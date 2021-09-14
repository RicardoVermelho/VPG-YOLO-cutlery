#!/usr/bin/env python

import time
import os
import random
import threading
import argparse
import matplotlib.pyplot as plt
import numpy as np
import scipy as sc
import cv2
from collections import namedtuple
import torch
from torch.autograd import Variable
from robot import Robot
from trainer import Trainer
from logger import Logger
import utils

import training_eval
from yolov5.inference import inferencev5, end_training_case
import ssl
ssl._create_default_https_context = ssl._create_unverified_context
from inference.depth2pc_final import depth2pc, getLatestFileInPath, getBBOXcoords
from predictions import compute_position
from pathlib import Path
from PIL import Image
import subprocess
import math

# Save previous images and heightmaps
prev_color_img = None
prev_depth_img = None
prev_color_heightmap = None
prev_depth_heightmap = None
prev_valid_depth_heightmap = None

# Initialize variables to count number of grasps and pushes
all_grasps = 0 # Total number of grasps
all_pushes = 0 # Total number of pushes
grasps_succ = 0 # Number of successful grasps
pushes_succ = 0 # Number of successful pushes

# Actual objects on table
objects_on_table = 3

# Initialize variable for action efficiency
action_eff = 0

# Home Dir
path_to_download_folder = str(os.path.join(Path.home(), "Downloads/VisualPushingGrasping"))
home_path = path_to_download_folder

def main(args):
    global home_path

    # --------------- Setup options ---------------
    is_sim = args.is_sim # Run in simulation?
    obj_mesh_dir = os.path.abspath(args.obj_mesh_dir) if is_sim else None # Directory containing 3D mesh files (.obj) of objects to be added to simulation
    num_obj = args.num_obj if is_sim else None # Number of objects to add to simulation
    tcp_host_ip = args.tcp_host_ip if not is_sim else None # IP and port to robot arm as TCP client (UR3)
    tcp_port = args.tcp_port if not is_sim else None
    rtc_host_ip = args.rtc_host_ip if not is_sim else None # IP and port to robot arm as real-time client (UR3)
    rtc_port = args.rtc_port if not is_sim else None
    if is_sim:
        workspace_limits = np.asarray([[0.05, 0.55], [-0.25, 0.25], [-0.0001, 0.4]])
    else:
        workspace_limits = np.asarray([[0.3, 0.748], [-0.224, 0.224], [-0.255, -0.1]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
    heightmap_resolution = args.heightmap_resolution # Meters per pixel of heightmap
    random_seed = args.random_seed
    force_cpu = args.force_cpu

    # ------------- Algorithm options -------------
    method = args.method # 'reactive' (supervised learning) or 'reinforcement' (reinforcement learning ie Q-learning)
    push_rewards = args.push_rewards if method == 'reinforcement' else None  # Use immediate rewards (from change detection) for pushing?
    future_reward_discount = args.future_reward_discount
    experience_replay = args.experience_replay # Use prioritized experience replay?
    heuristic_bootstrap = args.heuristic_bootstrap # Use handcrafted grasping algorithm when grasping fails too many times in a row?
    explore_rate_decay = args.explore_rate_decay
    grasp_only = args.grasp_only

    # -------------- Training options --------------
    training_iters = args.training_iters
    
    # -------------- Testing options --------------
    is_testing = args.is_testing
    max_test_trials = args.max_test_trials # Maximum number of test runs per case/scenario
    test_preset_cases = args.test_preset_cases
    test_preset_file = os.path.abspath(args.test_preset_file) if test_preset_cases else None

    # ------ Pre-loading and logging options ------
    load_snapshot = args.load_snapshot # Load pre-trained snapshot of model?
    snapshot_file = os.path.abspath(args.snapshot_file)  if load_snapshot else None
    continue_logging = args.continue_logging # Continue logging from previous session
    logging_directory = os.path.abspath(args.logging_directory) if continue_logging else os.path.abspath(home_path+'/logs')
    save_visualizations = args.save_visualizations # Save visualizations of FCN predictions? Takes 0.6s per training step if set to True


    # Set random seed
    np.random.seed(random_seed)

    # Initialize pick-and-place system (camera and robot)
    robot = Robot(is_sim, obj_mesh_dir, num_obj, workspace_limits,
                  tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
                  is_testing, test_preset_cases, test_preset_file)

    # Initialize trainer
    trainer = Trainer(method, push_rewards, future_reward_discount,
                      is_testing, load_snapshot, snapshot_file, force_cpu)

    # Initialize data logger
    logger = Logger(continue_logging, logging_directory)
    logger.save_camera_info(robot.cam_intrinsics, robot.cam_pose, robot.cam_depth_scale) # Save camera intrinsics and pose
    logger.save_heightmap_info(workspace_limits, heightmap_resolution) # Save heightmap parameters

    # Find last executed iteration of pre-loaded log, and load execution info and RL variables
    if continue_logging:
        trainer.preload(logger.transitions_directory)

    # Initialize variables for heuristic bootstrapping and exploration probability
    #no_change_count = [2, 2] if not is_testing else [0, 0]
    no_change_count = [0, 0]
    explore_prob = 0.5 if not is_testing else 0.0

    # Quick hack for nonlocal memory between threads in Python 2
    nonlocal_variables = {'executing_action' : False,
                          'primitive_action' : None,
                          'best_pix_ind' : None,
                          'push_success' : False,
                          'grasp_success' : False}
    
    # Variable to save previous obj position --> If there was a change we assume it was a successful push
    # Only for evaluating pushes
    previous_obj_pos = None

    # After a failed grasp, should we try it again?
    retry_grasp = False

    # Restart simulation and add new objects
    def sim_restart():
        robot.restart_sim()
        robot.add_objects()

    # Convert euler angles to axis angles and obtain the angle of a specified object
    def euler2axis(real_idx_object2grab):
        objects_orientation = robot.get_obj_positions_and_orientations()[1]

        # Grab the orientation of the object with the correct computed idx from Point Cloud Analysis
        object_pos_orientation = objects_orientation[real_idx_object2grab]

        # --> Euler angles (alpha beta gamma) to axis angles
        alpha = object_pos_orientation[0]
        beta = object_pos_orientation[1]
        gamma = object_pos_orientation[2]

        c1 = math.cos(alpha/2)
        c2 = math.cos(beta/2)
        c3 = math.cos(gamma/2)

        s1 = math.sin(alpha/2)
        s2 = math.sin(beta/2)
        s3 = math.sin(gamma/2)

        x = c1*c2*s3 - s1*s2*c3
        y = c1*s2*c3 + s1*c2*s3
        z = s1*c2*c3 - c1*s2*s3
        w = c1*c2*c3 - s1*s2*s3

        angle = 2*math.acos(w)

        obj_orientation_matrix = [x,y,z]
        return angle, obj_orientation_matrix
    
    def get_objects():
        global objects_on_table

        # Get objects in simulation
        object_positions = robot.get_obj_positions()

        # Filter for objects inside x-axis workspace
        object_positions = [obj for obj in object_positions if obj[0] >= 0.05 and obj[0] <= 0.55]
        # Filter for objects inside y-axis workspace
        object_positions = [obj for obj in object_positions if obj[1] >= -0.25 and obj[1] <= 0.25]
        # Filter for objects inside z-axis workspace
        object_positions = [obj for obj in object_positions if obj[2] >= -0.0001 and obj[2] <= 0.3]

        # If the robot grasped an object but it failed and yet the variable was decreased, needs to be corrected
        # Or if some objects are outside the defined boundaries for grasping and weren't grasped, then we have to update
        # the variabled that contains the number of objects on the table/simulation
        if objects_on_table != len(object_positions):
            objects_on_table = len(object_positions)
        return object_positions

    def get_objects_inside():
        # Get objects in simulation
        object_positions = robot.get_obj_positions()

        # Filter for objects inside x-axis, y-axis and z-axis workspace
        object_positions = [obj for obj in object_positions if (obj[0] >= 0.10 and obj[0] <= 0.50)]
        object_positions = [obj for obj in object_positions if (obj[1] >= -0.25 and obj[1] <= 0.25)]
        object_positions = [obj for obj in object_positions if obj[2] > -0.0001]
        return len(object_positions)

    # Parallel thread to process network output and execute actions
    # -------------------------------------------------------------
    def process_actions():
        while True:
            global objects_on_table
            global prev_color_img
            global prev_depth_img
            global prev_color_heightmap
            global prev_depth_heightmap
            global prev_valid_depth_heightmap

            if nonlocal_variables['executing_action']:
                print("_"*50 + "\n" + "Checking if there are objects in simulation to be grasped..."+ "\n" + "_"*50)
                while (get_objects_inside() == 0):
                    sim_restart()
                    objects_on_table = num_obj
                    print("_"*50)
                    print("Simulation restarted because there weren't objects to be grasped!")
                    print("_"*50)
                
                # If the robot grasped an object but it failed and yet the variable was decreased, needs to be corrected
                # Or if some objects are outside the defined boundaries for grasping and weren't grasped, then we have to update
                # the variabled that contains the number of objects on the table/simulation
                if objects_on_table != get_objects_inside() and objects_on_table > 0:
                    objects_on_table = get_objects_inside()
                elif objects_on_table == 0 or (objects_on_table > 0 and len(get_objects()) == 0):
                    print("No objects on simulation table! Restarting simulation...")
                    sim_restart()
                    objects_on_table = num_obj
                    print("Simulation restart completed! Resuming.")

                # Get new images before making inference, because there may have been
                # changes caused by the go through cup bug
                new_color_img, new_depth_img = robot.get_camera_data()
                new_depth_img = new_depth_img*robot.cam_depth_scale
                logger.save_images(trainer.iteration, new_color_img, new_depth_img, '0')

                # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
                new_color_heightmap, new_depth_heightmap = utils.get_heightmap(new_color_img, new_depth_img, robot.cam_intrinsics, robot.cam_pose, workspace_limits, heightmap_resolution)
                new_valid_depth_heightmap = new_depth_heightmap.copy()
                new_valid_depth_heightmap[np.isnan(new_valid_depth_heightmap)] = 0
                
                # Save RGB-D heightmaps
                logger.save_heightmaps(trainer.iteration, new_color_heightmap, new_valid_depth_heightmap, '0')
                
                # Execute inference on generated color image
                inferencev5(trainer.iteration)

                # Get objects in simulation
                all_objects = robot.get_obj_positions()
                
                all_objects2 = robot.get_obj_positions()
                new_all_objects2 = []
                for obj in all_objects2:
                    array = []
                    for num in obj:
                        array.append(round(num, 2))
                    new_all_objects2.append(array)

                # Filter for objects inside x-axis workspace
                all_objects = [obj for obj in all_objects if obj[0] >= 0.10 and obj[0] <= 0.55]
                # Filter for objects inside y-axis workspace
                all_objects = [obj for obj in all_objects if obj[1] >= -0.25 and obj[1] <= 0.25]
                # Filter for objects inside z-axis workspace
                all_objects = [obj for obj in all_objects if obj[2] > -0.0001 and obj[2] <= 0.4]

                # Execute convertion from depth image of object
                # with best confidence value to point cloud
                # and obtain its coordinates
                object2grab, obj_class, idx_object2grab = depth2pc(all_objects)

                real_idx_object2grab = 0
                tmp = [round(num, 2) for num in object2grab]
                for obj in new_all_objects2:
                    if obj[0] == tmp[0] and obj[1] == tmp[1]:
                        real_idx_object2grab = new_all_objects2.index(obj)
                
                '''print("Object index in array of objects: ", idx_object2grab)
                print("Object closer to the camera: ", object2grab)
                print("Object's class: ", obj_class), idx_object2grab'''
                
                # Calling global variables of grasps and pushes
                global all_grasps
                global all_pushes
                global grasps_succ
                global pushes_succ

                if nonlocal_variables['grasp_success'] == True or trainer.iteration == 0 or retry_grasp == True:
                    nonlocal_variables['primitive_action'] = 'grasp'
                    
                    print("\nATTEMPTING TO GRASP OBJECT FROM DEPTH CLOUD ANALYSIS...")
                    log_file = open(home_path+"/logs_latestFolder.txt", "r")
                    log_folder = log_file.readline()

                    # GET DEPTH HEIGHTMAP
                    path_depth_map = home_path+"/logs/"+log_folder+"/data/depth-heightmaps/"
                    depth_heightmap = str(getLatestFileInPath(os.path.relpath(path_depth_map)))
                    heightmap_to_crop = Image.open(path_depth_map+depth_heightmap)
                    
                    #depth_map = Image.open(home_path+"/inference/cropped_depth_heightmaps/"+getLatestFileInPath(home_path+"/inference/cropped_depth_heightmaps/"))
                    nonlocal_variables['best_pix_ind'] = trainer.grasp_heuristic(heightmap_to_crop)
                    print('Action: %s at (%d, %d, %d)' % (nonlocal_variables['primitive_action'], nonlocal_variables['best_pix_ind'][0], nonlocal_variables['best_pix_ind'][1], nonlocal_variables['best_pix_ind'][2]))

                    best_pix_x = nonlocal_variables['best_pix_ind'][2]
                    best_pix_y = nonlocal_variables['best_pix_ind'][1]
                    data_depthmap = np.asarray(heightmap_to_crop)
                    valid_depth_heightmap2 = data_depthmap.copy()
                    valid_depth_heightmap2[np.isnan(valid_depth_heightmap2)] = 0

                    angle, obj_orientation_matrix = euler2axis(real_idx_object2grab)

                    best_rotation_angle = obj_orientation_matrix[2]
                    
                    # Compute primitive position closer to the object2grab
                    best_pix_ind, primitive_position = compute_position(heightmap_resolution, workspace_limits, new_valid_depth_heightmap, object2grab, nonlocal_variables['primitive_action'], trainer, grasp_pred=grasp_predictions)
                    nonlocal_variables['best_pix_ind'] = best_pix_ind
                    
                    nonlocal_variables['grasp_success'] = robot.grasp(primitive_position, object2grab, best_rotation_angle, angle, workspace_limits, idx_object2grab, obj_class)
                    all_grasps += 1

                    all_objects_sim = robot.get_obj_positions()
                    # Filter for objects inside x-axis workspace
                    all_objects_sim = [obj for obj in all_objects_sim if obj[0] >= 0.10 and obj[0] <= 0.55]
                    # Filter for objects inside y-axis workspace
                    all_objects_sim = [obj for obj in all_objects_sim if obj[1] >= -0.25 and obj[1] <= 0.25]
                    # Filter for objects inside z-axis workspace
                    all_objects_sim = [obj for obj in all_objects_sim if obj[2] > -0.0001 and obj[2] <= 0.4]

                    if len(all_objects_sim) == objects_on_table:
                        nonlocal_variables['grasp_success'] = False

                    if nonlocal_variables['grasp_success'] == True:
                        no_change_count[1] = 0
                        grasps_succ += 1
                        objects_on_table -= 1
                    else:
                        no_change_count[1] += 1
                        retry_grasp = False
                    
                    trainer.executed_action_log.append([1, nonlocal_variables['best_pix_ind'][0], nonlocal_variables['best_pix_ind'][1], nonlocal_variables['best_pix_ind'][2]]) # 1 - grasp
                    logger.write_to_log('executed-action', trainer.executed_action_log)

                    # Save information for next training step
                    prev_color_img = new_color_img.copy()
                    prev_depth_img = new_depth_img.copy()
                    prev_color_heightmap = new_color_heightmap.copy()
                    prev_depth_heightmap = new_depth_heightmap.copy()
                    prev_valid_depth_heightmap = new_valid_depth_heightmap.copy()
                
                    print('Grasp successful: %r' % (nonlocal_variables['grasp_success']))
                    print("GRASP OBJECT ATTEMPT DONE!")
                else:
                    nonlocal_variables['primitive_action'] = 'push'
                    
                    print("\nATTEMPTING TO PUSH OBJECT FROM DEPTH CLOUD ANALYSIS...")
                    log_file = open(home_path+"/logs_latestFolder.txt", "r")
                    log_folder = log_file.readline()

                    # GET DEPTH HEIGHTMAP
                    path_depth_map = home_path+"/logs/"+log_folder+"/data/depth-heightmaps/"
                    depth_heightmap = str(getLatestFileInPath(os.path.relpath(path_depth_map)))
                    heightmap_to_crop = Image.open(path_depth_map+depth_heightmap)
                    
                    nonlocal_variables['best_pix_ind'] = trainer.push_heuristic(heightmap_to_crop)
                    print('Action: %s at (%d, %d, %d)' % (nonlocal_variables['primitive_action'], nonlocal_variables['best_pix_ind'][0], nonlocal_variables['best_pix_ind'][1], nonlocal_variables['best_pix_ind'][2]))

                    best_pix_x = nonlocal_variables['best_pix_ind'][2]
                    best_pix_y = nonlocal_variables['best_pix_ind'][1]
                    data_depthmap = np.asarray(heightmap_to_crop)
                    valid_depth_heightmap2 = data_depthmap.copy()
                    valid_depth_heightmap2[np.isnan(valid_depth_heightmap2)] = 0
                    
                    angle, obj_orientation_matrix = euler2axis(real_idx_object2grab)

                    best_rotation_angle = np.deg2rad(nonlocal_variables['best_pix_ind'][0]*(360.0/trainer.model.num_rotations))

                    previous_obj_pos = object2grab
                    
                    # Compute primitive position closer to the object2grab
                    best_pix_ind, primitive_position = compute_position(heightmap_resolution, workspace_limits, new_valid_depth_heightmap, object2grab, nonlocal_variables['primitive_action'], trainer, push_pred=push_predictions)
                    nonlocal_variables['best_pix_ind'] = best_pix_ind
                    
                    nonlocal_variables['push_success'] = robot.push(primitive_position, object2grab, best_rotation_angle, angle, workspace_limits)
                    all_pushes += 1

                    previous_obj_pos = [round(num, 2) for obj in all_objects for num in obj]
                    pos_objects_sim = robot.get_obj_positions()
                    # Filter for objects inside x-axis workspace
                    pos_objects_sim = [obj for obj in pos_objects_sim if obj[0] >= 0.10 and obj[0] <= 0.55]
                    # Filter for objects inside y-axis workspace
                    pos_objects_sim = [obj for obj in pos_objects_sim if obj[1] >= -0.25 and obj[1] <= 0.25]
                    # Filter for objects inside z-axis workspace
                    pos_objects_sim = [obj for obj in pos_objects_sim if obj[2] > -0.0001 and obj[2] <= 0.4]
                    pos_objects_sim = [round(num, 2) for obj in pos_objects_sim for num in obj]

                    if previous_obj_pos == pos_objects_sim:
                        nonlocal_variables['push_success'] = False

                    if nonlocal_variables['push_success'] == True:
                        pushes_succ += 1
                        no_change_count[0] = 0
                    else:
                        no_change_count[0] += 1 # If push was False right from the moment it was made
                    
                    trainer.executed_action_log.append([0, nonlocal_variables['best_pix_ind'][0], nonlocal_variables['best_pix_ind'][1], nonlocal_variables['best_pix_ind'][2]]) # 0 - push
                    logger.write_to_log('executed-action', trainer.executed_action_log)

                    # Save information for next training step
                    prev_color_img = new_color_img.copy()
                    prev_depth_img = new_depth_img.copy()
                    prev_color_heightmap = new_color_heightmap.copy()
                    prev_depth_heightmap = new_depth_heightmap.copy()
                    prev_valid_depth_heightmap = new_valid_depth_heightmap.copy()

                    retry_grasp = True
                
                    print('Push successful: %r' % (nonlocal_variables['push_success']))
                    print("PUSHED ATTEMPT DONE!")

                # Enter directory where depth heightmaps are saved and delete visualization images
                subprocess.call('rm -f ' + home_path + '/inference/labels/visualization.push.png', shell=True)
                subprocess.call('rm -f ' + home_path + '/inference/labels/visualization.grasp.png', shell=True)
                # Enter directory where output YoLo files are saved and delete visualization images
                subprocess.call('rm -f ' + home_path + '/inference/output/visualization.push.png', shell=True)
                subprocess.call('rm -f ' + home_path + '/inference/output/visualization.grasp.png', shell=True)
                # Enter directory where previous output YoLo files are saved and delete visualization images
                subprocess.call('rm -f ' + home_path + '/inference/previous_output/visualization.push.png', shell=True)
                subprocess.call('rm -f ' + home_path + '/inference/previous_output/visualization.grasp.png', shell=True)
                # Enter directory where depth images files are saved and delete visualization images
                subprocess.call('rm -f ' + home_path + "/logs/"+log_folder+"/data/depth-images/visualization.push.png", shell=True)
                subprocess.call('rm -f ' + home_path + "/logs/"+log_folder+"/data/depth-images/visualization.grasp.png", shell=True)
                # Enter directory where depth heightmaps files are saved and delete visualization images
                subprocess.call('rm -f ' + home_path + "/logs/"+log_folder+"/data/depth-heightmaps/visualization.push.png", shell=True)
                subprocess.call('rm -f ' + home_path + "/logs/"+log_folder+"/data/depth-heightmaps/visualization.grasp.png", shell=True)
            nonlocal_variables['executing_action'] = False

            time.sleep(0.01)
    action_thread = threading.Thread(target=process_actions)
    action_thread.daemon = True
    action_thread.start()
    exit_called = False
    # -------------------------------------------------------------
    # -------------------------------------------------------------

    # Start main training/testing loop
    # Total elapsed time
    totaltime = 0

    # Initialize object of class training eval for in training iterations evaluation
    eval_iter = training_eval.train_eval()

    # Do we continue simulation or restart it because some of the objects are outside the container?
    continue_sim = False

    while True:
        global all_grasps # Call global variable of all grasps
        global all_pushes # Call global variable of all pushes
        global grasps_succ # Call global variable of succ grasps
        global pushes_succ # Call global variable of succ pushes
        global action_eff  # Call global variable of action efficiency

        global objects_on_table
        global prev_color_img
        global prev_depth_img
        global prev_color_heightmap
        global prev_depth_heightmap
        global prev_valid_depth_heightmap

        print('\n%s iteration: %d' % ('Testing' if is_testing else 'Training', trainer.iteration))
        
        # Calculate initial time of the current iteration
        iteration_time_0 = time.time()

        ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        ### (RE)START SIMULATION AND CHECK IF ALL OBJECTS ARE INSIDE INITIAL CONTAINER ###
        # Make sure simulation is still stable (if not, reset simulation)
        if is_sim: robot.check_sim()
        
        # NOTE Uncomment after getting results with new objects
        if objects_on_table > 0 and continue_sim == False:
            while continue_sim == False:
                # Check if all objects are inside cup and not laying on the ground
                check_objects = robot.get_obj_positions()
                # Filter for objects with z-axis coordinate outside simulation workspace
                objects_checked_sim = [obj for obj in check_objects if obj[2] < -0.0001]
                #objects_checked_sim = [obj for obj in check_objects if obj[2] < 0.02]
                if len(objects_checked_sim) == 0:
                    continue_sim = True
                else:
                    continue_sim = False
                    sim_restart()
                    objects_on_table = num_obj

        ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        ### COMPUTE COLOR AND DEPTH IMAGES. COMPUTE VALID COLOR AND DEPTH HEIGHTMAPS ###
        # Get latest RGB-D image
        color_img, depth_img = robot.get_camera_data()
        depth_img = depth_img * robot.cam_depth_scale # Apply depth scale from calibration

        # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
        color_heightmap, depth_heightmap = utils.get_heightmap(color_img, depth_img, robot.cam_intrinsics, robot.cam_pose, workspace_limits, heightmap_resolution)
        valid_depth_heightmap = depth_heightmap.copy()
        valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0

        # Save RGB-D images and RGB-D heightmaps
        logger.save_images(trainer.iteration, color_img, depth_img, '0')
        logger.save_heightmaps(trainer.iteration, color_heightmap, valid_depth_heightmap, '0')

        ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        ### CHECK IF SIMULATION TABLE IS EMPTY AND DECIDE IF TO RESET IT OR NOT ###
        # Reset simulation or pause real-world training if table is empty
        stuff_count = np.zeros(valid_depth_heightmap.shape)
        stuff_count[valid_depth_heightmap > 0.02] = 1
        
        #empty_threshold = 300
        if is_sim and is_testing: # In testing cases, not in training simulation
            empty_threshold = 6

        print("Objects on table: ", objects_on_table)
        #If the sum of pixels from simulation is equal to threshold or there was no changes in simulation during 5 iterations
        if objects_on_table == 0 or (is_sim and no_change_count[0] + no_change_count[1] > num_obj+3):
            #Reset environment if in simulation
            if is_sim:
                # Get images from the spots where objects go after being grasped and run YoLo
                end_training_case(trainer.iteration, heightmap_resolution)

                # Restart simulation
                #print('Not enough objects in view (value: %d)! Repositioning objects.' % (np.sum(stuff_count)))
                print('No objects in simulation or maximum number of attempts reached... Restarting simulation!')
                sim_restart()
                # NOTE Uncomment after getting results with new objects
                continue_sim = False

                while continue_sim == False:
                    # Check if all objects are inside cup and not laying on the ground
                    check_objects = robot.get_obj_positions()
                    # Filter for objects with z-axis coordinate outside simulation workspace
                    objects_checked_sim = [obj for obj in check_objects if obj[2] < -0.0001]
                    #objects_checked_sim = [obj for obj in check_objects if obj[2] < 0.02]
                    if len(objects_checked_sim) == 0:
                        continue_sim = True
                    else:
                        sim_restart()
                        continue_sim = False
                
                no_change_count = [0,0]
                objects_on_table = num_obj

                if is_testing: # If at end of test run, re-load original weights (before test run)
                    trainer.model.load_state_dict(torch.load(snapshot_file))
            #Reset real-life robot
            else:
                # print('Not enough stuff on the table (value: %d)! Pausing for 30 seconds.' % (np.sum(stuff_count)))
                # time.sleep(30)
                print('Not enough stuff on the table (value: %d)! Flipping over bin of objects...' % (np.sum(stuff_count)))
                robot.restart_real()

            trainer.clearance_log.append([trainer.iteration])
            logger.write_to_log('clearance', trainer.clearance_log)
            if is_testing and len(trainer.clearance_log) >= max_test_trials:
                exit_called = True # Exit after training thread (backprop and saving labels)
            continue

        ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        ### PASS IMAGES THROUGH NETWORK, GET PUSH AND GRASP PREDICITIONS AND COMPUTE EXPERIENCE REPLAY ###
        if not exit_called:

            # Run forward pass with network to get affordances
            push_predictions, grasp_predictions, state_feat = trainer.forward(color_heightmap, valid_depth_heightmap, is_volatile=True)
            
            # Execute best primitive action on robot in another thread
            nonlocal_variables['executing_action'] = True

        # Run training iteration in current thread (aka training thread)
        if 'prev_color_img' in locals():

            # Detect changes
            depth_diff = abs(depth_heightmap - prev_depth_heightmap)
            depth_diff[np.isnan(depth_diff)] = 0
            depth_diff[depth_diff > 0.3] = 0
            depth_diff[depth_diff < 0.01] = 0
            depth_diff[depth_diff > 0] = 1
            change_threshold = 300
            change_value = np.sum(depth_diff)
            change_detected = change_value > change_threshold or prev_grasp_success
            print('Change detected: %r (value: %d)' % (change_detected, change_value))

            '''if change_detected:
                if prev_primitive_action == 'push':
                    no_change_count[0] = 0
                elif prev_primitive_action == 'grasp':
                    no_change_count[1] = 0
            else:
                if prev_primitive_action == 'push':
                    no_change_count[0] += 1
                elif prev_primitive_action == 'grasp':
                    no_change_count[1] += 1'''

            # Compute training labels
            label_value, prev_reward_value = trainer.get_label_value(prev_primitive_action, prev_push_success, prev_grasp_success, change_detected, prev_push_predictions, prev_grasp_predictions, color_heightmap, valid_depth_heightmap)
            trainer.label_value_log.append([label_value])
            logger.write_to_log('label-value', trainer.label_value_log)
            trainer.reward_value_log.append([prev_reward_value])
            logger.write_to_log('reward-value', trainer.reward_value_log)

            # Backpropagate
            trainer.backprop(prev_color_heightmap, prev_valid_depth_heightmap, prev_primitive_action, prev_best_pix_ind, label_value)

            # Adjust exploration probability
            if not is_testing:
                explore_prob = max(0.5 * np.power(0.9998, trainer.iteration),0.1) if explore_rate_decay else 0.5

            # Do sampling for experience replay
            if experience_replay and not is_testing:
                sample_primitive_action = prev_primitive_action
                if sample_primitive_action == 'push':
                    sample_primitive_action_id = 0
                    if method == 'reactive':
                        sample_reward_value = 0 if prev_reward_value == 1 else 1 # random.randint(1, 2) # 2
                    elif method == 'reinforcement':
                        sample_reward_value = 0 if prev_reward_value == 0.5 else 0.5
                elif sample_primitive_action == 'grasp':
                    sample_primitive_action_id = 1
                    if method == 'reactive':
                        sample_reward_value = 0 if prev_reward_value == 1 else 1
                    elif method == 'reinforcement':
                        sample_reward_value = 0 if prev_reward_value == 1 else 1

                # Get samples of the same primitive but with different results
                sample_ind = np.argwhere(np.logical_and(np.asarray(trainer.reward_value_log)[1:trainer.iteration,0] == sample_reward_value, np.asarray(trainer.executed_action_log)[1:trainer.iteration,0] == sample_primitive_action_id))
                
                if sample_ind.size > 0:

                    # Find sample with highest surprise value
                    if method == 'reactive':
                        sample_surprise_values = np.abs(np.asarray(trainer.predicted_value_log)[sample_ind[:,0]] - (1 - sample_reward_value))
                    elif method == 'reinforcement':
                        sample_surprise_values = np.abs(np.asarray(trainer.predicted_value_log)[sample_ind[:,0]] - np.asarray(trainer.label_value_log)[sample_ind[:,0]])
                    sorted_surprise_ind = np.argsort(sample_surprise_values[:,0])
                    sorted_sample_ind = sample_ind[sorted_surprise_ind,0]
                    pow_law_exp = 2
                    rand_sample_ind = int(np.round(np.random.power(pow_law_exp, 1)*(sample_ind.size-1)))
                    sample_iteration = sorted_sample_ind[rand_sample_ind]
                    print('Experience replay: iteration %d (surprise value: %f)' % (sample_iteration, sample_surprise_values[sorted_surprise_ind[rand_sample_ind]]))

                    # Load sample RGB-D heightmap
                    sample_color_heightmap = cv2.imread(os.path.join(logger.color_heightmaps_directory, '%06d.0.color.png' % (sample_iteration)))
                    sample_color_heightmap = cv2.cvtColor(sample_color_heightmap, cv2.COLOR_BGR2RGB)
                    sample_depth_heightmap = cv2.imread(os.path.join(logger.depth_heightmaps_directory, '%06d.0.depth.png' % (sample_iteration)), -1)
                    sample_depth_heightmap = sample_depth_heightmap.astype(np.float32)/100000

                    # Compute forward pass with sample
                    with torch.no_grad():
                        sample_push_predictions, sample_grasp_predictions, sample_state_feat = trainer.forward(sample_color_heightmap, sample_depth_heightmap, is_volatile=True)

                    # Load next sample RGB-D heightmap
                    next_sample_color_heightmap = cv2.imread(os.path.join(logger.color_heightmaps_directory, '%06d.0.color.png' % (sample_iteration+1)))
                    next_sample_color_heightmap = cv2.cvtColor(next_sample_color_heightmap, cv2.COLOR_BGR2RGB)
                    next_sample_depth_heightmap = cv2.imread(os.path.join(logger.depth_heightmaps_directory, '%06d.0.depth.png' % (sample_iteration+1)), -1)
                    next_sample_depth_heightmap = next_sample_depth_heightmap.astype(np.float32)/100000

                    sample_push_success = sample_reward_value == 0.5
                    sample_grasp_success = sample_reward_value == 1
                    sample_change_detected = sample_push_success
                    # new_sample_label_value, _ = trainer.get_label_value(sample_primitive_action, sample_push_success, sample_grasp_success, sample_change_detected, sample_push_predictions, sample_grasp_predictions, next_sample_color_heightmap, next_sample_depth_heightmap)

                    # Get labels for sample and backpropagate
                    sample_best_pix_ind = (np.asarray(trainer.executed_action_log)[sample_iteration,1:4]).astype(int)
                    trainer.backprop(sample_color_heightmap, sample_depth_heightmap, sample_primitive_action, sample_best_pix_ind, trainer.label_value_log[sample_iteration])

                    # Recompute prediction value and label for replay buffer
                    if sample_primitive_action == 'push':
                        trainer.predicted_value_log[sample_iteration] = [np.max(sample_push_predictions)]
                        # trainer.label_value_log[sample_iteration] = [new_sample_label_value]
                    elif sample_primitive_action == 'grasp':
                        trainer.predicted_value_log[sample_iteration] = [np.max(sample_grasp_predictions)]
                        # trainer.label_value_log[sample_iteration] = [new_sample_label_value]

                else:
                    print('Not enough prior training samples. Skipping experience replay.')

            # Save model snapshot
            if not is_testing:
                logger.save_backup_model(trainer.model, method)
                if trainer.iteration % 50 == 0:
                    logger.save_model(trainer.iteration, trainer.model, method)
                    if trainer.use_cuda:
                        trainer.model = trainer.model.cuda()

        # Sync both action thread and training thread
        while nonlocal_variables['executing_action']:
            time.sleep(0.01)

        if exit_called:
            break

        ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        ### SAVE INFORMATION FOR NEXT ITERATION FROM THE CURRENT ITERATION ###
        # Save information for next training step
        prev_color_img = color_img.copy()
        prev_depth_img = depth_img.copy()
        prev_color_heightmap = color_heightmap.copy()
        prev_depth_heightmap = depth_heightmap.copy()
        prev_valid_depth_heightmap = valid_depth_heightmap.copy()
        prev_push_success = nonlocal_variables['push_success']
        prev_grasp_success = nonlocal_variables['grasp_success']
        prev_primitive_action = nonlocal_variables['primitive_action']
        prev_push_predictions = push_predictions.copy()
        prev_grasp_predictions = grasp_predictions.copy()
        prev_best_pix_ind = nonlocal_variables['best_pix_ind']

        ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        ### COMPUTE METRICS' RESULTS ###
        print("\n"+"="*50)
        print("Evaluating grasp-to-all-ratio and push-to-all-ratio...\n")
        eval_iter.evaluate_iter(trainer.iteration, all_grasps, grasps_succ, all_pushes, pushes_succ, num_obj, nonlocal_variables['primitive_action'])
        print("="*50+"\n")

        # Calculate ending time of the current iteration
        iteration_time_1 = time.time()
        # Calculate total time took for the current iteration
        totaltime += iteration_time_1-iteration_time_0

        # Increment iteration value
        trainer.iteration += 1
        
        ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        ### CHECK IF IT IS TIME TO STOP THE SIMULATION (2500 ITERATIONS) ###
        # Stop criteria (VPG authors train their algorithm for 2500)
        if trainer.iteration == int(training_iters):
            g_ratios = eval_iter.get_grasp_ratios()
            print("Total average of grasp success: ", round(sum(g_ratios)/len(g_ratios), 2))

            p_ratios = eval_iter.get_push_ratios()
            print("Total average of push success: ", round(sum(p_ratios)/len(p_ratios), 2))

            # plotting the points 
            plt.plot(g_ratios, color='green', linestyle='solid', linewidth = 2,
                    marker='.', markerfacecolor='green', markersize=5)
            plt.plot(p_ratios, color='red', linestyle='solid', linewidth = 2,
                    marker='.', markerfacecolor='red', markersize=5)
            
            # setting x and y axis range
            plt.ylim([0,100])
            plt.xlim([0,250])
            
            # naming the x axis
            plt.xlabel('Number of training steps')
            # naming the y axis
            plt.ylabel('Performance Percentage')
            
            # giving a title to graph
            plt.title('Training results Visual Pushing-for-Grasping')

            plt.savefig(home_path+"/results_sim/results_vpg.png")
  
            plt.show(block=False)

            plt.pause(3)
            plt.close()

            print('Time elapsed: %f' % (iteration_time_1-iteration_time_0))
            break

        print('Time elapsed: %f' % (iteration_time_1-iteration_time_0))
    total_time = open(home_path+"/results_sim/total_time_sim.txt", "a")
    print("Total time elapsed in seconds: " + str(int(totaltime)))
    total_time.write("Total time elapsed in seconds: " + str(int(totaltime)) + "\n")
    print("Total time elapsed in minutes: " + str(int(totaltime/60)))
    total_time.write("Total time elapsed in minutes: " + str(int(totaltime/60)) + "\n")
    total_time.close()
    
    # Compress training session to a tar package
    #subprocess.call(['tar', '-czf', "vpg2_backup.tar.xz", home_path])

if __name__ == '__main__':

    # Parse arguments
    parser = argparse.ArgumentParser(description='Train robotic agents to learn how to plan complementary pushing and grasping actions for manipulation with deep reinforcement learning in PyTorch.')

    # --------------- Setup options ---------------
    parser.add_argument('--is_sim', dest='is_sim', action='store_true', default=False,                                    help='run in simulation?')
    parser.add_argument('--obj_mesh_dir', dest='obj_mesh_dir', action='store', default='objects/blocks',                  help='directory containing 3D mesh files (.obj) of objects to be added to simulation')
    parser.add_argument('--num_obj', dest='num_obj', type=int, action='store', default=3,                                help='number of objects to add to simulation')
    parser.add_argument('--tcp_host_ip', dest='tcp_host_ip', action='store', default='100.127.7.223',                     help='IP address to robot arm as TCP client (UR3)')
    parser.add_argument('--tcp_port', dest='tcp_port', type=int, action='store', default=30002,                           help='port to robot arm as TCP client (UR3)')
    parser.add_argument('--rtc_host_ip', dest='rtc_host_ip', action='store', default='100.127.7.223',                     help='IP address to robot arm as real-time client (UR3)')
    parser.add_argument('--rtc_port', dest='rtc_port', type=int, action='store', default=30003,                           help='port to robot arm as real-time client (UR3)')
    parser.add_argument('--heightmap_resolution', dest='heightmap_resolution', type=float, action='store', default=0.002, help='meters per pixel of heightmap')
    parser.add_argument('--random_seed', dest='random_seed', type=int, action='store', default=1234,                      help='random seed for simulation and neural net initialization')
    parser.add_argument('--cpu', dest='force_cpu', action='store_true', default=False,                                    help='force code to run in CPU mode')

    # ------------- Algorithm options -------------
    parser.add_argument('--method', dest='method', action='store', default='reinforcement',                               help='set to \'reactive\' (supervised learning) or \'reinforcement\' (reinforcement learning ie Q-learning)')
    parser.add_argument('--push_rewards', dest='push_rewards', action='store_true', default=False,                        help='use immediate rewards (from change detection) for pushing?')
    parser.add_argument('--future_reward_discount', dest='future_reward_discount', type=float, action='store', default=0.5)
    parser.add_argument('--experience_replay', dest='experience_replay', action='store_true', default=False,              help='use prioritized experience replay?')
    parser.add_argument('--heuristic_bootstrap', dest='heuristic_bootstrap', action='store_true', default=False,          help='use handcrafted grasping algorithm when grasping fails too many times in a row during training?')
    parser.add_argument('--explore_rate_decay', dest='explore_rate_decay', action='store_true', default=False)
    parser.add_argument('--grasp_only', dest='grasp_only', action='store_true', default=False)

    # -------------- Training options --------------
    parser.add_argument('--training_iters', dest='training_iters', action='store', default=2500)

    # -------------- Testing options --------------
    parser.add_argument('--is_testing', dest='is_testing', action='store_true', default=False)
    parser.add_argument('--max_test_trials', dest='max_test_trials', type=int, action='store', default=30,                help='maximum number of test runs per case/scenario')
    parser.add_argument('--test_preset_cases', dest='test_preset_cases', action='store_true', default=False)
    parser.add_argument('--test_preset_file', dest='test_preset_file', action='store', default='test-10-obj-01.txt')

    # ------ Pre-loading and logging options ------
    parser.add_argument('--load_snapshot', dest='load_snapshot', action='store_true', default=False,                      help='load pre-trained snapshot of model?')
    parser.add_argument('--snapshot_file', dest='snapshot_file', action='store')
    parser.add_argument('--continue_logging', dest='continue_logging', action='store_true', default=False,                help='continue logging from previous session?')
    parser.add_argument('--logging_directory', dest='logging_directory', action='store')
    parser.add_argument('--save_visualizations', dest='save_visualizations', action='store_true', default=False,          help='save visualizations of FCN predictions?')

    # Run main program with specified arguments
    args = parser.parse_args()
    main(args)
