import subprocess
from pathlib import Path
import os.path
import numpy as np
import cv2

import sys
sys.path.append('..')
import utils
from simulation import vrep

# Home Dir
path_to_download_folder = str(os.path.join(Path.home(), "Downloads/VisualPushingGraspingV2"))
home_path = path_to_download_folder
sim_client = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

### GET COLOR AND DEPTH IMAGES FROM VISION SENSOR DESTINED FOR FINAL COURSE OF EACH TRAINING CASE###
def get_vision_sensor_data(sim_client):
    # Get handle to camera
    sim_ret, cam_handle = vrep.simxGetObjectHandle(sim_client, 'end_iteration_sensor', vrep.simx_opmode_blocking)

    # Get camera pose and intrinsics in simulation
    sim_ret, cam_position = vrep.simxGetObjectPosition(sim_client, cam_handle, -1, vrep.simx_opmode_blocking)
    sim_ret, cam_orientation = vrep.simxGetObjectOrientation(sim_client, cam_handle, -1, vrep.simx_opmode_blocking)
    cam_trans = np.eye(4,4)
    cam_trans[0:3,3] = np.asarray(cam_position)
    cam_orientation = [-cam_orientation[0], -cam_orientation[1], -cam_orientation[2]]
    cam_rotm = np.eye(4,4)
    cam_rotm[0:3,0:3] = np.linalg.inv(utils.euler2rotm(cam_orientation))
    cam_pose = np.dot(cam_trans, cam_rotm) # Compute rigid transformation representating camera pose
    cam_intrinsics = np.asarray([[618.62, 0, 320], [0, 618.62, 240], [0, 0, 1]])
    cam_depth_scale = 1

    color_img = None
    depth_img = None

    # Get background image
    # Get color image from simulation
    sim_ret, resolution, raw_image = vrep.simxGetVisionSensorImage(sim_client, cam_handle, 0, vrep.simx_opmode_blocking)
    color_img = np.asarray(raw_image)
    color_img.shape = (resolution[1], resolution[0], 3)
    color_img = color_img.astype(np.float)/255
    color_img[color_img < 0] += 1
    color_img *= 255
    color_img = np.fliplr(color_img)
    color_img = color_img.astype(np.uint8)

    # Get depth image from simulation
    sim_ret, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(sim_client, cam_handle, vrep.simx_opmode_blocking)
    depth_img = np.asarray(depth_buffer)
    depth_img.shape = (resolution[1], resolution[0])
    depth_img = np.fliplr(depth_img)
    zNear = 0.01
    zFar = 10
    depth_img = depth_img * (zFar - zNear) + zNear

    # Get background image
    bg_color_img, bg_depth_img = color_img, depth_img
    bg_depth_img = bg_depth_img * cam_depth_scale

    return bg_color_img, bg_depth_img, cam_intrinsics, cam_pose

### SAVE BOTH COLOR AND DEPTH IMAGES TO DESIRED DIRECTORIES ###
def save_images(iteration, color_image, depth_image, mode):
    global home_path
    color_images_directory = home_path+"/inference/end_training_case/color-images"
    depth_images_directory = home_path+"/inference/end_training_case/depth-images"

    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
    cv2.imwrite(os.path.join(color_images_directory, '%06d.%s.color.png' % (iteration, mode)), color_image)
    depth_image = np.round(depth_image * 10000).astype(np.uint16) # Save depth in 1e-4 meters
    cv2.imwrite(os.path.join(depth_images_directory, '%06d.%s.depth.png' % (iteration, mode)), depth_image)

### SAVE COLOR AND DEPTH HEIGHTMAPS TO DESIRED DIRECTORIES ###
def save_heightmaps(iteration, color_heightmap, depth_heightmap, mode):
    global home_path
    color_heightmaps_directory = home_path+"/inference/end_training_case/color-heightmaps"
    depth_heightmaps_directory = home_path+"/inference/end_training_case/depth-heightmaps"

    color_heightmap = cv2.cvtColor(color_heightmap, cv2.COLOR_RGB2BGR)
    cv2.imwrite(os.path.join(color_heightmaps_directory, '%06d.%s.color.png' % (iteration, mode)), color_heightmap)
    depth_heightmap = np.round(depth_heightmap * 100000).astype(np.uint16) # Save depth in 1e-5 meters
    cv2.imwrite(os.path.join(depth_heightmaps_directory, '%06d.%s.depth.png' % (iteration, mode)), depth_heightmap)

def find_files(substring, path='.', extensions=[]):
    from os import listdir
    from re import search, IGNORECASE
    return [f for f in listdir(path) 
            if search(r'%s' % substring, f, IGNORECASE) 
            and any(search(r'%s$' % ext, f, IGNORECASE) 
                    for ext in extensions)]

def image_files(substring, path="."):
    return find_files(substring, path=path, extensions=['jpg', 'png'])

### METHOD TO APPLY INFERENCE AT THE BEGINING OF EACH ITERATION ###
def inferencev5(iteration):
	substring = str(iteration)
	path_to_download_folder = str(os.path.join(Path.home(), "Downloads/VisualPushingGraspingV2"))
	home_path = path_to_download_folder

	text_file = open(home_path+"/logs_latestFolder.txt", "r")
	log_folder = text_file.readline()

	source = home_path + '/logs/' + log_folder + '/data/color-images'

	images = image_files(substring, path=source)
	img_inference = images[-1] # Returns last image with the substring we want

	print("\n" + "*"*50)
	print("MAKING INFERENCE ON THE CURRENT COLOR IMAGE FROM LOG FOLDER...\n")
	subprocess.call('python3 ' + home_path + '/yolov5/detect.py --weights ' + home_path + '/yolov5/weights/ycb_dataset/best_yolov5x_clothing.pt --home_path ' + home_path + ' --output ' + home_path + '/inference/output  --img 640 --conf 0.4 --source ' + source + '/' + img_inference, shell=True)
	#subprocess.call('python3 ' + home_path + '/yolov5/detect.py --weights ' + home_path + '/yolov5/weights/turbosquid_models/best_yolov5x_clothing.pt --home_path ' + home_path + ' --output ' + home_path + '/inference/output  --img 640 --conf 0.4 --source ' + source + '/' + img_inference, shell=True)
	#subprocess.call('cp ' + home_path + '/inference/output/' + img_inference + ' ' + home_path + '/inference/previous_output', shell=True)
	print("\nINFERENCE ON CURRENT IMAGE ITERATION FINISHED!")
	print("*"*50 + "\n")

### METHOD TO GET/SAVE COLOR AND DEPTH IMAGES FROM VISION SENSOR DESTINED FOR FINAL COURSE OF EACH TRAINING CASE
### AND RUN YOLO OVER THE COLOR IMAGE 
def end_training_case(iteration, heightmap_res):
    global sim_client

    # Get color and depth images
    color_img, depth_img, cam_intrinsics, cam_pose = get_vision_sensor_data(sim_client)

    workspace_limits = np.asarray([[0.05, 0.55], [-0.25, 0.25], [-0.0001, 0.4]])
    heightmap_resolution = heightmap_res # Meters per pixel of heightmap

    # Save the images in designated folder
    # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
    color_heightmap, depth_heightmap = utils.get_heightmap(color_img, depth_img, cam_intrinsics, cam_pose, workspace_limits, heightmap_resolution)
    valid_depth_heightmap = depth_heightmap.copy()
    valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0

    # Save RGB-D images and RGB-D heightmaps
    save_images(iteration, color_img, depth_img, '0')
    save_heightmaps(iteration, color_heightmap, valid_depth_heightmap, '0')

    # Run inference over images from end of training iteration
    substring = str(iteration)
    source = home_path+"/inference/end_training_case/color-images"

    images = image_files(substring, path=source)
    img_inference = images[-1] # Returns last image with the substring we want

    output = home_path+"/inference/end_training_case/yolo_detect"

    print("\n" + "*"*50)
    print("MAKING INFERENCE ON THE COLOR IMAGE FROM FINAL TRAINING ITERATION FOLDER BEFORE RESTART...\n")

    #subprocess.call('python3 ' + home_path + '/yolov5/detect.py --weights ' + home_path + '/yolov5/weights/turbosquid_models/best_yolov5x_clothing.pt --home_path ' + home_path + ' --output ' + output + ' --img 640 --conf 0.4 --source ' + source + '/' + img_inference, shell=True)
    subprocess.call('python3 ' + home_path + '/yolov5/detect.py --weights ' + home_path + '/yolov5/weights/ycb_dataset/best_yolov5x_clothing.pt --home_path ' + home_path + ' --output ' + output + ' --img 640 --conf 0.4 --source ' + source + '/' + img_inference, shell=True)

    print("\nINFERENCE ON THE COLOR IMAGE FROM FINAL TRAINING ITERATION FOLDER FINISHED!")
    print("*"*50 + "\n")

    # Log file containing each iteration number where this process was made
    training_case = open(home_path+"/inference/end_training_case/end_training_case.txt", "a")
    training_case.write("End training case iteration: " + str(iteration) + "\n")
    training_case.close()