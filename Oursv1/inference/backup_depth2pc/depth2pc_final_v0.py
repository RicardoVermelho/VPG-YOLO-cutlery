##################################################
'''
############## DEPTH TO POINT CLOUD ##############
THIS MODULE IS DESIGNED TO GRAB THE BOUNDING BOX OF THE OBJECT
WITH BETTER CONFIDENCE VALUE, APPLY TO THE DEPTH IMAGE FOR THE SAME ITERATION
AND OBTAIN THE 3D LOCATION OF THE OBJECT IN THE ENVIRONMENT.
'''
##################################################

import os
import random
import cv2
import numpy as np
import math
from PIL import Image # to read images
from open3d import geometry
from open3d import camera 
from open3d import io
from open3d import visualization
from pathlib import Path
from sklearn.neighbors import NearestNeighbors # to compute k nearest neighbors of point cloud's central point

def euler2rotm(theta):
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])         
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])            
    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R

'''Given a path, this method searches
   for the newest file and returns it.
   https://gist.github.com/benhosmer/4634721 -> newest file in folder'''
def getLatestFileInPath(given_path):
    path = given_path
    os.chdir(path)
    files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)

    oldest = files[0]
    newest = files[-1]

    return newest

'''This method will obtain the index
   of the object with more confidence
   value and returns its index in the file'''
def getBestConfidenceIdx(path_to_newestFile):
    newest_f = open(getLatestFileInPath(path_to_newestFile), 'r')
    
    confidence = 0
    idx = 0

    for aline in newest_f:
        index = 0
        values = aline.split(" ")

        if confidence < float(values[1]):
            confidence = float(values[1])
            idx = index
        
        index += 1

    newest_f.close()
    return idx

'''The objective of this method is to obtain the
   coordinates of the bounding box correspondant to
   the object with the best confidence value for the
   current iteration'''
def getBBOXcoords(home_path, use_other_idx=False, other_idx=None):
    idx_bestConf = getBestConfidenceIdx(home_path+"/inference/labels")
    path_to_BBOXfile = home_path+"/inference/BBOX_xyxy"

    BBOX_file = open(getLatestFileInPath(path_to_BBOXfile), 'r')

    if use_other_idx == True:
        idx_bestConf = other_idx

    lines = BBOX_file.readlines()   # Read all lines of the file
    best_coords = lines[idx_bestConf]   # Get line contaning coordinates of best confidence

    BBOX_file.close()

    new_bcoords = best_coords.strip('][').split(', ')
    new_bcoords_final = []
    class_obj = 0

    for i in range(len(new_bcoords)):
        if i < 4:
            new_bcoords_final.append(float(new_bcoords[i]))
        if i == len(new_bcoords)-1:
            class_obj_tmp = new_bcoords[i].split('\n')
            class_obj_tmp2 = class_obj_tmp[0].split(']')
            class_obj_tmp3 = class_obj_tmp2[0]
            class_obj = class_obj_tmp3

    return new_bcoords_final, class_obj

'''Wtih this method we convert the depth image
   to point cloud, using the class and coordinates of the
   object with highest confidence value.'''
def depth2pc(color_img_pcloud, depth_img_pcloud):
    print("#"*50)
    print("STARTING CONVERTION FROM DEPTH IMAGE TO POINT CLOUD...\n")
    path_to_download_folder = str(os.path.join(Path.home(), "Downloads/VisualPushingGrasping"))
    home_path = path_to_download_folder

    BBOX_coords, obj_class = getBBOXcoords(home_path)
    obj_class = str(int(float(obj_class)))
    classes = {
        "0": "fork",
        "1": "knife",
        "2": "spoon"
    }
    final_class = classes.get(obj_class)
    
    log_file = open(home_path+"/logs_latestFolder.txt", "r")
    log_folder = log_file.readline()

    # MAKE CROP OF THE COLOR IMAGE USING BOUNDING BOX
    #path = home_path + '/logs/' + log_folder + '/data/color-images/'
    #color_file = str(getLatestFileInPath(os.path.relpath(path)))
    #color_to_crop = Image.open(path+color_file)
    color_to_crop = geometry.Image(color_img_pcloud)
    #cropped_color_img = color_to_crop.crop(BBOX_coords)
    #cropped_color_img = geometry.Image(np.asarray(cropped_color_img))

    # MAKE CROP OF THE DEPTH IMAGE USING BOUNDING BOX AND RECOMPUTE DEPTH VALUES
    path = home_path + '/logs/' + log_folder + '/data/depth-images/'
    depth_file = str(getLatestFileInPath(os.path.relpath(path)))
    depth_to_crop = Image.open(path+depth_file)
    depth_to_crop = geometry.Image(np.asarray(depth_to_crop).astype(np.uint16))
    #depth_to_crop = geometry.Image(depth_img_pcloud.astype(np.uint16))
    #print(depth_img_pcloud)
    '''cropped_depth_img = depth_to_crop.crop(BBOX_coords)

    print("Antes de aplicar zFar e zNear: ", np.max(cropped_depth_img))
    print("Antes de aplicar zFar e zNear: ", np.min(cropped_depth_img))
    zNear = 0.01
    zFar = 10
    cropped_depth_img = (np.asarray(cropped_depth_img) - zNear)/(zFar - zNear)
    cropped_depth_img = np.fliplr(cropped_depth_img)
    cropped_depth_img = geometry.Image((cropped_depth_img).astype(np.uint16))
    print("Depois de aplicar zFar e zNear: ", np.max(cropped_depth_img))
    print("Depois de aplicar zFar e zNear: ", np.min(cropped_depth_img))'''

    #cropped_depth_img.save(home_path+'/inference/cropped_depth_images/cropped_' + str(depth_file))

    # CREATE RGBD IMAGE USING OPEN3D-RGBDIMAGE MODULE
    rgbd_img = geometry.RGBDImage.create_from_color_and_depth(color_to_crop, depth_to_crop, depth_scale=10000)
    #rgbd_img = geometry.RGBDImage.create_from_color_and_depth(cropped_color_img, cropped_depth_img, depth_scale=10000)

    # READ CROPPED IMAGE AND GET WIDTH AND HEIGHT
    print("READING DEPTH IMAGE AND GENERATING POINT CLOUD")
    #depth = io.read_image(home_path+'/inference/cropped_depth_images/'+str(getLatestFileInPath(os.path.relpath(home_path+'/inference/cropped_depth_images'))))
    #depth = io.read_image(path+depth_file)
    w = np.shape(depth_to_crop)[1]
    print("depth w: ", w)
    h = np.shape(depth_to_crop)[0]
    print("depth h: ", h)

    # FOCAL LENGHT DISTANCE FOR THE CAMERA
    fx = 618.62
    fy = 618.62

    # SET INTRINSIC VALUES FOR THE CAMERA
    cam = camera.PinholeCameraIntrinsic()
    #cam.set_intrinsics(w,h,fx,fy,w/2,h/2) # ORIGINAL --> WORKS
    cam.intrinsic_matrix = [[fx, 0, w/2], [0, fy, h/2], [0, 0, 1]] # SECOND WAY OF DOING --> ALSO WORKS

    # GENERATE POINT CLOUD FROM CROPPED DEPTH IMAGE
    #points = geometry.PointCloud.create_from_rgbd_image(rgbd_img, cam)
    #points = geometry.PointCloud.create_from_depth_image(cropped_depth_img, cam, depth_scale=10000)
    points = geometry.PointCloud.create_from_depth_image(depth_to_crop, cam, depth_scale=10000)
    point_cloud_array = np.asarray(points.points)

    max_boundaries = np.asarray(points.get_max_bound())
    print("Point cloud Max Bounds: "+ str(max_boundaries))
    # Show min boundaries of the compute point cloud
    min_boundaries = np.asarray(points.get_min_bound())
    print("Point cloud Min Bounds: "+ str(min_boundaries))
    
    # CHANGE FROM CAMERA SYSTEM TO ROBOT SYSTEM COORDINATES
    cam_position = [0.8000519275665283, -1.539289951324463e-05, 0.5000017285346985]
    cam_trans = np.eye(4,4)
    cam_trans[0:3,3] = np.asarray(cam_position)
    cam_orientation = [3.141592502593994, 0.785398542881012, 1.5707974433898926]
    cam_orientation = [-cam_orientation[0], -cam_orientation[1], -cam_orientation[2]]
    cam_rotm = np.eye(4,4)
    cam_rotm[0:3,0:3] = np.linalg.inv(euler2rotm(cam_orientation))
    cam_pose = np.dot(cam_trans, cam_rotm) # Compute rigid transformation representating camera pose

    # CONVERT POINT CLOUD FROM CAMERA COORDINATE SYSTEM TO ROBOT COORDINATE SYSTEM
    point_cloud_array = np.transpose(np.dot(cam_pose[0:3,0:3],np.transpose(point_cloud_array)) + np.tile(cam_pose[0:3,3:],(1,point_cloud_array.shape[0])))
    zNear = 0.01
    zFar = 10
    point_cloud_array = (point_cloud_array-zNear)/(zFar-zNear)
    # Sort surface points by z value
    sort_z_ind = np.argsort(point_cloud_array[:,2])
    point_cloud_array = point_cloud_array[sort_z_ind]

    # Get point with highest z from point cloud
    highest_z = point_cloud_array[len(point_cloud_array)-1]
    print("Point with highest z from point cloud: ", highest_z)

    # Filter out surface points outside heightmap boundaries
    #heightmap_valid_ind = np.logical_and(np.logical_and(np.logical_and(np.logical_and(point_cloud_array[:,0] >= 0.05, point_cloud_array[:,0] < 0.55), point_cloud_array[:,1] >= -0.25), point_cloud_array[:,1] < 0.25), point_cloud_array[:,2] < 0.4)
    #point_cloud_array = point_cloud_array[heightmap_valid_ind]
    #print("len point cloud: " + str(len(point_cloud_array)))

    # First method to obtain the point in the center of the point cloud
    #centroid_1 = np.asarray(points.get_center())
    #print("Point cloud Center: " + str(centroid_1) + "\n")
    # Secnd method to obtain the point in the center of the point cloud
    x = [p[0] for p in point_cloud_array]
    y = [p[1] for p in point_cloud_array]
    z = [p[2] for p in point_cloud_array]
    centroid_2 = (sum(x) / len(point_cloud_array), sum(y) / len(point_cloud_array), sum(z) / len(point_cloud_array))
    print("Centroid of Point Cloud: " + str(centroid_2))

    max_bounds = [max(x), max(y), max(z)]
    min_bounds = [min(x), min(y), min(z)]

    print("Point Cloud Max Bounds: ", max_bounds)
    print("Point Cloud Min Bounds: ", min_bounds)
    
    pcloud = depth_file.split(".depth")
    #io.write_point_cloud(home_path+"/inference/point_clouds/point_cloud_"+str(pcloud[0])+".ply", points)
    print("FINISHED CONVERTING TO POINT CLOUD!")

    # GET MEAN VALUE OF THE 20% POINTS OF THE POINT CLOUD AND SAVE IT TO FILE
    # Compute k neighbors of centroid. Lets use k=30
    neigh = NearestNeighbors(n_neighbors=int(len(point_cloud_array)/5))
    neigh.fit(point_cloud_array)
    k_neighbors = neigh.kneighbors([highest_z], return_distance=False)

    # Compute mean point from k neighbors' coordinates
    meank_neighbor = [0, 0, 0]

    for i in k_neighbors[0]:
        pcdi_list = point_cloud_array[i].tolist()
        meank_neighbor[0] += pcdi_list[0]
        meank_neighbor[1] += pcdi_list[1]
        meank_neighbor[2] += pcdi_list[2]

    meank_neighbor = [meank_neighbor[0]/len(k_neighbors[0]), meank_neighbor[1]/len(k_neighbors[0]), meank_neighbor[2]/len(k_neighbors[0])]
    print("Mean k neighbor antes: ", meank_neighbor)

    # This step applies the correction to the mean point computed from point cloud
    '''meank_neighbor[0] = (meank_neighbor[0]-zNear)/(zFar-zNear)
    meank_neighbor[1] = (meank_neighbor[0]-zNear)/(zFar-zNear)
    meank_neighbor[2] = (meank_neighbor[0]-zNear)/(zFar-zNear)'''
    print("Mean K Neighbor depois: ", meank_neighbor)

    print("\nFINISHED COMPUTING AVERAGE POINT FROM K NEIGHBORS OF POINT CLOUDS' CENTROID!")
    print("#"*50)
    print("\n")
    return meank_neighbor, final_class