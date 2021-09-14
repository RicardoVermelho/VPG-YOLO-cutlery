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
from PIL import Image # to read images
from open3d import geometry
from open3d import camera 
from open3d import io
from open3d import visualization

# Know current library path - How To:
import pathlib
#print(pathlib.Path().absolute())

'''Given a path, this method searches
   for the newest file and returns it.
   https://gist.github.com/benhosmer/4634721 -> newest file in folder
'''
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
def getBBOXcoords():
    idx_bestConf = getBestConfidenceIdx("../inference/labels")
    path_to_BBOXfile = "../BBOX_xyxy"

    BBOX_file = open(getLatestFileInPath(path_to_BBOXfile), 'r')

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

'''This method plots box around an object present in a given image'''
def plot_one_box(x, path_to_img, img, color=None, label=None, line_thickness=None):
    # Plots one bounding box on image img
    tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
        cv2.imwrite(path_to_img, img) 

def crop_image(path_to_img, img, path_to_save, BBOX_coords, obj_class):
    final_image_rgb = cv2.imread(path_to_img)
    #image_bgr = cv2.cvtColor(final_image_rgb, cv2.COLOR_RGB2BGR)
    plot_one_box(BBOX_coords, path_to_img, final_image_rgb, color=(255, 0, 0), label=obj_class, line_thickness=3)
    '''final_image_bgr = cv2.imread(path+depth_file)
    image_rgb = cv2.cvtColor(final_image_bgr, cv2.COLOR_BGR2RGB)
    cv2.imwrite(path+depth_file, image_rgb)'''

    img_to_crop = Image.open(path_to_img)
    cropped_img = img_to_crop.crop(BBOX_coords)
    cropped_img.save(path_to_save + '/cropped_' + str(img))
    
'''Wtih this method we convert the depth image
   to point cloud, using the class and coordinates of the
   object with highest confidence value.'''
def depth2pc():
    home_path = str(pathlib.Path().absolute()).replace("inference","")
    
    BBOX_coords, obj_class = getBBOXcoords()
    obj_class = str(int(float(obj_class)))
    classes = {
        "0": "fork",
        "1": "knife",
        "2": "spoon"
    }
    final_class = classes.get(obj_class)

    # Compute mean pixel between both pixels that define the bounding box
    mean_pixel = [(BBOX_coords[0]+BBOX_coords[2])/2, (BBOX_coords[1]+BBOX_coords[3])/2]
    
    log_file = open("../../logs_latestFolder.txt", "r")
    log_folder = log_file.readline()

    path = home_path + 'logs/' + log_folder + '/data/depth-images/'
    depth_file = str(getLatestFileInPath(os.path.relpath(path)))

    crop_image(path+depth_file, depth_file, home_path+'inference/cropped_depth_images', BBOX_coords, final_class)

    #final_image_bgr = cv2.imread(home_path+'inference/cropped_depth_images'+str(getLatestFileInPath(os.path.relpath(home_path+'inference/cropped_depth_images'))))
    #h, w, c = final_image_bgr.shape

    #pil_img = Image.fromarray(final_image_bgr)
    #pil_img.save(home_path+'inference/cropped_depth_images'+str(getLatestFileInPath(os.path.relpath(home_path+'inference/cropped_depth_images'))))
    depth_rgb = np.array(Image.open(home_path+'inference/cropped_depth_images/'+str(getLatestFileInPath(os.path.relpath(home_path+'inference/cropped_depth_images')))))
    h, w, c = depth_rgb.shape
    fx = 575.8157496
    fy = 575.8157496

    #-->CORRIGIR PROBLEMA DE TYPEERROR, POSSIVELMENTE A MATRIZ DE INTRINSICS E DEPOIS VER COMO OBTER VALOR MEDIO DOS PRIMEIROS PONTOS DA POINT CLOUD
    depth = io.read_image(home_path+'inference/cropped_depth_images/'+str(getLatestFileInPath(os.path.relpath(home_path+'inference/cropped_depth_images'))))
    depth = cv2.cvtColor(np.asarray(depth), cv2.COLOR_BGR2RGB)
    cam = camera.PinholeCameraIntrinsic()
    cam.set_intrinsics(w, h, fx, fy, w/2, h/2)
    points = geometry.PointCloud.create_from_depth_image(depth, cam)
    #points = geometry.PointCloud.create_from_depth_image(depth_rgb, camera.PinholeCameraIntrinsic(camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    '''cam = camera.PinholeCameraParameters()
    cam.set_intrinsic(width=w, height=h, fx=575.8157496, fy=575.8157496, cx=w/2, cy=h/2)
    points = geometry.PointCloud.create_from_depth_image(depth_rgb, cam.intrinsic)'''
    visualization.draw_geometries([points])

depth2pc()