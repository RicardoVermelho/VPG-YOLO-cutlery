import os
import random
import cv2
import numpy as np
from PIL import Image # to read images

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

def getBBOXcoords():
    idx_bestConf = getBestConfidenceIdx("./labels")
    path_to_BBOXfile = "../BBOX_xyxy"

    BBOX_file = open(getLatestFileInPath(path_to_BBOXfile), 'r')

    lines = BBOX_file.readlines()   # Read all lines of the file
    best_coords = lines[idx_bestConf]   # Get line contaning coordinates of best confidence

    BBOX_file.close()

    new_bcoords = best_coords.strip('][').split(', ')
    new_bcoords_final = []

    for i in range(len(new_bcoords)):
        if i < 4:
            new_bcoords_final.append(float(new_bcoords[i]))

    return new_bcoords_final

img_to_crop = Image.open('000085.0.depth.png')
BBOX_coords = getBBOXcoords()
cropped_img = img_to_crop.crop(BBOX_coords)
cropped_img.save("cropped.png")