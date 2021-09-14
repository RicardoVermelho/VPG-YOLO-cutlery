import os
import subprocess
import os.path
import numpy as np
from yolov5.inference import inferencev5

def preprocess(iteration, all_objects, path2labels):
    inferencev5(iteration)
    
    os.chdir(path2labels)
    files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
    latest_label = files[-1]

    # If file is empty then get file previous to the latest one
    if os.stat(str(latest_label)).st_size == 0:
        print("File is empty... retrieving data from previous bbox file")
        subprocess.call('rm -f ' + path2labels + str(latest_label), shell=True)
        latest_label = files[-1]
    
    with open(latest_label) as f:
        bboxes = f.readlines()
        
        new_bbox = []
        new_bcoords_final = []
        new_bcoords_final_copy = {}

        for i in range(len(bboxes)):
            bbox_line = bboxes[i].strip('[]').split(', ')
            class_tmp = None
            for j in range(len(bbox_line)):
                if j < 4:
                    new_bbox.append(float(bbox_line[j]))
                if j == len(bbox_line)-1:
                    class_obj_tmp = bbox_line[j].split('\n')
                    class_obj_tmp2 = class_obj_tmp[0].split(']')
                    class_obj_tmp3 = class_obj_tmp2[0]
                    class_tmp = class_obj_tmp3
            new_bcoords_final.append(new_bbox)
            new_bcoords_final_copy[str(new_bbox)] = class_tmp
            new_bbox = []
            
        ious = []
        
        for a in range(0, len(new_bcoords_final)-1):
            bboxa = new_bcoords_final[a]
            for b in range(a+1, len(new_bcoords_final)):
                bboxb = new_bcoords_final[b]
                IoU = {a: bboxa, b: bboxb, 'iou': computeIoU(bboxa, bboxb)}
                ious.append(IoU)

        iou_boundary = 0.0       
        bbox_common, objclass, action = graspORpush(new_bcoords_final_copy, new_bcoords_final, ious, iou_boundary)
        closest_obj = getClosestObject(convertBBOx2Sim(bbox_common), all_objects)
        return action, closest_obj, objclass
                
def computeIoU(a, b, epsilon=1e-5):
    # COORDINATES OF THE INTERSECTION BOX
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])

    # AREA OF OVERLAP - Area where the boxes intersect
    width = (x2 - x1)
    height = (y2 - y1)
    # handle case where there is NO overlap
    if (width<0) or (height <0):
        return 0.0
    area_overlap = width * height

    # COMBINED AREA
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    area_combined = area_a + area_b - area_overlap

    # RATIO OF AREA OF OVERLAP OVER COMBINED AREA
    iou = area_overlap / (area_combined+epsilon)
    return iou

'''Between all the objects composing the pair of bboxes with iou less than 0.05
return the most common bbox amongst them.
If there are more than 1 object, return the first object.'''
def graspORpush(bboxes, new_bboxes, ious, iou_boundary):
    optimal_bbox = None
    objclass = None
    
    dictscontainingbbox = []
    for bbox in new_bboxes:
        count_isolations = 0
        bbox_dict = {tuple(bbox): count_isolations}
        
        for iou in ious:
            if bbox in iou.values():
                if iou['iou'] <= iou_boundary:
                    count_isolations += 1
        
        bbox_dict[tuple(bbox)] = count_isolations
        dictscontainingbbox.append(bbox_dict)
    
    (tmp_bbox, count_bIso), = dictscontainingbbox[0].items()
    idx_bIso = 0
    for i in range(1, len(dictscontainingbbox)):
        (tmp_bbox, tmp_count_bIso), = dictscontainingbbox[i].items()
        if count_bIso < tmp_count_bIso:
            count_bIso = tmp_count_bIso
            idx_bIso = i
    
    (optimal_bbox, counter), = dictscontainingbbox[idx_bIso].items()
    if counter == len(dictscontainingbbox)-1:
        tmp = list(bboxes.items())[idx_bIso]
        objclass = getObjectClass(tmp[1])
        return list(optimal_bbox), objclass, 'grasp'
    else:
        return fromAglomeratedBBOX(bboxes, ious)

'''Between all the objects composing the pair of bboxes with iou less than 0.05
return the most common bbox amongst them.
If there are more than 1 object, return the first object.
def isolatedObject(bboxes, new_bboxes, ious, iou_boundary):
    count_isolations = 0
    minimal_iou = 1
    optimal_bbox = None
    objclass = None
    
    dictscontainingbbox = []
    for bbox in new_bboxes:
        for iou in ious:
            if bbox in iou.values():
                dictscontainingbbox.append(iou)
        
        for i in range(len(dictscontainingbbox)):
            minimal_iou_tmp = round(dictscontainingbbox[i]['iou'], 2)
            if minimal_iou_tmp < iou_boundary:
                count_isolations += 1
                if minimal_iou_tmp < minimal_iou:
                    minimal_iou = minimal_iou_tmp
        
        if count_isolations == len(dictscontainingbbox):
            optimal_bbox = bbox
        
        count_isolations = 0
        dictscontainingbbox = []
    
    if optimal_bbox != None:
        str_optimalbbox = str(optimal_bbox)
        objclass = bboxes.get(str_optimalbbox)
        objclass = getObjectClass(objclass)
    
    return optimal_bbox, objclass'''

'''Between all the objects composing the pair of bboxes
return the bbox from the pair containing the highest iou.'''      
def fromAglomeratedBBOX(bboxes, ious):
    maximum_iou = 0
    bbox2push = None
    objclass = None

    for iou in ious:
        if maximum_iou < iou['iou']:
            maximum_iou = iou['iou']
            bbox2push =  list(iou.values())[0]
    
    str_bbox2push = str(bbox2push)
    objclass = bboxes.get(str_bbox2push)
    objclass = getObjectClass(objclass)
            
    return bbox2push, objclass, 'push'

def convertBBOx2Sim(bbox):
    simXmean = (0.05+0.55)/2
    # Find average point from bbox coordinates
    avg_x = (bbox[0]+bbox[2])/2
    avg_y = (bbox[1]+bbox[3])/2
    avg_point = [avg_x, avg_y]
    
    # Convert average pixel point coordinates to simulation coordinates
    #y_sim = -0.30 + ((avg_point[0]*0.30) / 320)
    y_sim = 0.30 - (avg_point[0]*0.30) / 320
    x_sim = ((avg_point[1]*simXmean) / 240)
    
    return [x_sim, y_sim]

def getClosestObject(coordsfrombbox, all_objects):
    x_diff = None # Keep lowest x difference between one object and coordsfrombbox
    y_diff = None # Keep lowest y difference between one object and coordsfrombbox
    closestobj = None # Keep idx of object with lowest x_diff and y_diff or only x_diff
    
    for i in range(len(all_objects)):
        xdiff_tmp = abs(all_objects[i][0] - coordsfrombbox[0])
        ydiff_tmp = abs(all_objects[i][1] - coordsfrombbox[1])
        if (x_diff == None and y_diff == None) or (xdiff_tmp < x_diff and ydiff_tmp < y_diff):
            x_diff = xdiff_tmp
            y_diff = ydiff_tmp
            closestobj = all_objects[i]
        elif xdiff_tmp < x_diff and not ydiff_tmp < y_diff:
            x_diff = xdiff_tmp
            closestobj = all_objects[i]
            
    return closestobj
    
def getObjectClass(objclass):
    class_obj = str(int(float(objclass)))
    classes = {
    "0": "fork",
    "1": "knife",
    "2": "spoon"
    }
    return classes.get(class_obj)