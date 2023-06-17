#!/usr/bin/env python3
import time
import cv2
import os
import rospy
import sys

sys.path.append(sys.path[0]+'/yolov5')
from detection import detector 

class DetectStop:

    def __init__(self):

        # folder path
        dir_path = "/home/puzzlebot/yolov5/samples/data/test/images" # adjust the path to your system! 
        weights = "best.engine" # adjust the path to your system! 
        print("Loading network")
        t = time.time()
        yolo = detector(weights,0.5)
        print(time.time() - t)
        # Iterate directory
        for path in os.listdir(dir_path):
            # check if current path is a file

            if os.path.isfile(os.path.join(dir_path, path)):

                source = cv2.imread(os.path.join(dir_path, path))
                t = time.time()
                try:
                    pred = yolo.detect(source) # acces the library to change threshoulds and other hyperparameters
                    for i, det in enumerate(pred):
                        print(det) # the output of this call is a table with N detections, the 4 elements of a bounding box, the confidence and the class ID
                        print(time.time()-t)
                except:
                    pass

if __name__ == "__main__":
    DetectStop()

