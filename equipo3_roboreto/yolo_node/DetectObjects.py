#!/usr/bin/env python3
import time
import cv2
import os
import rospy
import sys
import numpy as np
# from sensor_msgs.msg import Image
# from std_msgs.msg import Int32
# from cv_bridge import CvBridge

sys.path.append(sys.path[0]+'/yolov5')
from detection import detector 

class DetectStop:

    def __init__(self):

        # folder path
        # dir_path = "/home/puzzlebot/catkin_ws/src/yolo_rec/src/yolov5/data/images/bus.jpg" # adjust the path to your system! 
        weights = "/home/puzzlebot/catkin_ws/src/yolo_rec/src/signals.pt" # adjust the path to your system! 
        print("Loading network")
        t = time.time()
        yolo = detector(weights,0.5)
        print(time.time() - t)
        # Iterate directory
        # for path in os.listdir(dir_path):
            # check if current path is a file

            # if os.path.isfile(os.path.join(dir_path, path)):

    def search(self, img):
        print("search")
        source = cv2.imread(img)
        t = time.time()
        try:
            print("Hola 1")
            pred = yolo.detect(source) # acces the library to change threshoulds and other hyperparameters
            print("Hola 2")
            for i, det in enumerate(pred):
                print(det) # the output of this call is a table with N detections, the 4 elements of a bounding box, the confidence and the class ID
                print(time.time()-t)
        except:
            pass

# if __name__ == "__main__":
    # test = DetectStop()
    # test.search()

