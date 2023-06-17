#!/usr/bin/env python3
# -*- coding: utf-8 -*.
import time
import os
import sys
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
# from cv_bridge import CvBridge
import cv2 as cv
from geometry_msgs.msg import Point

sys.path.append(sys.path[0]+'/yolov5')
from detection import detector 

# from jetson_utils import videoSource

frame = np.array((350, 500, 3), dtype= np.uint8)
# begin = False
# bridge = CvBridge()

# weights = "/home/puzzlebot/catkin_ws/src/yolo_rec/src/signals.pt"

# def imgCallback(image_raw):
#   global frame, begin
#   frame = bridge.imgmsg_to_cv2(image_raw, desired_encoding="bgr8")
#   # frame = cv.flip(frame, -1)
#   raw_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
#   ima_pub.publish(raw_msg)
#   begin = True


#Stop Condition
def stop():
  #Cerrar Ventanas de cámara
  cv.destroyAllWindows()


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("object_detector")
    print("objectNode")

    #Initialise object detector
    weights = "/home/puzzlebot/catkin_ws/src/yolo_rec/src/signals.pt" # adjust the path to your system! 
    print("Loading network")
    t = time.time()
    yolo = detector(weights,0.5)
    print(time.time() - t)

    rate = rospy.Rate(120)
    rospy.on_shutdown(stop)
    #Setup Publishers to visualize the results of mask and thresholding
    ima_pub = rospy.Publisher('camera_raw', Image, queue_size=10)
    signal_pub = rospy.Publisher('signals', Int32, queue_size=10) 
    # rospy.Subscriber('/video_source/raw', Image, imgCallback)
    #Run the node
    # prevImg = img
    while not rospy.is_shutdown():
      tipo = -1
      try:
        img = cv.imread( "/home/puzzlebot/catkin_ws/src/yolo_rec/src/Image.jpg")
        
        pred = yolo.detect(img)
        print("imagen")
        for i, det in enumerate(pred):
          print("imagen2")
          if len(det) > 0:
            #for#x1 y1 x2 y2 fuerte prediccion, la clase de la se;al 
            for i in range(len(det)):
              print("imagen3")
              topx = det[i][0] 
              topy = det[1][1]
              botx = det[i][2]
              boty2 = det[i][3]
              predi = det[i][4]
              tipo = det[i][5]
              print("imagen5")
              print(int(tipo))
              
      except:
        signal_pub.publish(-1)
        # img = prevImg
        print("Failed to load image")
            
      signal_pub.publish(int(tipo))
      # if(begin):
      #   yolo.search(frame)
      # raw_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
      # ima_pub.publish(raw_msg)
          
          # colors_pub.publish(greenAmount, redAmount, yellowAmount)
          # green_pub.publish(green_detect)
        
      rate.sleep()