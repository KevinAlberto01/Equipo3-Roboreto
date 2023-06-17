#!/usr/bin/env python3
# -*- coding: utf-8 -*.
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
# from cv_bridge import CvBridge
import cv2 as cv
from geometry_msgs.msg import Point
import DetectObjects as DO
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
    yolo = DO.DetectStop()

    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)
    #Setup Publishers to visualize the results of mask and thresholding
    ima_pub = rospy.Publisher('camera_raw', Image, queue_size=10) 
    # rospy.Subscriber('/video_source/raw', Image, imgCallback)
    
    #Run the node
    # prevImg = img
    while not rospy.is_shutdown():
      try:
        img = "/home/puzzlebot/catkin_ws/src/yolo_rec/src/Image.jpg"
        yolo.search(img)
        print("imagen")
      except:
        # img = prevImg
        print("Failed to load image")
            
      
      # if(begin):
      #   yolo.search(frame)
      # raw_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
      # ima_pub.publish(raw_msg)
          
          # colors_pub.publish(greenAmount, redAmount, yellowAmount)
          # green_pub.publish(green_detect)
        
      rate.sleep()