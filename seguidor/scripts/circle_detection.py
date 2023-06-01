#!/usr/bin/env python
# -*- coding: utf-8 -*.
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2 as cv
from geometry_msgs.msg import Point

# Setup Variables, parameters and messages to be used (if required)
frame = np.array((350, 500, 3), dtype= np.uint8)
green_detect = np.array((350, 500, 3), dtype= np.uint8)

state = 0
begin = False

greenAmount = 0
redAmount = 0
yellowAmount = 0

lower_green = np.array([50, 90, 90])
upper_green = np.array([80, 255, 255])

lower_red1 = np.array([0, 160, 160])
upper_red1 = np.array([10, 255, 255])

lower_red2 = np.array([161, 155, 84])
upper_red2 = np.array([179, 255, 255])

lower_yellow = np.array([18, 120, 120])
upper_yellow = np.array([30, 255, 255])

bridge = CvBridge()

def detectRed(frame, hsv):
  global redAmount

  mask_red1 = cv.inRange(hsv, lower_red1, upper_red1)
  mask_red2 = cv.inRange(hsv, lower_red2, upper_red2)

  red_mask = cv.bitwise_or(mask_red1, mask_red2)
  red = cv.bitwise_and(frame, frame, mask=red_mask)

  red_gray = cv.cvtColor(red, cv.COLOR_BGR2GRAY)
  red_gray_blur = cv.medianBlur(red_gray,5)
  red_thres = cv.adaptiveThreshold(red_gray_blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2)
  blobs_red = detector.detect(red_thres)

  redAmount = 0

  # Iterar sobre los keypoints y contar aquellos con size mayor a 200
  for kp in blobs_red:
    if kp.size > 50:
      redAmount += 1

def detectGreen(frame, hsv):
  global greenAmount, green_detect

  # print("green")
  mask_green = cv.inRange(hsv, lower_green, upper_green)

  green = cv.bitwise_and(frame, frame, mask=mask_green)

  green_gray = cv.cvtColor(green, cv.COLOR_BGR2GRAY)
  green_gray_blur = cv.medianBlur(green_gray,5)
  green_thres = cv.adaptiveThreshold(green_gray_blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2)

  blobs_green = detector.detect(green_thres)
  greenAmount = 0

  # Iterar sobre los keypoints y contar aquellos con size mayor a 200
  for kp in blobs_green:
    if kp.size > 50:
      greenAmount += 1

  # green_detect = cv.drawKeypoints(frame, blobs_green, np.array([]), (255,0,0), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


def detectYellow(frame, hsv):
  global yellowAmount

  yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)
  yellow = cv.bitwise_and(frame, frame, mask=yellow_mask)

  yellow_gray = cv.cvtColor(yellow, cv.COLOR_BGR2GRAY)
  yellow_gray_blur = cv.medianBlur(yellow_gray,5)
  yellow_thres = cv.adaptiveThreshold(yellow_gray_blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2)
  blobs_yellow = detector.detect(yellow_thres)

  yellowAmount = 0

  # Iterar sobre los keypoints y contar aquellos con size mayor a 200
  for kp in blobs_yellow:
    if kp.size > 50:
      yellowAmount += 1
  

def colorsCallback(image_raw):
  global state, frame, begin
  frame = bridge.imgmsg_to_cv2(image_raw, desired_encoding="bgr8")
  raw_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
  ima_pub.publish(raw_msg)
  begin = True

def stateCallback(stateMsg):
  global state
  state = int(stateMsg.data)


#Stop Condition
def stop():
  #Cerrar Ventanas de cámara
  cv.destroyAllWindows()

  #Liberar cámara
  # cap.release()


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("line_follower")

    params = cv.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 1000
    params.maxArea = 1000000000
    params.filterByCircularity = True
    params.minCircularity = 0.8
    params.minThreshold = 10
    params.maxThreshold = 200

    detector = cv.SimpleBlobDetector_create(params)


    # cap = cv.VideoCapture(0)

    rate = rospy.Rate(100)

    rospy.on_shutdown(stop)
    #Setup Publishers to visualize the results of mask and thresholding

    green_pub = rospy.Publisher('green_detector', Image, queue_size=10)
    ima_pub = rospy.Publisher('camera_raw', Image, queue_size=10)

    colors_pub = rospy.Publisher('colors', Point, queue_size=1)    

    rospy.Subscriber('/video_source/raw', Image, colorsCallback)
    rospy.Subscriber('/state', Int32, stateCallback)
    
    #Run the node
    while not rospy.is_shutdown():
        #Leer imagen de la cámara
        # ret, frame = cap.read()
        if(begin):
          hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

          if(state == 0):
            detectGreen(frame, hsv)
          elif(state == 1):
            detectRed(frame, hsv)
            detectYellow(frame, hsv)
          elif(state == 2):
            detectRed(frame, hsv)
            detectGreen(frame, hsv)
          else:
            print("Sin acceso al control")
          
        colors_pub.publish(greenAmount, redAmount, yellowAmount)
        # green_pub.publish(green_detect)
        
        rate.sleep()