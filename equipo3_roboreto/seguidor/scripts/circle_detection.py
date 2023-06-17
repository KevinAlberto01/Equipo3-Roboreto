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
frame_new = np.array((350, 500, 3), dtype= np.uint8)

state = 0
begin = False

greenAmount = 0
redAmount = 0
yellowAmount = 0

lower_green = np.array([43, 70, 62])
upper_green = np.array([84, 255, 255])

lower_red1 = np.array([0, 99, 120])
upper_red1 = np.array([8, 236, 255])

lower_red2 = np.array([154, 178, 139])
upper_red2 = np.array([179, 255, 255])

lower_yellow = np.array([17, 102, 0])
upper_yellow = np.array([51, 255, 255])

bridge = CvBridge()

def detectRed(frame_new, hsv):
  global redAmount

  mask_red1 = cv.inRange(hsv, lower_red1, upper_red1)
  mask_red2 = cv.inRange(hsv, lower_red2, upper_red2)

  red_mask = cv.bitwise_or(mask_red1, mask_red2)
  red = cv.bitwise_and(frame_new, frame_new, mask=red_mask)

  red_gray = cv.cvtColor(red, cv.COLOR_BGR2GRAY)
  red_gray_blur = cv.medianBlur(red_gray,5)
  thres = cv.adaptiveThreshold(red_gray_blur, 100, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2)
  circles = cv.HoughCircles(red_gray_blur, cv.HOUGH_GRADIENT, 1, 20, param1=100, param2 = 15, minRadius=0, maxRadius = 0)  
  
  redAmount = 0

  # Iterar sobre los keypoints y contar aquellos con size mayor a 200
  if circles is not None:
    circles = np.round(circles[0, :]).astype(int)
    # print("red detected")
    #Dibujar círculos
    for (x, y, r) in circles:
      cv.circle(frame_new, (x,y), r, (0, 255, 0), 2)
      redAmount += 1

  red_img = bridge.cv2_to_imgmsg(frame_new, encoding="bgr8")
  red_pub.publish(red_img)
  

def detectGreen(frame_new, hsv):
  global greenAmount

  # print("green")
  mask_green = cv.inRange(hsv, lower_green, upper_green)

  green = cv.bitwise_and(frame_new, frame_new, mask=mask_green)

  green_gray = cv.cvtColor(green, cv.COLOR_BGR2GRAY)
  green_gray_blur = cv.medianBlur(green_gray,5)
  circles = cv.HoughCircles(green_gray_blur, cv.HOUGH_GRADIENT, 1, 20, param1=100, param2 = 15, minRadius=0, maxRadius = 0)  
  
  greenAmount = 0

  # Iterar sobre los keypoints y contar aquellos con size mayor a 200
  if circles is not None:
    circles = np.round(circles[0, :]).astype(int)
    # print("green detected")

    #Dibujar círculos
    for (x, y, r) in circles:
      cv.circle(frame, (x,y), r, (0, 255, 0), 2)
      greenAmount += 1

  green_img = bridge.cv2_to_imgmsg(frame_new, encoding="bgr8")
  green_pub.publish(green_img)
  


def detectYellow(frame_new, hsv):
  global yellowAmount

  yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)
  yellow = cv.bitwise_and(frame_new, frame_new, mask=yellow_mask)

  yellow_gray = cv.cvtColor(yellow, cv.COLOR_BGR2GRAY)
  yellow_gray_blur = cv.medianBlur(yellow_gray,5)
  circles = cv.HoughCircles(yellow_gray_blur, cv.HOUGH_GRADIENT, 1, 20, param1=100, param2=15,)

  yellowAmount = 0

 # Iterar sobre los keypoints y contar aquellos con size mayor a 200
  if circles is not None:
    circles = np.round(circles[0, :]).astype(int)
    # print("yellow detected")

    #Dibujar círculos
    for (x, y, r) in circles:
      cv.circle(frame_new, (x,y), r, (0, 255, 0), 2)
      yellowAmount += 1

  yellow_img = bridge.cv2_to_imgmsg(frame_new, encoding="bgr8")
  yellow_pub.publish(yellow_img)
  

  

def colorsCallback(image_raw):
  global state, frame, begin
  frame = bridge.imgmsg_to_cv2(image_raw, desired_encoding="bgr8")
  frame_new = cv.flip(frame, -1)
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
    rospy.init_node("circle_detection")

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
    yellow_pub = rospy.Publisher('yellow_detector', Image, queue_size=10)
    red_pub = rospy.Publisher('red_detector', Image, queue_size=10)
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
        greenAmount = 0
        redAmount = 0
        yellowAmount = 0
        # green_pub.publish(green_detect)
        
        rate.sleep()