#!/usr/bin/env python
# -*- coding: utf-8 -*.
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

bridge = CvBridge()

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=640,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )
    


#Stop Condition
def stop():
    print("Stopping")
    video_capture.release()
  #Cerrar Ventanas de cámara
#   cv.destroyAllWindows()


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("camera")

    # camera = videoSource("csi://0")
    print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    # video_capture = cv2.VideoCapture(0)

    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)
    #Setup Publishers to visualize the results of mask and thresholding
    ima_pub = rospy.Publisher('/video_source/raw', Image, queue_size=10) 
    image_path = "/home/puzzlebot/catkin_ws/src/yolo_rec/src/image.jpg"
    directory = "/home/puzzlebot/catkin_ws/src/yolo_rec/src/"
    os.chdir(directory)
    filename = 'Image.jpg'
    #Run the node
    while not rospy.is_shutdown():
        if video_capture.isOpened():
            ret_val, frame = video_capture.read()
            # Check to see if the user closed the window
            # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
            # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
            if not ret_val:
                break

            cv2.imwrite(filename, frame)
            # print(".")

            # frame = cv2.flip(frame, -1)
        
            raw_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ima_pub.publish(raw_msg)
        rate.sleep()