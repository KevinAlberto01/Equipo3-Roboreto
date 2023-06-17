#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from geometry_msgs.msg import Point

# Constants and Parameters
CONTOUR_THRESHOLD = 100
CENTRAL_LINE_THRESHOLD = 10
CONTOUR_REGION_HEIGHT_RATIO = 5/6
CONTOUR_REGION_WIDTH_RATIO = [3/16, 13/16]

# Topics
VIDEO_SOURCE_TOPIC = '/video_source/raw'
LINE_CENTER_TOPIC = 'line_center'
LINE_FOLLOWER_TOPIC = 'line_follower'

# Global Variables
bridge = CvBridge()
angle = 0
dist = 0

def line_callback(image_raw):
    global angle, dist
    
    img = bridge.imgmsg_to_cv2(image_raw, desired_encoding="bgr8")
    frame = cv.flip(img, -1)
    
    # # Crop region of interest for contour lines
    # contour_region_top = int(frame.shape[0] * CONTOUR_REGION_HEIGHT_RATIO)
    # contour_region_left = int(frame.shape[1] * CONTOUR_REGION_WIDTH_RATIO[0])
    # contour_region_right = int(frame.shape[1] * CONTOUR_REGION_WIDTH_RATIO[1])
    # contour_region = frame[contour_region_top:frame.shape[0], contour_region_left:contour_region_right]

    # # Convert to grayscale
    # gray = cv.cvtColor(contour_region, cv.COLOR_BGR2GRAY)

    # # Apply threshold to obtain binary image
    # _, thresh = cv.threshold(gray, CONTOUR_THRESHOLD, 255, cv.THRESH_BINARY_INV)

    # # Find contours of the contour lines
    # contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # # Filter contours based on area and shape
    # filtered_contours = []
    # for contour in contours:
    #     area = cv.contourArea(contour)
    #     perimeter = cv.arcLength(contour, True)
    #     circularity = 4 * np.pi * (area / (perimeter * perimeter))
    #     if area > CONTOUR_THRESHOLD and circularity < 1.2:
    #         filtered_contours.append(contour)

    # # Find the central line
    # central_line = None
    # if len(filtered_contours) > 1:
    #     central_line = max(filtered_contours, key=cv.contourArea)

    # # Calculate angle and distance based on central line
    # if central_line is not None:
    #     M = cv.moments(central_line)
    #     if M['m00'] != 0:
    #         cx = int(M['m10'] / M['m00'])
    #         center_x = frame.shape[1] // 2
    #         angle = np.arctan2(center_x - cx, frame.shape[0]) 
    #         dist = 5 if -CENTRAL_LINE_THRESHOLD < angle < CENTRAL_LINE_THRESHOLD else 0

    # else:
    #     angle = 0
    #     dist = 0

    # # Publish line center and line follower information
    # line_center_image = cv.drawContours(frame, filtered_contours, -1, (0, 255, 0), 3)
    # line_center_msg = bridge.cv2_to_imgmsg(line_center_image, encoding="bgr8")
    # line_pub.publish(line_center_msg)

    # follower_msg = Point()
    # follower_msg.x = dist
    # follower_msg.y = angle
    # follower_msg.z = 0
    # follower_pub.publish(follower_msg)


def stop():
    cv.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node("line_follower")

    bridge = CvBridge()

    rospy.on_shutdown(stop)

    ima_pub = rospy.Publisher('camera_raw', Image, queue_size=10)
    line_pub = rospy.Publisher(LINE_CENTER_TOPIC, Image, queue_size=10)
    follower_pub = rospy.Publisher(LINE_FOLLOWER_TOPIC, Point, queue_size=10)

    rospy.Subscriber(VIDEO_SOURCE_TOPIC, Image, line_callback)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()