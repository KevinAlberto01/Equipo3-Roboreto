#!/usr/bin/env python
# -*- coding: utf-8 -*.
import rospy
import cv2
import numpy as np
import utlis
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
 
curveList = []
avgVal=4
curve = 0 
prevCurve = 0
bridge = CvBridge()
a = 0.4

def lineCallback(image_raw):
  global curve
  frame = bridge.imgmsg_to_cv2(image_raw, desired_encoding="bgr8")
  
  img = cv2.resize(frame,(480,240))
  curve = getLaneCurve(img,display=1)
  follower_pub.publish(curve)
#   print(curve)

def getLaneCurve(img,display=1):
    global prevCurve
 
    imgCopy = img.copy()
    imgResult = img.copy()
    #### STEP 1
    imgThres = utlis.thresholding(img)
 
    #### STEP 2
    hT, wT, c = img.shape
    widthTop = 137
    heightTop = 194
    widthBottom = 90
    heightBottom = 240
    points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop),
                      (widthBottom , heightBottom ), (wT-widthBottom, heightBottom)])
    imgWarp = utlis.warpImg(imgThres,points,wT,hT)
 
    #### STEP 3
    middlePoint = utlis.getHistogram(imgWarp,minPer=0.5, display = False,region=4)
    curveAveragePoint = utlis.getHistogram(imgWarp, display = False, minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint
 
    #### SETP 4
    curveList.append(curveRaw)
    if len(curveList)>avgVal:
        curveList.pop(0)
    curve = (sum(curveList)/len(curveList))
 
    #### STEP 5
    if display != 0:
        imgInvWarp = utlis.warpImg(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
        imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
        midY = 450
        cv2.putText(imgResult, str(curve), (wT // 2 - 80, 85), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3)
        cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
        cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)
        for x in range(-30, 30):
            w = wT // 20
            cv2.line(imgResult, (w * x + int(curve // 50), midY - 10),
                     (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)
        #fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
        #cv2.putText(imgResult, 'FPS ' + str(int(fps)), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (230, 50, 50), 3);
    
    #cv2.imshow('Resutlt', imgResult)
    imgStacked_msg = bridge.cv2_to_imgmsg(imgResult, encoding="bgr8")
    # raw_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    line_pub.publish(imgStacked_msg)
    
    

    #### NORMALIZATION
    # curve = a*curve + (1-a)*prevCurve
    # prevCurve = curve
    fcurve = float(curve)/100
    if fcurve>1: fcurve == 1
    if fcurve<-1:fcurve == -1
    # print (curve)
    return fcurve
 
#Stop Condition
def stop():
  #Cerrar Ventanas de cámara
  cv2.destroyAllWindows()

  #Liberar cámara
  # cap.release()

 
if __name__ == '__main__':
    rospy.init_node("line_follower")
    intialTrackBarVals = [202, 151, 166, 229 ]
    # utlis.initializeTrackbars(intialTrackBarVals)

    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    line_pub = rospy.Publisher('line_center', Image, queue_size=10)

    follower_pub = rospy.Publisher('line_follower', Float32, queue_size=10)    

    rospy.Subscriber('/video_source/raw', Image, lineCallback)

    while not rospy.is_shutdown():
        #cv2.imshow('Vid',img)

        rate.sleep()