#!/usr/bin/env python3
import cv2
import numpy as np
 
def thresholding(img):
    imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lowermask = np.array([50, 4,41])
    uppermask = np.array([189,99,151])
    maskWhite = cv2.inRange(imgHsv,lowermask,uppermask)
    return maskWhite
 
def warpImg(img,points,w,h,inv = False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1,pts2)
    imgWarp = cv2.warpPerspective(img,matrix,(w,h))
    return imgWarp
 
def nothing(a):
    pass
 
def getHistogram(img,minPer=0.1, display = False,region=1):
 
    if region ==1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0]//region:,:], axis=0)
 
    #print(histValues)
    maxValue = np.max(histValues)
    minValue = minPer*maxValue
 
    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))
    #print(basePoint)
 
    return basePoint