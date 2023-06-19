#!/usr/bin/env python
# -*- coding: utf-8 -*.
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2 as cv

# Setup Variables, parameters and messages to be used (if required)

cordObjeto_x = 0
cordObjeto_y = 0

cordRobot_x = 0
cordRobot_y = 0

#Stop Condition
def stop():
  #Cerrar Ventanas de cámara
  cv.destroyAllWindows()

  #Liberar cámara
  cap.release()


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("color_detection")

    #Set Blob Detector Parameters
    params = cv.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 10
    params.maxArea = 1000000000
    params.filterByCircularity = True
    params.minCircularity = 0.6
    params.minThreshold = 10
    params.maxThreshold = 200

    detector = cv.SimpleBlobDetector_create(params)

    bridge = CvBridge()

    #Obtain image from both cameras
    cap = cv.VideoCapture(4)
    cap2 = cv.VideoCapture(2) # check this

    rate = rospy.Rate(30)

    rospy.on_shutdown(stop)
    #Setup Publishers to visualize the results of mask and thresholding
    ima_pub = rospy.Publisher('camera_raw', Image, queue_size=10)
    ima_pub2 = rospy.Publisher('camera_raw2', Image, queue_size=10)

    both_detection_pub = rospy.Publisher('both_detection', Image, queue_size=10)
    both_detection_pub2 = rospy.Publisher('both_detection2', Image, queue_size=10)

    object_position_pub = rospy.Publisher("object_position", Point, queue_size=0) 
    
    #Initialize object previous and current coordinates
    cordObjeto_x = 0
    cordObjeto_y = 0
    cordObjeto_z = 0

    prev_objeto_x = 0
    prev_objeto_z = 0
    prev_objeto_y = 0
    #Run the node
    while not rospy.is_shutdown():
        #Leer imagen de la cámara
        ret, frame = cap.read()
        ret2, frame2 = cap2.read()
        
        #Convertir en mensaje de ROS y publicar la imagen original
        raw_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        ima_pub.publish(raw_msg)

        raw_msg2 = bridge.cv2_to_imgmsg(frame2, encoding="bgr8")
        ima_pub2.publish(raw_msg2)

        #Convertir a HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv2 = cv.cvtColor(frame2, cv.COLOR_BGR2HSV)

        #Declarar y aplicar máscaras de color
        lower_green = np.array([50, 70, 70])
        upper_green = np.array([80, 255, 255])

        mask_green = cv.inRange(hsv, lower_green, upper_green)
        
        mask_green2 = cv.inRange(hsv2, lower_green, upper_green)

        green = cv.bitwise_and(frame, frame, mask=mask_green)
        green2 = cv.bitwise_and(frame2, frame2, mask=mask_green2)
        
        #Cambiar la escala de colores a grises
        green_gray = cv.cvtColor(green, cv.COLOR_BGR2GRAY)
        green_gray2 = cv.cvtColor(green2, cv.COLOR_BGR2GRAY)
        
        #Aplicar un blur a las imágenes
        green_gray_blur = cv.medianBlur(green_gray,5)
        green_gray_blur2 = cv.medianBlur(green_gray2,5)
        
        #Uso de threshold inverso para eliminar ruido y círculos llenos
        green_thres = cv.adaptiveThreshold(green_gray_blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2)
        green_thres2 = cv.adaptiveThreshold(green_gray_blur2, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2)
        
        #Aplicar el detector de blobs para encontrar los círculos de la imagen
        object_blobs = detector.detect(green_thres)
        object_blobs2 = detector.detect(green_thres2)
        
        #Obtener las coordenadas de los círculos detectados con radio mayor a 30 pixeles y dibujarlos. La cámara 1 obtiene las coordenadas en X
        #la cámara 2 obtiene las coordenadas en Y y Z.
        for kp in object_blobs:
          if kp.size > 30:
            both_detect = cv.drawKeypoints(frame, object_blobs, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cordObjeto_x = kp.pt[0]

        for kp in object_blobs2:
          if kp.size > 30:
            both_detect2 = cv.drawKeypoints(frame2, object_blobs2, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cordObjeto_y = kp.pt[0]
            cordObjeto_z = kp.pt[1]
        
        #Convertir la imágen de los círculos detectados a un mensaje de ros y publicarlos en el tópico correspondiente
        detection_msg = bridge.cv2_to_imgmsg(both_detect, encoding="bgr8")
        detection_msg2 = bridge.cv2_to_imgmsg(both_detect2, encoding="bgr8")

        both_detection_pub.publish(detection_msg)
        both_detection_pub2.publish(detection_msg2)
        
        #Si el círculo detectado se encuentra a una distancia de 100 pixeles en 3 dimensiones, actualizar el valor de posición enviado al planer del xARM
        if (np.sqrt(np.square(prev_objeto_x - cordObjeto_x) + np.square(prev_objeto_y - cordObjeto_y) + np.square(prev_objeto_z - cordObjeto_z)) > 100) :

          #Publicar las coordenadas del objeto en escala del espacio del xARM
          # 27 cm en x, 38 cm en y, máximo de altura z de 20 cm.
          object_position_pub.publish((cordObjeto_x)/640*0.27, (cordObjeto_y-320)/640*0.38, (1 - cordObjeto_z/480)*0.2)
          prev_objeto_x = cordObjeto_x
          prev_objeto_z = cordObjeto_z
          prev_objeto_y = cordObjeto_y
        
        rate.sleep()