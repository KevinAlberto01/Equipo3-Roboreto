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

angle = 0
dist = 0

begin = False
bridge = CvBridge()

frame = np.array((350, 500, 3), dtype= np.uint8)

def lineCallback(image_raw):
  global angle, dist, frame, begin
  frame = bridge.imgmsg_to_cv2(image_raw, desired_encoding="bgr8")
  raw_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
  # line_pub.publish(raw_msg)
  frame = frame[frame.shape[0]*5/6:frame.shape[0], frame.shape[1]*3/16:frame.shape[1]*13/16]
  begin = True
      
      

#Stop Condition
def stop():
  #Cerrar Ventanas de cámara
  cv.destroyAllWindows()

  #Liberar cámara
  # cap.release()


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("line_follower")
    # cap = cv.VideoCapture(0)

    rate = rospy.Rate(100)

    rospy.on_shutdown(stop)
    #Setup Publishers to visualize the results of mask and thresholding
    ima_pub = rospy.Publisher('camera_raw', Image, queue_size=10)
    line_pub = rospy.Publisher('line_center', Image, queue_size=10)

    follower_pub = rospy.Publisher('line_follower', Point, queue_size=10)    

    rospy.Subscriber('/video_source/raw', Image, lineCallback)
    
    #Run the node
    while not rospy.is_shutdown():
        #Leer imagen de la cámara
        # ret, frame = cap.read()
        if(begin):
          # print("0")
          #  Convertir la imagen al espacio de color HSV
          # cv.rectangle(frame,(0,0),(frame.shape[1],frame.shape[0]*5/6),(255,255,255),-1)
          
          # hsv_image = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

          # Definir los límites inferiores y superiores del rango negro
          # lower_black = np.array([0, 0, 0])
          # upper_black = np.array([60, 30, 50])

          # mask = cv.inRange(hsv_image, lower_black, upper_black)


          # Aplicar la máscara inversa a la imagen original para obtener solo los píxeles negros
          # result = cv.bitwise_and(frame, frame, mask=mask)

          gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

          # Aplicar umbral adaptativo para convertir la imagen en blanco y negro
          _, thresh = cv.threshold(gray, 90, 255, cv.THRESH_BINARY_INV)

          # Encontrar los contornos de la línea
          contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2:]
          # cont = cv.drawContours(frame, contours, -1, (0,255,0), 3)

          # Si se detecta al menos un contorno
          if len(contours) > 0:
            # Encontrar el contorno más grande (la línea principal)
            main_contour = max(contours, key=cv.contourArea)

            # Calcular el centro del contorno principal
            M = cv.moments(main_contour)
            if(M['m00'] == 0):
              dist = 0
              angle = 0
              # print("1")
            else:
              cx = int(M['m10'] / M['m00'])

              # Calcular el ángulo entre el centro de la imagen y la coordenada x del centro del contorno
              center_x = frame.shape[1] // 2
              angle = np.arctan2(center_x - cx, frame.shape[0]) 

              # Dibujar un círculo en el centro del contorno principal
              circle = cv.circle(frame, (cx, frame.shape[0] // 2), 5, (0, 0, 255), -1)
              line_center = bridge.cv2_to_imgmsg(circle, encoding="bgr8")
              line_pub.publish(line_center)
              # print("2")
              # Mover el robot en función del ángulo
              if angle < 0.1 or angle > -0.1:
                dist = 5

              else:
                dist = 0    
              #elif angle < -0.1:
                  # Girar a la izquierda
                
                
          else:
            # No se detectó ninguna línea, detener el robot
            dist = 0
            angle = 0
            circle = cv.circle(frame, (frame.shape[1]//2, frame.shape[0] // 2), 5, (0, 0, 255), -1)
            line_center = bridge.cv2_to_imgmsg(circle, encoding="bgr8")
            print("no")
            line_pub.publish(line_center)
            # print("3")
        
        follower_pub.publish(dist, angle, 0)

        
        rate.sleep()