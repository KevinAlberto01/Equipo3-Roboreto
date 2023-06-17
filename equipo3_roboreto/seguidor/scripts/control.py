#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

#Setup path parameters
speedR = rospy.get_param("/speed", 0.08)
kw = rospy.get_param("/kw", 1.2)
iw = rospy.get_param("/iw", 0.00)
dw = rospy.get_param("/dw", 0.00)

kl = rospy.get_param("/kl", 1.0)

#Setup variables
lVel = 0.0

time_elapsed = 0.0
init_time = 0.0
prev_Time = 0.0
dt = 0.0

signal = -1

msg = Twist()
msg.linear.x = 0
msg.angular.z = 0

#Flags
corOp = 1
stop = 0
state = 0
stateTemp = 0

#Variables del puzzlebot
r = 0.05
l = 0.19

currPosition = np.array([0.0, 0.0])
prevLine = np.array([0.0, 0.0])
lineCenter = np.array([0.0, 0.0])
angle = 0.0
currAngle = 0.0

wr = 0.0
wl = 0.0

#Stoplight
green = 0
red = 0
yellow = 0

#Error parameters
errorPa = 0
errorIa = 0
errorDa = 0
prev_errorA = 0

uw = 0

#ToDo
wMax = 0
lMax = 0

wMaxF = speedR
lMaxF = speedR

wMaxC = speedR/2
lMaxC = speedR/2

wMin = 0.01
lMin = 0.01

fTime = 0.0


#Callbacks
def wrCallback(ar):
   global wr
   wr = ar.data

def wlCallback(al):
   global wl
   wl = al.data

def stoplight(colors):
   global green, red, yellow
   green = colors.x
   red = colors.y
   yellow = colors.z

def followerCallback(follower):
   global angle
   angle = follower.data
   # print(angle)

def signalCallback(signalMsg):
   global signal
   signal = signalMsg.data


#Setup speed
# lVel = speedR

#Determina si la velocidad es adecuada o se sale de rango
# if (lVel < 0.1):
#    lVel = 0.0
#    corOp = 0
#    print("Speed too low for correct operation")
#    print("Recommended minimum speed: 0.1")
# if (lVel > 0.25):
#    lVel = 0.0
#    corOp = 0
#    print("Speed too high for correct operation")
#    print("Recommended maximum speed: 0.25")

def signStop():
   global state
   state = 0

def signForward():
   fTime = (rospy.get_time())
   while(fTime + (3) > rospy.get_time()):
      msg.angular.z = 0
      msg.linear.x = lMaxF
      pubVel.publish(msg)
   
   print("End forward")
   state = 0
   # msg.angular.z = 0
   # msg.linear.x = 0
   # pubVel.publish(msg)
   # rospy.sleep(1)
   
   
def signWork():
   global state
   state = 2

def signLeft():
   fTime = (rospy.get_time())
   while((rospy.get_time())+(0.4/lMaxF) > fTime):
      msg.angular.z = -wMaxF
      msg.linear.x = lMaxF

def signRight():
   fTime = (rospy.get_time())
   while((rospy.get_time())+(0.4/lMaxF) > fTime):
      msg.angular.z = wMaxF
      msg.linear.x = lMaxF


def checkSign():
   global signal
   if(signal == 4):
      print("Stopped")
      # signStop()
   elif(signal == 0):
      print("Forward")
      # signForward()
   elif(signal == 5):
      print("Work")
      # signWork()
   elif(signal == 7):
      print("Left")
      # signLeft()
   elif(signal == 2):
      print("Right")
      # signRight()

   
#Realiza los calculos necesarios para ver a donde moverse
def go():
   global angle, wMax, lMax
   wMax = wMaxF
   lMax = lMaxF
   move(angle)

#Realiza los calculos necesarios para ver a donde moverse a una menor velocidad
def caution():
   global angle, wMax, lMax
   wMax = wMaxC
   lMax = lMaxC
   move(angle)

def halt():
   global angle, wMax, lMax
   wMax = 0
   lMax = 0
   move(0)

#Controlador de velocidades angulares y lineales
#Resuelve primero la diferencia de angulo y luego resuelve la diferencia de posicion
def move(angle):
   global errorIa, prev_errorA
   global kw, iw, dw, wr, wl, uw, kl
   
   # print(angle)
   errorPa = -angle
   errorIa = errorIa + errorPa*dt
   errorDa = (errorPa - prev_errorA)/dt

   uw = kw * errorPa + iw *errorIa + dw * errorDa
   if(angle > 0):
      ul = np.abs(kl / errorPa)
   else:
      ul = lMax

   if(ul < lMin): ul = lMin
   if(ul > lMax): ul = lMax

   # if(uw > ul): uw = ul
   if(uw > wMax): uw = wMax
   if(uw < wMin and uw > -wMin): uw = 0
   if(uw < -wMax): uw = -wMax

   # print(ul)

   msg.linear.x = ul

   if (np.abs(angle) > 0.00001):
      msg.angular.z = uw
   else:  
      msg.angular.z = 0.0
   

   prev_errorA = errorPa
   

def wrapToPi(ang):
   result = np.fmod((ang + np.pi),(2*np.pi))
   if(result < 0):
      result += 2 * np.pi
   return result - np.pi

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    state = 0
    pubVel.publish(msg)
    pubState.publish(state)
    print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Controller")
    loop_rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers
    pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubState = rospy.Publisher('/state', Int32, queue_size = 1)
    rospy.Subscriber('/wl', Float32, wlCallback)
    rospy.Subscriber('/wr', Float32, wrCallback)
    rospy.Subscriber('/colors', Point, stoplight)
    rospy.Subscriber('/line_follower', Float32, followerCallback)
    rospy.Subscriber('/signals', Int32, signalCallback)
    print("The Controller is Running")

    #Run the node
    while not rospy.is_shutdown():
      time_elapsed = float(rospy.get_time())
      dt = float(time_elapsed - prev_Time)
      #Evita saltos grandes en el tiempo
      if(dt > 1.0 or dt == 0): dt = 0.01

      #Revisa si el puzzlebot puede operar correctamente
      if(corOp):

         if(state == 0):
            # print("Stop")
            halt()
            if(green > 0):
               state = 1
            checkSign()
         elif(state == 1):
            print("Go")
            go()
            if(red > 0):
               state = 0
            elif(yellow > 0):
               state = 2
            checkSign()
         elif(state == 2):
            print("Caution")
            caution()
            if(red > 0):
               state = 0
            elif(green > 0):
               state = 1
            checkSign()
         

      pubVel.publish(msg)
      pubState.publish(state)
      prev_Time = float(time_elapsed)

      loop_rate.sleep()