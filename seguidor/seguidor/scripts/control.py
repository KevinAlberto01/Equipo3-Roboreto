#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

#Setup path parameters
speedR = rospy.get_param("/speed", 0.25)
kw = rospy.get_param("/kw", 1.2)
iw = rospy.get_param("/iw", 0.00)
dw = rospy.get_param("/dw", 0.00)

#Setup variables
lVel = 0.0

time_elapsed = 0.0
init_time = 0.0
prev_Time = 0.0
dt = 0.0

msg = Twist()
msg.linear.x = 0
msg.angular.z = 0

#Flags
corOp = 1
stop = 0
state = 0

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
wMax = 0.4
lMax = 0.25

wMaxF = 0.4

wMaxC = 0.15

wMin = 0.1
lMin = 0.1


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
   global dist, angle
   dist, angle = [follower.x, follower.y]


# def followerCallback(follower):
#    global lineCenter, prevLine, currPosition
#    lineCenter = [follower.x, follower.y]
#    if(lineCenter != prevLine):
#       prevLine = lineCenter
#       currPosition = [0.0, 0.0]


#Setup speed
lVel = speedR

#Determina si la velocidad es adecuada o se sale de rango
if (lVel < 0.1):
   lVel = 0.0
   corOp = 0
   print("Speed too low for correct operation")
   print("Recommended minimum speed: 0.1")
if (lVel > 0.25):
   lVel = 0.0
   corOp = 0
   print("Speed too high for correct operation")
   print("Recommended maximum speed: 0.25")

   
#Realiza los calculos necesarios para ver a donde moverse
def go():
   global angle, wMax
   wMax = wMaxF
   move(angle)

#Realiza los calculos necesarios para ver a donde moverse a una menor velocidad
def caution():
   global angle, wMax
   wMax = wMaxC
   move(angle)

# Calcula el error entre la ubicacion actual del robot y su destino
# Calcula el error de orientacion entre la actual y la necesaria para tener el punto de frente
def difPos():
   global currAngle, angle

   xd = lineCenter[0] - currPosition[0]
   yd = lineCenter[1] - currPosition[1]

   if(xd > 0.2 or xd < -0.2 or yd > 0.2 or yd < -0.2):
      angle = wrapToPi(np.arctan2(yd,xd) - currAngle)
      if (np.abs(angle) < 0.1):
         angle = 0
   else:
      angle = 0.0

   # print(angle)

def updatePos():
   global currPosition, currAngle, lVel, wr, wl
   tAngle = r * (wr - wl) / l * dt
   currAngle = tAngle + currAngle
   tDistance = r * (wr + wl) / 2 * dt
   currPosition[0] = (np.cos(currAngle) * tDistance) + currPosition[0]
   currPosition[1] = (np.sin(currAngle) * tDistance) + currPosition[1]
   #print(currPosition)
   #print(currAngle)

#Controlador de velocidades angulares y lineales
#Resuelve primero la diferencia de angulo y luego resuelve la diferencia de posicion
def move(angle):
   global errorIa, prev_errorA
   global kw, iw, dw, wr, wl, uw
   
   errorPa = angle
   errorIa = errorIa + errorPa*dt
   errorDa = (errorPa - prev_errorA)/dt

   uw = kw * errorPa + iw *errorIa + dw * errorDa

   if(uw > wMax): uw = wMax
   if(uw < wMin and uw > -wMin): uw = 0
   if(uw < -wMax): uw = -wMax

   print(uw)

   if (np.abs(angle) > 0.1):
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
    rospy.Subscriber('/line_follower', Point, followerCallback)
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
            print("Stop")
            move(0)
            msg.linear.x = 0.0
            if(green > 0):
               state = 1
               msg.linear.x = lVel
         elif(state == 1):
            print("Go")
            go()
            if(red > 0):
               state = 0
            elif(yellow > 0):
               state = 2
               msg.linear.x = lVel/2
         elif(state == 2):
            print("Caution")
            caution()
            if(red > 0):
               state = 0
               msg.linear.x = 0.0
            elif(green > 0):
               state = 1
               msg.linear.x = lVel
         

      pubVel.publish(msg)
      pubState.publish(state)
      prev_Time = float(time_elapsed)

      loop_rate.sleep()

