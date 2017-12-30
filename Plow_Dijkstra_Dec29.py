# Save as server.py 
# Message Receiver
import os
from socket import *
import socket
import serial
import time
from time import sleep
#import matplotlib
import math
from math import sqrt
from math import cos
from math import sin
import sys
from picamera.array import PiRGBArray
from picamera import PiCamera
import stopsignDetection
import cv2
import grid_example_dijkstra_singleI as dijI
import grid_example_dijkstra_TripleI as dijT
import matplotlib.pyplot as plt
#import msvcrt


def fixAngle(tol, wpX, wpY, xAct, yAct):
    thetaGood = False
    thetaPlow = 90
    sensact = 90
    counter = 0
    stateChange = False
    movementChar = 's'
    while not thetaGood:
        counter = counter+1
        #while arduino.inWaiting():
        while True:
            #readline = arduino.readline()
            readline = 2.1
            sensact = float(readline)
        if sensact>180:
            sensact-=360
        thetaPlow = sensact+90
        if thetaPlow>180:
            thetaPlow-=360
        thetaDesired = math.atan2(wpY-y,wpX-x)
        thetaDif = thetaPlow-thetaDesired
        if not (abs(thetaDif)<tol or abs(thetaDif)>(360-tol)): #if not within the tolerance
            if not stateChange:
                if thetaDif>180:
                    movementChar = 'l'
                if thetaDif>=0 and thetaDif<180:
                    movementChar = 'r'
                if thetaDif<0 and thetaDif>=-180:
                    movementChar = 'l'
                if thetaDif<-180:
                    movementChar = 'r'
                stateChange = True
                arduino.write(movementChar)
            if(counter%10==0):
                print("Turn to the: " + movementChar)
                print('thetaPlow: ' + str(thetaPlow)+' thetaDesired: ')+str(thetaDesired)+' thetaDif: ' + str(thetaDif)
        else: #If the plow direction is within the tolerance
            print('theta is good')
            return True

#COMPORT = int(input("Port number: "))
#arduino = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
#arduino.port = "COM{}".format(COMPORT)
print ("Opening Serial port...")
time.sleep(2)
#print (arduino.readline())
time.sleep(2)
print ("Initialization complete")

#waypoints = [[0,3],[0,4],[0,5],[0,6],[0,7],[0,8],[0,9],[0,10],[0,11],[0,12],[0,13],[0,14],[0,13],[0,12],[0,11],[0,10],[0,9],[0,8],[0,7],[0,6],[0,5],[0,4],[0,3]]
#waypoints =[[0,3],[1,4],[1,5],[0,6],[-1,5],[0,3]]
#waypoints = [[0,3],[0,7],[0,10], [0,5]]
#waypoints = [[3,3],[0,-2],[0,4]]

host = ""
port = 13000
buf = 1024
addr = (host, port)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("", 8090))
s.listen(5)

#camera setup:
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))
time.sleep(2)
orientationTol = 15
orientation = 100
myByte = 0
offset = 3.14159/2

print "Waiting to receive messages..."

#Receive the obstacle list over wifi:
connection, address = s.accept()
data = connection.recv(128)
(ob1X, ob1Y, ob2X, ob2Y) = data.split() #receive obstacle list
ob1X = float(ob1X)
ob1Y = float(ob1Y)
ob2X = float(ob2X)
ob2Y = float(ob2Y)
obsList = []
obsList.append([ob1X,ob1Y])
obsList.append([ob2X,ob2Y])
print(obsList)
#waypoints = dijI.generateFullSingleI(obsList)
waypoints = dijT.generateTripleI(obsList)
xcoordlist = [] #coordinate lists for plotting
ycoordlist = []
for waypoint in waypoints:
    xcoordlist.append(float(waypoint.x))
    ycoordlist.append(float(waypoint.y))
plt.xlim(-1,15)
plt.ylim(-1,15)
plt.scatter(xcoordlist,ycoordlist)
plt.savefig('figure.png')

for waypoint in waypoints:
    wpY = waypoint.x
    wpX = waypoint.y
    #print("X: " + str(wpX)+" Y: " + str(wpY))
    while True:
#this chunk of code gets the location of the marker
        for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port=True):
            scene = frame.array
            #cv2.imshow("scene", scene)
            #key = cv2.waitKey(1) & 0xFF
            rawCapture.truncate(0)
            StopSignChances = stopsignDetection.detectStopSign(scene)
            print(StopSignChances)
            break
        if(StopSignChances<.15):
            #arduino.write('s')
            print("Stop Sign Detected! Stopping for 15 seconds.")
            time.sleep(15)
            continue

        connection, address = s.accept()
        data = connection.recv(128)
        (xstr, zstr, ystr, IDstr) = data.split() #note that our field coordinates are (x,y).
        x = float(xstr)/39.4
        y = float(ystr)/39.4
        z = float(zstr)/39.4
        ID = int(IDstr)
#This chunk of code corrects the marker location to the center of the "marker cube"
#Assumes marker one is on the back of the plow, with three facing forward. If it was a compass, NWSE would be 3412.
        #while arduino.inWaiting():
        while True:
            anglestr = arduino.readline()
            ts = float(anglestr) #theta sensor in degrees
            act = ts+90 #plow angle in degrees (90 is straight up field)
            break
        if act>=360:
            act = act-360
        x = x+cos(ts+offset*(ID-1))*12/39.4
        y = y+sin(ts+offset*(ID-1))*12/39.4
#This calculates distance to the waypoint and sets the distance tolerance
        distToWP = sqrt(pow((wpY-y),2)+pow((wpX-x),2)) #meters
        distTol = .75 #in meters        
        print("(X,Y): ("+str(x)+","+str(y)+"). Desired: ("+str(wpX)+","+str(wpY)+").    Orientation: " +str(act))

#This is navigation control:
        if(distToWP>distTol): #If we're too far away from the wapoint
            headingCorrect = fixAngle(orientationTol, wpX, wpY, x, y)#fix angle is able to fix the angle without relying on a positional update.  Should make it work a bit better!
            if(headingCorrect): #If heading is correct, drive towards the waypoint.
                print("Drive forward")
                #arduino.write('f')
                time.sleep(.25)
        else:   #If we're close enough, stop for two seconds. 
            #arduino.write('s')
            print('waypoint reached!')
            time.sleep(2)
            break

    print('next wp')
UDPSock.close()
os._exit(0)


#point in right direction
#if within right direction tolerance
#go in a straight line towards next waypoint until
#distance < tolerance
#go to next iteration of the for loop, or the next waypoint
#if done, stop


