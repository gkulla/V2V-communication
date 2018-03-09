#!/usr/bin/python
from __future__ import print_function
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Sensor, Car,Temperature_Sensor, GPS, XBee
import RPi.GPIO as GPIO
import time
import os
import threading
from threading import Thread
import atexit
import math as Math
from math import sin, cos, sqrt, atan2, radians

Flag = False
Flag2 = False

######################
# Temperature Sensor #
######################

MYTempSensor = Temperature_Sensor('/sys/bus/w1/devices/28-0316744489ff/w1_slave')
temperature = MYTempSensor.read_temp()
print("Sorrounding temperature is: ", temperature, "deg")


# create a default object, no changes to I2C address 
mh = Adafruit_MotorHAT(addr=0x60)
MyToyCar = Car(0x60, 150)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
atexit.register(turnOffMotors)

##################################
# Creating 5 object sensors      #
# We specify the trigger pins    #
# and the echo pins which we     #
# assign from the raspberry pi   #
##################################

FSensor   = Sensor(23, 24)
SL_Sensor = Sensor(12, 26) #Gpio 4 changed to Gpio 26
SR_Sensor = Sensor(18, 25)
FL_Sensor = Sensor(17, 22)
FR_Sensor = Sensor( 5, 13)

def Sensor_distance(Sensor):
        dist = Sensor.measure_average()
        return dist

def GPS_Data_Processing(lat1, lon1, lat2, lon2):
        p = 0.017453292519943295
        c = Math.cos
        a = 0.5 - Math.cos((lat2-lat1)*p)/2+Math.cos(lat1*p)*Math.cos(lat2*p)*(1-Math.cos((lon2-lon1)*p))/2
        print('Distance: ', 12742*Math.asin(Math.sqrt(a))*1000*100, 'cm')
        return (12742*Math.asin(Math.sqrt(a))*100000)

speedSound = 33100+(0.6*temperature)
print("Ultrasonic Measurements")
print("Speed of sound is", speedSound/100,"m/s at ",temperature,"deg")

p = Pool(5) #declaration of a pool of sensors
Sensors = []
Sensors = [FSensor,FR_Sensor,FL_Sensor,SL_Sensor,SR_Sensor]

MyGPS = GPS()
MyGPS.start()
xbee = XBee("/dev/tty50", 9600)

def fun():
        while true:
                Front_sensor_variable = FSensor.measure_average()
                lat = MyGPS.lat_var
                lon = MyGPS.lon_var
                speed = MyGPS.speed_var
                sent = xbee.SendStr(str(lat)+",",+str(lon)+","+str(Front_sensor_variable),0x0005)

                #a[]
                print("Sent_Address: ", xbee.received_address)
                b = [xbee.sent_address, lat, lon, Front_sensor_variable]
                print(b)
                a.append(b)
                time.sleep(0.3)
                Msg = xbee.Receive()
                if Msg:
                        content = Msg[7:-1].decode('ascii')
                        print('Address_received: ', xbee.received_address)

                        xbee.sensor_decode(content)
                        c = [xbee.received_address, xbee.latitude_received, xbee.longitude_received,xbee.front_variable_received]
                        print(c)
                        a.append(c)
                        print('------Routing Table------')
                        print(a)
                        if(GPS_Data_Processing(lat,lon,xbee.latitude_received, xbee_longitude_received) < 250):
                                if(xbee.front_variable_received < 50):
                                        global Flag
                                        Flag = True
                                else:
                                        global Flag2
                                        Flag2 = True
                        else:
                                global Flag
                                Flag = False
                                global Flag2
                                Flag2 = False

threading.Thread(target=fun).start()
                
def Xbee_Behind(flag):
        MyToyCar.DeAccelerate(150)
        MyToyCar.Left(230, 150)
        MyToyCar.Right(230, 150)

        while(flag):
                MyToyCar.MoveForward()

def Xbee_Intersect(flag2):
        MyToyCar.DeAccelerate(150)
        time.sleep(10)
        MyToyCar.Accelerate(150)
        while(flag2):
                MyToyCar.MoveForward()                    
                        
try:
  while True:
        ret = p.map(Sensor_distance, Sensors)
        distance = ret[0]
        FR_distance = ret[1]
        FL_distance = ret[2]
        SL_distance = ret[3]
        SR_distance = ret[4]
       
        #1 forward-------------------------------------------------------------------------------------------------
        if(distance > 65 and FR_distance > 25 and FL_distance > 25):
               MyToyCar.MoveForward()
               print("Forward!!!...")
               print("Distance : {0:5.1f}".format(distance))
               print("Distance right: {0:5.1f}".format(SR_distance))
               print("Distance left: {0:5.1f}".format(SL_distance))
               print("Distance Front Left: {0:5.1f}".format(FL_distance))
               print("Distance Front Right: {0:5.1f}".format(FR_distance))
               if(Flag):
                       print("XBEE ACTIVATED!!!...")
                       Xbee_Behind(Flag)
               if(Flag2):
                        print("XBEE INTERSECTION ACTIVATED!!!...")
                        Xbee_Intersect(Flag2)
               #1/1 forward/FL --------------------------------------------------
               if(distance > 65 and FL_distance < 45 and FR_distance > 45 and SL_distance > 25 and SR_distance > 25):
                       print("Forward/(FL)Right!!!...")
                       MyToyCar.TurnRight()
                       print("Turn Right!!!...")
                       print("Distance : {0:5.1f}".format(distance))
                       print("Distance right: {0:5.1f}".format(SR_distance))
                       print("Distance left: {0:5.1f}".format(SL_distance))
                       print("Distance Front Left: {0:5.1f}".format(FL_distance))
                       print("Distance Front Right: {0:5.1f}".format(FR_distance))
                       if(Flag):
                               print("XBEE ACTIVATED!!!...")
                               Xbee_Behind(Flag)
                       if(Flag2):
                               print("XBEE INTERSECTION ACTIVATED!!!...")
                               Xbee_Intersect(Flag2)
               #1/2 forward/FR ---------------------------------------------------
               elif(distance > 65 and FL_distance > 45 and FR_distance < 45 and SL_distance > 25 and SR_distance > 25):
                       print("Forward/(FR)Left!!!...")
                       MyToyCar.TurnLeft()
                       print("Turn Right!!!...")
                       print("Distance : {0:5.1f}".format(distance))
                       print("Distance right: {0:5.1f}".format(SR_distance))
                       print("Distance left: {0:5.1f}".format(SL_distance))
                       print("Distance Front Left: {0:5.1f}".format(FL_distance))
                       print("Distance Front Right: {0:5.1f}".format(FR_distance))
                       if(Flag):
                               print("XBEE ACTIVATED!!!...")
                               Xbee_Behind(Flag)
                       if(Flag2):
                               print("XBEE INTERSECTION ACTIVATED!!!...")
                               Xbee_Intersect(Flag2)
               #1/3 forward/SR-----------------------------------------------------------
               elif(distance > 65 and FL_distance > 45 and FR_distance > 45 and SL_distance > 25 and SR_distance < 15):
                       print("Forward/Left!!!...")
                       MyToyCar.TurnLeft()
                       print("Turn Right!!!...")
                       print("Distance : {0:5.1f}".format(distance))
                       print("Distance right: {0:5.1f}".format(SR_distance))
                       print("Distance left: {0:5.1f}".format(SL_distance))
                       print("Distance Front Left: {0:5.1f}".format(FL_distance))
                       print("Distance Front Right: {0:5.1f}".format(FR_distance))
                       if(Flag):
                               print("XBEE ACTIVATED!!!...")
                               Xbee_Behind(Flag)
                       if(Flag2):
                               print("XBEE INTERSECTION ACTIVATED!!!...")
                               Xbee_Intersect(Flag2)
               #1/4 forward/SL---------------------------------------------------------------------------
               elif(distance > 65 and FL_distance > 45 and FR_distance > 45 and SL_distance < 15 and SR_distance >25):
                       print("Forward/(SR)Right!!!...")
                       MyToyCar.TurnRight()
                       print("Turn Right!!!...")
                       print("Distance : {0:5.1f}".format(distance))
                       print("Distance right: {0:5.1f}".format(SR_distance))
                       print("Distance left: {0:5.1f}".format(SL_distance))
                       print("Distance Front Left: {0:5.1f}".format(FL_distance))
                       print("Distance Front Right: {0:5.1f}".format(FR_distance))
                       if(Flag):
                               print("XBEE ACTIVATED!!!...")
                               Xbee_Behind(Flag)
                       if(Flag2):
                               print("XBEE INTERSECTION ACTIVATED!!!...")
                               Xbee_Intersect(Flag2)
               #1/5 forward/SR & FR----------------------------------------------------------------------------------
               elif(distance > 65 and FL_distance > 45 and FR_distance < 45 and SL_distance > 25 and SR_distance < 15):
                       print("Forward/(SR & FR)Left!!!...")
                       MyToyCar.TurnLeft()
                       print("Turn Right!!!...")
                       print("Distance : {0:5.1f}".format(distance))
                       print("Distance right: {0:5.1f}".format(SR_distance))
                       print("Distance left: {0:5.1f}".format(SL_distance))
                       print("Distance Front Left: {0:5.1f}".format(FL_distance))
                       print("Distance Front Right: {0:5.1f}".format(FR_distance))
                       if(Flag):
                               print("XBEE ACTIVATED!!!...")
                               Xbee_Behind(Flag)
                       if(Flag2):
                               print("XBEE INTERSECTION ACTIVATED!!!...")
                               Xbee_Intersect(Flag2)
               #1/6 forward/SL & FL--------------------------------------------------------------------
               elif(distance > 65 and FL_distance < 45 and FR_distance > 45 and SL_distance < 15 and SR_distance > 25):
                       print("Forward/Right(SL & FL)!!!...")
                       MyToyCar.TurnRight()
                       print("Turn Right!!!...")
                       print("Distance : {0:5.1f}".format(distance))
                       print("Distance right: {0:5.1f}".format(SR_distance))
                       print("Distance left: {0:5.1f}".format(SL_distance))
                       print("Distance Front Left: {0:5.1f}".format(FL_distance))
                       print("Distance Front Right: {0:5.1f}".format(FR_distance))
                       if(Flag):
                               print("XBEE ACTIVATED!!!...")
                               Xbee_Behind(Flag)
                       if(Flag2):
                               print("XBEE INTERSECTION ACTIVATED!!!...")
                               Xbee_Intersect(Flag2)
        #2 backward--------------------------------------------------------------
        elif(distance < 20 or FL_distance < 25 or FR_distance < 25):
             MyToyCar.MoveBackward()
             print("Backawrd!!!...")
             print("Distance : {0:5.1f}".format(distance))
             print("Distance right: {0:5.1f}".format(SR_distance))
             print("Distance left: {0:5.1f}".format(SL_distance))
             print("Distance Front Left: {0:5.1f}".format(FL_distance))
             print("Distance Front Right: {0:5.1f}".format(FR_distance))
        elif((SL_distance > SR_distance) and SL_distance > 15 and ((distance >= 15 and distance <= 65) and (FL_distance >= 20 and FL_distance <= 105) and (FR_distance >= 25 and FR_distance <= 105))):
             MyToyCar.TurnLeft()
             print("Turn Left!!!...")
             print("Distance : {0:5.1f}".format(distance))
             print("Distance right: {0:5.1f}".format(SR_distance))
             print("Distance left: {0:5.1f}".format(SL_distance))
             print("Distance Front Left: {0:5.1f}".format(FL_distance))
             print("Distance Front Right: {0:5.1f}".format(FR_distance))
        elif((SL_distance < SR_distance) and SR_distance > 15 and ((distance >= 15 and distance <= 65) and (FL_distance >= 20 and FL_distance <= 105) and (FR_distance >= 25 and FR_distance <= 105))):
             MyToyCar.TurnRight()
             print("Turn Right!!!...")
             print("Distance : {0:5.1f}".format(distance))
             print("Distance right: {0:5.1f}".format(SR_distance))
             print("Distance left: {0:5.1f}".format(SL_distance))
             print("Distance Front Left: {0:5.1f}".format(FL_distance))
             print("Distance Front Right: {0:5.1f}".format(FR_distance))
      

if __name__ ==  '__main__':
        main()
GPIO.cleanup()


