#!/usr/bin/env python
from __future__ import division
import signal
import socket
import time
import string
import sys
import getopt
import math
import threading
from array import *
import smbus
import select
import os
import struct
import logging
from Adafruit_I2C import Adafruit_I2C
from Adafruit_BMP085 import BMP085
from Adafruit_ADXL345 import ADXL345
from ThinkBowl_ITG3205 import ITG3205
from ThinkBowl_HMC5883L import HMC5883L
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import subprocess
from datetime import datetime
import shutil
import ctypes
from ctypes.util import find_library

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
def todeg(x):
    return x*(180/math.pi)


############################################################################################
#
# Main
#
############################################################################################

#initilize the sensors
accel = ADXL345(0x53)
gyro = ITG3205(0x68)
compass = HMC5883L(0x1e)
bmp = BMP085(0x77)

#assume quad start at level
#i_pitch = 0
#i_roll = 0
i_yaw = 0


#keep track of time
elapsed_time = 0.0
start_time = time.time()
current_time = start_time

#While Loop
while True:
    #update time
    current_time = time.time()
    delta_time = current_time - start_time - elapsed_time
    elapsed_time = current_time - start_time

    #Read Gyro and Accelerometer Data
    ax, ay, az = accel.read()
    gx, gy, gz = gyro.getDegPerSecAxes()

    #print("Ax: "+str(ax)+", Ay: "+str(ay)+", Az: "+str(az)) #debug
    #print("Gx: "+str(gx)+", Gy: "+str(gy)+", Gz: "+str(gz)) #debug
    
    epitch, eroll, eyaw = accel.getEulerAngles(ax, ay, az)
    
    epitch = todeg(epitch)
    eroll = todeg(eroll)
    eyaw = todeg(eyaw)
    
    print("epitch: "+str(epitch)+", eroll: "+str(eroll)+", eyaw: "+str(eyaw)) #debug
    
    #Calculate accurate data with complementary filter
    #i_pitch = round(i_pitch + (gy*delta_time),1)
    #i_roll = round(i_roll + (gx*delta_time),1)
    i_yaw = round(i_yaw + (gz*delta_time),1)

    #print("i_pitch: "+str(i_pitch)+", i_roll: "+str(i_roll)+", i_yaw: "+str(i_yaw)) #debug
    
    pitch = round(epitch,1)
    roll = round(eroll,1)
    yaw = i_yaw

    print("pitch: "+str(pitch)+", roll: "+str(roll)+", yaw: "+str(yaw)) #debug

    time.sleep(0.1) #debug
  
  
#Exit and shut down everything