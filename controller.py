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
    
def constrain(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

############################################################################################
# PID algorithm to take input accelerometer readings, and target accelermeter requirements, and
# as a result feedback new rotor speeds.
############################################################################################
class PID:

        def __init__(self, p_gain, i_gain, d_gain):
                self.last_error = 0.0
                self.last_time = time.time()
                self.p_gain = p_gain
                self.i_gain = i_gain
                self.d_gain = d_gain

                self.i_error = 0.0
                self.i_err_min = 0.0
                self.i_err_max = 0.0
                if i_gain != 0.0:
                        self.i_err_min = -250.0 / i_gain
                        self.i_err_max = +250.0 / i_gain


        def Compute(self, input, target):

                now = time.time()
                dt = (now - self.last_time)

                #---------------------------------------------------------------------------
                # Error is what the PID alogithm acts upon to derive the output
                #---------------------------------------------------------------------------
                error = target - input

                #---------------------------------------------------------------------------
                # The proportional term takes the distance between current input and target
                # and uses this proportially (based on Kp) to control the ESC pulse width
                #---------------------------------------------------------------------------
                p_error = error

                #---------------------------------------------------------------------------
                # The integral term sums the errors across many compute calls to allow for
                # external factors like wind speed and friction
                #---------------------------------------------------------------------------
                self.i_error += (error + self.last_error) * dt
                if self.i_gain != 0.0 and self.i_error > self.i_err_max:
                        self.i_error = self.i_err_max
                elif self.i_gain != 0.0 and self.i_error < self.i_err_min:
                        self.i_error = self.i_err_min
                i_error = self.i_error

                #---------------------------------------------------------------------------
                # The differential term accounts for the fact that as error approaches 0,
                # the output needs to be reduced proportionally to ensure factors such as
                # momentum do not cause overshoot.
                #---------------------------------------------------------------------------
                d_error = (error - self.last_error) / dt

                #---------------------------------------------------------------------------
                # The overall output is the sum of the (P)roportional, (I)ntegral and (D)iffertial terms
                #---------------------------------------------------------------------------
                p_output = self.p_gain * p_error
                i_output = self.i_gain * i_error
                d_output = self.d_gain * d_error

                #---------------------------------------------------------------------------
                # Store off last input (for the next differential calulation) and time for calculating the integral value
                #---------------------------------------------------------------------------
                self.last_error = error
                self.last_time = now

                #---------------------------------------------------------------------------
                # Return the output, which has been tuned to be the increment / decrement in ESC PWM
                #---------------------------------------------------------------------------
                return p_output + i_output + d_output
                
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

#declare RC receiver values (1000-2000us)
rcthr = 0.0
rcyaw = 0.0
rcpitch = 0.0
rcroll = 0.0

#initialize PIDs (Pgain, Igain, Dgain)
ps_pid = PID(4.5, 0, 0)
rs_pid = PID(4.5, 0, 0)
ys_pid = PID(4.5, 0, 0)

pr_pid = PID(0.7, 0, 0)
rr_pid = PID(0.7, 0, 0)
yr_pid = PID(0.7, 0, 0)

#initial motor values, 400Hz, 1us-2us, 40%-80%
motor1 = 40.0    #Front Left (CW)    (+roll,-pitch,-yaw)
motor2 = 40.0    #Front Right (CCW)  (-roll,-pitch,+yaw)
motor3 = 40.0    #Back Right (CW)    (-roll,+pitch,-yaw)
motor4 = 40.0    #Back Left (CCW)    (+roll,+pitch,+yaw)

#initialize output pins
PWM.start("P9_14", motor1, 400, 0)
PWM.start("P9_21", motor2, 400, 0)
PWM.start("P9_42", motor3, 400, 0)
PWM.start("P8_13", motor4, 400, 0)

time.sleep(3)    #wait 3s for ESC to start, keep at minimum thro

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

    #read RC receiver values from PRU
    rcthr = 1400.0
    rcpitch = 1500.0
    rcroll = 1500.0
    rcyaw = 1500.0

    rcpitch = map(rcpitch, 1000, 2000, -45, 45)
    rcroll = map(rcroll, 1000, 2000, -45, 45)
    rcyaw = map(rcyaw, 1000, 2000, -180, 180)
    
    #print("rcpitch: "+str(rcpitch)+", rcroll: "+str(rcroll)+", rcyaw: "+str(rcyaw)) #debug
    
    #Read Gyro and Accelerometer Data
    ax, ay, az = accel.read()
    gx, gy, gz = gyro.getDegPerSecAxes()

    #print("Ax: "+str(ax)+", Ay: "+str(ay)+", Az: "+str(az)) #debug
    #print("Gx: "+str(gx)+", Gy: "+str(gy)+", Gz: "+str(gz)) #debug
    
    epitch, eroll, eyaw = accel.getEulerAngles(ax, ay, az)
    
    epitch = todeg(epitch)
    eroll = todeg(eroll)
    eyaw = todeg(eyaw)
    
    #print("ex: "+str(epitch)+", ey: "+str(eroll)+", ez: "+str(eyaw)) #debug
    #mx, my, mz = compass.getAxes()
    
    #Calculate accurate data with complementary filter
    #i_pitch = round(i_pitch + (gy*delta_time),1)
    #i_roll = round(i_roll + (gx*delta_time),1)
    i_yaw = round(i_yaw + (gz*delta_time),1)
    
    pitch = round(epitch,1)
    roll = round(eroll,1)
    yaw = i_yaw
    
    #print("pitch: "+str(pitch)+", roll: "+str(roll)+", yaw: "+str(yaw)) #debug

    #Calculate PID stab and rate of each axis, 6 total
    pitchstab = ps_pid.Compute(pitch, rcpitch)
    rollstab = rs_pid.Compute(roll, rcroll)
    yawstab = ys_pid.Compute(yaw, rcyaw)

    pitchout = pr_pid.Compute(gx, pitchstab)
    rollout = rr_pid.Compute(gy, rollstab)
    yawout = yr_pid.Compute(gz, yawstab)

    #print("pitchout: "+str(pitchout)+", rollout: "+str(rollout)+", yawout: "+str(yawout)) #debug

    #Combine user input values and PID outputs to obtain individual motor speed
    motor1 = map(rcthr + rollout - pitchout - yawout, 1000.0, 2000.0, 40.0, 80.0)
    motor2 = map(rcthr - rollout - pitchout + yawout, 1000.0, 2000.0, 40.0, 80.0)
    motor3 = map(rcthr - rollout + pitchout - yawout, 1000.0, 2000.0, 40.0, 80.0)
    motor4 = map(rcthr + rollout + pitchout + yawout, 1000.0, 2000.0, 40.0, 80.0)
    
    motor1 = constrain(motor1, 40.0, 80.0)
    motor2 = constrain(motor2, 40.0, 80.0)
    motor3 = constrain(motor3, 40.0, 80.0)
    motor4 = constrain(motor4, 40.0, 80.0)

    #print("Front Left: "+str(motor1)) #debug
    #print("Front Right: "+str(motor2)) #debug
    #print("Back Right: "+str(motor3)) #debug
    #print("Back Left: "+str(motor4)) #debug    

    #Update each motor with new speed
    PWM.set_duty_cycle("P9_14", motor1)
    PWM.set_duty_cycle("P9_21", motor2)
    PWM.set_duty_cycle("P9_42", motor3)
    PWM.set_duty_cycle("P8_13", motor4)

    #time.sleep(0.1) #debug
  
  
#Exit and shut down everything
PWM.stop("P9_14")
PWM.stop("P8_13")
PWM.stop("P9_21")
PWM.stop("P9_42")
PWM.cleanup()
