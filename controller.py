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
                return p_output, i_output, d_output
                
############################################################################################
#
# Main
#
############################################################################################

motor1 = 0    #Front Left (CW)    (+roll,+pitch,-yaw)
motor2 = 0    #Front Right (CCW)  (-roll,+pitch,+yaw)
motor3 = 0    #Back Right (CW)    (-roll,-pitch,-yaw)
motor4 = 0    #Back Left (CCW)    (+roll,-pitch,-yaw)

#initialize output pins

#While Loop

  #Read Gyro and Accelerometer Data





  #Calculate accurate data with complementary filter





  #Calculate PID of each axis




  #Combine user input values and PID outputs to obtain individual motor speed
  
  
#Exit and shut down everything





#Update each motor with new speed
