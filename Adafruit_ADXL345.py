#!/usr/bin/python

# Python library for ADXL345 accelerometer.

# Copyright 2013 Adafruit Industries

# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

from Adafruit_I2C import Adafruit_I2C


class ADXL345:

    # Minimal constants carried over from Arduino library

    ADXL345_ADDRESS          = 0x53

    ADXL345_REG_DEVID        = 0x00 # Device ID
    ADXL345_REG_DATA_FORMAT  = 0x31
    ADXL345_REG_DATAX0       = 0x32 # X-axis data 0 (6 bytes for X/Y/Z)
    ADXL345_REG_DATAY0       = 0x34
    ADXL345_REG_DATAZ0       = 0x36
    ADXL345_REG_POWER_CTL    = 0x2D # Power-saving features control

    ACCELEROMETER_GAIN       = 0.004 
    GRAVITY                  = 9.813

    ADXL345_OFFSETX          = 0
    ADXL345_OFFSETY          = 17
    ADXL345_OFFSETZ          = 20

    ADXL345_DATARATE_0_10_HZ = 0x00
    ADXL345_DATARATE_0_20_HZ = 0x01
    ADXL345_DATARATE_0_39_HZ = 0x02
    ADXL345_DATARATE_0_78_HZ = 0x03
    ADXL345_DATARATE_1_56_HZ = 0x04
    ADXL345_DATARATE_3_13_HZ = 0x05
    ADXL345_DATARATE_6_25HZ  = 0x06
    ADXL345_DATARATE_12_5_HZ = 0x07
    ADXL345_DATARATE_25_HZ   = 0x08
    ADXL345_DATARATE_50_HZ   = 0x09
    ADXL345_DATARATE_100_HZ  = 0x0A # (default)
    ADXL345_DATARATE_200_HZ  = 0x0B
    ADXL345_DATARATE_400_HZ  = 0x0C
    ADXL345_DATARATE_800_HZ  = 0x0D
    ADXL345_DATARATE_1600_HZ = 0x0E
    ADXL345_DATARATE_3200_HZ = 0x0F

    ADXL345_RANGE_2_G        = 0x00 # +/-  2g (default)
    ADXL345_RANGE_4_G        = 0x01 # +/-  4g
    ADXL345_RANGE_8_G        = 0x02 # +/-  8g
    ADXL345_RANGE_16_G       = 0x03 # +/- 16g


    def __init__(self,address=0x53, busnum=-1, debug=False):

        self.accel = Adafruit_I2C(self.address, busnum, debug)

        if self.accel.readU8(self.ADXL345_REG_DEVID) == 0xE5:
	    # Full resolution, +/-16g, 4mg/LSB.
	    self.accel.write8(self.ADXL345_REG_DATA_FORMAT, 0x0B)
            # Enable the accelerometer
            self.accel.write8(self.ADXL345_REG_POWER_CTL, 0x08)

    def readS16(self, register):
        "Reads a signed 16-bit value"
        hi = self.accel.readS8(register+1)
        lo = self.accel.readU8(register)
        return (hi << 8) + lo

    def setRange(self, range):
        # Read the data format register to preserve bits.  Update the data
        # rate, make sure that the FULL-RES bit is enabled for range scaling
        format = ((self.accel.readU8(self.ADXL345_REG_DATA_FORMAT) & ~0x0F) |
          range | 0x08)
        # Write the register back to the IC
        self.accel.write8(self.ADXL345_REG_DATA_FORMAT, format)


    def getRange(self):
        return self.accel.readU8(self.ADXL345_REG_DATA_FORMAT) & 0x03


    def setDataRate(self, dataRate):
        # Note: The LOW_POWER bits are currently ignored,
        # we always keep the device in 'normal' mode
        self.accel.write8(self.ADXL345_REG_BW_RATE, dataRate & 0x0F)


    def getDataRate(self):
        return self.accel.readU8(self.ADXL345_REG_BW_RATE) & 0x0F

    def readRaw(self):
	accel_x = self.readS16(self.ADXL345_RED_DATAX0)
	accel_y = self.readS16(self.ADXL345_RED_DATAY0)
	accel_z = self.readS16(self.ADXL345_RED_DATAZ0)
	return (accel_x, acel_y, accel_z)


    # Read the accelerometer
    def read(self):
        accel_x = (self.readS16(self.ADXL345_REG_DATAX0) + self.ADXL345_OFFSETX) * self.ACCELEROMETER_GAIN
        accel_y = (self.readS16(self.ADXL345_REG_DATAY0) + self.ADXL345_OFFSETY) * self.ACCELEROMETER_GAIN
        accel_z = (self.readS16(self.ADXL345_REG_DATAZ0) + self.ADXL345_OFFSETZ) * self.ACCELEROMETER_GAIN
        return (accel_x, accel_y, accel_z)


    def getEulerAngles(self, fax, fay, faz):
        #---------------------------------------------------------------------------
        # What's the angle in the x and y plane from horizontal in radians?
        # Note fax, fay, fax are all the calibrated outputs reading 0, 0, 0 on
        # horizontal ground as a measure of speed in a given direction.  For Euler we
        # need to re-add gravity of 1g so the sensors read 0, 0, 1 for a horizontal setting
        #---------------------------------------------------------------------------
        pitch = -math.atan2(fax, faz)
        roll = math.atan2(fay,  faz)
        tilt = math.atan2(faz, math.pow(math.pow(fax, 2) + math.pow(fay, 2), 0.5))

        return pitch, roll, tilt
