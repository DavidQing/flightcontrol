import time
from Adafruit_ADXL345 import ADXL345

accel = ADXL345(0x53)

print '[Accelerometer X, Y, Z]'
print accel.read()
while True:
  print accel.read()
  time.sleep(1) # Output is fun to watch if this is commented out
