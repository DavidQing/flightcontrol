from ThinkBowl_ITG3205 import ITG3205
from time import *

gyro = ITG3205(0x68)

while True:
	(itgready, dataready) = gyro.getInterruptStatus()	
	if dataready:
		#temp = gyro.getDieTemperature()
		(x, y, z) = gyro.getRadPerSecAxes() 
		#print("Temp: "+str(temp))
		print("X:    "+str(x))
		print("Y:    "+str(y))
		print("Z:    "+str(z))
		print("")

	sleep(1)
