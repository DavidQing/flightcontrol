from ThinkBowl_HMC5883L import HMC5883L

compass = HMC5883L(0x1e)

compass.setContinuousMode()
compass.setDeclination(9,54)

# To get degrees and minutes into variables
(degrees, minutes) = compass.getDeclination()
(degress, minutes) = compass.getHeading()

# To get string of degrees and minutes
declination = compass.getDeclinationString()
heading = compass.getHeadingString()

# To scaled axes
(x, y, z) = compass.getAxes()

print("X:    "+str(x))
print("Y:    "+str(y))
print("Z:    "+str(z))
print("")
