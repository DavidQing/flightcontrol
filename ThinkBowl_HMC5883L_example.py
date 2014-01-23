from ThinkBowl_HMC5883L import HMC5883L

comp = HMC5883L(0x1e)

hmc5883l.setContinuousMode()
hmc5883l.setDeclination(9,54)

# To get degrees and minutes into variables
(degrees, minutes) = hmc5883l.getDeclination()
(degress, minutes) = hmc5883l.getHeading()

# To get string of degrees and minutes
declination = hmc5883l.getDeclinationString()
heading = hmc5883l.getHeadingString()

# To scaled axes
(x, y, z) = hmc5883l.getAxes()
