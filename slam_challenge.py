import brickpi
from movementlib import *
from config import *
from challenge_lib import *

interface = brickpi.Interface()
interface.initialize()
setupRobot(interface)
setupSensors(interface)


#moveForwardcm(30.0,interface)
#rotateLeftDegrees(360,interface)
#
#rotateRightDegrees(90, interface)
# a = sonar360Reading(interface)
# rotateSensor(0.6,interface)
# # learn_location(interface)
(x,y),theta = recognize_location(interface)
print x,y,theta

interface.X_POS = x
interface.Y_POS = y
interface.THETA = math.radians(theta)

initialiseParticles(interface.X_POS, interface.Y_POS, interface.THETA)

moveGivenPoint(interface,(x,y))
