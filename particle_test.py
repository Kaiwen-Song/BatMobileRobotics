import brickpi
from config import *
from movementlib import *

interface = brickpi.Interface()
interface.initialize()

setupRobot(interface, x_pos =84, y_pos = 30, theta = 0)
setupSensors(interface)
# navigateToWayPoint(interface, 180,30)
# navigateToWayPoint(interface, 180,54)
# navigateToWayPoint(interface, 138,54)
# navigateToWayPoint(interface, 138,168)
# navigateToWayPoint(interface, 114,168)
# navigateToWayPoint(interface, 114,84)
# navigateToWayPoint(interface, 84,84)
# navigateToWayPoint(interface, 84,30)

rotateSensor(370, interface)
rotateSensor(-370, interface)


# moveForwardcm(10, interface)
# moveForwardcm(10, interface)
# moveForwardcm(10, interface)
# moveForwardcm(10, interface)
# rotateRightDegrees(90, interface)
# print getPosition()
# moveForwardcm(10, interface)
# moveForwardcm(10, interface)
# moveForwardcm(10, interface)
# moveForwardcm(10, interface)
# rotateRightDegrees(90, interface)
# print getPosition()
# moveForwardcm(10, interface)
# moveForwardcm(10, interface)
# moveForwardcm(10, interface)
# moveForwardcm(10, interface)
# rotateRightDegrees(90, interface)
# print getPosition()
# moveForwardcm(10, interface)
# moveForwardcm(10, interface)
# moveForwardcm(10, interface)
# moveForwardcm(10, interface)
# rotateRightDegrees(90, interface)
# print getPosition()
#
# setupRobot(interface, x_pos =0, y_pos = 0, theta = 0)

# while True:
#   x,y = input("enter x, y : ")
#   navigateToWayPoint(interface, x, y)
# #
