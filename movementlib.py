import brickpi
import time
from particles_ver2 import *
from config import *
import math
# Method that waits for the motors to reach the angles before continuing
def __idleUntilReached(interface):
    while not interface.motorAngleReferencesReached(WHEEL_MOTORS):
        dummy = 0
def __checkBumperUntilReached(interface):
    while not interface.motorAngleReferencesReached(WHEEL_MOTORS):
        t_sensor_left = interface.getSensorValue(TOUCH_SENSOR[0])
        t_sensor_right = interface.getSensorValue(TOUCH_SENSOR[1])
        print(t_sensor_left, t_sensor_right)
        current_target_references = interface.getMotorAngleReferences(WHEEL_MOTORS)
        if t_sensor_left or t_sensor_right:
            stopMovement(interface)
            #backs out
            moveForwardcm(-10, interface)
            if t_sensor_left and t_sensor_right:
                rotateRightDegrees(180,interface)
                #back out and change direction
            elif t_sensor_left:
                rotateRightDegrees(90, interface)
                moveForwardcm(10, interface)
                rotateLeftDegrees(90, interface)
                interface.setMotorAngleReferences(current_target_references)
                #back out, rotate right, move forward by a car body and rotate left and try again
            else:
                rotateLeftDegrees(90, interface)
                moveForwardcm(10, interface)
                rotateRightDegrees(90, interface)
                interface.setMotorAngleReferences(current_target_references)
                #back out, rotate left, move forward by a lenght and rotate right and try move forward
    time.sleep(0.5)

def __checkSensorUntilReached(interface):
    while not interface.motorAngleReferencesReached(WHEEL_MOTORS):
        sonar = interface.getSensorValue(SONAR_SENSOR[0])
        time.sleep(0.5)


# Best values found for 40cm and 90 degrees rotation clockwise

DRIVE_MODES = [__idleUntilReached, __checkBumperUntilReached, __checkSensorUntilReached]

def setupRobot(interface, DRIVE_MODE = 0, x_pos =0, y_pos = 0, theta=0):
    # Setup Brick Pi
    # Enable the motors and fetch the MotorParams holders
    interface.motorEnable(WHEEL_MOTORS[0])
    interface.motorEnable(WHEEL_MOTORS[1])
    motorParams = []
    motorParams.append(interface.MotorAngleControllerParameters())
    motorParams.append(interface.MotorAngleControllerParameters())

    # Setup the params for the motors that are equals
    motorParams[0].maxRotationAcceleration = motorParams[1].maxRotationAcceleration = 10.0
    motorParams[0].maxRotationSpeed = motorParams[1].maxRotationSpeed = 10.0
    motorParams[0].pidParameters.minOutput = motorParams[1].pidParameters.minOutput = -255
    motorParams[0].pidParameters.maxOutput = motorParams[1].pidParameters.maxOutput = 255

    # Setup the params for the motors that differ using our optimal values
    motorParams[0].pidParameters.k_p = KP0
    motorParams[0].pidParameters.k_i = KI0
    motorParams[0].pidParameters.k_d = KD0
    motorParams[0].feedForwardGain = FFG0
    motorParams[0].minPWM = PWM0

    motorParams[1].pidParameters.k_p = KP1
    motorParams[1].pidParameters.k_i = KI1
    motorParams[1].pidParameters.k_d = KD1
    motorParams[1].feedForwardGain = FFG1
    motorParams[1].minPWM = PWM1

    interface.setMotorAngleControllerParameters(WHEEL_MOTORS[0], motorParams[0])
    interface.setMotorAngleControllerParameters(WHEEL_MOTORS[1], motorParams[1])
    #interface.setMotorAngleReferences(motors, [0,0])
    interface.DRIVE_MODE = DRIVE_MODE
    #return (motors, interface)
    interface.X_POS = x_pos
    interface.Y_POS = y_pos
    interface.THETA = theta

    #canvas.drawParticles()

def setupSensors(interface):
    interface.motorEnable(SENSOR_MOTOR[0])
    motorParam = interface.MotorAngleControllerParameters()
    motorParam.maxRotationAcceleration = 2.0
    motorParam.maxRotationSpeed = 2
    motorParam.pidParameters.minOutput = -255
    motorParam.pidParameters.maxOutput = 255
    motorParam.pidParameters.k_p = KP_SENSOR_MOTOR
    motorParam.pidParameters.k_i = KI_SENSOR_MOTOR
    motorParam.pidParameters.k_d = KD_SENSOR_MOTOR
    motorParam.feedForwardGain = FFG_SENSOR_MOTOR
    motorParam.minPWM = PWM_SENSOR_MOTOR

    interface.setMotorAngleControllerParameters(SENSOR_MOTOR[0], motorParam)

    #interface.sensorEnable(TOUCH_SENSOR[0],brickpi.SensorType.SENSOR_TOUCH)
    #interface.sensorEnable(TOUCH_SENSOR[1], brickpi.SensorType.SENSOR_TOUCH)
    interface.sensorEnable(SONAR_SENSOR[0], brickpi.SensorType.SENSOR_ULTRASONIC)
    #interface.setMotorAngleReference(SENSOR_MOTOR[0], 0)

def __moveForward(interface):
    # Get CM to move foward
    distanceToMove = float(input("Please enter a distance in cm to move foward in\n"))
    moveForwardcm(distanceToMove, interface)
    return

# Move the robot n cm forward
def moveForwardcm(distance, interface):
    # radianValue = distance * (FORWARD_40_CM_VALUE/40)
    # __moveForwardExplicitly(radianValue, interface)
    # updateAllParticlesStraight(distance)
    # canvas.drawParticles(ALL_PARTICLES)
    print "moving forward"
    radianValue =  distance * (FORWARD_40_CM_VALUE/40)
    __moveForwardExplicitly(radianValue, interface)
    updateAllParticlesStraight(distance)
    #obtain sonar reading
    sonar_reading = getSonarMeasurement(interface)
    updateWeights(sonar_reading)
    resampleParticles()
    #canvas.drawParticles(ALL_PARTICLES)


def moveBackwardcm(distance, interface):
    # radianValue = - distance * (FORWARD_40_CM_VALUE/40)
    # __moveForwardExplicitly(radianValue, interface)
    # updateAllParticlesStraight(-distance)
    # canvas.drawParticles(ALL_PARTICLES
    radianValue = -distance * (FORWARD_40_CM_VALUE/40)
    __moveForwardExplicitly(radianValue, interface)
    updateAllParticlesStraight(distance)
    #obtain sonar reading
    sonar_reading = getSonarMeasurement(interface)
    updateWeights(sonar_reading)
    resampleParticles()
    #canvas.drawParticles(ALL_PARTICLES)

# Move the robot forward using the radian vlaues explicity
def __moveForwardExplicitly(radian,  interface):
    interface.increaseMotorAngleReferences(WHEEL_MOTORS, [radian, radian])
    DRIVE_MODES[interface.DRIVE_MODE](interface)

def __rotateClockwise(interface):
    # Get degress to rotate clockwise
    degreesToMove = float(input("Please enter a degrees to turn clockwise in\n"))
    rotateRightDegrees(degreesToMove,interface)
    #canvas.drawParticles()

def rotateSensor(radians, interface):
    interface.increaseMotorAngleReferences(SENSOR_MOTOR,[radians])
    while not interface.motorAngleReferencesReached(SENSOR_MOTOR):
        continue
    # interface.setMotorRotationSpeedReferences(SENSOR_MOTOR, [-0.5])

# Angle in degrees you want the robots to rotate clockwise
def rotateRightDegrees(angle, interface):
    radianValue = -angle * (ROTATE_90_DEGREES_VALUE/90)
    __rotateExplicitly(radianValue,interface)
    updateAllParticlesRotate(math.radians(angle))
    sonar_reading = getSonarMeasurement(interface)
    updateWeights(sonar_reading)
    resampleParticles()
    #canvas.drawParticles(ALL_PARTICLES)
def rotateLeftDegrees(angle, interface):
    radianValue = angle * (ROTATE_90_DEGREES_VALUE/90)
    __rotateExplicitly(radianValue, interface)
    updateAllParticlesRotate(math.radians(-angle))
    sonar_reading = getSonarMeasurement(interface)
    updateWeights(sonar_reading)
    resampleParticles()
    #canvas.drawParticles(ALL_PARTICLES)

# Rotate the robot using radians values explicity
def __rotateExplicitly(radian, interface):
    interface.increaseMotorAngleReferences(WHEEL_MOTORS,[radian,-radian])
    DRIVE_MODES[interface.DRIVE_MODE](interface)
    return

def sonar360Reading(interface):
    interface.increaseMotorAngleReferences(SENSOR_MOTOR,[-SONAR_360_VAL])
    readings = []
    while not interface.motorAngleReferencesReached(SENSOR_MOTOR):
        r = getSonarMeasurement(interface)
        print r
        readings.append(r)
    print "length: ", len(readings)
    return readings

def __drawSquare(interface):
    # Get number of times to draw the suqare
    numberOfSquares = int(input("Please enter the number of squares to draw\n"))
    for i in range(0, numberOfSquares):
        __drawSingleSquare(interface)
    return

def __testMovement(interface):
    # Go forwards 40cm and then backwards 40cm
    __moveForwardExplicitly(FORWARD_40_CM_VALUE, motors, interface)
    time.sleep(0.5)
    __moveForwardExplicitly(-FORWARD_40_CM_VALUE, motors, interface)
    return

def __testRotation(interface):
    # Rotate 90 degrees clockwise then anitclockwise
    __rotateExplicitly(ROTATE_90_DEGREES_VALUE, interface)
    time.sleep(0.5)
    __rotateExplicitly(-ROTATE_90_DEGREES_VALUE, interface)
    return

def __drawSingleSquare(interface):
    # Draws a 40cm square
    __moveForwardExplicitly(FORWARD_40_CM_VALUE, interface)
    __rotateExplicitly(ROTATE_90_DEGREES_VALUE, interface)
    __rotateExplicitly(ROTATE_90_DEGREES_VALUE, interface)
    __moveForwardExplicitly(FORWARD_40_CM_VALUE, interface)
    __rotateExplicitly(ROTATE_90_DEGREES_VALUE, interface)
    __moveForwardExplicitly(FORWARD_40_CM_VALUE, interface)
    __rotateExplicitly(ROTATE_90_DEGREES_VALUE, interface)
    __moveForwardExplicitly(FORWARD_40_CM_VALUE, interface)
    __rotateExplicitly(ROTATE_90_DEGREES_VALUE, interface)
    return

def stopMovement(interface):
    interface.setMotorPwm(WHEEL_MOTORS,[0, 0])

#after the robot stops moving take 10 readings and return the median distance
def getSonarMeasurement(interface):
    values = []
    for i in range(0,5):
        sonar_val = interface.getSensorValue(SONAR_SENSOR[0])
    #   print sonar_val
        if sonar_val:
            (reading,_) = sonar_val
            values.append(reading)
        time.sleep(0.032)
    values.sort()
    return values[len(values)/2] - SONAR_OFFSET
    # return values

def navigateToWayPoint(interface, X, Y):
    [x, y, theta] = getPosition()
    x_diff = X-x
    y_diff = Y-y
    #now adjusts and adapts to new predictions in position as we are trying to reach the destination
    #only stops if it believes it is within an acceptable distance away from the target
    while abs(x_diff)>2.5 or abs(y_diff)>2.5:
        #print "YDIFF" + str(math.sqrt(pow(y_diff,2.) + pow(x_diff,2.)))
        curr_orientation = math.degrees(theta)
        world_angle_of_target = math.degrees(math.atan2(y_diff, x_diff))
        angle_to_rotate = ((world_angle_of_target - curr_orientation) % 360 + 540 ) % 360 - 180
        rotateRightDegrees(angle_to_rotate, interface)
        #moves in a max of 20 cm step now
        distance_to_move = min(math.sqrt(pow(y_diff,2.) + pow(x_diff,2.)),40.0)
        #print "diff" + str(min(math.sqrt(pow(y_diff,2.) + pow(x_diff,2.)),20.))
        moveForwardcm(distance_to_move, interface)
        [x, y, theta] = getPosition()
        print "current position: ", x,y
        x_diff = X - x
        y_diff = Y - y

def moveGivenPoint(interface, point):
    paths = {
        LOCATIONS[1-1] : [LOCATIONS[2-1], LOCATIONS[3-1], LOCATIONS[4-1], LOCATIONS[5-1], LOCATIONS[1-1]],
        LOCATIONS[2-1] : [LOCATIONS[3-1], LOCATIONS[4-1], LOCATIONS[1-1], LOCATIONS[5-1], LOCATIONS[4-1], LOCATIONS[2-1]],
        LOCATIONS[3-1] : [LOCATIONS[2-1], LOCATIONS[1-1], LOCATIONS[4-1], LOCATIONS[5-1], LOCATIONS[4-1], LOCATIONS[3-1]],
        LOCATIONS[4-1] : [LOCATIONS[3-1], LOCATIONS[2-1], LOCATIONS[1-1], LOCATIONS[5-1], LOCATIONS[4-1]],
        LOCATIONS[5-1] : [LOCATIONS[4-1], LOCATIONS[3-1], LOCATIONS[2-1], LOCATIONS[1-1], LOCATIONS[5-1]]
    }
    points_to_navigate = paths[point]
    for (x, y) in points_to_navigate:
        navigateToWayPoint(interface, x, y)
        time.sleep(1)
    #    stopMovement(interface)


# This file will provide the support for performing 3 different actions:
# 1. Move the robot forward a distance of n
# 2. Rotate the robot m degrees in a clockwise fashio
# 3. Draw a square

# The following "map" allows us to choose which function to run in the
# main method called

def moveWithUserInput(interface):
    actions = [["1. Move forward n amounts in cm (Negative cm to move backwards)", __moveForward],
    		["2. Rotate clockwise in place by n degrees (Negative degreees to move anticlockwise)", __rotateClockwise],
    		["3. Draw n number of squares with 40cm long sides", __drawSquare],
    		["4. Move forward with length of 40cm and backwards with length of 40cm", __testMovement],
    		["5. Rotate 90 degrees clockwise and then rotate 90 degrees anticlockwise", __testRotation]]

    # Wait for input of the right method, and call the method to process the
    # request
    while True:
	for action in actions:
		print(action[0])

	print("-----------------------------------------------------------------------------------")
        inputCode = int(input("Press a number between 1 and 5 for the desired action \n"))
        actions[inputCode-1][1](interface)
	print("-----------------------------------------------------------------------------------")
