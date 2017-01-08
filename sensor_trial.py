import brickpi
import time
import movementlib
from config import *

interface=brickpi.Interface()
interface.initialize()

movementlib.setupRobot(interface, 1)
movementlib.setupSensors(interface)

while True:
	l = interface.getSensorValue(TOUCH_SENSOR[0])
	r = interface.getSensorValue(TOUCH_SENSOR[1])
	if l :
		print l
	else:
		print "Failed left"
	if r:
		print r
	else:
		print "failed right"
	time.sleep(0.5)

interface.terminate()
