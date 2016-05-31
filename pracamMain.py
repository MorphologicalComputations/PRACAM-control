import logging
from robot import Pracam
from webcam import Cam
import time
import cv2
import sys
import numpy as np

logger = logging.getLogger('PRACAM')
logger.setLevel(logging.DEBUG)

fileHandler = logging.FileHandler('logoutput.log')
fileHandler.setLevel(logging.DEBUG)

consoleHandler = logging.StreamHandler()
consoleHandler.setLevel(logging.INFO)

formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
fileHandler.setFormatter(formatter)
consoleHandler.setFormatter(formatter)

logger.addHandler(fileHandler)
logger.addHandler(consoleHandler)


cameraAvailable = True
trackingTime = 10
speed = 100
minimumTimePerStep = 1000000 / speed * 21 / 20
Offset1 = 0
Offset2 = 0
Offset3 = 0
Offset4 = 0
pracam = Pracam(logger)
if cameraAvailable:
	cam = Cam(logger, trackingTime = trackingTime)
pracam.setSpeed(speed)
pracam.setMinimumTimePerStep(minimumTimePerStep)
pracam.setOffset(Offset1, Offset2, Offset3, Offset4)
#print pracam.getVar('Offset1')
#print pracam.getVar('Offset2')
#print pracam.getVar('Offset3')
#print pracam.getVar('Offset4')
#print pracam.getVar('Speed')
#print pracam.getVar('MinTimePerStep')
if True:
	try:
		pracam.power()
		
		#print pracam.getVar('Angle1')
		#print pracam.getVar('Angle2')
		#print pracam.getVar('Angle3')
		#print pracam.getVar('Angle4')

		#time.sleep(20)
		if cameraAvailable:
			cam.open()
		pracam.lauch()
		if cameraAvailable:
			cam.run()
			print cam.totalDistance
		else:
			time.sleep(trackingTime)
		pracam.reset()
		pracam.powerOff(wait = True)
	except:
		pracam.powerOff()
		e = sys.exc_info()
		logger.error(str(e[0]) + " " + e[1] + " " + str(e[2]))
		#logger.error(e[1])
