import serial, time, numpy as np
class Pracam:
	# PC - Arduino
	charStatereq = 'A'
	charPowerOn = 'B'
	charLaunchreq = 'C'
	charStopreq = 'D'
	charReseteq = 'E'
	charPowerOff = 'F'

	# Arduino - PC
	charPowerOnok = 'J'
	charLaunchok = 'K'
	charStopok = 'L'
	charResetok = 'M'
	charPowerOffok = 'N'

	# Universal
	charSet = 's'
	charAsk = 'a'
	charReply = 'r'
	charStateresp = 't'
	charDisplay = 'd'
	charErrorMess = 'e'

	#variables
	varAngle1 = 'a'
	varAngle2 = 'b'
	varAngle3 = 'c'
	varAngle4 = 'd'
	varSpeed = 'e'
	varMinTimePerStep = 'f'
	varOffset1 = 'g'
	varOffset2 = 'h'
	varOffset3 = 'i'
	varOffset4 = 'j'


	def __init__(self, logger, timeout = 0.3):
		self.logger = logger
		self.baudrate = 9600 # TEMP 115200
		self.bluetooth = serial.Serial('COM6', self.baudrate, timeout= timeout)
		time.sleep(1) #give the connection a second to settle
		self.logger.info("bluetooth connection has started")

	def power(self):
		self.bluetooth.write(self.charPowerOn)
		self.logger.info("order to switch power on")
		self.processUntilChar(self.charPowerOnok, wait = True)
		self.logger.info("robot power has been switched on")

	def lauch(self):
		self.bluetooth.write(self.charLaunchreq)
		self.logger.info("order to launch robot")
		self.processUntilChar(self.charLaunchok)
		self.logger.info("robot has been launched")

	def stop(self):
		self.bluetooth.write(self.charStopreq)
		self.logger.info("order to stop robot")
		self.processUntilChar(self.charStopok)
		self.logger.info("robot has been stopped")
		
	def reset(self):
		self.bluetooth.write(self.charReseteq)
		self.logger.info("order to reset robot")
		self.processUntilChar(self.charResetok, wait = True)
		self.logger.info("robot has been reset")

	def setMinimumTimePerStep(self, time):
		message = self.charSet
		message += self.varMinTimePerStep
		message += str(int(time))
		message += '#'
		self.bluetooth.write(message)
		self.logger.debug("robot minimum time per step has been set" + message)
	
	def setSpeed(self, speed, check = True):
		message = self.charSet
		message += self.varSpeed
		message += str(int(speed))
		message += '#'
		if check:
			self.bluetooth.write(message)
			self.bluetooth.write(self.charAsk + self.varSpeed)
			self.processUntilChar(self.charReply)
			anwser = self.getMessage()
			assert anwser == str(int(speed))
		self.logger.debug("robot speed has been set")

	def setOffset(self, offset1, offset2, offset3, offset4):
		message = self.charSet + self.varOffset1
		message += str(radiansToSteps(offset1))
		message += '#'
		self.bluetooth.write(message)
		
		message = self.charSet + self.varOffset2
		message += str(radiansToSteps(offset2))
		message += '#'
		self.bluetooth.write(message)

		message = self.charSet + self.varOffset3
		message += str(radiansToSteps(offset3))
		message += '#'
		self.bluetooth.write(message)

		message = self.charSet + self.varOffset4
		message += str(radiansToSteps(offset4))
		message += '#'
		self.bluetooth.write(message)
		self.logger.debug("robot offset has been set")

	def powerOff(self, wait = True):
		self.bluetooth.write(self.charPowerOff)
		self.logger.info("order to switch power off")
		self.processUntilChar(self.charPowerOffok, wait)
		self.logger.info("power has been switched off")

	def getState(self):
		self.bluetooth.send(self.statereq)
		self.logger.debug("status asked")
		self.processUntilChar(self.charStateresp)
		state = self.bluetooth.read(1)
		self.logger.debug("status received")
		return int(state)

	def processUntilChar(self, char, wait = False):
		received = ''
		while received is not char:
			received = self.bluetooth.read(1)
			if not (received is char or (wait and received is "")):
				self.logger.info("other character received " + received + " instead of " + char)
				self.processMessage(received)
				time.sleep(0.1)
			elif received is '':
				time.sleep(0.5)
			

	def getVar(self, name):
		if name == 'Angle1':
			askcode = self.varAngle1
		elif name == 'Angle2':
			askcode = self.varAngle2
		elif name == 'Angle3':
			askcode = self.varAngle3
		elif name == 'Angle4':
			askcode = self.varAngle4
		elif name == 'Speed':
			askcode = self.varSpeed
		elif name == 'MinTimePerStep':
			askcode = self.varMinTimePerStep
		elif name == 'Offset1':
			askcode = self.varOffset1
		elif name == 'Offset2':
			askcode = self.varOffset2
		elif name == 'Offset3':
			askcode = self.varOffset3
		elif name == 'Offset4':
			askcode = self.varOffset4
		else:
			raise NotImplementedError, "the requested option " + name + " has not been implemented " 
		self.bluetooth.write(self.charAsk + askcode)
		self.logger.debug("variable value has been requested :" + name)
		self.processUntilChar(self.charReply)
		value = self.getMessage()
		self.logger.debug("variable value has been received :" + name)
		return float(value)


	def processMessage(self, received = 'default'):
		if received is 'default':
			received = self.bluetooth.read(1)
		if received is "":
			self.logger.critical("no message was read, communication with pracam has broken down")
		if received is self.charDisplay:
			message = self.getMessage()
			self.logger.debug("display command" + message)
		elif received is self.charErrorMess:
			message = self.getMessage()
			self.logger.error("error message" + message)
		elif received is self.charStateresp or self.charReply:
			self.logger.critical("response given with no question asked: " + received)
		else: 
			self.logger.critical("invalid response received" + received)

	def getMessage(self):
		received = ""
		lastChar = ""
		counter = 0
		while lastChar is not "#" and counter is not 100:
			received = received + lastChar
			lastChar = self.bluetooth.read(1)
			counter += 1
		if counter == 100:
			self.logger.error("maximum message length exceeded")
		return received

def radiansToSteps(rad):
	fullRotation = 200 * (5 + 2.0 / 11)
	return int(rad / (2 * np.pi) * fullRotation )

