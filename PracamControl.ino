//#include <Stepper.h>
//#include <Wire.h>
#include <LiquidCrystal.h>
#include <Dwenguino.h>
#include <ctype.h>

// gebruik maken van initial angle

#pragma region Globals
class BluetoothAdapter;
class Motor;
class Robot;

#define Enable 8

#define pinDir0 5
#define pinStep0 2

#define pinDir1 6
#define pinStep1 3

#define pinDir2 7
#define pinStep2 4

#define pinDir3 13
#define pinStep3 12

#define ledBlt 32
#define ledEnable 33
#define ledRunning 34
#define ledReset 35

#define noMotors 4
# define statePowerOff 0
# define statePowerOn 1
# define stateRunning 2
# define stateResetting 3

int speed = 100; 
long minTimePerStep = 8000; 
int motorOffset1 = 0;
int motorOffset2 = 0;
int motorOffset3 = 0;
int motorOffset4 = 0;

long timePerStep() {
	return 1000000 / speed;
}// number of microsseconds per step

int state = statePowerOff;
long bluetoothInterval = 100000;

unsigned long launchTime = 0;

#pragma endregion

#pragma region Simple
unsigned long microsLaunch() {
	return micros() - launchTime;
}

void print(String message); // DECLARED AFTER BLUETOOTH ADAPTER

void print(String message, int millisdelay) {
	print(message);
	delay(millisdelay);
}

void print(long num, int millisdelay)
{
	print(String(num), millisdelay);
}

void print(int num, int millisdelay)
{
	print(String(num), millisdelay);
}

long toLong(String value) {
	const int strLen = value.length() + 1;
	char charArray[strLen];
	value.toCharArray(charArray, strLen);
	return atol(charArray);
}

bool highBT = false;
void ledBTToggle() {
	if (highBT)
		digitalWrite(ledBlt, LOW);
	else digitalWrite(ledBlt, HIGH);
	highBT = !highBT;
}

#pragma endregion

#pragma region WaveForm
long desirAngle(int motorNum) {
	int motorOffSet = 0;
	switch(motorNum){
		case 0: motorOffSet = motorOffset1; break;
		case 1: motorOffSet = motorOffset2; break;
		case 2: motorOffSet = motorOffset3; break;
		case 3: motorOffSet = motorOffset4; break;}
		
	return microsLaunch() / timePerStep() + motorOffSet;
}

#pragma endregion

#pragma region Bluetooth
class BluetoothAdapter {
	// different Types
	const static char
		// MessageLess 
		// PC 2 Arduino
		charStatereq = 'A',
		charPowerOn = 'B',
		charLaunchreq = 'C',
		charStopreq = 'D',
		charResetreq = 'E',
		charPowerOff = 'F',

		// Arduino 2 PC
		charPowerOnok = 'J',
		charLaunchok = 'K',
		charStopok = 'L',
		charResetok = 'M',
		charPowerOffok = 'N',

		// Universal
		charOk = 'ok',

		// Message
		charSet = 's',
		charAsk = 'a',
		charReply = 'r',
		charStateresp = 't',
		charDisplay = 'd',
		charErrormess = 'e';

	// other characters
	static const char
		endOfMessage = '#',
		nothingReceived = '?';

public:
	// different variables
	const static char
		firstVar = 'a',
		// char 'a' to 'd' are reserved for motor angles
		varSpeed = 'e',
		varMinTimePerStep = 'f',
		varOffset1 = 'g',
		varOffset2 = 'h',
		varOffset3 = 'i',
		varOffset4 = 'j',
		lastVar = 'j';

	const static int arrayVarSize = lastVar - firstVar + 1;

	void*  pointerArray[arrayVarSize];
	char  typeArray[arrayVarSize];

	BluetoothAdapter();
	int varAddress(char);
	char varAddress(int);
	void addVarToArray(char, int*);
	void addVarToArray(char, long*);
	void init() { Serial1.begin(9600); }; // TEMP 115200

	int read();
	String retrieveMessage();
	char retrieveChar(bool);
	void send(String mess) { Serial1.print(mess); };
	void send(char type) { send(String(type)); };
	void sendStarted() { send(charLaunchok); };
	void sendStopped() { send(charStopok); };
	void sendReset() { send(charResetok); };
	void sendPowerOn() { send(charPowerOnok); };
	void sendPowerOff() { send(charPowerOffok); };
	void error(String);
	void setVar(char, String);
	String sendVar(char);
};

BluetoothAdapter::BluetoothAdapter() { 
	addVarToArray(varSpeed, &speed);
	addVarToArray(varMinTimePerStep, &minTimePerStep);
	addVarToArray(varOffset1, &motorOffset1);
	addVarToArray(varOffset2, &motorOffset2);
	addVarToArray(varOffset3, &motorOffset3);
	addVarToArray(varOffset4, &motorOffset4);
	}

int BluetoothAdapter::varAddress(char name) {
	if (name >= firstVar && name <= lastVar) {
		return name - firstVar;
	}
	else {
		error("invallid varname: " + name);
		return -1;
	}
}

char BluetoothAdapter::varAddress(int num) {
	if (num >= 0 && num <= 26) {
		return num + firstVar;
	}
	else {
		error("invallid varnum: " + num);
		return ' ';
	}
}

void BluetoothAdapter::addVarToArray(char ref, int* pointer) {
	int address = varAddress(ref);
	pointerArray[address] = pointer;
	typeArray[address] = 'i';
}

void BluetoothAdapter::addVarToArray(char ref, long* pointer) {
	int address = varAddress(ref);
	pointerArray[address] = pointer;
	typeArray[address] = 'l';
}

int BluetoothAdapter::read() {
	char type = retrieveChar(false);
	if (type == nothingReceived) { return -1; }
	else if (isupper(type)) {
		switch (type) {

		case charStatereq:
			send(charStateresp + String(state));
			break;

		case charPowerOn:
			return -5;

		case charLaunchreq:
			return -2;

		case charStopreq:
			return -3;

		case charResetreq:
			return -4;

		case charPowerOff:
			return -6;

		default:
			error(" an invalid bluetooth command has been received:" + type);
		}
	}
	else {
		switch (type) {
		case charSet:
		{char ref = retrieveChar(true);
		String message = retrieveMessage();
		setVar(ref, message);
		break; }

		case charAsk:
		{char ref = retrieveChar(true);
		sendVar(ref);
		break; }

		case charReply:
			error(" reply not yet implemented");
			break;

		case charStateresp:
			error(" status request not yet implemented");
			break;

		case charDisplay: {
			String message = retrieveMessage();
			print(message);
			send(message);
			break; }

		}
	}
	return 1;
}

String BluetoothAdapter::retrieveMessage() {
	char newChar = ' ';
	String data = "";
	while (newChar != endOfMessage) {
		newChar = retrieveChar(true);

		if (newChar != endOfMessage)
			data = data + newChar;
	}
	return data;
}

char BluetoothAdapter::retrieveChar(bool compulsory) {
	delayMicroseconds(10); // NOTE delay introduced
	if (Serial1.available() > 0)
		return Serial1.read();

	else if (compulsory)
		error("no bits can be read from the serial connection");
	return nothingReceived;
}

void BluetoothAdapter::error(String errorMessage) {
	send(charErrormess + errorMessage + endOfMessage);
	print("error!");
}

void BluetoothAdapter::setVar(char ref, String value) {
	int address = varAddress(ref);
	void* pointer = pointerArray[address];
	char type = typeArray[address];
	if (type == 'i') {
		*(int*)pointer = value.toInt();
	}
	if (type == 'l') {
		*(long*)pointer = toLong(value);
	}
}

String BluetoothAdapter::sendVar(char ref)
{
	String message = "";
	int address = varAddress(ref);
	void* pointer = pointerArray[address];
	char type = typeArray[address];
	if (type == 'i') {
		message = charReply + String(*(int*)pointer)+ endOfMessage;
	}
	if (type == 'l') {
		message = charReply + String(*(long*)pointer) + endOfMessage;
	}

	send(message);
	return message;

}

BluetoothAdapter blt = BluetoothAdapter();

void print(String message) {
	dwenguinoLCD.clear();
	dwenguinoLCD.print(message);
	//blt.send(message);
}

#pragma endregion

#pragma region Motor
class Motor {
	const static int resolution = 200 * (5 + 2.0 / 11); // Number of Steps per full revolution

public:
	uint8_t pinDir;
	uint8_t pinStep;
	int currentDirection;
	int motorNum;
	long angle;

	Motor() {};
	Motor(int, int, int);

	void potentialStep();
	void step(int);
	void reset();
};

Motor::Motor(int motorNum, int pinDir, int pinStep) {
	this->motorNum = motorNum;
	this->pinDir = pinDir;
	this->pinStep = pinStep;

	angle = 0;
	currentDirection = 1;
	digitalWrite(pinDir, LOW);
}

void Motor::step(int newDirection) {
	// perform rotation
	digitalWrite(pinStep, HIGH);
	delayMicroseconds(5);
	digitalWrite(pinStep, LOW);
	if (newDirection == 1) {
		if (currentDirection = -1)
			digitalWrite(pinDir, LOW);
		angle++;
	}

	else if (newDirection == -1) {
		if (currentDirection = -1)
			digitalWrite(pinDir, HIGH);
		angle--;
	}
}

void Motor::reset() {
	// set motor back to zero
	while (angle % resolution != 0) {
		step(1);
		delayMicroseconds(minTimePerStep);
	}
	angle = 0;
}

void Motor::potentialStep() {
	long desiredAngle = desirAngle(motorNum);
	if (desiredAngle > angle + 1) {
		step(1);
	}
	else if (desiredAngle < angle - 1) {
		step(-1);
	}
};

#pragma endregion

#pragma region Robot
class Robot {
	unsigned long nextEvent;
	unsigned long nextBluetooth;
	unsigned long nextMotor;
	int nextTask;
	
public:
	Motor cluster[noMotors];
	Robot() {};
	Robot(bool);
	void performEvent();
	void scheduleNextEvent();
	void launch();
	void readBluetooth();
	void reset();
	void stop(boolean);
	void powerOn();
	void powerOff();
	void initialAngle(int [], boolean );
	void calibrate();
};

Robot::Robot(bool a) {
	// set Pins
	pinMode(ledBlt, OUTPUT);
	pinMode(ledEnable, OUTPUT);
	pinMode(Enable, OUTPUT);
	pinMode(pinDir0, OUTPUT);
	pinMode(pinDir1, OUTPUT);
	pinMode(pinDir2, OUTPUT);
	pinMode(pinDir3, OUTPUT);
	pinMode(pinStep0, OUTPUT);
	pinMode(pinStep1, OUTPUT);
	pinMode(pinStep2, OUTPUT);
	pinMode(pinStep3, OUTPUT);

	// configure bluetooth
	blt = BluetoothAdapter();
	blt.init();

	// set motors
	cluster[0] = Motor(0, pinDir0, pinStep0);
	cluster[1] = Motor(1, pinDir1, pinStep1);
	cluster[2] = Motor(2, pinDir2, pinStep2);
	cluster[3] = Motor(3, pinDir3, pinStep3);

	// initialize state waiting
	print("power off");
	state = statePowerOff;
	nextBluetooth = micros() + bluetoothInterval;
	digitalWrite(Enable, HIGH);
	digitalWrite(ledRunning, LOW);
	digitalWrite(ledEnable, LOW);
}

void Robot::scheduleNextEvent() {
	// determine Time of next event by enumerating all scheduled events;
	// std Bluetooth time;
	nextEvent = nextBluetooth;
	nextTask = -1;

	// check motors if in launch state
	if (state == stateRunning && nextEvent > nextMotor){
		nextEvent = nextMotor;
		nextTask = 0;}
}

void Robot::initialAngle(int desiredAngle[], boolean calibration) {
	if (calibration)
		calibrate();
	boolean reached = false;
	while (!reached) {
		delayMicroseconds(minTimePerStep);
		for (int i = 0; i < noMotors; i++) {
			reached = true; // reset as soon as one motor is not finished
			if (cluster[i].angle <= desiredAngle[i]){
				cluster[i].step(1);
				reached = false;}
			}
		}
}

void Robot::performEvent() {
	if (micros() < nextEvent) {
		unsigned long timeDiff = nextEvent - micros();
		if (timeDiff > 10000) { // limited value of the timer (16 bit)
			delay(timeDiff / 1000);
		}
		else delayMicroseconds(timeDiff);
	}
	// update motor settings
	if (nextTask == 0) {
		for (int i = 0; i < noMotors; i++)
		cluster[i].potentialStep();
		nextMotor = micros() + minTimePerStep;}

	else if (nextTask == -1) {
		readBluetooth();
		nextBluetooth = micros() + bluetoothInterval;}
}

void Robot::stop(boolean sendBlt) {
	state = statePowerOn;
	print("waiting");
	if (sendBlt)
		blt.sendStopped();
	digitalWrite(ledRunning, LOW);
}

void Robot::launch() {
	// schedule motorEvents
	launchTime = micros();
	nextMotor = micros();
	state = stateRunning;
	blt.sendStarted();
	digitalWrite(ledRunning, HIGH);
	print("running");
}

void Robot::reset() {
	print("resetting");
	state = stateResetting;
	digitalWrite(ledReset, HIGH);
	for (int i = 0; i < noMotors; i++) {
		cluster[i].reset();
	}
	digitalWrite(ledReset, LOW);
	blt.sendReset();
	stop(false);
}

void Robot::powerOn() {
	digitalWrite(Enable, LOW);
	print("waiting");
	state = statePowerOn;
	digitalWrite(ledEnable, HIGH);
	int offset[noMotors] = { motorOffset1, motorOffset2, motorOffset3, motorOffset4 };
	initialAngle(offset, true);
	blt.sendPowerOn();
}

void Robot::calibrate() {
	for (int i = 0; i < noMotors; i++) {
		cluster[i].angle = 0;
	}
}

void Robot::powerOff() {
	print("power off");
	digitalWrite(Enable, HIGH);
	state = statePowerOff;
	blt.sendPowerOff();
	digitalWrite(ledEnable, LOW);
	digitalWrite(ledRunning, LOW);
}

void Robot::readBluetooth() {
	ledBTToggle();
	int output;
	do { output = blt.read(); } while (output > 0);

	if (output == -2) { // launch
		if (state == statePowerOn) {
			launch();
		}
		else if (state == statePowerOff) blt.error("power is switched off");
		else if (state == stateRunning) blt.error("robot is already running");
		else if (state == stateResetting) blt.error("robot is resetting");
	}

	else if (output == -3) { // stop
		if (state == stateRunning) {
			stop(true);
		}
		else blt.error("robot was not running");
	}

	else if (output == -4) { //reset
		if (state == stateRunning || state == statePowerOn) {
			reset();
		}
		else blt.error("robot was not running");
	}

	else if (output == -5) { //power on
		if (state == statePowerOff) {
			powerOn();
		}
		else blt.error("power was already switched on");
	}

	else if (output == -6) { //power off
		powerOff();
	}
}

#pragma endregion

#pragma region MAIN

Robot pracam;

void setup() {
	pracam = Robot(true);
	for (int i = 0; i < 1; i++) {
		long Motor::*pAngle = &Motor::angle;
		blt.addVarToArray(blt.varAddress(i), &(pracam.cluster[i].*pAngle));
	}
}

void loop() {
	pracam.scheduleNextEvent();
	pracam.performEvent();
}
#pragma endregion


