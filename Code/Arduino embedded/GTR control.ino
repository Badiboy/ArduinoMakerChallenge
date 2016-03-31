//Set board controling GTR
//#define GTR_MEGA
#define GTR_UNO
//If we have HC-SR04, we can connect it to the board to detect barriers
#define ULTRASONIC_INCLUDE

#include <Firmata.h>
#include <Wire.h>
#ifdef ULTRASONIC_INCLUDE
#include <Ultrasonic.h>
#endif

/* Directions enum */
typedef enum
{
	forward,
	back,
	right,
	left
}state;

/* GTR controling types */
typedef enum
{
	automatic,		// Autonomus driving
	gamepad,		// Controlling by gamepad
	phone			// Controlling by phone
}mode;

// Array to swiching control type, containing signal sequence
const int keyArray[6] = { -1, 4, 3, 4, 1, 2 };

/* Timer variables */
unsigned long currentMillis;        // Store the current value from millis()
unsigned long previousMillis;       // For comparison with currentMillis
int samplingInterval = 19;          // Main loop interval (in ms)

unsigned long timer;				// Previous millis() value
state carState;						// For autonomus mode only: current car state
mode carMode;						// Current control type
boolean stopper;					// Flag to indicate a needing for stop function execution
boolean barrierAhead;				// Flag to indicate a barrier

int numberOfKey;					// Iterator at keyArray
int keyValue;						// Current gamepad value

// Defining connectors pins depending on board
#ifdef GTR_MEGA
const int rightSideWatch = 50;		// Right IR Line sensor
const int leftSideWatch = 48;		// Left IR Line sensor

const int rightRotate = 31;			// Pin to turn right
const int leftRotate = 33;			// Pin to turn left
const int gas = 35;					// Pin to move forward
const int stop = 29;				// Pin to move backward

const int buttonForward = 34;		// Value of button "forward" of a gamepad
const int buttonBack = 28;			// Value of button "backward" of a gamepad
const int buttonLeft = 32;			// Value of button "left" of a gamepad
const int buttonRight = 30;			// Value of button "right" of a gamepad
#ifdef ULTRASONIC_INCLUDE
Ultrasonic ultrasonic(52, 53);	// Ultrasonic conection. 52 - Trig, 53 - Echo
#endif
#endif // GTR_MEGA

#ifdef GTR_UNO
const int rightSideWatch = 3;	// Right IR Line sensor
const int leftSideWatch = 2;	// Left IR Line sensor

const int rightRotate = 13;		// Pin to turn right
const int leftRotate = 11;		// Pin to turn left
const int gas = 7;				// Pin to move forward
const int stop = 9;				// Pin to move backward

const int buttonForward = 6;	// Value of button "forward" of gamepad
const int buttonBack = 8;		// Value of button "backward" of gamepad
const int buttonLeft = 10;		// Value of button "left" of gamepad
const int buttonRight = 12;		// Value of button "right" of gamepad
#ifdef ULTRASONIC_INCLUDE
Ultrasonic ultrasonic(5, 4);// Ultrasonic coonection. 5 - Trig, 4 - Echo
#endif
#endif //GTR_UNO

/* Firmata Part start*/
#define MAX_QUERIES 8
#define MINIMUM_SAMPLING_INTERVAL 10
#define REGISTER_NOT_SPECIFIED -1
#define I2C_WRITE B00000000
#define I2C_READ B00001000
#define I2C_READ_CONTINUOUSLY B00010000
#define I2C_STOP_READING B00011000
#define I2C_READ_WRITE_MODE_MASK B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000

/* i2c data */
struct i2c_device_info
{
	byte addr;
	byte reg;
	byte bytes;
};

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written

/* for i2c read continuous more */
i2c_device_info query[MAX_QUERIES];

byte i2cRxData[32];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
unsigned int i2cReadDelayTime = 0;  // default delay time between i2c read request and Wire.requestFrom()


void outputPort(byte portNumber, byte portValue, byte forceSend)
{
	// pins not configured as INPUT are cleared to zeros
	portValue = portValue & portConfigInputs[portNumber];
	// only send if the value is different than previously sent
	if (forceSend || previousPINs[portNumber] != portValue)
	{
		Firmata.sendDigitalPort(portNumber, portValue);
		previousPINs[portNumber] = portValue;
	}
}

void checkDigitalInputs(void)
{
	/* Using non-looping code allows constants to be given to readPort().
	 * The compiler will apply substantial optimizations if the inputs
	 * to readPort() are compile-time constants. */
	if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
	if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
	if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
	if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
	if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
	if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
	if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
	if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
	if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
	if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
	if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
	if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
	if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
	if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
	if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
	if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

void digitalWriteCallback(byte port, int value)
{
	byte pin, lastPin, mask = 1, pinWriteMask = 0;

	if (port < TOTAL_PORTS)
	{
		// create a mask of the pins on this port that are writable.
		lastPin = port * 8 + 8;
		if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
		for (pin = port * 8; pin < lastPin; pin++)
		{
			// do not disturb non-digital pins (eg, Rx & Tx)
			if (IS_PIN_DIGITAL(pin))
			{
				// only write to OUTPUT and INPUT (enables pullup)
				// do not touch pins in PWM, ANALOG, SERVO or other modes
				if (pinConfig[pin] == OUTPUT || pinConfig[pin] == INPUT)
				{
					pinWriteMask |= mask;
					pinState[pin] = ((byte)value & mask) ? 1 : 0;
				}
			}
			mask = mask << 1;
		}
		writePort(port, (byte)value, pinWriteMask);
	}
}

void readAndReportData(byte address, int theRegister, byte numBytes)
{
	// allow I2C requests that don't require a register read
	// for example, some devices using an interrupt pin to signify new data available
	// do not always require the register read so upon interrupt you call Wire.requestFrom()  
	if (theRegister != REGISTER_NOT_SPECIFIED)
	{
		Wire.beginTransmission(address);
#if ARDUINO >= 100
		Wire.write((byte)theRegister);
#else
		Wire.send((byte)theRegister);
#endif
		Wire.endTransmission();
		delayMicroseconds(i2cReadDelayTime);  // delay is necessary for some devices such as WiiNunchuck
	}
	else
	{
		theRegister = 0;  // fill the register with a dummy value
	}

	Wire.requestFrom(address, numBytes);  // all bytes are returned in requestFrom

	// check to be sure correct number of bytes were returned by slave
	if (numBytes == Wire.available())
	{
		i2cRxData[0] = address;
		i2cRxData[1] = theRegister;
		for (int i = 0; i < numBytes; i++)
		{
#if ARDUINO >= 100
			i2cRxData[2 + i] = Wire.read();
#else
			i2cRxData[2 + i] = Wire.receive();
#endif
		}
	}
	else
	{
		if (numBytes > Wire.available())
		{
			Firmata.sendString("I2C Read Error: Too many bytes received");
		}
		else
		{
			Firmata.sendString("I2C Read Error: Too few bytes received");
		}
	}

	// send slave address, register and received bytes
	Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
}

void reportDigitalCallback(byte port, int value)
{
	if (port < TOTAL_PORTS)
	{
		reportPINs[port] = (byte)value;
	}
	// do not disable analog reporting on these 8 pins, to allow some
	// pins used for digital, others analog.  Instead, allow both types
	// of reporting to be enabled, but check if the pin is configured
	// as analog when sampling the analog inputs.  Likewise, while
	// scanning digital pins, portConfigInputs will mask off values from any
	// pins configured as analog
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins()
{
	isI2CEnabled = false;
	// disable read continuous mode for all devices
	queryIndex = -1;
	// uncomment the following if or when the end() method is added to Wire library
	// Wire.end();
}

void setPinModeCallback(byte pin, int mode)
{
	if (pinConfig[pin] == I2C && isI2CEnabled && mode != I2C)
	{
		// disable i2c so pins can be used for other functions
		// the following if statements should reconfigure the pins properly
		disableI2CPins();
	}

	if (IS_PIN_DIGITAL(pin))
	{
		if (mode == INPUT) {
			portConfigInputs[pin / 8] |= (1 << (pin & 7));
		}
		else
		{
			portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
		}
	}
	pinState[pin] = 0;
	switch (mode)
	{
	case ANALOG:
		if (IS_PIN_ANALOG(pin))
		{
			if (IS_PIN_DIGITAL(pin))
			{
				pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
				digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
			}
			pinConfig[pin] = ANALOG;
		}
		break;
	case INPUT:
		if (IS_PIN_DIGITAL(pin))
		{
			pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
			digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
			pinConfig[pin] = INPUT;
		}
		break;
	case OUTPUT:
		if (IS_PIN_DIGITAL(pin))
		{
			digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
			pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
			pinConfig[pin] = OUTPUT;
		}
		break;
	case PWM:
		if (IS_PIN_PWM(pin))
		{
			pinMode(PIN_TO_PWM(pin), OUTPUT);
			analogWrite(PIN_TO_PWM(pin), 0);
			pinConfig[pin] = PWM;
		}
		break;
	case I2C:
		if (IS_PIN_I2C(pin))
		{
			// mark the pin as i2c
			// the user must call I2C_CONFIG to enable I2C for a device
			pinConfig[pin] = I2C;
		}
		break;
	default:
		Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
	}
	// TODO: save status to EEPROM here, if changed
}

void enableI2CPins()
{
	byte i;
	// is there a faster way to do this? would probaby require importing 
	// Arduino.h to get SCL and SDA pins
	for (i = 0; i < TOTAL_PINS; i++)
	{
		if (IS_PIN_I2C(i))
		{
			// mark pins as i2c so they are ignore in non i2c data requests
			setPinModeCallback(i, I2C);
		}
	}

	isI2CEnabled = true;

	// is there enough time before the first I2C request to call this here?
	Wire.begin();
}

void analogWriteCallback(byte pin, int value)
{
	if (pin < TOTAL_PINS)
	{
		switch (pinConfig[pin])
		{
		case PWM:
			if (IS_PIN_PWM(pin))
				analogWrite(PIN_TO_PWM(pin), value);
			pinState[pin] = value;
			break;
		}
	}
}

void sysexCallback(byte command, byte argc, byte *argv)
{
	byte mode;
	byte slaveAddress;
	byte slaveRegister;
	byte data;
	unsigned int delayTime;

	switch (command)
	{
	case I2C_REQUEST:
		mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
		if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK)
		{
			Firmata.sendString("10-bit addressing mode is not yet supported");
			return;
		}
		else
		{
			slaveAddress = argv[0];
		}

		switch (mode)
		{
		case I2C_WRITE:
			Wire.beginTransmission(slaveAddress);
			for (byte i = 2; i < argc; i += 2)
			{
				data = argv[i] + (argv[i + 1] << 7);
#if ARDUINO >= 100
				Wire.write(data);
#else
				Wire.send(data);
#endif
			}
			Wire.endTransmission();
			delayMicroseconds(70);
			break;
		case I2C_READ:
			if (argc == 6)
			{
				// a slave register is specified
				slaveRegister = argv[2] + (argv[3] << 7);
				data = argv[4] + (argv[5] << 7);  // bytes to read
				readAndReportData(slaveAddress, (int)slaveRegister, data);
			}
			else
			{
				// a slave register is NOT specified
				data = argv[2] + (argv[3] << 7);  // bytes to read
				readAndReportData(slaveAddress, (int)REGISTER_NOT_SPECIFIED, data);
			}
			break;
		case I2C_READ_CONTINUOUSLY:
			if ((queryIndex + 1) >= MAX_QUERIES)
			{
				// too many queries, just ignore
				Firmata.sendString("too many queries");
				break;
			}
			queryIndex++;
			query[queryIndex].addr = slaveAddress;
			query[queryIndex].reg = argv[2] + (argv[3] << 7);
			query[queryIndex].bytes = argv[4] + (argv[5] << 7);
			break;
		case I2C_STOP_READING:
			byte queryIndexToSkip;
			// if read continuous mode is enabled for only 1 i2c device, disable
			// read continuous reporting for that device
			if (queryIndex <= 0)
			{
				queryIndex = -1;
			}
			else
			{
				// if read continuous mode is enabled for multiple devices,
				// determine which device to stop reading and remove it's data from
				// the array, shifiting other array data to fill the space
				for (byte i = 0; i < queryIndex + 1; i++)
				{
					if (query[i].addr = slaveAddress)
					{
						queryIndexToSkip = i;
						break;
					}
				}

				for (byte i = queryIndexToSkip; i < queryIndex + 1; i++)
				{
					if (i < MAX_QUERIES)
					{
						query[i].addr = query[i + 1].addr;
						query[i].reg = query[i + 1].addr;
						query[i].bytes = query[i + 1].bytes;
					}
				}
				queryIndex--;
			}
			break;
		default:
			break;
		}
		break;
	case I2C_CONFIG:
		delayTime = (argv[0] + (argv[1] << 7));

		if (delayTime > 0)
		{
			i2cReadDelayTime = delayTime;
		}

		if (!isI2CEnabled)
		{
			enableI2CPins();
		}

		break;
	case SAMPLING_INTERVAL:
		if (argc > 1)
		{
			samplingInterval = argv[0] + (argv[1] << 7);
			if (samplingInterval < MINIMUM_SAMPLING_INTERVAL)
			{
				samplingInterval = MINIMUM_SAMPLING_INTERVAL;
			}
		}
		else
		{
			//Firmata.sendString("Not enough data");
		}
		break;
	case EXTENDED_ANALOG:
		if (argc > 1)
		{
			int val = argv[1];
			if (argc > 2) val |= (argv[2] << 7);
			if (argc > 3) val |= (argv[3] << 14);
			analogWriteCallback(argv[0], val);
		}
		break;
	case CAPABILITY_QUERY:
		Serial.write(START_SYSEX);
		Serial.write(CAPABILITY_RESPONSE);
		for (byte pin = 0; pin < TOTAL_PINS; pin++)
		{
			if (IS_PIN_DIGITAL(pin))
			{
				Serial.write((byte)INPUT);
				Serial.write(1);
				Serial.write((byte)OUTPUT);
				Serial.write(1);
			}
			if (IS_PIN_ANALOG(pin))
			{
				Serial.write(ANALOG);
				Serial.write(10);
			}
			if (IS_PIN_PWM(pin))
			{
				Serial.write(PWM);
				Serial.write(8);
			}
			if (IS_PIN_I2C(pin))
			{
				Serial.write(I2C);
				Serial.write(1);  // to do: determine appropriate value 
			}
			Serial.write(127);
		}
		Serial.write(END_SYSEX);
		break;
	case PIN_STATE_QUERY:
		if (argc > 0)
		{
			byte pin = argv[0];
			Serial.write(START_SYSEX);
			Serial.write(PIN_STATE_RESPONSE);
			Serial.write(pin);
			if (pin < TOTAL_PINS)
			{
				Serial.write((byte)pinConfig[pin]);
				Serial.write((byte)pinState[pin] & 0x7F);
				if (pinState[pin] & 0xFF80) Serial.write((byte)(pinState[pin] >> 7) & 0x7F);
				if (pinState[pin] & 0xC000) Serial.write((byte)(pinState[pin] >> 14) & 0x7F);
			}
			Serial.write(END_SYSEX);
		}
		break;
	case ANALOG_MAPPING_QUERY:
		Serial.write(START_SYSEX);
		Serial.write(ANALOG_MAPPING_RESPONSE);
		for (byte pin = 0; pin < TOTAL_PINS; pin++)
		{
			Serial.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
		}
		Serial.write(END_SYSEX);
		break;
	}
}

/* Firmata Part end*/

void setup()
{
	byte pin;

	/* Setup of Firmata */

	Firmata.setFirmwareVersion(0, 2);
	Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
	Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
	Firmata.attach(SET_PIN_MODE, setPinModeCallback);
	Firmata.attach(START_SYSEX, sysexCallback);
	//Firmata.begin(9600);

	/* GTR Part setup*/

	//IR Line sensors
	pinMode(rightSideWatch, INPUT);
	pinMode(leftSideWatch, INPUT);

	//Pins to controll GTR car
	pinMode(rightRotate, OUTPUT);
	pinMode(leftRotate, OUTPUT);
	pinMode(gas, OUTPUT);
	pinMode(stop, OUTPUT);

	//Pins to getting signals from gamepad
	pinMode(buttonForward, INPUT);
	pinMode(buttonBack, INPUT);
	pinMode(buttonLeft, INPUT);
	pinMode(buttonRight, INPUT);

	stopper = false;		// At the beginning we dont't need in stop function
	carMode = gamepad;		// At the beginning we control car by gamepad
	carState = forward;		// Initial car state in autonommus mode
	numberOfKey = 0;

	Serial.begin(9600);

	// If we have no ultrasonic - we have never any barriers
#ifndef ULTRASONIC_INCLUDE
	barrierAhead = 1;
#endif //ULTRASONIC_INCLUDE
}

/* GTR Control methods */

//Method to switch current control mode
//L-Left, R-Right, U-Up, D-Down
//Swithing to mobile control: L R L U D R
//Swithing to gamepad control: L R L U D L
//Swithing to mobile control: L R L U D U
void modeSwitcher() {

	keyValue = digitalRead(buttonForward) + digitalRead(buttonBack) * 2 + digitalRead(buttonRight) * 3 + digitalRead(buttonLeft) * 4;

	if (keyValue && keyValue != keyArray[numberOfKey]) {
		if (keyArray[numberOfKey + 1] == keyValue || numberOfKey == 5) {
			numberOfKey++;
		}
		else {
			numberOfKey = 0;
		}
	}

	if (keyValue && numberOfKey == 6) {
		if (digitalRead(buttonForward)) {
			carMode = automatic;
		}
		else if (digitalRead(buttonRight)) {
			carMode = phone;
		}
		else if (digitalRead(buttonLeft)) {
			carMode = gamepad;
		}
		digitalWrite(stop, LOW);
		numberOfKey = 0;
	}
}

//In gamepad controlling mode we directly control car considering barriers. 
void modeGamepad() {
	digitalWrite(gas, digitalRead(buttonForward) * barrierAhead);
	digitalWrite(stop, digitalRead(buttonBack));
	digitalWrite(rightRotate, digitalRead(buttonRight));
	digitalWrite(leftRotate, digitalRead(buttonLeft));
}

//In autonomus mode we directly control car considering barriers
void modeAuto() {

	switch (carState) {
	case forward: {
		moveForward();
		break;
	}
	case left: {
		moveLeft();
		break;
	}
	case right: {
		moveRight();
		break;
	}
	default:
		break;
	}
}

//Function to stop driving if there are some barriers and moving backward 200ms
void stopDriving() {

	if (stopper && digitalRead(gas)) {
		digitalWrite(gas, LOW);
		digitalWrite(stop, HIGH);
		delay(200);
		digitalWrite(stop, LOW);
	}
	stopper = false;
}

#ifdef ULTRASONIC_INCLUDE
//Function to detect a barrier at a distance of no more than 30cm
void isBarrier() {
	Serial.println(ultrasonic.Ranging(CM));
	if (ultrasonic.Ranging(CM) < 30) {
		barrierAhead = 0;
		stopDriving();
	}
	else {
		barrierAhead = 1;
		stopper = true;
	}
}
#endif 

//Moving forward if there is no barrier
void moveForward() {

	digitalWrite(gas, HIGH * barrierAhead);
	if (!digitalRead(leftSideWatch)) {
		carState = left;
	}
	else if (!digitalRead(rightSideWatch)) {
		carState = right;
	}
}

//Moving left if there is no barrier
void moveLeft()
{
	digitalWrite(leftRotate, HIGH);
	// Move Left until right sensor has crossed the line
	while (digitalRead(rightSideWatch) && !carMode)
	{
#ifdef ULTRASONIC_INCLUDE
		isBarrier();
#endif
		// Move left with delay to reduce speed
		if (millis() - timer > 200)
		{
			timer = millis();
			digitalWrite(gas, !digitalRead(gas) * barrierAhead);
		}
		modeSwitcher();
	}
	digitalWrite(gas, HIGH * barrierAhead);
	digitalWrite(leftRotate, LOW);
	carState = forward;
}

//Moving right if there is no barrier
void moveRight()
{

	digitalWrite(rightRotate, HIGH);
	timer = millis();
	// Move Right until left sensor has crossed the line
	while (digitalRead(leftSideWatch) && !carMode)
	{
#ifdef ULTRASONIC_INCLUDE
		isBarrier();
#endif
		// Move right with delay to reduce speed
		if (millis() - timer > 200)
		{
			timer = millis();
			digitalWrite(gas, !digitalRead(gas) * barrierAhead);
		}
		modeSwitcher();
	}
	digitalWrite(gas, HIGH * barrierAhead);
	digitalWrite(rightRotate, LOW);
	carState = forward;
}

void firmataPart()
{
	byte pin;
	/* DIGITALREAD - as fast as possible, check for changes and output them to the
	 * FTDI buffer using Serial.print()  */
	checkDigitalInputs();
	while (Firmata.available())
		Firmata.processInput();

	currentMillis = millis();
	if (currentMillis - previousMillis > samplingInterval)
	{
		previousMillis += samplingInterval;
		// report i2c data for all device with read continuous mode enabled
		if (queryIndex > -1)
		{
			for (byte i = 0; i < queryIndex + 1; i++)
			{
				readAndReportData(query[i].addr, query[i].reg, query[i].bytes);
			}
		}
	}
}

void loop()
{

#ifdef ULTRASONIC_INCLUDE
	isBarrier();
#endif

	if (carMode == automatic)
	{
		modeAuto();
	}
	else if (carMode == gamepad)
	{
		modeGamepad();
	}
	else
	{
		//if control type is "phone" we should use firmata
		//to control GTR
		firmataPart();
	}

	modeSwitcher();
}