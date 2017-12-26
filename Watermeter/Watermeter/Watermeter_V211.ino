/*
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Ralph Bisschops
* Based on original Watermeter sketch V 1.1 by Henrik Ekblad and GizMoCuz
* Copyright (C) 2017 Ralph Bisschops
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
*******************************
*
* REVISION HISTORY
* Version 0.1 - Ralph Bisschops (first alpha release)
* Version 0.2 - Ralph Bisschops (second alpha release)
* Version 0.3 - Ralph Bisschops (updated EEPROM procedures)
* Version 0.4 - Ralph Bisschops (updated layout)
* Version 0.5 - Ralph Bisschops (prep for 2.0.0)
* Version 0.6 - Ralph Bisschops (uses library 2.1.1)
* Version 1.0 - Ralph Bisschops (production version)
*
* DESCRIPTION
* Use the TCRT5000 sensor to measure volume and flow of your house watermeter.
* You need to set the correct pulsefactor of your meter (pulses per m3).
* The sensor starts by fetching current volume reading from gateway (VAR 1).
* The sensor can receive updated thresholds from the gateway.
* VAR 2 sents upper threshold level, VAR 3 sents lower threshold level.
* Threshold levels are stored in EEPROM for use after power failure.
* Reports both volume and flow back to gateway.
*
* Unfortunately millis() won't increment when the Arduino is in
* sleep mode. So we cannot make this sensor sleep if we also want
* to calculate/report flow.
*/

// Enable debug prints to serial monitor
#include <MyConfig.h>
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RF24_CHANNEL	1
//#define MY_DEBUG_VERBOSE_RF24

#include <SPI.h>
#include <MySensors.h>

// Node and sketch information
#define SKETCH_VER            "1.0"				// Sketch version
#define SKETCH_NAME           "Watermeter"		// Optional child sensor name
#define CHILD_ID              1					// Id of the sensor child
#define CHILD_NAME            "Watermeter"		// Optional child sensor name
#define NODE_REPEATER         false				// Set true if a repeater node (i.e. always turned on)
#define SLEEP_MODE            false				// flowvalue can only be reported when sleep mode is false.

// Sketch settings
#define PULSE_FACTOR          1000				// Number of blinks per m3 of your meter (One rotation/liter)
#define MAX_FLOW              40				// Max flow (l/min) value to report. This filters outliers.

// Input and output definitions
#define DIGITAL_INPUT_SENSOR  3					// The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)
#define ANALOG_INPUT_SENSOR   A0				// The analog input you you attached your sensor. 

#define SENSOR_INTERRUPT DIGITAL_INPUT_SENSOR-2	// Usually the interrupt = pin -2 (on uno/nano anyway)

MyMessage flowMsg(CHILD_ID, V_FLOW);
MyMessage volumeMsg(CHILD_ID, V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID, V_VAR1);
MyMessage analogValue(CHILD_ID, V_VAR5);

// Global vars
unsigned long sendFrequency = 30000;			// Minimum time between send (in milliseconds). We don't want to spam the gateway.
volatile unsigned int highThreshold = 500;		// higher threshold for analog readings
volatile unsigned int lowThreshold = 400;       // lower threshold for analog readings
volatile unsigned int debugLevel = 0;			// sets the debug level, 0 = basic info. 1 = streaming level info. 2 = sent level streaming to gateway.
volatile uint32_t pulseCount = 0;
volatile unsigned long lastBlink = 0;
volatile double flow = 0;
boolean pcReceived = false;
unsigned long oldPulseCount = 0;
unsigned long newBlink = 0;
double ppl = ((double)PULSE_FACTOR) / 1000;		// Pulses per liter
double oldflow = 0;
double volume = 0;
double oldvolume = 0;
unsigned long lastSend = 0;
unsigned long lastPulse = 0;
int sensorValue;
boolean sensorState;

void setup()
{
	// initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion) 
	pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);

	pulseCount = oldPulseCount = 0;

	// Fetch last known pulse count value from gw
	request(CHILD_ID, V_VAR1);

	lastSend = lastPulse = millis();

	// attachInterrupt(SENSOR_INTERRUPT, onPulse, FALLING);

	// Fetch debug level from EEPROM
	debugLevel = loadState(4);
	debugMessage("Debug level fetched from EEPROM, value: ", String(debugLevel));

	// Fetch the last set thresholds from EEPROM
	int high = readEeprom(0); //loadState (0);
	int low = readEeprom(2); //loadState (2);
	if (high > 0 || low > 0) {
		highThreshold = high;
		lowThreshold = low;
		debugMessage("High threshold fetched from EEPROM, value: ", String(highThreshold));
		debugMessage("Low threshold fetched from EEPROM, value: ", String(lowThreshold));
	}
	else {
		debugMessage("High threshold set to standard value: ", String(highThreshold));
		debugMessage("Low threshold set to standard value: ", String(lowThreshold));
	}
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo(SKETCH_NAME, SKETCH_VER);

	// Register this device as Waterflow sensor
	present(CHILD_ID, S_WATER);
}

void loop()
{
	unsigned long currentTime = millis();

	// Check the analog sensor values and change state when thresholds are passed
	checkThreshold();

	// Only send values at a maximum frequency or woken up from sleep
	if (SLEEP_MODE || (currentTime - lastSend > sendFrequency)) {
		lastSend = currentTime;

		if (!pcReceived) {
			//Last Pulsecount not yet received from controller, request it again
			request(CHILD_ID, V_VAR1);
			return;
		}

		if (!SLEEP_MODE && flow != oldflow) {
			oldflow = flow;
			debugMessage("l/min:", String(flow));

			// Check that we dont get unreasonable large flow value. 
			// could hapen when long wraps or false interrupt triggered
			if (flow < ((unsigned long)MAX_FLOW)) {
				// Send flow value to gw
				send(flowMsg.set(flow, 2));
			}
		}

		// No Pulse count received in 2 min 
		if (currentTime - lastPulse > 120000) {
			flow = 0;
		}

		// Pulse count has changed
		if ((pulseCount != oldPulseCount) || (!SLEEP_MODE)) {
			oldPulseCount = pulseCount;
			if (debugLevel < 2) {
				debugMessage("pulsecount: ", String(pulseCount));
				// Send  pulsecount value to gw in VAR1
				send(lastCounterMsg.set(pulseCount));
			}
			double volume = ((double)pulseCount / ((double)PULSE_FACTOR));
			if ((volume != oldvolume) || (!SLEEP_MODE)) {
				oldvolume = volume;
				if (debugLevel < 2) {
					debugMessage("volume: ", String(volume, 3));
					// Send volume value to gw
					send(volumeMsg.set(volume, 3));
				}
			}
		}
	}
	if (SLEEP_MODE) {
		sleep(sendFrequency);
	}
}

void debugMessage(String header, String content)
{
	// DEBUG code ------
	Serial.print(header);
	Serial.println(content);
	// DEBUG code ------   
}

void receive(const MyMessage &message)
{
	switch (message.type) {
	case V_VAR1: {
		unsigned long gwPulseCount = message.getULong();
		pulseCount += gwPulseCount;
		flow = oldflow = 0;
		debugMessage("Received last pulse count from gw: ", String(pulseCount));
		pcReceived = true;
	}
				 break;
	case V_VAR2: {
		highThreshold = message.getULong();
		storeEeprom(0, highThreshold);
		debugMessage("Received new high threshold from gw: ", String(highThreshold));
	}
				 break;
	case V_VAR3: {
		lowThreshold = message.getULong();
		storeEeprom(2, lowThreshold);
		debugMessage("Received new low threshold from gw: ", String(lowThreshold));
	}
				 break;
	case V_VAR4: {
		debugLevel = message.getULong();
		saveState(4, debugLevel);
		debugMessage("Received new debug state from gw: ", String(debugLevel));
	}
				 break;
	default: {
		debugMessage("Received invalid message from gw! ", "");
	}
	}
	debugMessage("Stored in EEPROM", "");
	debugMessage("Pos 0: ", String(loadState(0)));
	debugMessage("Pos 1: ", String(loadState(1)));
	debugMessage("Pos 2: ", String(loadState(2)));
	debugMessage("Pos 3: ", String(loadState(3)));
	debugMessage("Pos 4: ", String(loadState(4)));
}

void storeEeprom(int pos, int value) {
	// function for saving the values to the internal EEPROM
	// value = the value to be stored (as int)
	// pos = the first byte position to store the value in
	// only two bytes can be stored with this function (max 32.767)
	saveState(pos, ((unsigned int)value >> 8));
	pos++;
	saveState(pos, (value & 0xff));
}

int readEeprom(int pos) {
	// function for reading the values from the internal EEPROM
	// pos = the first byte position to read the value from 

	int hiByte;
	int loByte;

	hiByte = loadState(pos) << 8;
	pos++;
	loByte = loadState(pos);
	return (hiByte | loByte);
}

void onPulse()
{
	if (!SLEEP_MODE) {
		unsigned long newBlink = micros();
		unsigned long interval = newBlink - lastBlink;

		if (interval != 0) {
			lastPulse = millis();
			if (interval < 500000L) {
				// Sometimes we get interrupt on RISING,  500000 = 0.5 sec debounce (max 120 l/min)
				return;
			}
			flow = (60000000.0 / interval) / ppl;
		}
		lastBlink = newBlink;
	}
	pulseCount++;
}

void checkThreshold() {

	sensorValue = getAverage();
	if (debugLevel > 0) {
		debugMessage("val = ", String(sensorValue, DEC));
		if (debugLevel == 2) {
			send(analogValue.set(sensorValue));
		}

	}
	if ((sensorState == true) && (sensorValue < lowThreshold)) {
		pulseCount++;
		sensorState = !sensorState;
		debugMessage("pulsCount + 1", "");
		debugMessage("Sensor state: ", String(sensorState));
	}
	if ((sensorState == false) && (sensorValue > highThreshold)) {
		sensorState = !sensorState;
		debugMessage("Sensor state: ", String(sensorState));
	}
}

int getAverage()
{
	int cycles = 0;
	int val = 0;		// variable to store current values from the input
	int newVal = 0;     // variable to store new values from the input
	int average = 0;    // variable to store the peak value
	int count = 0;      // variable for loop

						// take 10 samples over half a second 
						// for each sample the input is taken 10 times
	for (cycles = 0; cycles < 10; cycles++) {
		// read input 10 times and get the sum
		// "ANALOG_INPUT_SENSOR" was defined in main program as shown below:
		// "#define ANALOG_INPUT_SENSOR A0"
		for (count = 0; count < 10; count++) {
			val = val + analogRead(ANALOG_INPUT_SENSOR);
		}
		// get average of readings
		val = (val / 10);

		// add reading to newVal
		newVal += val;
		val = 0;
		// measure samples over half a second or "newVal" will
		// almost always be the same resulting in wrong average
		delay(50); // 10 cycles of 50ms gives  10 samples in 500ms
	}
	// set average to average of newValue1
	average = newVal / 10;

	// return the value of average to main program
	return average;
}
