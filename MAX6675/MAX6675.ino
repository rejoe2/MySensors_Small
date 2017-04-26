/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * 
 * DESCRIPTION
 * Pressure sensor example using BMP085 module  
 * http://www.mysensors.org/build/pressure
 *
 * This is an example that demonstrates how to report the battery level for a sensor
 * Instructions for measuring battery capacity on A0 are available here:
 * http://www.mysensors.org/build/battery *
 */

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#include <SPI.h>
#include <MySensors.h>  
#include <Wire.h>
#include "max6675.h"

int ktcSO = 11;
int ktcCS = 10;
int ktcCLK = 13;

MAX6675 ktc(ktcCLK, ktcCS, ktcSO);

int BATTERY_SENSE_PIN = A0;  // select the input pin for the battery sense point

int oldBatteryPcnt = 0;


#define TEMP_CHILD 1

float lastTemp=-127;
const unsigned long SLEEP_TIME = 30000; //Sende alle 5 Min.

bool metric;

MyMessage tempMsg(TEMP_CHILD, V_TEMP);

void before() {
#if defined(__AVR_ATmega2560__)
	analogReference(INTERNAL1V1);
#else
	analogReference(INTERNAL);
#endif
	pinMode (ktcCS, OUTPUT);
	digitalWrite(ktcCS, HIGH);
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Beta-User Temp", "0.1");

  // Register sensors to gw (they will be created as child devices)
  present(TEMP_CHILD, S_TEMP);
}

void setup() {
		metric = getControllerConfig().isMetric;
}


void loop() 
{
	float temperature = ktc.readCelsius();

	if (!metric) 
	{
		// Convert to fahrenheit
		temperature = temperature * 9.0 / 5.0 + 32.0;
	}
#ifdef MY_DEBUG
	Serial.print("Temperature = ");
	Serial.print(temperature);
	Serial.println(metric ? " *C" : " *F");
#endif	

	if (temperature != lastTemp) 
	{
		send(tempMsg.set(temperature, 1));
		lastTemp = temperature;
	}
	int sensorValue = analogRead(BATTERY_SENSE_PIN);
#ifdef MY_DEBUG
	Serial.println(sensorValue);
#endif

	// 1M, 470K divider across battery and using internal ADC ref of 1.1V
	// Sense point is bypassed with 0.1 uF cap to reduce noise at that point
	// ((1e6+470e3)/470e3)*1.1 = Vmax = 3.44 Volts
	// 3.44/1023 = Volts per bit = 0.003363075

	int batteryPcnt = sensorValue / 10;

#ifdef MY_DEBUG
	float batteryV  = sensorValue * 0.003363075;
	Serial.print("Battery Voltage: ");
	Serial.print(batteryV);
	Serial.println(" V");

	Serial.print("Battery percent: ");
	Serial.print(batteryPcnt);
	Serial.println(" %");
#endif

	if (oldBatteryPcnt != batteryPcnt) {
		// Power up radio after sleep
		sendBatteryLevel(batteryPcnt);
		oldBatteryPcnt = batteryPcnt;
	}
	sleep(SLEEP_TIME);
}

