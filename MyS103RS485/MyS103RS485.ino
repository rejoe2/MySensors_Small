/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters.
   Each repeater and gateway builds a routing tables in EEPROM which keeps track of
   the network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************

   REVISION HISTORY
   Version 1.0 - idefix

   DESCRIPTION
   Arduino BMP Temperature and Pressure sensor
   communicate using I2C Protocol
   this library enable 2 slave device addresses
   connect the sensor as follows :

     VCC  >>> 5V
     Gnd  >>> Gnd
     ADDR >>> NC or GND
     SCL  >>> A5
     SDA  >>> A4
  +2 Motion sensors, 1 to switch light on, if auto mode is set (S_CUSTOM)
*/


// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
//#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Enable RS485 transport layer
#define MY_RS485

// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 2

// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600

#define MY_TRANSPORT_WAIT_READY_MS 6000
#define MY_NODE_ID 103

//#include <SPI.h>
#include <MySensors.h>
#include <Bounce2.h>

//#include <BH1750.h>
//#include <Wire.h>
//#include <Adafruit_BMP085.h>

#define CHILD_CONFIG 0
MyMessage mOnTime(CHILD_CONFIG, V_VAR1);   //Receive configuration: On-Time
MyMessage mNightMode(CHILD_CONFIG, V_VAR2);   //Night=1; Day=0
//MyMessage mTest1(CHILD_CONFIG, V_VAR3);   //Test funktionality for oher projects
//MyMessage mTest2(CHILD_CONFIG, V_VAR4);   //#2
//MyMessage mTest3(CHILD_CONFIG, V_VAR5);   //#3

uint16_t pirOnTime = 300; // Default on-time in case of motion: 5 minutes
boolean NightMode = 0; // Default lowest light level for switch-on in case of motion detection
//char* Test1;
//char* Test2 ="";
//char* Test3 ="";

#define FIRST_PIR_ID 20
#define MAX_PIRS 2
const uint8_t pirPin[] = {4, 6};   //  switch around pins to your desire
Bounce debouncer[MAX_PIRS];
MyMessage pirMsg(0, V_TRIPPED);
bool oldPir[MAX_PIRS] = {false};

#define RELAY_PIN 5  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS 1 // Total number of attached relays
#define CHILD_ID_RELAY 1   // Id of the sensor child
#define RELAY_ON 0
#define RELAY_OFF 1
boolean onOff = false;
MyMessage RelayMsg(CHILD_ID_RELAY, V_LIGHT);
//MyMessage msgRelay(RELAY_1 - 3, V_LIGHT);
unsigned long lastSwitchOn = 0;

void before()
{
  // Then set relay pins in output mode
  pinMode(RELAY_PIN, OUTPUT);
  // Switch water pump off when starting up
  digitalWrite(RELAY_PIN, RELAY_OFF);

  for (uint8_t i = 0; i < MAX_PIRS; i++) {
    debouncer[i] = Bounce();                        // initialize debouncer
    debouncer[i].attach(pirPin[i], INPUT_PULLUP);
    debouncer[i].interval(5);
  }
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Garage", "0.4");

  // Register all sensors to gateway (they will be created as child devices)
  for (int i = 0; i < MAX_PIRS; i++) { //i < numSensors &&
    present(FIRST_PIR_ID + i, S_MOTION);
  }
  present(CHILD_CONFIG, S_CUSTOM);  //
  present(CHILD_ID_RELAY, S_LIGHT); 
}

void setup()
{
  request(CHILD_CONFIG, V_VAR1);
  request(CHILD_CONFIG, V_VAR2);
  //  request(CHILD_CONFIG, V_VAR3);
};


void loop() {
  
  unsigned long currentTime = millis();

  bool pir[MAX_PIRS];
  bool bounceUpdate[MAX_PIRS] = {false}; //true, if button pressed
  for (uint8_t i = 0; i < MAX_PIRS; i++) {
    debouncer[i].update();
    pir[i] = debouncer[i].read() == HIGH;
    if (pir[i] != oldPir[i]) {
      send(pirMsg.setSensor(FIRST_PIR_ID + i).set( pir[i])); // Send tripped value to gw
      if (i == 0 && pir[i]) {
        if (digitalRead(RELAY_PIN) == RELAY_OFF ) {
          // Send in the new temperature
          digitalWrite(RELAY_PIN, RELAY_ON);
          send(RelayMsg.set(true)); // Send new state and request ack back
        }
      }
      oldPir[i] = pir[i];
    }
  }

  if (pir[0] == 1) lastSwitchOn = currentTime;

  //Switch Relay off after motion has stopped
  if (currentTime - lastSwitchOn > pirOnTime*1000) {
    if (digitalRead(RELAY_PIN) == RELAY_ON ) {
      // Send in the new state
      digitalWrite(RELAY_PIN, RELAY_OFF);
      send(RelayMsg.set(false)); // Send new state 
    }
  }

};


void receive(const MyMessage &message) {
  if (message.type == V_STATUS) {
    // Change relay state
    digitalWrite(RELAY_PIN, message.getBool() ? RELAY_ON : RELAY_OFF);
    // Write some debug info
#ifdef MY_DEBUG
    Serial.print("Incoming change for sensor:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());
#endif
    if (message.getBool()) {
      lastSwitchOn  = millis();
    }
  }

  else if (message.type == V_VAR1) {
    if (message.getInt() > 0) pirOnTime = message.getInt();
#ifdef MY_DEBUG
    Serial.print(F("Received OnTime: "));
    Serial.println(pirOnTime);
#endif
  }
  else if (message.type == V_VAR2) {
    NightMode = message.getBool();
    if (NightMode == 0 && onOff) {
      digitalWrite(RELAY_PIN, RELAY_OFF);
      send(RelayMsg.set(false));
      onOff = false;
    }
#ifdef MY_DEBUG
    Serial.print(F("Received Night Mode: "));
    Serial.println(NightMode);
#endif
  }
  /*  else if (message.type == V_VAR3) {
      Test1 = (char*) message.getString();
    #ifdef MY_DEBUG
      Serial.println(F("Received Config:"));
      Serial.println(Test1);
    #endif
    }*/
}
