/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************

   DESCRIPTION

   Example sketch showing how to send in DS1820B OneWire temperature readings back to the controller
   http://www.mysensors.org/build/temp
   Enhanced Version also sending the Dallas-ROM-ID, MySensors Version >=2.1.0
   + 4 Relays
*/

// Enable debug prints to serial monitor
//#define MY_DEBUG

// Enable RS485 transport layer
#define MY_RS485

// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 3

// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 38400 //9600
#define MY_RS485_SOH_COUNT 3
#define MY_RS485_HWSERIAL Serial

// Enable and select radio type attached
//#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
#define MY_NODE_ID 101
#define MY_TRANSPORT_WAIT_READY_MS 3000

#include <SPI.h>
#include <MySensors.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define ONE_WIRE_BUS 8 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 4
uint8_t DS_First_Child_ID = 7; //First Child-ID to be used by Dallas Bus; set this to be higher than other Child-ID's who need EEPROM storage to avoid conflicts
unsigned long TempSendFreqency = 180000; // Sleep time between reads (in milliseconds)
unsigned long lastTemp = 0;
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature.
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors = 0;
bool receivedConfig = false;
bool metric = true;
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
int resolution = 10; // precision: 12 bits = 0.0625°C, 11 bits = 0.125°C, 10 bits = 0.25°C, 9 bits = 0.5°C
int conversionTime = 0;
// Initialize temperature message
MyMessage msgTemp(0, V_TEMP);
MyMessage msgId(0, V_ID);

char* charAddr = "Check for faults";
#define SEND_ID

#define RELAY_1  4  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS 4 // Total number of attached relays
#define RELAY_ON 0 // GPIO value to write to turn on attached relay
#define RELAY_OFF 1 // GPIO value to write to turn off attached relay

void before()
{
  // 12 bits = 750 ms, 11 bits = 375ms, 10 bits = 187.5ms, 9 bits = 93.75ms
  conversionTime = 750 / (1 << (12 - resolution));
  // Startup up the OneWire library
  sensors.begin();
  for (int sensor = 1, pin = RELAY_1; sensor <= NUMBER_OF_RELAYS; sensor++, pin++) {
    // Then set relay pins in output mode
    pinMode(pin, OUTPUT);
    digitalWrite(pin, RELAY_OFF);
  }
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Garage hinten", "0.2");

  // Fetch the number of attached temperature sensors
  numSensors = sensors.getDeviceCount();

  // Present all sensors to controller
  for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {
    sensors.getAddress(tempDeviceAddress, i);
    charAddr = addrToChar(tempDeviceAddress);
    present(i + 1, S_TEMP, charAddr);
#ifdef MY_DEBUG
    Serial.println(charAddr);
#endif
  }
  for (int sensor = 1, pin = RELAY_1; sensor <= NUMBER_OF_RELAYS; sensor++, pin++) {
    // Register all sensors to gw (they will be created as child devices)
    present(sensor, S_BINARY);
  }
  metric = getControllerConfig().isMetric;
  for (int sensor = 1, pin = RELAY_1; sensor <= NUMBER_OF_RELAYS; sensor++, pin++) {
    // Set relay to last known state (using eeprom storage)
    request(sensor, V_STATUS);
  }
}

void setup()
{
  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);

  for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {
    sensors.getAddress(tempDeviceAddress, i);
#ifdef SEND_ID
    // 8 will assure a length of 16 of the sent ROM-ID
    send(msgId.setSensor(i + 1).set(tempDeviceAddress, 8));
#endif
    sensors.setResolution(tempDeviceAddress, resolution);
  }
}

void loop() {
  unsigned long currentTime = millis();  // Fetch temperatures from Dallas sensors
  if (currentTime - lastTemp > TempSendFreqency) {
    sensors.requestTemperatures();

    // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
    wait(conversionTime);

    // Read temperatures and send them to controller
    for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {

      // Fetch and round temperature to one decimal
      float temperature = static_cast<float>(static_cast<int>((metric ? sensors.getTempCByIndex(i) : sensors.getTempFByIndex(i)) * 10.)) / 10.;

      // Only send data if temperature has changed and no error
#if COMPARE_TEMP == 1
      if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) {
#else
      if (temperature != -127.00 && temperature != 85.00) {
#endif

        // Send in the new temperature
        send(msgTemp.setSensor(i + 1).set(temperature, 1));
        wait(20);
        // Save new temperatures for next compare
        lastTemperature[i] = temperature;
      }
    }
    lastTemp = millis();
  }
  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  //wait(SLEEP_TIME);
}

char* addrToChar(uint8_t* data) {
  String strAddr = String(data[0], HEX); //Chip Version; should be higher than 16
  byte first ;
  int j = 0;
  for (uint8_t i = 1; i < 8; i++) {
    if (data[i] < 16) strAddr = strAddr + 0;
    strAddr = strAddr + String(data[i], HEX);
    strAddr.toUpperCase();
  }
  for (int j = 0; j < 16; j++) {
    charAddr[j] = strAddr[j];
  }
  return charAddr;
}

void receive(const MyMessage & message)
{
  if (message.type == V_STATUS) {
    // Change relay state
    digitalWrite(message.sensor - 1 + RELAY_1, message.getBool() ? RELAY_ON : RELAY_OFF);
    // Store state in eeprom
    //saveState(message.sensor, message.getBool());
    // Write some debug info
#ifdef MY_DEBUG
    Serial.print("Incoming change for sensor:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());
#endif
  }
}

