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
#define SN "Garage hinten"
#define SV "0.2.3"
  
// Enable debug prints to serial monitor
//#define MY_DEBUG

// Enable RS485 transport layer
#define MY_RS485

// Define this to enables DE-pin management on defined pin
//#define MY_RS485_DE_PIN 3

// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 19200 //57600 //38400 //9600
//#define MY_RS485_SOH_COUNT 3
#define MY_RS485_HWSERIAL Serial
#define MY_SPLASH_SCREEN_DISABLED

// Enable and select radio type attached
//#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
#define MY_NODE_ID 95
#define MY_TRANSPORT_WAIT_READY_MS 3000

//#include <SPI.h>
#include <MySensors.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Bounce2.h>

#define COMPARE_TEMP 0 // Send temperature only if changed? 1 = Yes 0 = No
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

#define DIGITAL_INPUT_SENSOR 2  // The digital input you attached your water sensor.  (Only 2 and 3 generates interrupt!)
#define PULSE_FACTOR 1000       // Nummber of blinks per m3 of your meter (One rotation/10 liter)
#define SENSOR_INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define CHILD_ID_WATER 16              // Id of the sensor child
#define MAX_FLOW 100          // Max volume value to report. This filetrs outliers.

MyMessage flowMsg(CHILD_ID_WATER, V_FLOW);
MyMessage volumeMsg(CHILD_ID_WATER, V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID_WATER, V_VAR1);
unsigned long SEND_FREQUENCY = 300000; // Minimum time between send (in milliseconds). We don't want to spam the gateway.

double ppl = ((double)PULSE_FACTOR) / 1000;      // Pulses per liter

volatile unsigned long pulseCount = 0;
volatile unsigned long lastBlink = 0;
volatile double flow = 0;
boolean pcReceived = false;
unsigned long oldPulseCount = 0;
unsigned long newBlink = 0;
double oldflow = 0;
double volume = 0;
double oldvolume = 0;
unsigned long lastSend = 0;
unsigned long lastPulse = 0;
#define SLEEP_MODE false        // Watt-value can only be reported when sleep mode is false.

unsigned long lastTempSend = 0;

#define FIRST_PIR_ID 17
#define MAX_PIRS 4 //Pump running, button, water level, PIR
const uint8_t pirPin[] = {A0, A1, A2, A3};   //  switch around pins to your desire
Bounce debouncer[MAX_PIRS];
MyMessage pirMsg(0, V_TRIPPED);
bool oldPir[MAX_PIRS] = {false};

unsigned long maxTimeDryRun = 30000;
unsigned long startTimeDryRun = 0;
volatile unsigned long dryPulseCount = 0;

#define RELAY_1  4  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS 4 // Total number of attached relays
#define RELAY_ON 0 // GPIO value to write to turn on attached relay
#define RELAY_OFF 1 // GPIO value to write to turn off attached relay

//For pump
#define RELAY_PIN 7 
MyMessage RelayMsg(NUMBER_OF_RELAYS, V_LIGHT);


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
  
  // initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion)
  pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);
  digitalWrite(DIGITAL_INPUT_SENSOR, HIGH);
  pulseCount = oldPulseCount = 0;
  attachInterrupt(SENSOR_INTERRUPT, onPulse, FALLING);
  
  for (uint8_t i = 0; i < MAX_PIRS; i++) {
    debouncer[i] = Bounce();                        // initialize debouncer
    debouncer[i].attach(pirPin[i], INPUT_PULLUP);
    debouncer[i].interval(5);
  }
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SN, SV);
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
  present(CHILD_ID_WATER, S_WATER);
  for (int i = 0; i < MAX_PIRS; i++) { //i < numSensors &&
    present(FIRST_PIR_ID + i, S_MOTION);
  }
  // Fetch last known pulse count value from gw
  request(CHILD_ID_WATER, V_VAR1);

  for (int i = 0; i < MAX_PIRS; i++) { //i < numSensors &&
    present(FIRST_PIR_ID + i, S_MOTION);
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
  lastSend = lastPulse = millis();
}

void loop() {
  unsigned long currentTime = millis();  // Fetch temperatures from Dallas sensors
  
  if (currentTime - lastTemp > TempSendFreqency) {
    sensors.requestTemperatures();
    sendHeartbeat();

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
  
  
  bool pir[MAX_PIRS];
  bool bounceUpdate[MAX_PIRS] = {false}; //true, if button pressed
  for (uint8_t i = 0; i < MAX_PIRS; i++) {
    debouncer[i].update();
    pir[i] = debouncer[i].read() == HIGH;
    if (pir[i] != oldPir[i]) {
      send(pirMsg.setSensor(FIRST_PIR_ID + i).set( pir[i])); // Send tripped value to gw
      if (i == 0 && !pir[i]) {
        startTimeDryRun = currentTime;
      }  
      if (i == 1 && pir[i]) {
        startTimeDryRun = currentTime;
        bool inverted = !digitalRead(RELAY_PIN);
        digitalWrite(RELAY_PIN, inverted);
        send(RelayMsg.set(!inverted)); // Send new state 
      }
      oldPir[i] = pir[i];
    }
  } 
  if (digitalRead(RELAY_PIN) == RELAY_ON && !pir[0]) {
    if (currentTime - startTimeDryRun > maxTimeDryRun) {
      if (dryPulseCount == 1) { //XXX Change to 0 when flow meter is attached
        digitalWrite(RELAY_PIN, RELAY_OFF);
        send(RelayMsg.set(false)); // Send new state and request ack back
      }
    } else {
        startTimeDryRun = currentTime;
        dryPulseCount = 0;
    }
  }
    // Only send values at a maximum frequency or woken up from sleep
  if (currentTime - lastSend > SEND_FREQUENCY)
  {
    lastSend = currentTime;
    if (flow != oldflow) {
      oldflow = flow;
#ifdef MY_DEBUG_LOCAL
      Serial.print("l/min:");
      Serial.println(flow);
#endif
      // Check that we dont get unresonable large flow value.
      // could hapen when long wraps or false interrupt triggered
      if (flow < ((unsigned long)MAX_FLOW)) {
        send(flowMsg.set(flow, 2));                   // Send flow value to gw
      }
    }

    // No Pulse count received in 2min
    if (currentTime - lastPulse > 120000) {
      flow = 0;
    }

    // Pulse count has changed
    if (pulseCount != oldPulseCount) {
      oldPulseCount = pulseCount;
#ifdef MY_DEBUG
      Serial.print(F("pulsecnt:"));
      Serial.println(pulseCount);
#endif
      if (!pcReceived) {
        request(CHILD_ID_WATER, V_VAR1);
      }
      send(lastCounterMsg.set(pulseCount));                  // Send  pulsecount value to gw in VAR1

      double volume = ((double)pulseCount / ((double)PULSE_FACTOR));
      if (volume != oldvolume) {
        oldvolume = volume;
#ifdef MY_DEBUG
        Serial.print(F("vol:"));
        Serial.println(volume, 3);
#endif
        send(volumeMsg.set(volume, 3));               // Send volume value to gw
      }
    }
  }
}

void onPulse()
{
  if (!SLEEP_MODE)
  {
    unsigned long newBlink = micros();
    unsigned long interval = newBlink - lastBlink;

    if (interval != 0)
    {
      lastPulse = millis();
      if (interval < 1000000L) {
        // Sometimes we get interrupt on RISING,  1000000 = 1sek debounce ( max 60 l/min)
        return;
      }
      flow = (60000000.0 / interval) / ppl;
    }
    lastBlink = newBlink;
  }
  pulseCount++;
  dryPulseCount++;
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

    if (message.sensor == RELAY_1+NUMBER_OF_RELAYS-1) {
      startTimeDryRun = millis();
    }
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
