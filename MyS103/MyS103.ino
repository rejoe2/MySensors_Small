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
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
//#define MY_TRANSPORT_RELAXED
#define MY_TRANSPORT_WAIT_READY_MS 3000
#define MY_NODE_ID 103

#include <SPI.h>
#include <MySensors.h>
//#include <BH1750.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

#define CHILD_CONFIG 0
MyMessage mOnTime(CHILD_CONFIG, V_VAR1);   //Receive configuration: On-Time
MyMessage mNightMode(CHILD_CONFIG, V_VAR2);   //Night=1; Day=0
//MyMessage mTest1(CHILD_CONFIG, V_VAR3);   //Test funktionality for oher projects
//MyMessage mTest2(CHILD_CONFIG, V_VAR4);   //#2
//MyMessage mTest3(CHILD_CONFIG, V_VAR5);   //#3

uint16_t OnTime = 300; // Default on-time in case of motion: 5 minutes
boolean NightMode = 0; // Default lowest light level for switch-on in case of motion detection
//char* Test1;
//char* Test2 ="";
//char* Test3 ="";

const float ALTITUDE = 200; // <-- adapt this value to your own location's altitude.

#define BARO_CHILD 10
#define TEMP_CHILD 11
// Sleep time between reads (in seconds). Do not change this value as the forecast algorithm needs a sample every minute.
const unsigned long SLEEP_TIME = 60000;

const char *weather[] = { "stable", "sunny", "cloudy", "unstable",
                          "thunderstorm", "unknown"
                        };
enum FORECAST
{
  STABLE = 0,     // "Stable Weather Pattern"
  SUNNY = 1,      // "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,     // "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,   // "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4, // "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5     // "Unknown (More Time needed)
};

Adafruit_BMP085 bmp = Adafruit_BMP085();      // Digital Pressure Sensor

float lastPressure = -1;
float lastTemp = -1;
int lastForecast = -1;

const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];

// this CONVERSION_FACTOR is used to convert from Pa to kPa in forecast algorithm
// get kPa/h be dividing hPa by 10
#define CONVERSION_FACTOR (1.0/10.0)

int minuteCount = 0;
bool firstRound = true;
// average value is used in forecast algorithm.
float pressureAvg;
// average after 2 hours is used as reference value for the next iteration.
float pressureAvg2;

float dP_dt;
boolean metric;
MyMessage tempMsg(TEMP_CHILD, V_TEMP);
MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);
MyMessage forecastMsg(BARO_CHILD, V_FORECAST);

#define DIGITAL_INPUT_SENSOR1 2   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define INTERRUPT1 DIGITAL_INPUT_SENSOR1-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define CHILD_ID_M1 20   // Id of the first motion sensor child
#define DIGITAL_INPUT_SENSOR2 3   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define INTERRUPT2 DIGITAL_INPUT_SENSOR2-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define CHILD_ID_M2 21   // Id of the 2. sensor child
MyMessage msg_M1(CHILD_ID_M1, V_TRIPPED);
MyMessage msg_M2(CHILD_ID_M2, V_TRIPPED);
boolean tripped1 = 0;
boolean tripped2 = 0;

#define RELAY_1  4  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS 1 // Total number of attached relays
#define RELAY_ON 0
#define RELAY_OFF 1
boolean onOff = false;
MyMessage msgRelay(RELAY_1 - 3, V_LIGHT);
uint16_t switchtime ;
uint8_t roundscounter = 0;

uint16_t loopTime = 0;
uint16_t lastBMP = 0; // store last time of measurement

void before()
{
  pinMode(DIGITAL_INPUT_SENSOR1, INPUT);      // sets the motion sensor digital pin as input
  //digitalWrite(DIGITAL_INPUT_SENSOR2, HIGH);  //set defined state
  attachInterrupt(INTERRUPT1, onMotion1, CHANGE);
  pinMode(DIGITAL_INPUT_SENSOR2, INPUT);      // sets the motion sensor digital pin as input
  //digitalWrite(DIGITAL_INPUT_SENSOR2, HIGH); //set defined state
  attachInterrupt(INTERRUPT2, onMotion2, CHANGE);
  for (int sensor = 1, pin = RELAY_1; sensor <= NUMBER_OF_RELAYS; sensor++, pin++) {
    // Then set relay pins in output mode
    pinMode(pin, OUTPUT);
    // Set all relays to off
    digitalWrite(pin, RELAY_OFF);
  };
};

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Garage", "0.3");

  // Register all sensors to gateway (they will be created as child devices)
  present(BARO_CHILD, S_BARO);
  present(TEMP_CHILD, S_TEMP);
  present(CHILD_ID_M1, S_MOTION);
  present(CHILD_ID_M2, S_MOTION);
  present(CHILD_CONFIG, S_CUSTOM);  //
  for (int sensor = 1, pin = RELAY_1; sensor <= NUMBER_OF_RELAYS; sensor++, pin++) {
    // Register all sensors to gw (they will be created as child devices)
    present(sensor, S_BINARY);
  };
};


void setup()
{
  request(CHILD_CONFIG, V_VAR1);
  request(CHILD_CONFIG, V_VAR2);
  //  request(CHILD_CONFIG, V_VAR3);
  metric = getControllerConfig().isMetric;
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  };
};


void loop() {
  loopTime = millis();
  if (loopTime - switchtime > OnTime * 1000 && onOff ) {
    digitalWrite(RELAY_1, RELAY_OFF);
    send(msgRelay.set(false));
    onOff = false;
  }
  if (loopTime - lastBMP > SLEEP_TIME ) {
    lastBMP = loopTime;
    float pressure = bmp.readSealevelPressure(ALTITUDE) / 100.0;
    float temperature = bmp.readTemperature();

    if (!metric)
    {
      // Convert to fahrenheit
      temperature = temperature * 9.0 / 5.0 + 32.0;
    };

    int forecast = sample(pressure);
#ifdef MY_DEBUG
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(metric ? " *C" : " *F");
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");
    Serial.print("Forecast = ");
    Serial.println(weather[forecast]);
#endif

    if (temperature != lastTemp)
    {
      send(tempMsg.set(temperature, 1));
      lastTemp = temperature;
    };

    if (pressure != lastPressure)
    {
      send(pressureMsg.set(pressure, 0));
      lastPressure = pressure;
    };

    if (forecast != lastForecast)
    {
      send(forecastMsg.set(weather[forecast]));
      lastForecast = forecast;
    };

    roundscounter++;
    //check for configuration update every 10 min
    if (roundscounter == 10) {
      request(CHILD_CONFIG, V_VAR1);
      request(CHILD_CONFIG, V_VAR2);
      roundscounter = 0;
    }
    Serial.println(roundscounter);
  }
};

void onMotion1 () {
  // Read digital motion value
  boolean tripped1 = digitalRead(DIGITAL_INPUT_SENSOR1) == HIGH;
  send(msg_M1.set(tripped1 ? "1" : "0")); // Send tripped values to gw
  if (NightMode && tripped1 ) {
    digitalWrite(RELAY_1, RELAY_ON);
    send(msgRelay.set(true));
    onOff = true;
    switchtime = millis();
  };
#ifdef MY_DEBUG
  Serial.println(tripped1);
  Serial.println(switchtime);
  Serial.println(OnTime);
#endif
}

void onMotion2 () {
  // Read digital motion value
  boolean tripped2 = digitalRead(DIGITAL_INPUT_SENSOR2) == HIGH;
#ifdef MY_DEBUG
  Serial.println(tripped2);
#endif
  send(msg_M2.set(tripped2 ? "1" : "0"));
}

void receive(const MyMessage &message) {
  if (message.type == V_STATUS) {
    // Change relay state
    digitalWrite(message.sensor - 1 + RELAY_1, message.getBool() ? RELAY_ON : RELAY_OFF);
    // Write some debug info
#ifdef MY_DEBUG
    Serial.print("Incoming change for sensor:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());
#endif
    if (message.getBool()) {
      switchtime = millis();
    }
  }

  else if (message.type == V_VAR1) {
    if (message.getInt() > 0) OnTime = message.getInt();
#ifdef MY_DEBUG
    Serial.print(F("Received OnTime: "));
    Serial.println(OnTime);
#endif
  }
  else if (message.type == V_VAR2) {
    NightMode = message.getBool();
    if (NightMode == 0 && onOff) {
      digitalWrite(RELAY_1, RELAY_OFF);
      send(msgRelay.set(false));
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

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
  {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}



// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185)
  {
    minuteCount = 6;
  }

  if (minuteCount == 5)
  {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else
    {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) //first time initial 3 hour
    {
      dP_dt = change; //note this is for t = 1 hour
    }
    else
    {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else
    {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else
    {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    }
    else
    {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 3; // note this is for t = 3 hour
    }
    else
    {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
  {
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25))
  {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25)
  {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
  {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt > (-0.05)) && (dP_dt < 0.05))
  {
    forecast = STABLE;
  }
  else
  {
    forecast = UNKNOWN;
  }

  // uncomment when debugging
  //Serial.print(F("Forecast at minute "));
  //Serial.print(minuteCount);
  //Serial.print(F(" dP/dt = "));
  //Serial.print(dP_dt);
  //Serial.print(F("kPa/h --> "));
  //Serial.println(weather[forecast]);

  return forecast;
}
