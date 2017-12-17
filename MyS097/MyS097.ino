/*
 * Changelog: Kommentare zum weiteren Vorgehen eingefügt
 * BME280
 * Standort: Raum hinter Garage
 */

#define SN "BME280"
#define SV "0.2"

#define MY_DEBUG
#define MY_DEBUG_LOCAL
// Enable RS485 transport layer
#define MY_RS485
//#define MY_RS485_HWSERIAL Serial
// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 2
// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 38400 //9600
#define MY_RS485_SOH_COUNT 3

#define MY_NODE_ID 97
#define MY_TRANSPORT_WAIT_READY_MS 20000
#include <Arduino.h>
//#include <SPI.h>

#define MY_BME_ENABLED
#ifdef MY_BME_ENABLED
#include <BME280I2C.h> // From Library Manager, comes with the BME280-lib by Tyler Glenn
#define MY_FORECAST
#endif

#include <Wire.h>
#include <MySensors.h>

//für die millis()-Berechnung, wann wieder gesendet werden soll
unsigned long SEND_FREQUENCY = 180000; // Sleep time between reads (in milliseconds)
unsigned long lastSend = 0;

#define BARO_CHILD 1
#define TEMP_CHILD 2
#define HUM_CHILD 3
BME280I2C bme;
unsigned long lastSendBme = 0;
//#define SEALEVELPRESSURE_HPA (1013.25)


float lastPressure = -1;
float lastTemp = -1;
float lastHum = -1;
float temperature(NAN), humidity(NAN), pressureBme(NAN);
uint8_t pressureUnit(1);                                          
// unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi

MyMessage tempMsg(TEMP_CHILD, V_TEMP);
MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);
MyMessage humMsg(HUM_CHILD, V_HUM);

//bme: Value according to MySensors for forecast accuracy; do not change
unsigned long bmeDelayTime = 60000;

const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
uint8_t lastForecast = -1;
const uint8_t LAST_SAMPLES_COUNT = 5;
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
MyMessage forecastMsg(BARO_CHILD, V_FORECAST);

bool metric = true;

void before()
{
  Wire.begin();
  bme.begin();
}

void presentation() {
  sendSketchInfo(SN, SV);
  present(BARO_CHILD, S_BARO);
  present(TEMP_CHILD, S_TEMP);
  present(HUM_CHILD, S_HUM);
}

void setup() {
  metric = getControllerConfig().isMetric;
}

void loop()
{
  unsigned long currentTime = millis();

  // Only send values at a maximum frequency
  if (currentTime - lastSend > bmeDelayTime) {
    bme.read(pressureBme, temperature, humidity, metric, pressureUnit); //Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
    if (isnan(temperature)) {
#ifdef MY_DEBUG_LOCAL
      Serial.println("Failed reading temperature");
#endif
    } else if (temperature != lastTemp) {
      // Only send temperature if it changed since the last measurement
      lastTemp = temperature;
      send(tempMsg.set(temperature, 1));
#ifdef MY_DEBUG_LOCAL
      Serial.print("T: ");
      Serial.println(temperature);
#endif
    }
    
    if (isnan(humidity)) {
#ifdef MY_DEBUG_LOCAL
      Serial.println("Failed reading humidity");
#endif
    } else if (humidity != lastHum) {
    // Only send humidity if it changed since the last measurement
      lastHum = humidity;
      send(humMsg.set(humidity, 1));
#ifdef MY_DEBUG
      Serial.print("H: ");
      Serial.println(humidity);
#endif
    }

    if (isnan(pressureBme)) {
#ifdef MY_DEBUG_LOCAL
      Serial.println("Failed reading pressure");
#endif
      if (pressureBme != lastPressure) {
        send(pressureMsg.set(pressureBme, 2));
        lastPressure = pressureBme;
      }
    }

    int forecast = sample(pressureBme);
    if (pressureBme != lastPressure) {
      send(pressureMsg.set(pressureBme, 2));
      lastPressure = pressureBme;
    }
    if (forecast != lastForecast){
      send(forecastMsg.set(weather[forecast]));
     lastForecast = forecast;
    }
    lastSend = currentTime;
  }
}

void receive(const MyMessage &message) {
  
}

enum FORECAST
{
    STABLE = 0,            // "Stable Weather Pattern"
    SUNNY = 1,            // "Slowly rising Good Weather", "Clear/Sunny "
    CLOUDY = 2,            // "Slowly falling L-Pressure ", "Cloudy/Rain "
    UNSTABLE = 3,        // "Quickly rising H-Press",     "Not Stable"
    THUNDERSTORM = 4,    // "Quickly falling L-Press",    "Thunderstorm"
    UNKNOWN = 5            // "Unknown (More Time needed)
};

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++) {
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
  else if ((dP_dt >(-0.05)) && (dP_dt < 0.05))
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
