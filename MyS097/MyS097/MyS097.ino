/**
  3-fach Motion
  2x1Wire bus
  Wasserzähler 
  Relays
  
  Konfiguration:
  - ID 0, V_VAR1: autoMode = 0; 0=Manual Mode, 1=Switch internal pump automatically according to external and internal temperatures
  - ID 102:
  -- V_VAR1: tempMaxPump = 45 = upper temp level at warmwater circulation; pump will be switched off if higher temp is measured
  -- V_VAR2: tempMaxHeatingPump = 75 = emergency temperature to switch internal heating pump to highest level
  -- V_VAR3: tempLowExtToLevelIII = 0 = External low temperature to switch internal heating pump to highest level
  -- V_VAR4: tempLowExtToLevelII = 11 = External highest temperature to switch internal heating pump from lowest to medium level; Hysteresis for level I/II: +/- 0,5 degree
  -- V_VAR5: lastPumpSwitch = if true: Reset timer (Heartbeat functionality) for autoMode to prevent automatic switches for the next hour
*/

// Enable debug prints to serial monitor
#define MY_DEBUG
//#define MY_DEBUG_LOCAL

// Enable and select radio type attached
//#define MY_RADIO_NRF24
//#define MY_RF24_PA_LEVEL RF24_PA_MAX
//#define MY_RADIO_RFM69

// Enable RS485 transport layer
#define MY_RS485

// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 2

// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600
#define MY_NODE_ID 97
#define MY_TRANSPORT_WAIT_READY_MS 3000
#include <MySensors.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#define ONE_WIRE_BUS[2]
const uint8_t oneWirePins[3]= {5,6}; // Pin where dallas sensor is connected 
/* 10 = 
 * 11 = 
 * 12 = Warmwasser
 */
#define MAX_ATTACHED_DS18B20 8
unsigned long SLEEP_TIME = 300000; // Sleep time between reads (in milliseconds)
unsigned long relaysOnTime = 90000; //Check cyclus for fast temperature rise

OneWire oneWire[3] = {10,11}; //{oneWirePins[]}; // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors[3]; // Pass the oneWire reference to Dallas Temperature.
float lastTemperature[MAX_ATTACHED_DS18B20];
//int numSensors = 0;
boolean receivedConfig = false;
boolean metric = true;
// Initialize temperature message
MyMessage DallasMsg(0, V_TEMP);
int  resolution = 10;
int  conversionTime = 1000;

uint8_t DSStartAddress[2] = {20,30};

// arrays to hold device addresses
DeviceAddress dallasAddresses[] = {
  {0x28, 0x28, 0x6F, 0xE5, 0x5, 0x0, 0x0, 0x8A}, // Aussentemperatur Süd 28.286FE5050000.8A
  {0x28, 0x42, 0x6F, 0xE5, 0x5, 0x0, 0x0, 0x86} // Schildkröten 28.426FE5050000.86*/
  /*{0x28, 0xFF, 0x36, 0x98, 0x54, 0x14, 0x1, 0xC1}, // Sensor Internal Heating Pump
  {0x28, 0xF8, 0x24, 0xE5, 0x5, 0x0, 0x0, 0xD7}, // Warmwasser 28.F824E5050000.D7
  {0x28, 0xF9, 0x61, 0xE7, 0x5, 0x0, 0x0, 0xDC}, //{0x28, 0xF8, 0x24, 0xE5, 0x05, 0x0, 0x0, 0xDC}, // Warmwasser an der Umwälzpumpe 28.F961E7050000.DC
  {0x28, 0xFF, 0x0, 0x4A, 0x54, 0x14, 0x1, 0xDF}, // Heizung Rücklauf 28.FF004A541401.DF
  {0x28, 0xFF, 0xCE, 0x69, 0x36, 0x16, 0x4, 0xE3}, // Heizung Vorlauf 28FFCE69361604E3
  {0x28, 0xDC, 0x1A, 0xE6, 0x5, 0x0, 0x0, 0x13}, // Aussentemperatur Nord DC1AE6050000.13*/
  
};
int AS = 0; //Aussentemperatur Sued => [0] => PIN 10
int TS = 1; //Schidkroeten => [0] => PIN 10

/*int HP = 0; //Internal Heating Pump => [2] => PIN 12
int WW = 1; //binary # of warmwater sensor  => [2]=> PIN 12
int WP = 2; //WarmWater Pump => [1] => PIN 11
int RL = 3; //Rücklauf => [0] => PIN 10
int VL = 4; //Vorlauf => [0] => PIN 10
int AN = 5; //Aussentemperatur Nord => [0] => PIN 10*/

//#define CHILD_ID_CONFIG 102   // Id for the temp-settings
#define CHILD_ID_CONFIG 0   // Id for automatic mode
boolean autoMode = 0; //0=Manual Mode, 1=Switch internal pump automatically according to external and internal temperatures

#define DIGITAL_INPUT_SENSOR 3  // The digital input you attached your light sensor.  (Only 2 and 3 generates interrupt!)
#define PULSE_FACTOR 100       // Nummber of blinks per m3 of your meter (One rotation/10 liter)
#define SLEEP_MODE false        // Watt-value can only be reported when sleep mode is false.
#define MAX_FLOW 100          // Max volume value to report. This filetrs outliers.
#define SENSOR_INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define CHILD_ID_WATER 0              // Id of the sensor child
unsigned long SEND_FREQUENCY = 300000; // Minimum time between send (in milliseconds). We don't want to spam the gateway.

#define RELAY_PIN  4  // Arduino Digital I/O pin number for relay 
#define CHILD_ID_RELAY 1   // Id of the sensor child
#define RELAY_ON 0
#define RELAY_OFF 1

unsigned long tempDelayPump = 30000;

MyMessage flowMsg(CHILD_ID_WATER, V_FLOW);
MyMessage volumeMsg(CHILD_ID_WATER, V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID_WATER, V_VAR1);

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

unsigned long lastCheckWater = 0;
unsigned long lastCheckPump = 0;
unsigned long lastCheckHeatingPump = 0;
unsigned long lastTempAll = 0;
int DeltaL3 = 14; //Temperature Delta for switching to internal pump level 3
int DeltaL2 = 11; //Temperature Delta for switching to internal pump level 3
unsigned long waitTimePumpSwitch = 600000; //10 min as minimum time to switch internal pump
unsigned long lastPumpSwitch = 0; //use this also for heartbeat?

//set defaults
int tempMaxPump = 45; //upper temp level at warmwater circle pump
int tempMaxHeatingPump = 75; //temperature to switch internal heating pump to highest level
int tempLowExtToLevelIII = 0; //External low temperature to switch internal heating pump to highest level
int tempLowExtToLevelII = 11; ////External highest temperature to switch internal heating pump to medium level
int val = 2;

int oldValue = 0;
bool state;
MyMessage RelayMsg(CHILD_ID_RELAY, V_LIGHT);

void before() {

  // initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion)
  pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);
  digitalWrite(DIGITAL_INPUT_SENSOR, HIGH);
  pulseCount = oldPulseCount = 0;
  attachInterrupt(SENSOR_INTERRUPT, onPulse, FALLING);

  // Then set relay pins in output mode
  pinMode(RELAY_PIN, OUTPUT);
  // Switch water pump off when starting up
  digitalWrite(RELAY_PIN, RELAY_OFF);

  conversionTime = 750 / (1 << (12 - resolution));
  // Startup up the OneWire library
  for (uint8_t i=0; i<2; i++) {
    sensors[i].setOneWire(&oneWire[i]);
    sensors[i].begin();

  // requestTemperatures() will not block current thread
    sensors[i].setWaitForConversion(false);

    // Fetch the number of attached temperature sensors and set resolution
    for (uint8_t j=0; j < sensors[i].getDeviceCount(); j++) {
      sensors[i].setResolution(j, resolution);
    }
  }
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Keller", "0.01");
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_WATER, S_WATER);
  present(CHILD_ID_RELAY, S_LIGHT);
  present(CHILD_ID_CONFIG, S_CUSTOM);
  //present(CHILD_ID_CONFIG0, S_CUSTOM); //for automatic mode

  // Present all sensors to controller
  for (int i = 0; i < 2; i++) { //i < numSensors &&
    for (uint8_t j=0; j < sensors[i].getDeviceCount(); j++) {
      present(DSStartAddress[i] + j, S_TEMP);
    }
  }
}

void setup() {
  // Fetch last known pulse count value from gw
  request(CHILD_ID_WATER, V_VAR1);
  request(CHILD_ID_CONFIG, V_VAR1);
  request(CHILD_ID_CONFIG, V_VAR2);
  request(CHILD_ID_CONFIG, V_VAR3);
  request(CHILD_ID_CONFIG, V_VAR4);
  lastSend = lastPulse = lastCheckWater = lastCheckPump = lastCheckHeatingPump = lastTempAll = millis();
}

void loop()
{
  unsigned long currentTime = millis();

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
#ifdef MY_DEBUG_LOCAL
      Serial.print("pulsecnt:");
      Serial.println(pulseCount);
#endif
      send(lastCounterMsg.set(pulseCount));                  // Send  pulsecount value to gw in VAR1

      double volume = ((double)pulseCount / ((double)PULSE_FACTOR));
      if (volume != oldvolume) {
        oldvolume = volume;
#ifdef MY_DEBUG_LOCAL
        Serial.print("vol:");
        Serial.println(volume, 3);
#endif
        send(volumeMsg.set(volume, 3));               // Send volume value to gw
      }
    }
  }

/*    if (digitalRead(RELAY_PIN) != RELAY_ON && lastTemperature[WW] + 0.2 < temperature && temperature != -127.00 && temperature != 85.00 && lastTemperature[WP] < tempMaxPump) {
      // Send in the new temperature
      send(DallasMsg.setSensor(WW + 20).set(temperature, 1));
      digitalWrite(RELAY_PIN, RELAY_ON);
      send(RelayMsg.set(true)); // Send new state and request ack back
      // Write some debug info
#ifdef MY_DEBUG
      Serial.print("Int. change:");
      Serial.print(temperature);
      Serial.print("C, Rel. status: ON");
      Serial.println();
#endif
    }*/

/*    if (temperature != -127.00 && temperature != 85.00) lastTemperature[WW] = temperature;
    lastCheckWater = currentMillisShort;

    // Fetch and round temperature to one decimal
    temperature = static_cast<float>(static_cast<int>(sensors[2].getTempC(dallasAddresses[HP]) * 10.)) / 10.;

    //Emergency switching function heating pump
    //switch heating pump to max, if temperature at exit of burner is above warning level
    if (val < 3 && temperature != -127.00 && temperature != 85.00 && temperature > tempMaxHeatingPump) {
      // Send in the new temperature
      send(DallasMsg.setSensor(HP + 20).set(temperature, 1));
      val = 3;
      internalServo(val);
      // Write some debug info
#ifdef MY_DEBUG_LOCAL
      Serial.print("Int. change:");
      Serial.print(temperature);
      Serial.print("C, Servo. status: ");
      Serial.print(val);
      Serial.println();
#endif
    }
    lastTemperature[HP] = temperature;
  }*/


  //Loop for switching off relay
  
  unsigned long currentMillisPump = millis();
  if (digitalRead(RELAY_PIN) != RELAY_ON && currentMillisPump - lastCheckPump > tempDelayPump) {
    // switch Relay off, if it's on
    if (digitalRead(RELAY_PIN) != RELAY_OFF) {
      // Send in the new temperature
      digitalWrite(RELAY_PIN, RELAY_OFF);
      send(RelayMsg.set(false)); // Send new state
#ifdef MY_DEBUG_LOCAL
      // Write some debug info
      Serial.println("Int. change: status: OFF");
#endif
    }
  }

  //Loop for regular temperature sensing
  unsigned long currentMillis = millis();
  if (currentMillis - lastTempAll > SLEEP_TIME) {
    sensors[0].requestTemperatures();
    wait(conversionTime);
  
    /*
    // Read temperatures and send them to controller
    for (int i = 3; i < MAX_ATTACHED_DS18B20; i++) { //i < numSensors &&

      // Fetch and round temperature to one decimal
      //if (sensors[0].getAddress(dallasAddresses[i], 0)) {
        float temperature = static_cast<float>(static_cast<int>(sensors[0].getTempC(dallasAddresses[i]) * 10.)) / 10.;
        //float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;

        // Only send data if temperature has no error
        if ( temperature != -127.00 && temperature != 85.00) {
          // Send in the new temperature
          send(DallasMsg.setSensor(i + 20).set(temperature, 1));
          // Save new temperatures for next compare
          lastTemperature[i] = temperature;
          //wait(40);
#ifdef MY_DEBUG_LOCAL
          // Write some debug info
          Serial.print("Temperature ");
          Serial.print(i);
          Serial.print(" : ");
          Serial.println(temperature);
#endif
        }
      //}
    }
    //Send remaining Temps?
    send(DallasMsg.setSensor(WP + 20).set(lastTemperature[WP], 1));
    send(DallasMsg.setSensor(WW + 20).set(lastTemperature[WW], 1));
    send(DallasMsg.setSensor(HP + 20).set(lastTemperature[HP], 1));
    lastTempAll = currentMillis;*/
  }
}

void receive(const MyMessage & message) {
  if (message.sensor == CHILD_ID_WATER) {
    if (message.type == V_VAR1) {
      pulseCount = message.getULong();
      flow = oldflow = 0;
      //Serial.print("Rec. last pulse count from gw:");
      //Serial.println(pulseCount);
      pcReceived = true;
    }
  }
  else if (message.sensor == CHILD_ID_RELAY) {

    if (message.type == V_LIGHT) {
      // Change relay state
      state = message.getBool();
      digitalWrite(RELAY_PIN, state ? RELAY_ON : RELAY_OFF);
#ifdef MY_DEBUG
      // Write some debug info
      Serial.print("Gw change relay:");
      Serial.print(message.sensor);
      Serial.print(", New status: ");
      Serial.println(message.getBool());
#endif
    }
  }
  else if (message.sensor == CHILD_ID_CONFIG) {
    if (message.type == V_VAR1) {
      int tempMaxPump = message.getInt(); //upper temp level at warmwater circle pump
    }
    else if (message.type == V_VAR2) {
      int tempMaxHeatingPump = message.getInt(); //temperature to switch internal heating pump to highest level
    }
    else if (message.type == V_VAR3) {
      int tempLowExtToLevelIII = message.getInt(); //External low temperature to switch internal heating pump to highest level
    }
    else if (message.type == V_VAR4) {
      int tempLowExtToLevelII = message.getInt(); //External highest temperature to switch internal heating pump to medium level
    }
    else if (message.type == V_VAR5) {
      if (message.getBool()) {
        lastPumpSwitch = millis(); //if true: Reset timer (Heartbeat functionality)
#ifdef MY_DEBUG_LOCAL
        // Write some debug info
        Serial.print("Timer reset to ");
        Serial.print(lastPumpSwitch);
#endif
      } else {
        lastPumpSwitch = 0; //if false: switch immediately if necessary
#ifdef MY_DEBUG_LOCAL
        // Write some debug info
        Serial.print("Timer deleted, lastPumpSwitch=");
        Serial.print(lastPumpSwitch);
#endif
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
}


