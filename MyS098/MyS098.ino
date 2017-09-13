/**
  Zwischenkeller-Node
  - Wasserzähler
  - 3xPIR:
  - 1 Relais
  - 3x1Wire Bus (DS18B20)
  Konfiguration:
  - ID 0, V_VAR1: On-Time
  - ID 0, V_VAR2: Temp-Measure Invervall
*/

// Enable debug prints to serial monitor
#define MY_DEBUG
#define MY_DEBUG_LOCAL

// Enable RS485 transport layer
#define MY_RS485

// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 2

// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600

#define MY_NODE_ID 98
#define MY_TRANSPORT_WAIT_READY_MS 3000
#include <MySensors.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Bounce2.h>

#define MAX_ATTACHED_DS18B20 12
#define MAX_BUS_DS18B20 3
unsigned long TempSendFreqency = 300000; // Sleep time between reads (in milliseconds)
unsigned long lastTemp = 0;

OneWire oneWire[MAX_BUS_DS18B20] = {10, 12, 11}; //{oneWirePins[]} - setup oneWire instances to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors[MAX_BUS_DS18B20]; // Pass the oneWire reference to Dallas Temperature.
float lastTemperature[MAX_ATTACHED_DS18B20];
const uint8_t DS_BUS_START[MAX_BUS_DS18B20 + 1] = {0, 2, 7, MAX_ATTACHED_DS18B20};
boolean receivedConfig = false;
boolean metric = true;
// Initialize temperature message
MyMessage DallasMsg(0, V_TEMP);
int  resolution = 11;
int  conversionTime = 1000;

// arrays to hold device addresses
DeviceAddress dallasAddresses[] = {
  {0x28, 0x28, 0x6F, 0xE5, 0x5, 0x0, 0x0, 0x8A}, // Aussentemperatur Süd 28.286FE5050000.8A
  {0x28, 0xFF, 0xE9, 0x3C, 0x36, 0x16, 0x4, 0x13}, // Schildkröten
  {0x28, 0xFF, 0x20, 0x38, 0xB5, 0x16, 0x5, 0x14}, //#2 VL Neubau
  {0x28, 0xFF, 0xA2, 0xB9, 0xB3, 0x16, 0x3, 0x6},// RL Neubau
  {0x28, 0xFF, 0x12, 0xB8, 0xB3, 0x16, 0x3, 0xC6},// RL Bodenkonvektor
  {0x28, 0xFF, 0x34, 0xA, 0xB5, 0x16, 0x5, 0x38},// VL Bad EG
  {0x28, 0xFF, 0xB6, 0x45, 0xB5, 0x16, 0x5, 0xA},// RL Bad EG
  {0x28, 0xFF, 0xC3, 0x73, 0xC1, 0x16, 0x4, 0xDB},// #7 VL Altbau
  {0x28, 0xFF, 0x97, 0xA6, 0xB3, 0x16, 0x4, 0x71},// RL Altbau
  {0x28, 0xFF, 0xAA, 0x47, 0xB5, 0x16, 0x5, 0x50},// VL WZ Ost
  {0x28, 0xFF, 0xC1, 0x19, 0xB5, 0x16, 0x5, 0xFC},// RL WZ Ost
  {0x28, 0xFF, 0xC3, 0x2E, 0xB3, 0x16, 0x5, 0x60} // RL Wohnzimmer West
};

#define CHILD_ID_CONFIG 0   // Id for Node parameters (onTime)

#define DIGITAL_INPUT_SENSOR 3  // The digital input you attached your water sensor.  (Only 2 and 3 generates interrupt!)
#define PULSE_FACTOR 100       // Nummber of blinks per m3 of your meter (One rotation/10 liter)
#define SLEEP_MODE false        // Watt-value can only be reported when sleep mode is false.
#define MAX_FLOW 100          // Max volume value to report. This filetrs outliers.
#define SENSOR_INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define CHILD_ID_WATER 6              // Id of the sensor child
unsigned long SEND_FREQUENCY = 300000; // Minimum time between send (in milliseconds). We don't want to spam the gateway.

#define FIRST_PIR_ID 2
#define MAX_PIRS 3
const uint8_t pirPin[] = {4, 5, 6};   //  switch around pins to your desire
Bounce debouncer[MAX_PIRS];
MyMessage pirMsg(0, V_TRIPPED);
bool oldPir[MAX_PIRS] = {false};

#define RELAY_PIN  13  // Arduino Digital I/O pin number for relay 
#define CHILD_ID_RELAY 1   // Id of the sensor child
#define RELAY_ON 0
#define RELAY_OFF 1

unsigned long pirOnTime = 30000; //time pir sets light on
unsigned long lastSwitchOn = 0;

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

unsigned long lastTempSend = 0;

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
  for (uint8_t i = 0; i < MAX_BUS_DS18B20; i++) {
    sensors[i].setOneWire(&oneWire[i]);
    sensors[i].begin();

    // requestTemperatures() will not block current thread
    sensors[i].setWaitForConversion(false);

    // Fetch the number of attached temperature sensors and set resolution
    for (uint8_t j = 0; j < sensors[i].getDeviceCount(); j++) {
      sensors[i].setResolution(j, resolution);
    }
  }
  for (uint8_t i = 0; i < MAX_PIRS; i++) {
    debouncer[i] = Bounce();                        // initialize debouncer
    debouncer[i].attach(pirPin[i], INPUT_PULLUP);
    debouncer[i].interval(5);
  }
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Zwischenkeller", "0.1");
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_WATER, S_WATER);
  present(CHILD_ID_RELAY, S_LIGHT);
  present(CHILD_ID_CONFIG, S_CUSTOM);

  for (int i = 0; i < MAX_ATTACHED_DS18B20; i++) { //i < numSensors &&
    present(20 + i, S_TEMP);
  }
  for (int i = 0; i < MAX_PIRS; i++) { //i < numSensors &&
    present(FIRST_PIR_ID + i, S_MOTION);
  }
}

void setup() {
  // Fetch last known pulse count value from gw
  request(CHILD_ID_WATER, V_VAR1);
  request(CHILD_ID_CONFIG, V_VAR1);
  request(CHILD_ID_CONFIG, V_VAR2);
  request(CHILD_ID_CONFIG, V_VAR3);
  request(CHILD_ID_CONFIG, V_VAR4);
  request(CHILD_ID_CONFIG, V_VAR5);
  lastSend = lastPulse = millis();
}

void loop()
{
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

  //Switch Relay off after motion has stopped
  if (currentTime - lastSwitchOn > pirOnTime) {
    if (digitalRead(RELAY_PIN) == RELAY_ON ) {
      // Send in the new temperature
      digitalWrite(RELAY_PIN, RELAY_OFF);
      send(RelayMsg.set(false)); // Send new state and request ack back
    }
  }


  //Loop for regular temperature sensing
  if (currentTime - lastTemp > TempSendFreqency) {
    for (int j = 0; j < MAX_BUS_DS18B20; j++) { 
      sensors[j].requestTemperatures();
      wait(200);
    }
  //  wait(conversionTime);
  }
  if (currentTime - lastTemp - conversionTime > TempSendFreqency) {
    // Read temperatures and send them to controller
    for (int j = 0; j < MAX_BUS_DS18B20; j++) { 
      for (int i = DS_BUS_START[j]; i < DS_BUS_START[j + 1]; i++) { 
        float temperature = static_cast<float>(static_cast<int>(sensors[j].getTempC(dallasAddresses[i]) * 10.)) / 10.;
        if ( temperature != -127.00 && temperature != 85.00) {
          // Send in the new temperature
          send(DallasMsg.setSensor(i + 20).set(temperature, 1));
          // Save new temperatures for next compare
          lastTemperature[i] = temperature;
#ifdef MY_DEBUG_LOCAL
          // Write some debug info
          Serial.print(F("Temperature "));
          Serial.print(i);
          Serial.print(" : ");
          Serial.println(temperature);
#endif
        }
      }
      wait(500);
    }
    lastTemp = currentTime;
  }
}

void receive(const MyMessage & message) {
  if (message.sensor == CHILD_ID_WATER) {
    if (message.type == V_VAR1) {
      if (pcReceived){
        pulseCount = message.getULong();
        flow = oldflow = 0;
      } else {
        pulseCount = pulseCount+message.getULong();
      }
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
      lastSwitchOn = millis();
#ifdef MY_DEBUG
      // Write some debug info
      Serial.print(F("Gw change relay:"));
      Serial.print(message.sensor);
      Serial.print(F(", New status: "));
      Serial.println(message.getBool());
#endif
    }
  }
  else if (message.sensor == CHILD_ID_CONFIG) {
    if (message.type == V_VAR1) {
      pirOnTime = message.getInt() * 1000; //set new onTime for pir
    }
    else if (message.type == V_VAR2) {
      if (message.getInt() > 10) {
        TempSendFreqency = message.getInt() * 1000; //set new temp meassure intervall
      }
    }
    else if (message.type == V_VAR3) {

    }
    else if (message.type == V_VAR4) {

    }
    else if (message.type == V_VAR5) {

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

