/**
  Zwischenkeller-Node
  - Wasserzähler
  - 3xPIR:
  - 1 Relais
  - 2 DS18B20	
 Konfiguration:
  - ID 0, V_VAR1: On-Time
*/

// Enable debug prints to serial monitor
#define MY_DEBUG
//#define MY_DEBUG_LOCAL

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

#define MAX_ATTACHED_DS18B20 10
unsigned long SLEEP_TIME = 300000; // Sleep time between reads (in milliseconds)
unsigned long lastTemp = 0;

OneWire oneWire[2] = {10,11}; //{oneWirePins[]}setup oneWire instances to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors[2]; // Pass the oneWire reference to Dallas Temperature.
float lastTemperature[MAX_ATTACHED_DS18B20];
//int numSensors = 0;
boolean receivedConfig = false;
boolean metric = true;
// Initialize temperature message
MyMessage DallasMsg(0, V_TEMP);
int  resolution = 10;
int  conversionTime = 1000;

// arrays to hold device addresses
DeviceAddress dallasAddresses[0][] = {
    {0x28, 0x28, 0x6F, 0xE5, 0x5, 0x0, 0x0, 0x8A}, // Aussentemperatur Süd 28.286FE5050000.8A
  {0x28, 0x42, 0x6F, 0xE5, 0x5, 0x0, 0x0, 0x86} // Schildkröten 28.426FE5050000.86*/
};

int AS = 0; //Aussentemperatur Sued => [0] => PIN 10
int TS = 1; //Schidkroeten => [0] => PIN 10

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

#define RELAY_PIN  12  // Arduino Digital I/O pin number for relay 
#define CHILD_ID_RELAY 1   // Id of the sensor child
#define RELAY_ON 0
#define RELAY_OFF 1

unsigned long pirOnTime = 30000; //Check cyclus for fast temperature rise
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

unsigned long lastTempSend= 0;

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
  for (uint8_t i=0; i<3; i++) {
    sensors[i].setOneWire(&oneWire[i]);
    sensors[i].begin();

  // requestTemperatures() will not block current thread
    sensors[i].setWaitForConversion(false);

    // Fetch the number of attached temperature sensors and set resolution
    for (uint8_t j=0; j < sensors[i].getDeviceCount(); j++) {
      sensors[i].setResolution(j, resolution);
    }
  }
  for (uint8_t i=0; i<MAX_PIRS; i++) {
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
  
  // Present all sensors to controller
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
        debounce[i].update();
		pir[i] = debounce[i].read() == LOW;
		if (pir[i] != oldPir[i]) {
			send(pirMsg.set(FIRST_PIR_ID+i, pir[i]));  // Send tripped value to gw
			if (i == 0 && pir[i]) {
				if (digitalRead(RELAY_PIN) = RELAY_OFF ) {
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

  //Switch Relay off after motion has stopped
  if (currentTime - lastSwitchOn> pirOnTime) {
    if (digitalRead(RELAY_PIN) = RELAY_ON ) {
      // Send in the new temperature
      digitalWrite(RELAY_PIN, RELAY_OFF);
      send(RelayMsg.set(false)); // Send new state and request ack back
    }
  }

  //Loop for regular temperature sensing
  if (currentTime - lastTemp > SLEEP_TIME) {
	sensors[0].requestTemperatures();
    sensors[1].requestTemperatures();
    wait(conversionTime);
    // Read temperatures and send them to controller
    //First bus: Manual
	for (int j = 0; j < 2; i++) { //i < numSensors &&
		float temperature = static_cast<float>(static_cast<int>(sensors[j].getTempC(dallasAddresses[0]) * 10.)) / 10.;
		if ( temperature != -127.00 && temperature != 85.00) {
			// Send in the new temperature
			send(DallasMsg.setSensor(j + 20).set(temperature, 1));
			// Save new temperatures for next compare
          lastTemperature[j] = temperature;
#ifdef MY_DEBUG_LOCAL
          // Write some debug info
          Serial.print("Temperature ");
          Serial.print(i);
          Serial.print(" : ");
          Serial.println(temperature);
#endif
        }
	}
	//second bus
	for (int i = 3; i < MAX_ATTACHED_DS18B20; i++) { //i < numSensors &&
		// Fetch and round temperature to one decimal
		float temperature = static_cast<float>(static_cast<int>(sensors[1].getTempC(dallasAddresses[i]) * 10.)) / 10.;
        
		// Only send data if temperature has no error
		if ( temperature != -127.00 && temperature != 85.00) {
			// Send in the new temperature
			send(DallasMsg.setSensor(i + 20).set(temperature, 1));
			// Save new temperatures for next compare
			lastTemperature[i] = temperature;
#ifdef MY_DEBUG_LOCAL
			// Write some debug info
			Serial.print("Temperature ");
			Serial.print(i);
			Serial.print(" : ");
			Serial.println(temperature);
#endif
		}
    }
    lastTemp = currentTime;
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
	  onTime = millis();
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
      onTime = message.getInt(); //set new onTime for pir
    }
    else if (message.type == V_VAR2) {
      
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

void internalServo(int val1)
{
  myservo.attach(SERVO_DIGITAL_OUT_PIN);
  attachedServo = true;
  //myservo.write(SERVO_MAX + (SERVO_MIN - SERVO_MAX) / 100 * val1); // sets the servo position 0-180
  myservo.write(SERVO_MAX + (SERVO_MIN - SERVO_MAX) / 100 * (130 - val1 * 40)); // sets the servo position 0-180
  send(ServoMsg.set(val1));
  timeOfLastChange = millis();
}