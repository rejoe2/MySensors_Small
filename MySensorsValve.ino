/*
  MySesors MultiValve Sketch for use with HG7881 Motor Driver

  This example shows how to drive a valve with using HG7881 (L9110) Dual or Four
  Channel Motor Driver Module.

  Connections:
   (e.g., first valve)
    Arduino digital output "dirPin[0]" to motor driver input A-IA.
    Arduino digital output "pwmPin[0]" to motor driver input A-IB.
    Motor driver VCC to operating voltage (e.g. 3V for valves from Pearl origin).
    Motor driver GND to common ground.
    Motor driver MOTOR A screw terminals to a single valve.
*/

#define SN "MultiValve"
#define SV "0.0.1"

//#define MY_DEBUG
//#define MY_DEBUG_LOCAL //Für lokale Debug-Ausgaben
#define MY_DEBUG_ACTUAL //Für lokale Debug-Ausgaben
// Enable RS485 transport layer
#define MY_RS485
//#define MY_RS485_HWSERIAL Serial
// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 2
// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 19200 //57600 //38400 //9600
#define MY_RS485_SOH_COUNT 1
#define MY_RS485_HWSERIAL Serial
#define MY_SPLASH_SCREEN_DISABLED
#define MY_TRANSPORT_WAIT_READY_MS 3000
#define MY_NODE_ID 111
#include <Bounce2.h>

#include <MySensors.h>

//für die millis()-Berechnung, wann wieder gesendet werden soll
unsigned long SEND_FREQUENCY = 60000; // Sleep time between reads (in milliseconds)
unsigned long lastSend = 0;

#define VALVE_0_ID 2
#define MAX_VALVES 4
#define ALL_OFF_PIN 4
#define BUTTON_ALL_ID 10

const uint8_t dirPin[] = {13, A1, A3, A5};    //  switch around pins to your desire
const uint8_t pwmPin[] = {A0, A2, A4, A6};    //  switch around pins to your desire
const uint8_t buttonPin[] = { 5, 6, 7, 10};   //  switch around pins to your desire

class Valve             // valve class, store all relevant data (equivalent to struct)
{
  public:
    uint8_t buttonPin;      // physical pin number of button
    uint8_t dirPin;         // physical pin number of direction PIN
    uint8_t pwmPin;         // physical pin number of relay
    bool    valveState;    // valve status 
};

Valve Valves[MAX_VALVES];
MyMessage msg[MAX_VALVES];
MyMessage buttonMsg(BUTTON_ALL_ID, V_TRIPPED);

Bounce debounce[MAX_VALVES];

// INSTANTIATE A Button OBJECT FROM THE Bounce2 NAMESPACE
Bounce2::Button button = Bounce2::Button();

#define DIGITAL_INPUT_SENSOR 3  // The digital input you attached your water sensor.  (Only 2 and 3 generates interrupt!)
#define PULSE_FACTOR 100       // Nummber of blinks per m3 of your meter (One rotation/10 liter)
#define SLEEP_MODE false        // Watt-value can only be reported when sleep mode is false.
#define MAX_FLOW 100          // Max volume value to report. This filetrs outliers.
#define SENSOR_INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define CHILD_ID_WATER 20              // Id of the sensor child
unsigned long SEND_FREQUENCY_FLOW = 300000; // Minimum time between send (in milliseconds). We don't want to spam the gateway.
MyMessage flowMsg(CHILD_ID_WATER, V_FLOW);
MyMessage volumeMsg(CHILD_ID_WATER, V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID_WATER, V_VAR1);
double ppl = ((double)PULSE_FACTOR) / 1000;      // Pulses per liter

bool metric = true;

void before()
{
  // Initialize In-/Outputs
  for (uint8_t i = 0; i < MAX_VALVES; i++) {
      closeValve(i);
      debounce[i] = Bounce();
      debounce[i].attach(INPUT_PINS[i], INPUT_PULLUP);
      debounce[i].interval(5);
  }

  button.attach( ALL_OFF_PIN , INPUT_PULLUP ); // USE INTERNAL PULL-UP
  button.interval(5);
  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  button.setPressedState(LOW); 
}

void presentation() {
  sendSketchInfo(SN, SV);
  present(BUTTON_ALL_ID, S_SPRINKLER, "all_off_button");
  for (uint8_t i = 0; i < MAX_VALVES; i++) {
    present(VALVE_0_ID+i, S_SPRINKLER);
  }
}

void setup() {
  metric = getControllerConfig().isMetric;
}

void loop()
{
  button.update();
  if ( button.read() == LOW ) {
    for (uint8_t i = 0; i < MAX_VALVES; i++) {
        closeValve(i);
        send(msg[VALVE_0_ID+i].set(Valves[i].valveState ? true : false));
    }
  }

  for (uint8_t i = 0; i < MAX_VALVES; i++) {
    debounce[i].update();

    int value = debouncer[i].read();
    
    if ( value == LOW) {
        Valves[i].relayState ? closeValve(i) : openValve(i);
        send(msg[VALVE_0_ID+i].set(Valves[i].relayState ? true : false));
    }
  }

  unsigned long currentTime = millis();

}

void receive(const MyMessage &message) {
  if ( message.sensor >= VALVE_0_ID && message.sensor <= VALVE_0_ID+MAX_VALVES ) {
    uint8_t valve_i = message.sensor-VALVE_0_ID;
    if (message.isAck()) {
#ifdef MY_DEBUG_LOCAL
    Serial.println(F("Ack child1 from gw rec."));
#endif
    }
    if (message.type == V_LIGHT) {
      message.getBool() ? openValve(valve_i) : closeValve(valve_i);
    }
  }

}

void closeValve( uint8_t valve ) {
  digitalWrite(Valves[valve].dirPin, LOW);
  digitalWrite(Valves[valve].pwmPin, HIGH);
  delay(20);
  digitalWrite(Valves[valve].pwmPin, LOW);
  Valves[valve].valveState = 0;
}

void openValve( uint8_t valve ) {
  digitalWrite(Valves[valve].dirPin, HIGH);
  digitalWrite(Valves[valve].pwmPin, HIGH);
  delay(20);
  digitalWrite(Valves[valve].dirPin, LOW);
  digitalWrite(Valves[valve].pwmPin, LOW);
  Valves[valve].valveState = 1;
}

