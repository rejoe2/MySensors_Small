/*
  MySesors MultiValve Sketch for use with HG7881 Motor Driver

  This example shows how to drive a valve with using HG7881 (L9110) Dual or Four
  Channel Motor Driver Module.
  
  See info at the end of file for wiring etc..

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
#define DO_BREAK // set motor break on after switching valve
#define KEEP_OPEN 50 // ms to switch from open to close vice versa

#include <MySensors.h>

//für die millis()-Berechnung, wann wieder gesendet werden soll
unsigned long SEND_FREQUENCY = 60000; // Sleep time between reads (in milliseconds)
unsigned long lastSend = 0;

#define VALVE_0_ID 2
#define MAX_VALVES 4
#define ALL_OFF_PIN 4
#define BUTTON_ALL_ID 10

const uint8_t iaPin[] = {5, 6, 10, 11};         //  switch around pins to your desire
//nano pwm: 3, 5, 6, 9, 10, 11
const uint8_t ibPin[] = {A4, A5, A6, A7};       //  switch around pins to your desire
const uint8_t buttonPin[] = { 7, A1, A2, A3};   //  switch around pins to your desire

class Valve             // valve class, store all relevant data (equivalent to struct)
{
  public:
    uint8_t buttonPin;      // physical pin number of button
    uint8_t ibPin;         // physical pin number of direction PIN
    uint8_t iaPin;         // physical pin number of iaPin
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
      debounce[i].attach(buttonPin[i], INPUT_PULLUP);
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

    int value = debounce[i].read();
    
    if ( value == LOW) {
        Valves[i].valveState ? closeValve(i) : openValve(i);
        send(msg[VALVE_0_ID+i].set(Valves[i].valveState ? true : false));
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
  digitalWrite(Valves[valve].iaPin, HIGH);
  digitalWrite(Valves[valve].ibPin, LOW);
  delay(KEEP_OPEN);
#ifdef DO_BREAK
  digitalWrite(Valves[valve].iaPin, HIGH);
  digitalWrite(Valves[valve].ibPin, HIGH);
#else
  digitalWrite(Valves[valve].ibPin, LOW);
  digitalWrite(Valves[valve].iaPin, LOW);
#endif
  Valves[valve].valveState = 0;
}

void openValve( uint8_t valve ) {
  digitalWrite(Valves[valve].iaPin, LOW);
  digitalWrite(Valves[valve].ibPin, HIGH);
  delay(KEEP_OPEN);
#ifdef DO_BREAK
  digitalWrite(Valves[valve].iaPin, HIGH);
  digitalWrite(Valves[valve].ibPin, HIGH);
#else
  digitalWrite(Valves[valve].iaPin, LOW);
  digitalWrite(Valves[valve].ibPin, LOW);
#endif
  Valves[valve].valveState = 1;
}

/*
****
Description from https://www.bananarobotics.com/shop/HG7881-(L9110)-Dual-Channel-Motor-Driver-Module
NOTE: Other docs mention 10V to be the upper voltage limit
****

The HG7881 (L9110S) Dual Channel Motor Driver Module is a compact board that can be used to drive very small robots.

This tiny module has two independent HG7881 (L9110S) motor driver chips which can each drive up 800mA of continuous current. The boards can be operated from 2.5V to 12V enabling this module to be used with both 3.3V and 5V microcontrollers.

A set of male header pins is used to connect this module to your robot's microcontroller brain. The motors are attached via two sets of screw terminals.

A PWM Pulse Width Modulation signal is used to control the speed of a motor and a digital output is used to change its direction. This module can also be used to drive a single four line two phase stepper motor. Four holes make this board easy to mount onto your robot or other project.
Motor Control Interface Pin 	Description
B-IA 	Motor B Input A (IA)
B-IB 	Motor B Input B (IB)
GND 	Ground
VCC 	Operating Voltage 2.5-12V
A-IA 	Motor A Input A (IA)
A-IB 	Motor A Input B (IB)

 
Motor Truth Table 
IA 	IB 	Motor State
L 	L 	Off
H 	L 	Forward
L 	H 	Reverse
H 	H 	Off

We recommend applying a PWM signal to input IA to control the motor speed and a digital output to input IB to control its direction.

Note that the actual direction that "forward" and "reverse" turn your motor will depend on how it is oriented and wired. If your motor spins the wrong way, either swap the motor wires that connect to the output terminals or change the way the IA and IB bits get set in your program.

*******

For pwm pins see https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
=> nano: 3, 5, 6, 9, 10, 11

*/
