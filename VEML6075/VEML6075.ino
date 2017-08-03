/*
Original by scalz@mysensors: https://forum.mysensors.org/topic/6952/veml6070-and-veml6075-uv-sensors/25#
Needs a modified VEML6075-lib: https://github.com/rejoe2/VEML6075
*/
// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

//#define MY_NODE_ID 10

#include <MySensors.h>

#include <VEML6075.h>


VEML6075 veml6075 = VEML6075();
bool sensorFound = false;

#define CHILD_ID_UVI 1
#define CHILD_ID_UVA 2
#define CHILD_ID_UVB 3

uint32_t SLEEP_TIME = 30*1000; // Sleep time between reads (in milliseconds)

MyMessage uviMsg(CHILD_ID_UVI, V_UV);
MyMessage uvaMsg(CHILD_ID_UVA, V_UV);
MyMessage uvbMsg(CHILD_ID_UVB, V_UV);


void presentation()
{
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo("UV Sensor", "1.3");

    // Register all sensors to gateway (they will be created as child devices)
    present(CHILD_ID_UVI, S_UV);
    present(CHILD_ID_UVA, S_UV);
    present(CHILD_ID_UVB, S_UV);
}

void setup()
{
  if (!veml6075.begin()) {
    Serial.println(F("VEML6075 not found!"));
  } else 
    sensorFound = true;
#ifdef MY_DEBUG
    uint16_t devid = veml6075.getDevID();
    Serial.print(F("Device ID = "));
    Serial.println(devid, HEX);
    Serial.println(F("----------------"));
#endif
}

void loop()
{
  if (sensorFound) {
    unsigned long lastSend =0;
    float uvi;
    float uva;
    float uvb;
    static float uviOld = -1;
    static float uvaOld = -1;
    static float uvbOld = -1;
    
    veml6075.sleep(false); // power up veml6075

    // Poll sensor
    veml6075.poll();

    uva = veml6075.getUVA();
    uvb = veml6075.getUVB();
    uvi = veml6075.getUVIndex();
    
#ifdef MY_DEBUG    
    Serial.print(F("UVA = "));
    Serial.println(uva, 2);
    Serial.print(F("UVB = "));
    Serial.println(uvb, 2);
    Serial.print(F("UV Index = "));
    Serial.println(uvi, 1);
endif

    if (uvi != uviOld) {
        send(uviMsg.set(uvi,2));
        uviOld = uvi;
    }
    if (uva != uvaOld) {
        send(uvaMsg.set(uva,2));
        uvaOld = uva;
    }
    if (uvb != uvbOld) {
        send(uvbMsg.set(uvb,2));
        uvbOld = uvb;
    }    
  }
  veml6075.sleep(true); // power down veml6075
  sleep(SLEEP_TIME);
}
