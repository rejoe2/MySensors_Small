/*
   --------------------------------------------------------------------------------------------------------------------
   Example sketch/program showing An Arduino Door Access Control featuring RFID, EEPROM, Relay
   --------------------------------------------------------------------------------------------------------------------
   This is a MFRC522 library example; for further details and other examples see: https://github.com/miguelbalboa/rfid

   This example showing a complete Door Access Control System

  Simple Work Flow (not limited to) :
                                     +---------+
  +----------------------------------->READ TAGS+^------------------------------------------+
  |                              +--------------------+                                     |
  |                              |                    |                                     |
  |                              |                    |                                     |
  |                         +----v-----+        +-----v----+                                |
  |                         |MASTER TAG|        |OTHER TAGS|                                |
  |                         +--+-------+        ++-------------+                            |
  |                            |                 |             |                            |
  |                            |                 |             |                            |
  |                      +-----v---+        +----v----+   +----v------+                     |
  |         +------------+READ TAGS+---+    |KNOWN TAG|   |UNKNOWN TAG|                     |
  |         |            +-+-------+   |    +-----------+ +------------------+              |
  |         |              |           |                |                    |              |
  |    +----v-----+   +----v----+   +--v--------+     +-v----------+  +------v----+         |
  |    |MASTER TAG|   |KNOWN TAG|   |UNKNOWN TAG|     |GRANT ACCESS|  |DENY ACCESS|         |
  |    +----------+   +---+-----+   +-----+-----+     +-----+------+  +-----+-----+         |
  |                       |               |                 |               |               |
  |       +----+     +----v------+     +--v---+             |               +--------------->
  +-------+EXIT|     |DELETE FROM|     |ADD TO|             |                               |
        +----+     |  EEPROM   |     |EEPROM|             |                               |
                   +-----------+     +------+             +-------------------------------+


   Use a Master Card which is act as Programmer then you can able to choose card holders who will granted access or not

 * **Easy User Interface**

   Just one RFID tag needed whether Delete or Add Tags. You can choose to use Leds for output or Serial LCD module to inform users.

 * **Stores Information on EEPROM**

   Information stored on non volatile Arduino's EEPROM memory to preserve Users' tag and Master Card. No Information lost
   if power lost. EEPROM has unlimited Read cycle but roughly 100,000 limited Write cycle.

 * **Security**
   To keep it simple we are going to use Tag's Unique IDs. It's simple and not hacker proof.

   @license Released into the public domain.

   Typical pin layout used:
   -----------------------------------------------------------------------------------------
               MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
               Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
   Signal      Pin          Pin           Pin       Pin        Pin              Pin
   -----------------------------------------------------------------------------------------
   RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
   SPI SS      SDA(SS)      10            53        D10        10               10
   SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
   SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
   SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
*/

#define SN "RFID Reader"
#define SV "0.1"

#include <EEPROM.h>     // We are going to read and write PICC's UIDs from/to EEPROM
#include <SPI.h>        // RC522 Module uses SPI protocol
#include <MFRC522.h>  // Library for Mifare RC522 Devices
#include <Bounce2.h>

#define MY_DEBUG
//#define MY_DEBUG_LOCAL
// Enable RS485 transport layer
#define MY_RS485
//#define MY_RS485_HWSERIAL Serial
//#define MY_SPLASH_SCREEN_DISABLED

// Define this to enables DE-pin management on defined pin
//#define MY_RS485_DE_PIN 2
// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 19200 //57600 //38400 //9600
//#define MY_RS485_SOH_COUNT 3

#define MY_NODE_ID 94
#define MY_TRANSPORT_WAIT_READY_MS 20000
#include <Arduino.h>

#include <MySensors.h>

//für die millis()-Berechnung, wann wieder gesendet werden soll
unsigned long SEND_FREQUENCY = 180000; // Sleep time between reads (in milliseconds)
unsigned long lastSend = 0;

#define RF_INIT_DELAY   125
#define ONE_SEC         1000
#define MAX_CARDS       18
#define PROG_WAIT       10
#define HEARTBEAT       10

/*Pin definitions*/
#define LED_PIN         A3
#define GARAGEPIN       4
#define SWITCH_PIN      5
#define RST_PIN    7   //  MFRC 
#define SS_PIN    10   //  MFRC 

MFRC522      mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
MFRC522::Uid olduid;
MFRC522::Uid masterkey = { 10, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  0 };

byte       countValidCards  = 0;
MFRC522::Uid validCards[MAX_CARDS];

void       ShowCardData(MFRC522::Uid* uid);
bool       sameUid(MFRC522::Uid* old, MFRC522::Uid* check);
void       copyUid(MFRC522::Uid* src, MFRC522::Uid* dest);
bool       isValidCard(MFRC522::Uid* uid);
int        releasecnt = 0;

#define    CHILD_ID_ALARM    1
#define    CHILD_ID_LOCK     2
#define CHILD_ID_TAGID 4 // Id of the sensor child

//volatile String tagid = String();

Bounce     debouncer = Bounce();

int        oldSwitchValue = -1;
int        switchValue = 0;
long       timer = -1;
bool       programmode = false;
bool       ledon;
int        programTimer = 0;
bool       armed = true;
unsigned long lastTime = 0;

MyMessage  lockMsg(CHILD_ID_LOCK,          V_LOCK_STATUS);
MyMessage  lockArmMsg(CHILD_ID_ALARM,      V_ARMED);
MyMessage  wrongMsg(CHILD_ID_ALARM,        V_TRIPPED);
MyMessage tagMsg(CHILD_ID_TAGID, V_IR_RECEIVE);

void before()
{
  pinMode(GARAGEPIN, OUTPUT);     // Initialise in/output ports

  // Make sure MFRC will be disabled on the SPI bus
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // Setup the button
  pinMode(SWITCH_PIN, INPUT);
  // Activate internal pull-up
  digitalWrite(SWITCH_PIN, HIGH);

  // After setting up the button, setup debouncer
  debouncer.attach(SWITCH_PIN);
  debouncer.interval(5);
}

void presentation() {

  sendSketchInfo(SN, SV);
  present(CHILD_ID_LOCK, S_LOCK);      delay(RF_INIT_DELAY);
  present(CHILD_ID_ALARM, S_MOTION);   delay(RF_INIT_DELAY);
  present(CHILD_ID_TAGID, S_IR);
  recallEeprom();

  // Init MFRC RFID sensor
  SPI.begin();      // Init SPI bus
  mfrc522.PCD_Init();   // Init MFRC522
  ShowReaderDetails();          // Show details of PCD - MFRC522 Card Reader details

  send(lockArmMsg.set(armed));
  Serial.println(F("Init done..."));

}

void setup() {

}

void loop()
{
  timer++;
  debouncer.update();

  // Get the update value
  int switchValue = debouncer.read();
  if (switchValue != oldSwitchValue) {
    // Send in the new value
    Serial.print (F("Switch "));
    Serial.println (switchValue);

    if (switchValue && programmode) {
      lastTime     = millis() / 1000;
    }

    if (!switchValue && programmode && lastTime > 0) {
      if ( (millis() / 1000) - lastTime > 3) {
        Serial.println(F("Reset all cards"));
        countValidCards  = 0;
        blinkFast(50);
      } else {
        Serial.println(F("Program off"));
        digitalWrite(LED_PIN, LOW);
        programmode = false;

        storeEeprom();
      }
    }

    if (!switchValue)   {
      programTimer = 0;
    }
    oldSwitchValue = switchValue;
  }

  if (programmode && ((timer % (ONE_SEC / HEARTBEAT)) == 0 ))  {
    ledon = !ledon;
    digitalWrite(LED_PIN, ledon);
    programTimer++;

    // Stop program mode after 20 sec inactivity
    if (programTimer > PROG_WAIT)  {
      programmode = false;
      digitalWrite(LED_PIN, false);
      Serial.println(F("Program expired"));
    }
  }

  if ((timer % (200 / HEARTBEAT)) == 0 )   {
    // Look for new cards
    if ( ! mfrc522.PICC_IsNewCardPresent()) {
      if (releasecnt > 0)   {
        releasecnt--;
        if (!releasecnt)  {
          olduid.size = 0;
          Serial.println(F("release"));
        }
      }
      return;
    }
    releasecnt = 5;

    // Select one of the cards
    if ( ! mfrc522.PICC_ReadCardSerial()) {
      return;
    }

    // Dump debug info about the card; PICC_HaltA() is automatically called
    //mfrc522.PICC_DumpToSerial(&(mfrc522.uid));

    getTagId(&(mfrc522.uid));
        
    if (!olduid.size || !sameUid(&(mfrc522.uid), &olduid))  {
      ShowCardData(&(mfrc522.uid));
      copyUid(&(mfrc522.uid), &olduid);
      if ( isValidCard(&olduid) )   {
        OpenDoor(programmode);
      } else  {

        if (sameUid(&(mfrc522.uid), &masterkey)) {
          // Only switch in program mode when mastercard is found AND the program button is pressed
          if (switchValue)  {
            Serial.println(F("Program mode"));
            programmode = true;
            programTimer = 0;
            lastTime     = 0;
          }
        } else {
          if (programmode) {
            Serial.println(F("new card"));
            programTimer = 0;

            if (countValidCards < MAX_CARDS)
            {
              // Add card to list...
              copyUid(&(mfrc522.uid), &validCards[countValidCards]);
              countValidCards++;
              blinkFast(15);
            }
          } else {
            Serial.println(F("Invalid card"));
            if (armed) {
              send(wrongMsg.set(1));
              delay(2000);
              send(wrongMsg.set(0));
              //send(tagMsg.set(tagid)); // Send id of the rfid-tag  to gw
            }
          }
        }
      }
    }
  }
}

void ShowCardData(MFRC522::Uid* uid) {
  Serial.print(F("Card UID:"));
  for (byte i = 0; i < uid->size; i++) {
    if (uid->uidByte[i] < 0x10) {
      Serial.print(F(" 0"));
    } else {
      Serial.print(F(" "));
    }
    Serial.print(uid->uidByte[i], HEX);
  }
  Serial.println();
}

void getTagId(MFRC522::Uid* uid) {
   String tagid=("");
   for (byte i = 0; i < uid->size; i++) {
    if (uid->uidByte[i] < 0x10) {
      if (i>0) tagid+=(",");
    } else {
      tagid+=(" ");
    }
    tagid+=("0x");
    tagid+=(String(uid->uidByte[i], HEX));
  }
  Serial.print("tagid: ");
  Serial.println(tagid);
  char TAGID[tagid.length() + 1];
  tagid.toCharArray(TAGID, tagid.length() + 1);
  
  send(tagMsg.set(TAGID)); // Send id of the rfid-tag  to gw
}

void copyUid(MFRC522::Uid* src, MFRC522::Uid* dest)
{
  dest->size = src->size;
  dest->sak  = src->sak;

  for (byte i = 0; i < src->size; i++) {
    dest->uidByte[i] = src->uidByte[i];
  }
}

bool sameUid(MFRC522::Uid* old, MFRC522::Uid* check)
{
  if (old->size != check->size) {
    return false;
  }
  for (byte i = 0; i < old->size; i++) {
    if (old->uidByte[i] != check->uidByte[i]) {
      return false;
    }
  }
  return true;
}

bool isValidCard(MFRC522::Uid* uid)
{
  for (byte i = 0; i < countValidCards; i++)  {
    if (validCards[i].size != uid->size)  {
      break;
    }
    for (int j = 0; j < uid->size; j++) {
      if (validCards[i].uidByte[j] != uid->uidByte[j])  {
        break;
      }
      if (j == (uid->size - 1)) {
        return true;
      }
    }
  }
  return false;
}


void storeEeprom()
{
  byte address = 0;
  saveState(address++, countValidCards);

  for (byte i = 0; i < countValidCards; i++) {
    saveState(address++, validCards[i].size);
    for (byte j = 0; j < 10; j++) {
      saveState(address++, validCards[i].uidByte[j]);
    }
  }
}

void recallEeprom()
{
  byte address = 0;

  countValidCards = loadState(address++);
  if (countValidCards > MAX_CARDS) {
    Serial.println(F("Not a valid EEPROM reading set to default"));
    countValidCards = 0;
    storeEeprom();
    return;
  }

  for (byte i = 0; i < countValidCards; i++)  {
    validCards[i].size = loadState(address++);
    for (byte j = 0; j < 10; j++)  {
      validCards[i].uidByte[j] = loadState(address++);
    }
  }

}

void blinkFast(int times)
{
  for (int i = 0; i < times; i++) {
    ledon = !ledon;
    digitalWrite(LED_PIN, ledon);
    delay(100);
  }
}

void OpenDoor(bool fakeOpen)
{
  Serial.println(F("Open door!"));
  send(lockMsg.set(false));

  if (!fakeOpen) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(GARAGEPIN, HIGH);
  }
  delay(1000);

  if (!fakeOpen) {
    digitalWrite(GARAGEPIN, LOW);
    digitalWrite(LED_PIN, LOW);
  }

  send(lockMsg.set(true));
}

void ShowReaderDetails() {
  // Get the MFRC522 software version
  byte v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  Serial.print(F("MFRC522 Software Version: 0x"));
  Serial.print(v, HEX);
  if (v == 0x91) {
    Serial.print(F(" = v1.0"));
  } else if (v == 0x92) {
    Serial.print(F(" = v2.0"));
  } else {
    Serial.print(F(" (unknown)"));
  }
  Serial.println("");

  // When 0x00 or 0xFF is returned, communication probably failed
  if ((v == 0x00) || (v == 0xFF)) {
    Serial.println(F("WARNING: Communication failure, is the MFRC522 properly connected?"));
  }
}


void incomingMessage(const MyMessage &message)
{
  if (message.type == V_LOCK_STATUS) {
    // Change relay state
    if (!message.getBool())  {
      OpenDoor(false);
    }

    // Write some debug info
    Serial.print(F("Lock status: "));
    Serial.println(message.getBool());
  }
  else
  {
    if (message.type == V_ARMED)  {
      // Change relay state
      armed = message.getBool();

      // Write some debug info
      Serial.print(F("Arm status: "));
      Serial.println(message.getBool());
    }
    else
    {
      // Write some debug info
      Serial.print(F("Incoming msg type: "));
      Serial.print(message.type);
      Serial.print(F(" id: "));
      Serial.print(message.sensor);
      Serial.print(F(" content: "));
      Serial.println(message.getInt());
    }
  }
}
