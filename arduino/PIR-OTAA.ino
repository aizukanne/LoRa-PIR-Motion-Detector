/********************************************************************************
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2).
 * 
 * To use this sketch, first register your application and device with
 * the server, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

const int nodeID = 2;                       //  node ID for this node
const uint8_t networkGroup = 1;             //  Distribution network group for this node


#define GPS 1                        // set this to 1 if GPS module is available
//uncomment this for dev mode
#define DEVMODE 1

//--------------------------------------------------------------------------------------
// Variable declaration 
//--------------------------------------------------------------------------------------

typedef struct { float latitude, longitude; int nodeID; byte networkGroup, intrusion;} PayloadTX; 

PayloadTX myData;                // create an instance
int ledPin = 7;                  // choose the pin for the LED
int inputPin = 9;               // choose the input pin (for PIR sensor)
int pirState = LOW;              // we start, assuming no motion detected
int val = 0;                     // variable for reading the pin status

#ifdef GPS
  #include <NMEAGPS.h>
  #include <SoftwareSerial.h>
  SoftwareSerial gpsPort(9, 10); // pin 9 to GPS TX, pin 10 to GPS RX
#endif
//------------------------------------------------------------


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}


static osjob_t sendjob;
static osjob_t readjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, LMIC_UNUSED_PIN},
};


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);    
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            do_reconnect();
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            do_reconnect();
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void detect_motion() {
  val = digitalRead(inputPin);  // read input value
  if (val == HIGH) {
    if (pirState == LOW){
        // we have just turned on
        Serial.println("Motion detected!");
        myData.intrusion = 1;
        digitalWrite(ledPin, HIGH);  // flash LED ON
        delay(50);
        digitalWrite(ledPin, LOW);  // flash LED OFF
        // We only want to print on the output change, not state
        pirState = HIGH;
    }
  }else{
    if (pirState == HIGH){
        // we have just turned off
        Serial.println("Motion ended!");
        myData.intrusion = 0;
        digitalWrite(ledPin, LOW); // turn LED OFF
        // We only want to print on the output change, not state
        pirState = LOW;
    }
  }
}

void get_gps(){
    NMEAGPS  gps; // This parses the GPS characters
    gps_fix  fix; // This holds on to the latest values
    Serial.println(F("Searching for GPS Signal "));
    bool newdata = false;
    unsigned long start = millis();
    while (newdata == false) {
      if (gps.available( gpsPort )) {
        fix = gps.read();
        digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
        if (fix.valid.location) {
          #if defined(DEVMODE)
            Serial.println();
            Serial.println(F("GPS Signal aqcuired."));
            Serial.print( F("Location: ") );          
            Serial.print( fix.latitude(), 6 );
            Serial.print( ',' );
            Serial.println( fix.longitude(), 6 );
          #endif
          myData.latitude = fix.latitude();
          myData.longitude = fix.longitude();
          newdata = true;
        }
        Serial.print(".");
        delay (2000);
      }
    }
}

void do_send(osjob_t* j){
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    detect_motion();
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1,(byte*)&myData, sizeof(myData), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void do_connect(){
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC.dn2Dr = DR_SF9;
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);  
    //Start first transmission to do the Join
    const char first = "Join";
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(LMIC.opmode);
      Serial.println(OP_TXRXPEND);
      Serial.println(F("[LMIC] OP_TXRXPEND, not sending"));
    } else {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, (byte*)&myData, sizeof(myData), 1);
    }
}

void do_reconnect(){
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC.dn2Dr = DR_SF9;
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    LMIC_setTxData2(1,(byte*)&myData, sizeof(myData), 0);
}

void setup() {
   Serial.begin(115200);
   #if defined(DEVMODE)  
     Serial.println(F("Starting"));
     Serial.println(F("Intrusion Detection System v1.0"));
     Serial.println(F("www.your-website.com"));
     Serial.print(F("Node: ")); 
     Serial.print(nodeID); 
     Serial.print(F(" Freq: ")); 
     Serial.print(F("868MHz"));
     Serial.print(F(" Network: ")); 
     Serial.println(networkGroup);
   #endif
   pinMode(ledPin, OUTPUT);      // declare LED as output
   pinMode(inputPin, INPUT);     // declare sensor as input
   myData.nodeID = nodeID;
   myData.networkGroup = networkGroup;

  if (GPS == 1){
    get_gps();
  } else {
        myData.latitude = 0.0;
        myData.longitude = 0.0;
  }

    do_connect();
      
}

void loop() {
  os_runloop_once(); 
}
