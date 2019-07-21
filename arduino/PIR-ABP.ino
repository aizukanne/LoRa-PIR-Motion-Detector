/********************************************************************************
 *
 * This uses ABP (Activation By Personalization).
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


#define GPS                      // comment this out if GPS module is not available
//uncomment this for dev mode
#define DEVMODE

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


// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x2B, 0xAA, 0x11, 0xED, 0x2A, 0xEC, 0xE8, 0xA3, 0x09, 0xF1, 0x13, 0x70, 0x4A, 0x8C, 0xD8, 0x81 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x012abc48 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }



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
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            do_connect(); 
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

void do_connect() {
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    //LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF12;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
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

  #if defined(GPS)
    get_gps();
  #else
    myData.latitude = 0.0;
    myData.longitude = 0.0;
  #endif

    do_connect();
      
}

void loop() {
  os_runloop_once(); 
}
