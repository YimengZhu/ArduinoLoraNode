#include<lmic.h>
#include<hal/hal.h>
#include<SPI.h>
#include<TinyGPS.h>

// Program EUI issued by TTN. Using little-endian format.
static const u1_t PROGMEM APPEUI[8] = {0x93, 0x9E, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// Device EUI in TTN, also using little-endian format.
static const u1_t PROGMEM DEVEUI[8] = {0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23 , 0x01};
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// Program Key issued by TTN, using big endian format as a block of memory.
static const u1_t PROGMEM APPKEY[16] = {0x23, 0x60, 0xCB, 0x06, 0x84, 0xCB, 0x35, 0x7F, 0xBF, 0x93, 0xB9, 0x81, 0x95, 0xA4, 0x9E, 0x8A};
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

TinyGPS gps;
String latString = "";
String lonString = "";
char gps_lon[20]={"\0"};  
char gps_lat[20]={"\0"}; 

static uint8_t payload[] = "";
static osjob_t sendjob;

// Schedule TX interval, might be longer due to duty cycle limitation.
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

bool readGPS() {
  Serial.println("Reading GPS data.");

  bool newData = false;

  //variable for failed gps reading 
  unsigned long chars;
  unsigned short sentences, failed;
  // Read and parse GPS data every secound, by success report some key values.
  for(unsigned long start = millis(); millis() - start < 1000;) {
    while(Serial.available()) {
      char c = Serial.read();
      if(gps.encode(c))
        newData = true;  
    }
  }

  // If the gps data can be read and parsed successfully, set up the payload
  if(newData) {
    float flat, flon;
    unsigned long age;
    
    gps.f_get_position(&flat, &flon, &age);

    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());

    flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;          
    flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6; 
    
    // Once the GPS fixed,send the data to the server.
    latString +=dtostrf(flat, 0, 6, gps_lat); 
    lonString +=dtostrf(flon, 0, 6, gps_lon);
    strcat(strcat(gps_lon,","),gps_lat);
    strcpy(gps_lat,gps_lon);
    strcpy(payload, gps_lat);
    delay(400);
    return true; 
  } else {
    gps.stats(&chars, &sentences, &failed);                                                                                                                                                                                                                                                                                                                                                                          
    Serial.print(" CHARS=");
    Serial.print(chars);
    Serial.print(" SENTENCES=");
    Serial.print(sentences);
    Serial.print(" CSUM ERR=");
    Serial.println(failed);
    if (chars == 0) {
      Serial.println("** No characters received from GPS: check wiring **");
    }
    return false;
  }
}

// Event handler
void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");

  switch (ev) {
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
      // Disable link check validation (automatically enabled during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows.)"));
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Recieved ack"));
      }
      if (LMIC.dataLen) {
        Serial.print(F("Recieved "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload."));
      }
      // Setup next TX after TX_INVERVAL time
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println("EV_LOST_TSYNC");
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

void do_send(osjob_t* j) {
  // check if there is a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, no sending"));
  } else {
    // Prepare upstream data transmission at next possible time.
    if(readGPS()) {
      LMIC_setTxData2(2, payload, sizeof(payload) - 1, 0);
      Serial.println(F("Packet queued."));
    } else {
      Serial.println("No GPS Singnal.");  
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("Starting..."));

#ifdef VCC_ENABLE
  // For Pinoccio Scout
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  //LMIC init
  os_init();
  //Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Start job
  uint8_t greet[] = "GPS Tracker starting...";
  LMIC_setTxData2(1, greet, sizeof(greet) - 1, 0);
}

void loop() {
  os_runloop();
}






































