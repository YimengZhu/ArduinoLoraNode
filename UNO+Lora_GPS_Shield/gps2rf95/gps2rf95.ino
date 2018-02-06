// Library for serial communication of GPS
// https://www.arduino.cc/en/Reference/SoftwareSerial
#include <SoftwareSerial.h> 

// Arduino GPS Library
// http://arduiniana.org/libraries/tinygps/ 
#include <TinyGPS.h>

// Library for communication with SPI devices with Arduino as master
// https://www.arduino.cc/en/Reference/SPI
#include <SPI.h> 

// Driver to send and receive unaddreSerialed unreliable datagrams via Lora
// http://www.airspayce.com/mikem/arduino/RadioHead/claSerialRH__RF95.html
#include <RH_RF95.h>

RH_RF95 rf95;
TinyGPS gps;
// SoftwareSerial instance, set Pin3 as rxPin and Pin4 as txPin
//SoftwareSerial Serial(3, 4);
String latstring = "";
String lonstring = "";
char databuf[100];
uint8_t dataoutgoing[100];
char gps_lon[20] = {"\0"};
char gps_lat[20] = {"\0"};

void setup() {
  Serial.begin(9600);
  Serial.println("cao from serial");
  Serial.begin(9600);
  Serial.println("cao from Serial");

  if (!rf95.init()) {
    Serial.println("Init failed!");
    }
    
  Serial.print("Simple TinyGPS library v. "); 
  Serial.println(TinyGPS::library_version());
}

void loop() {
  Serial.println("cao");

  // Sending data to rf95_server
  Serial.println("Sending data to rf95_server.");
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // parse GPS data every secound
  for(unsigned long start = millis(); millis() - start < 1000;) {
    while (Serial.available()) {
      char c = Serial.read();
      // check if a new valid sentence come in
      if(gps.encode(c)) {
        newData = true;
        }
      }
  }

  // extract the gps data
  if(newData) {
    float flat, flon;
    unsigned long age;
        
    gps.f_get_position(&flat, &flon, &age);

    // check if the data is fixed
    Serial.print("LAT = ");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0: flat, 6);
      
    Serial.print("LON = ");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0: flon, 6);

    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());

    flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;          
    flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6; 

    // dtostrf(floatvar, StirngLen, NumAfterDecimal, charbuf) will convert a float to a char array
    // floatvar: the float number to be converted
    // StirngLen: the length of the array after convert
    // NumAfterDecimal: the number of char ater the decimal point, which is used for describe precision
    // charbuf: the array to store the result.
    latstring += dtostrf(flat, 0, 6, gps_lat);
    lonstring += dtostrf(flon, 0, 6, gps_lon);
       
    //print the data
    Serial.println(strcat(strcat(gps_lon, " ,"), gps_lat));
    strcpy(gps_lat, gps_lon);
    Serial.println(gps_lat);

        
    // send the data to the server
    strcpy((char *)dataoutgoing, gps_lat);
    rf95.send(dataoutgoing, sizeof(dataoutgoing));

    //wait for reply
    uint8_t indatabuf[RH_RF95_MAX_MESSDDDDDDDDDDDDAGE_LEN];
    uint8_t len = (indatabuf);

    if(rf95.waitAvailableTimeout(3000)) {
          
      if(rf95.recv(indatabuf, &len)) {
        Serial.print("Reply meSerialage: ");
        Serial.println((char*)indatabuf);
        } 
      else {
        Serial.println("Recieving meSerialage failed!");
        }
      } 
    else {
          Serial.println("No reply, is rf95_server running?");
      }

    delay(400);
    }

  gps.stats(&chars, &sentences, &failed);
  Serial.print("CHARS = ");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0){
     Serial.println("** No characters received from GPS: check wiring **");
    }
}















