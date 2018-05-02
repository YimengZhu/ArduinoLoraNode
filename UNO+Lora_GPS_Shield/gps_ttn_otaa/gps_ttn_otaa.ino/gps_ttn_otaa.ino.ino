#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial ss(3, 4); 

float flat, flon;
uint8_t payload[20];

void setup()
{
  Serial.begin(9600);  
  ss.begin(9600); 
  Serial.println("Starting");
}


void loop()
{
  updateGPS();

  Serial.println();
  smartdelay(1000);
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
    {
      gps.encode(ss.read());
    }
  } while (millis() - start < ms);
}

static int updateGPS() {
  unsigned long age;

  gps.f_get_position(&flat, &flon, &age);

  if ((flat == TinyGPS::GPS_INVALID_F_ANGLE) || 
      (flon == TinyGPS::GPS_INVALID_F_ANGLE)) {
    Serial.println("GPS data invalid.");    
    return -1;
  } else {
    char lat_char[10] = {"\0"};
    char lon_char[10] = {"\0"};
    String slat = dtostrf(flat, 10, 4, lat_char);
    String slon = dtostrf(flon, 10, 4, lon_char);
    
    String payload_string = slat + slon;
    payload_string.toCharArray(payload, 20);
        
    Serial.print("payload: ");
    Serial.println((char*) payload);
    return 0;  
  }
}


