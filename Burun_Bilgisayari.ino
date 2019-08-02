#include <TinyGPS++.h> // Library über http://arduiniana.org/libraries/tinygpsplus/ downloaden und installieren
#include <HardwareSerial.h> // sollte bereits mit Arduino IDE installiert sein
#include <config.h>
#include "SSD1306.h"
#include <LoRa.h>
#include<Arduino.h>

#define SS      18
#define RST     14
#define DI0     26
#define BAND    433E6  //915E6 

TinyGPSPlus gps;
 

HardwareSerial SerialGPS(2);
 

struct GpsDataState_t {
  double originLat = 0;
  double originLon = 0;
  double originAlt = 0;
  double distMax = 0;
  double dist = 0;
  double altMax = -999999;
  double altMin = 999999;
  double spdMax = 0;
  double prevDist = 0;
};
GpsDataState_t gpsState = {};
 

#define TASK_SERIAL_RATE 1000 // ms
uint32_t nextSerialTaskTs = 0;
uint32_t nextOledTaskTs = 0;
double a,b,c = 0.0;


void setup() {

  Serial.begin(9600);
 

  SerialGPS.begin(9600, SERIAL_8N1,16,17);
  delay(50);
  //SPI.begin(5,19,27,18);  
  LoRa.setPins(SS,RST,DI0);
  Serial.println("LoRa Sender");
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initial OK!");
}

 


void loop() {
 
  static int p0 = 0;
 
  // GPS Koordinaten von Modul lesen
  gpsState.originLat = gps.location.lat();
  gpsState.originLon = gps.location.lng();
  gpsState.originAlt = gps.altitude.meters();
 
  long writeValue;
 
  gpsState.distMax = 0;
  gpsState.altMax = -999999;
  gpsState.spdMax = 0;
  gpsState.altMin = 999999;
 

  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
 


 

  if (nextSerialTaskTs < millis()) {
    a = gps.location.lat();
    b = gps.location.lng();
    c = gps.altitude.meters();
    Serial.print("LAT=");  Serial.println(a, 6);
    Serial.print("LONG="); Serial.println(b, 6);
    Serial.print("ALT=");  Serial.println(c);
    Serial.print("Sats=");  Serial.println(gps.satellites.value());
    Serial.print("DST: ");
    Serial.println(gpsState.dist, 1);
    nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
    //LoRa.beginPacket();
    
    //LoRa.print("Latitude ");
   // LoRa.println(a);
    //LoRa.print("Longtitude ");
    //LoRa.println(b);
    
  
   // LoRa.endPacket();
    
  }
    delay(2000);
    LoRa.beginPacket();
    
    LoRa.print("Enlem: ");
    LoRa.println(a,6);
    LoRa.print("Boylam: ");
    LoRa.println(b,6);
    LoRa.println("Burun Bilgisayarı Gps Verisi");
    
  
    LoRa.endPacket();
}
