#include <config.h>

#include "TinyGPS++.h"//arduiniana.org/libraries/tinygpsplus/ downloaden und installieren
#include <HardwareSerial.h>
#include<Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include<Arduino.h>

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

#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

//OLED pins to ESP32 GPIOs via this connecthin:
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16



#define SS      18
#define RST     14
#define DI0     26
#define BAND    433E6  //915E6 

boolean tepe_noktasi = false;
int16_t h_yeni = 0;
int16_t h_eski = 0;
int i; 
int counter = 0;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
unsigned long startMillis_2;  //some global variables available anywhere in the program
unsigned long currentMillis_2;
const unsigned long period = 1500;
const unsigned long period2 = 5000;
void setup() {
   
   
  Serial.begin(115200);
 

  //SerialGPS.begin(115200, SERIAL_8N1, 16,17);
 
  while (!Serial); //If just the the basic function, must connect to a computer
  // Initialising the UI will init the display too.
          
  SerialGPS.begin(9600,SERIAL_8N1,16,17);

  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
  while (1) {
    Serial.println("asd");
    }
  }
  
  pinMode(12,OUTPUT); //Send success, LED will bright 1 second
  //pinMode(16,OUTPUT);
  //digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  //digitalWrite(16, HIGH);
  //SPI.begin(5,19,27,18);
  LoRa.setPins(SS,RST,DI0);
  Serial.println("LoRa Sender");
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initial OK!");
  delay(300);
  pinMode(25, OUTPUT);
  digitalWrite (25,LOW);
}
  
void loop() {
  static int p0 = 0;
 
  //PS Koordinaten von Modul lesen
 gpsState.originLat = gps.location.lat();
  gpsState.originLon = gps.location.lng();
  gpsState.originAlt = gps.altitude.meters();
 
  // Aktuelle Position in nichtflüchtigen ESP32-Speicher schreiben
  long writeValue;

 
  gpsState.distMax = 0;
  gpsState.altMax = -999999;
  gpsState.spdMax = 0;
  gpsState.altMin = 999999;

   while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
   
   }

  if (nextSerialTaskTs < millis()) {
    Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
    Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
    Serial.print("ALT=");  Serial.println(gps.altitude.meters());
    Serial.print("Sats=");  Serial.println(gps.satellites.value());
    Serial.print("DST: ");
    Serial.println(gpsState.dist, 1);
    nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
  }
    
   

  
  //Serial.print("Altitude = ");
  //Serial.print(bmp.readAltitude());
  //Serial.println(" meters");
  //startMillis_2 = millis();
 

  h_eski = h_yeni;
 
  delay(500);

  Serial.print("Real altitude = ");
  h_yeni = bmp.readAltitude(102000);
  Serial.print(h_yeni);
  Serial.println(" meters");
  if(h_yeni >= 12000){
    tepe_noktasi = true; 
    }
  
  if(tepe_noktasi && (h_yeni - h_eski < -3)){
    
    startMillis = millis();
    while(1){
      
    
    if(currentMillis - startMillis >= period){
      break;
      }
      
      digitalWrite (25,HIGH);
      delayMicroseconds(2000);
      digitalWrite (25,LOW);
      delay(18);
      currentMillis = millis();
      
    }  
  }
  
  Serial.println();
 
  LoRa.beginPacket();
 
 
  LoRa.print("Yukseklik ");
  LoRa.println(h_yeni);
  LoRa.println("Yedek Sistem Gps Verisi.");
  LoRa.endPacket();
  
}

