#include <config.h>
#include "SSD1306.h"
#include<Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include<Arduino.h>
#include <TinyGPS++.h> 
#include <HardwareSerial.h>



int motor1Pin1 = 33; 
int motor1Pin2 = 32; 

//int motor1Pin3 = 36; 
//int motor1Pin4 = 37; 
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

//OLED pins to ESP32 GPIOs via this connecthin:
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16

SSD1306  display(0x3c, 4, 15);

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


char text[40];
boolean tepe_noktasi = false;
boolean tepe_noktasi2 = false;
int16_t h_yeni = 0;
int16_t h_eski = 0;
int i,j,k,d; 
int counter = 0;
const int MPU=0X68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ ;
unsigned long startMillis = 0;  //some global variables available anywhere in the program
unsigned long currentMillis = 0;
unsigned long startMillis_2 = 0; //some global variables available anywhere in the program
unsigned long currentMillis_2 = 0;
const unsigned long period = 15000;
const unsigned long period2 = 1500;
bool yedek = false;
void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(2, OUTPUT);

   

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x68);
  Wire.write(0); //MPU-6050 çalıştırıldı 
  Wire.endTransmission(true);
  Serial.begin(9600);
 
  //while (!Serial); //If just the the basic function, must connect to a computer
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(5,5,"LoRa Sender");
  display.display();
  Serial.begin(9600);
  SerialGPS.begin(9600, SERIAL_8N1,16,17);
  delay(50);
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");

  while (1) {
    Serial.println("sdaa");}
  }
  
  pinMode(12,OUTPUT); //Send success, LED will bright 1 second
      
  delay(50); 
  SPI.begin(5,19,27,18);
  LoRa.setPins(SS,RST,DI0);
  Serial.println("LoRa Sender");
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initial OK!");
  display.drawString(5,20,"LoRa Initializing OK!");
  display.display();
  delay(300);
  digitalWrite(2,LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  pinMode(25, OUTPUT);
  digitalWrite (25,LOW);
}
  
void loop() {
  while(1){

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
    
      
  if(h_yeni == -7174){
    yedek = true;
    
    if(yedek){
      digitalWrite(2,HIGH);
      LoRa.beginPacket();
      LoRa.println("ANA SİSTEMDEN VERİ GELMİYOR YEDEK SİSTEM AKTİF EDİLDİ!!!!!!!!!!");
      LoRa.endPacket();
      break;
      }
  }
    
    
  verileriOku();
  Serial.print(" | ivmeX = "); Serial.print(AcX);
  Serial.print(" | ivmeY = "); Serial.print(AcY);
  Serial.print(" | ivmeZ = "); Serial.print(AcZ);
  
  Serial.print(" | GyroX = "); Serial.print(GyX);
  Serial.print(" | GyroY = "); Serial.print(GyY);
  Serial.print(" | GyroZ = "); Serial.println(GyZ);

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(3, 5, "AcX = ");
  display.drawString(50, 5, String(AcX));
  display.drawString(3, 25,"AcY = ");
  display.drawString(50, 25, String(AcY));
  display.drawString(3, 45, "AcZ = ");
  display.drawString(50, 45, String(AcZ));
  display.display();
  
  
  h_eski = h_yeni;
 
  delay(1000);

  Serial.print("Real altitude = ");
  h_yeni = bmp.readAltitude(102000);
  
  //sprintf(text,"%d,\n",h_yeni);
  Serial.print(h_yeni);
  Serial.println(" meters");
  if(h_yeni >= 350){ // Kod yüklenirken 2250 metre parametresi girilmelidir.
    tepe_noktasi = true; 
    Serial.println("Tepe Noktasina Ulaşıldı.");
    }
  if(h_yeni >= 200){ // Kod yüklenirken 1600 metre parametresi girilmelidir.
     tepe_noktasi2 = true; 
    
    }
  
  if(tepe_noktasi && (h_yeni - h_eski < -3) && (h_yeni - h_eski > -300) && (h_yeni>0) && (h_eski>0)){
    
    j = 0;
    Serial.println(j);
    while(1){
      j = j + 1;
     
      
    
    if(j == 2){
       Serial.println(j);
       LoRa.beginPacket();
       LoRa.print("Birinci Parasüt Acildi.");
       LoRa.endPacket();
       break;
      }
      for(i = 0;i<=20;i++){
      digitalWrite (13,HIGH);
      delayMicroseconds(2000);
      digitalWrite (13,LOW);
      delay(18);
      
      
      }
   
    }  
  }

  if(tepe_noktasi2 && h_yeni <= 110 && (h_yeni - h_eski < -1) && (h_yeni - h_eski > -300) && (h_yeni>0) && (h_eski>0) ){
    k = 0;
    while(1){
      k = k+1;
      if(k == 2){
        
        startMillis = millis();
        LoRa.beginPacket();
        LoRa.print("İkinci Parasüt Acildi.");
        LoRa.endPacket();
        tepe_noktasi = false;
        
        break;
        }
      
       //delay(4500);
       digitalWrite(motor1Pin1, HIGH);
       digitalWrite(motor1Pin2, LOW); 
       
     }
    
    }
 
  j = 0;
  k = 0;
  Serial.println();
 
  LoRa.beginPacket();
  
  
 
  LoRa.print("Yukseklik:(Ana): ");
  LoRa.println(h_yeni);
  LoRa.print("Enlem: ");
  LoRa.println(a,6);
  LoRa.print("Boylam: ");
  LoRa.println(b,6);
  LoRa.println("Ana Uçus Bilgisayarı Gps Verisi");
  
  LoRa.endPacket();
  
}
}}
void verileriOku(){
  Wire.beginTransmission(MPU);
  
  Wire.write(0x3B); 
  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);

  
  AcX=Wire.read()<<8|Wire.read()/16384;   
  AcY=Wire.read()<<8|Wire.read()/16384; 
  AcZ=Wire.read()<<8|Wire.read()/16384; 
   
  GyX=Wire.read()<<8|Wire.read()/33; 
  GyY=Wire.read()<<8|Wire.read()/33;  
  GyZ=Wire.read()<<8|Wire.read()/33;
  
  Serial.println();
  delay(40);
 
}
