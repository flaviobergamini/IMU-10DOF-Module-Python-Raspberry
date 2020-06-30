 
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define rxGPS 3 
#define txGPS 5

TinyGPSPlus gps;
SoftwareSerial serialGPS = SoftwareSerial(rxGPS, txGPS);

void setup() {
  pinMode(rxGPS, INPUT);
  pinMode(txGPS, OUTPUT);

  Serial.begin(9600);
  serialGPS.begin(4800);
}

void loop()
{
  while (serialGPS.available())
  {
    int recebe = serialGPS.read();
    
    if(gps.encode(recebe)){
      Serial.print("Latitude: ");  
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: "); 
      Serial.println(gps.location.lng(), 6);
      Serial.print("Altitude: ");  
      Serial.println(gps.altitude.meters());
    }
  }
}
