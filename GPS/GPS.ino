 
#include <SoftwareSerial.h>
#include <TinyGPS.h>

#define rxGPS 3 
#define txGPS 5

TinyGPS gps;
SoftwareSerial serialGPS = SoftwareSerial(rxGPS, txGPS);

//String GPS_String = "";

long lat, lon;
unsigned long fix_age, time, date, speed, course;

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
      gps.get_position(&lat, &lon, &fix_age);
      Serial.print("Latitude: ");
      Serial.println(lat);
      Serial.print("Longitude: ");
      Serial.println(lon);
      Serial.println("---------------------------");
      gps.get_datetime(&date, &time, &fix_age);
      Serial.print("Data: ");
      Serial.println(date);
      Serial.print("Hora: ");
      Serial.println(time);
      Serial.println("***************************");
      Serial.print("ID Satelite: ");
      Serial.println(gps.satellites());
      Serial.println("***************************");
    }
    /*
    if (recebe != '\n' && recebe != '\r')
    {
      GPS_String  = recebe;
      Serial.print(GPS_String);
    }
    else
    {
      Serial.print("\n");
    } */
  }
}
