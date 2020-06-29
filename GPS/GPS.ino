#include <SoftwareSerial.h>

#define rxGPS 3 
#define txGPS 5
SoftwareSerial serialGPS = SoftwareSerial(rxGPS, txGPS);
String GPS_String = "";

void setup() {
  pinMode(rxGPS, INPUT);
  pinMode(txGPS, OUTPUT);

  Serial.begin(9600);
  serialGPS.begin(4800);
}

void loop()
{
  if (serialGPS.available())
  {
    char recebe = serialGPS.read();
    if (recebe != '\n' && recebe != '\r')
    {
      GPS_String  = recebe;
      Serial.print(GPS_String);
    }
    else
    {
      Serial.print("\n");
    }
  }
}
