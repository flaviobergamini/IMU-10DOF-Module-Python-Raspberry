#include "Wire.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "I2Cdev.h"
#include "MPU9250.h"
#include "BMP280.h"
#include <avr/wdt.h>  // Usando o watchdog do MCU para fazer um reset da placa quando necessário

// Definindo pinos de RX e TX do Módulo GPS
#define rxGPS 3 
#define txGPS 5

TinyGPSPlus gps;
SoftwareSerial serialGPS = SoftwareSerial(rxGPS, txGPS);

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev   I2C_M;

uint8_t buffer_m[6];


int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;



float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

//variaveis usadas no GPS
float latitude, longitude;


#define sample_num_mdate  5000

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

float temperature;
float pressure;
float atm;
float altitude;
BMP280 bmp280;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
     pinMode(rxGPS, INPUT);
     pinMode(txGPS, OUTPUT);
    
     Serial.begin(115200);
     serialGPS.begin(4800);

    // initialize device
    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    bmp280.init();

    // See datasheet
    //bmp280.setSampling(BMP280::MODE_NORMAL,  // mode
    //              BMP280::SAMPLING_X2,  // temperature, not more than x2
    //              BMP280::SAMPLING_X16, // pressure                  
    //              BMP280::FILTER_X16,   // filter
    //              BMP280::STANDBY_MS_500); // standby

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

    delay(1000);
    //Serial.println("     ");

    //  Mxyz_init_calibrated ();
    wdt_enable (WDTO_2S);
}

void reset(){
  while (true) ; // reset do WDT
} 

void loop() {
  
    getAccel_Data();
    getGyro_Data();
    getCompassDate_calibrated(); // compass data has been calibrated here
    getHeading();               //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
    getTiltHeading();
    
      while (serialGPS.available() > 0)               //Realiza a leitura da Latitude e longitude no GPS
      {
        int recebe = serialGPS.read();
        
        if(gps.encode(recebe)){ 
          latitude = gps.location.lat();             //Latitude
          longitude = gps.location.lng();            //Longitude
        }
      }

      char caracter;
      if(Serial.available() > 0) {
        caracter = Serial.read();
        temperature = bmp280.getTemperature(); //Get the temperature, bmp180ReadUT MUST be called first
        pressure = bmp280.getPressure();//Get the temperature
        altitude = bmp280.calcAltitude(pressure); //Uncompensated caculation - in Meters
        atm = pressure / 101325;
        
        switch(caracter){ 
          case 'm':     
                        if(String(Mxyz[0]) == "" || String(Mxyz[1]) == "" || String(Mxyz[2]) == ""){
                            Serial.println("EEntrou");
                            wdt_reset ();  
                            reset();
                        }else{
                            Serial.print(Mxyz[0]);
                            Serial.print(",");
                            Serial.print(Mxyz[1]);
                            Serial.print(",");
                            Serial.println(Mxyz[2]);
                        }
                        break;
    
          case 'a':     
                        Serial.print(Axyz[0]);
                        Serial.print(",");
                        Serial.print(Axyz[1]);
                        Serial.print(",");
                        Serial.println(Axyz[2]);
                        break;
    
          case 'g':     Serial.print(Gxyz[0]);
                        Serial.print(",");
                        Serial.print(Gxyz[1]);
                        Serial.print(",");
                        Serial.println(Gxyz[2]); 
                        break;
                        
          case 'x':     Serial.println(heading);           //Angulo entre o norte magnetico e a projeção da posição X-Axis
                        break;
                        
          case 'y':     Serial.println(tiltheading);     //Angulo entre o norte magnetico e a projeção da posição X-Axis no plano horizontal
                        break;
    
          case 'h':     Serial.println(altitude, 2);      //Altura em metros
                        break;

          case 'z':     Serial.print(latitude, 6);         //Envia para a serial os dados de Latitude do GPS
                        Serial.print(",");
                        Serial.println(longitude, 6);
                        break;
        }
    }
}


void getHeading(void) {
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}

void getTiltHeading(void) {
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0)    tiltheading += 360;
}

void Mxyz_init_calibrated () {
    Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
    Serial.print("  ");
    Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
    Serial.print("  ");
    Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
    while (!Serial.find("ready"));
    Serial.println("  ");
    Serial.println("ready");
    Serial.println("Sample starting......");
    Serial.println("waiting ......");

    get_calibration_Data ();

    Serial.println("     ");
    Serial.println("compass calibration parameter ");
    Serial.print(mx_centre);
    Serial.print("     ");
    Serial.print(my_centre);
    Serial.print("     ");
    Serial.println(mz_centre);
    Serial.println("    ");
}

void get_calibration_Data () {
    for (int i = 0; i < sample_num_mdate; i++)
    {
        get_one_sample_date_mxyz();
        /*
        Serial.print(mx_sample[2]);
        Serial.print(" ");
        Serial.print(my_sample[2]);                            //you can see the sample data here .
        Serial.print(" ");
        Serial.println(mz_sample[2]);
        */

        if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
        if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
        if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

        if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
        if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];

    }

    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];

    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];

    mx_centre = (mx_max + mx_min) / 2;
    my_centre = (my_max + my_min) / 2;
    mz_centre = (mz_max + mz_min) / 2;

}

void get_one_sample_date_mxyz() {
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}

void getAccel_Data(void) {
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void) {
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void) {
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;
    
    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassDate_calibrated () {
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}
