#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <HardwareSerial.h>
#include <EBYTE.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPS++.h>


//=Barometer&Temp=
Adafruit_BMP280 bmp;
float bmpOffset;
float atmPressure = 1014;
float temperature = 0;
float pressure = 0;
float altitude = 0;

//=radio=


int Chan = 22;
unsigned long Last;


//=servos=
#define PIN_A2 2
#define PIN_A3 3

Servo servoX, servoY;

int posX = 0;
int posY = 0;

//=IMU=
#define BNO08X_RESET -1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

//=GPS=

static const int RXPinGPS = 7, TXPinGPS = 8;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

float targetLat = 54.679734;
float targetLon = 25.258749;
float delta_lon, delta_lat;
float bearing = 0;

TinyGPSCustom pdop(gps, "GNGLL", 1);
TinyGPSCustom hdop(gps, "GNGLL", 3);


struct RECEIVE_DATA
{
  float X;
  float Y;
  float theta;
  int idle;
};

struct SEND_DATA
{
  float temperature;
  float pressure;
  float altitude;
  float roll;
  float pitch;
  float yaw;
};

RECEIVE_DATA ControlData;
SEND_DATA FeedbackData;

EBYTE Transceiver(&Serial2, PIN_M0, PIN_M1, PIN_AX);


void InitServos(){
  servoX.write(posX);
  servoY.write(posY);
  servoX.attach(PIN_A2);
  servoY.attach(PIN_A3);
  Serial.println("Servos reset.!");
}

void InitGPS(){
  Serial1.begin(GPSBaud, SERIAL_8N1, RXPinGPS, TXPinGPS);
}

//============

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println(" ");
    Serial.println("Arduino Nano Esp32-S3 booted successfully!");
    Serial.println("Waiting for data...");
    //Flash();
    //InitRadio();
    //InitIMU();
    InitGPS();
    //InitBMP();
    delay(3000);
    InitServos();
}

void InitIMU(){
    while (!bno08x.begin_I2C()) {
    Serial.println("Warning! Waiting For IMU..");
    delay(500);
  }
  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}


float GetBearing(float lat1, float lon1, float lat2, float lon2){

    delta_lon = lon1 - lon2;
    delta_lat = lat1 - lat2;
    // theta = atan2(sin(delta_lon * 57296 / 1000) * cos(lat2 * 57296 / 1000),
    //                    cos(lat1 * 57296 / 1000) * sin(lat2 * 57296 / 1000) - 
    //                    sin(lat1 * 57296 / 1000) * cos(lat2 * 57296 / 1000) * 
    //                    cos(radians(delta_lon)));
    bearing = degrees(atan2(delta_lon, delta_lat));
    //bearing = theta * 57296 / 1000;
    return bearing;
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void InitRadio(){
  Serial.println("Starting communication to ground!");
  Serial2.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);
  Serial2.setRxBufferSize(256);
  Serial2.setTxBufferSize(256);
  
  
  //delay(1000);
  Transceiver.init();
  //Transceiver.SetAddressH(1);
  //Transceiver.SetAddressL(0);
  Transceiver.SetMode(MODE_NORMAL);
  Chan = 23;
  Transceiver.SetChannel(Chan);
  //Transceiver.SetUARTBaudRate(9600);
  //Transceiver.SetAirDataRate(62500);
  Transceiver.SaveParameters(TEMPORARY);
  Serial.println(Transceiver.GetChannel());
  Serial.println(Transceiver.GetUARTBaudRate());
  Transceiver.PrintParameters();
}

void InitBMP(){
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    while (1) delay(10);
  }

  // bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  //                 Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  //                 Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
  //                 Adafruit_BMP280::FILTER_X8,      /* Filtering. */
  //                 Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */
  bmpOffset = bmp.readAltitude(atmPressure);
  Serial.println(F("Altimeter reset and is active!"));
  
}

void Flash(){
  for (int i = 3; i<9; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
    delay(500);
  }
  for (int i = 3; i<9; i++) {
    digitalWrite(i, LOW);
  }
  delay(500);
  for (int i = 3; i<9; i++) {
    digitalWrite(i, HIGH);
  }
  delay(500);
  for (int i = 3; i<9; i++) {
    digitalWrite(i, LOW);
  }
  delay(500);
}

//============
float GetYaw(){
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }

    // Serial.print(ypr.yaw);                Serial.print("\t");
    // Serial.print(ypr.pitch);              Serial.print("\t");
    // Serial.println(ypr.roll);
    return ypr.yaw;
  }
}

void loop() {

  //FeedbackData.yaw = GetYaw();
  Serial.print("Yaw: ");
  Serial.println(FeedbackData.yaw);

  // Serial.print(F(" DIR=")); Serial.print(GetBearing(targetLat, targetLon, atof(pdop.value())/100, atof(hdop.value())/100));
  // Serial.print(F(" LAT_T=")); Serial.print(targetLat,7);
  // Serial.print(F(" LON_T=")); Serial.print(targetLon,7);
  Serial.print(F(" LAT=")); Serial.print(atof(pdop.value())/100,7);
  Serial.print(F("\tLON=")); Serial.println(atof(hdop.value())/100,7);
  delay(100);

  while (Serial1.available() > 0){
    gps.encode(Serial1.read());
  }


  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();

  // FeedbackData.altitude = 100;
  // Serial.print("To get: ");
  // Serial.println(Transceiver.available());
  // if(Transceiver.available()){
    
  //   Transceiver.GetStruct(&ControlData, sizeof(ControlData));
  //   Serial.print("Got X: ");
  //   Serial.println(ControlData.X);
  //   //Last = millis();

  // }
  
  // delay(100);
  // if(Serial2.availableForWrite()){
  //   Serial.print("Available: ");
  //   Serial.println(Serial2.availableForWrite());
  //   Transceiver.SendStruct(&FeedbackData, sizeof(FeedbackData));
  //   Serial.print("Available: ");
  //   Serial.println(Serial2.availableForWrite());
  // }
  // delay(100);

    

}

void GetBMPStatus(){
    //Serial.print(F("Temperature = "));
    temperature = bmp.readTemperature();

    //Serial.print(F("Pressure = "));
    pressure = bmp.readPressure();

    //Serial.print(F("Approx altitude = "));
    altitude = bmp.readAltitude(atmPressure) - bmpOffset;
}

//============
