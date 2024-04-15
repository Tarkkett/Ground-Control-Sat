#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>
#include <cmath>
#include <Adafruit_BME280.h>


//=BME280=Temp=Pressure=Humidity
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

//=XBEE3=radio=
#define Xbee_Baud 57600

//SG90nMG90=servos=
#define PIN_A2 4 //high
#define PIN_A3 5 //low

Servo servoX, servoY;

int posX = 0;
int posY = 0;

//BNO08x=IMU=
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

//NEO-8M=GPS=

TinyGPSPlus gps;
static const int RXPinGPS = 2, TXPinGPS = 3;
static const uint32_t GPSBaud = 57600;


float targetLat = 54.67578568748265;//54.67836624970636;
float targetLon = 25.223520410616086;//25.263374017806708;

float delta_lon, delta_lat;
float bearing = 0;

TinyGPSCustom pdop(gps, "GNGLL", 1);
TinyGPSCustom hdop(gps, "GNGLL", 3);

//Send/Receive parameters for XBEE
struct RECEIVE_DATA
{
  float theta;
  int idle;
};

struct SEND_DATA
{
  float longtitude;
  float latitude;
  float sat_cnt;
  float seconds;
  float minutes;

  float temperature;
  float pressure;
  float altitude;

  float roll;
  float pitch;
  float yaw;

  float idle;
};

RECEIVE_DATA ControlData;
SEND_DATA FeedbackData;

//============

void setup() {
    Serial.begin(115200);
    // while (!Serial){
    //   delay(10);
    // } 
    pinMode(0, OUTPUT);
    //pinMode(1, INPUT);
    Serial.println(" ");
    Serial.println("Arduino Nano Esp32-S3 booted successfully!");
    Serial.println("Starting all systems...");
    // //Flash();
    InitRadio();
    InitIMU();
    InitGPS();
    InitServos();
    InitBME();
    // delay(3000);
    
}

void InitServos(){
  // servoX.write(posX);
  // servoY.write(posY);
  servoX.attach(PIN_A2);
  servoY.attach(PIN_A3);
  Serial.println("Servos reset!");
}

void InitGPS(){
  Serial1.begin(GPSBaud, SERIAL_8N1, RXPinGPS, TXPinGPS);
  while(!Serial1){
    Serial.println("Warning! Waiting for GPS...");
    delay(500);
  };
  Serial.println("Gps Port open! Seeking data...");
}

void InitIMU(){
    while (!bno08x.begin_I2C()) {
    Serial.println("Warning! Waiting For IMU..");
    delay(500);
  }
  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);
}

void InitRadio(){
  Serial.println("Starting communication to ground!");
  Serial0.begin(Xbee_Baud);
  while (!Serial0) {
    Serial.println("Can't open data transfer port RX/TX on Serial0!");
  }
  Serial.println("Communication to ground success!");

}

/*void Flash(){
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
}*/

void InitBME(){
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1) delay(10);
  }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();
  Serial.println("BMR sensors init() success!");
}

float GetBearing(float lat1, float lon1, float lat2, float lon2){

    delta_lon = lon1 - lon2;
    delta_lat = lat1 - lat2;
    /*theta = atan2(sin(delta_lon * 57296 / 1000) * cos(lat2 * 57296 / 1000),
                       cos(lat1 * 57296 / 1000) * sin(lat2 * 57296 / 1000) - 
                       sin(lat1 * 57296 / 1000) * cos(lat2 * 57296 / 1000) * 
                       cos(radians(delta_lon)));*/
    bearing = atan2(delta_lon, delta_lat);
    //bearing = theta * 57296 / 1000;
    return bearing;
}

void GetBMEData(sensors_event_t *temp_event, sensors_event_t *pressure_event, sensors_event_t *humidity_event){
  bme_temp->getEvent(temp_event);
  bme_pressure->getEvent(pressure_event);
  bme_humidity->getEvent(humidity_event);
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

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
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

float GetPitch(){
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
    return ypr.pitch;
  }
}

float GetRoll(){
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
    return ypr.roll;
  }
}

float MapToFloat(float x, float in_min, float in_max, long out_min, long out_max){
  return (float)(x - in_min)*(out_max - out_min) / (float)(in_max - in_min) + out_min;
}

//input Deg
void SetServos(int angle){
  float sin = std::sin(radians(angle));
  float cos = std::cos(radians(angle));
  Serial.print("Sin: ");Serial.println(sin);
  Serial.print("Cos: ");Serial.println(cos);
  servoX.write(int(MapToFloat(sin, -1, 1, 0, 180)));
  servoY.write(int(MapToFloat(cos, -1, 1, 0, 180)));
  Serial.print("Sin mapped: ");Serial.println(MapToFloat(sin, -1, 1, 0, 180));
  Serial.print("Cos mapped: ");Serial.println(MapToFloat(cos, -1, 1, 0, 180));
}


void loop() {

  while (Serial1.available() > 0){
    gps.encode(Serial1.read());
  }

  if (gps.location.isValid())
  {
    FeedbackData.latitude = gps.location.lat();
    FeedbackData.longtitude = gps.location.lng();
  }
  else{
    //Serial.println("INVALID!");
  }

  sensors_event_t temp_event, pressure_event, humidity_event;
  FeedbackData.yaw = GetYaw();
  GetBMEData(&temp_event, &pressure_event, &humidity_event);
  //Serial.print("X: "); Serial.println(sin(GetBearing(targetLat, targetLon, FeedbackData.latitude, FeedbackData.longtitude) + radians(GetYaw())));
  //Serial.print("Y: "); Serial.println(cos(GetBearing(targetLat, targetLon, FeedbackData.latitude, FeedbackData.longtitude) + radians(GetYaw())));

  servoX.write(MapToFloat(sin(radians(GetBearing(targetLat, targetLon, FeedbackData.latitude, FeedbackData.longtitude)) + radians(GetYaw())), 1, -1, 0, 180));
  servoY.write(MapToFloat(cos(radians(GetBearing(targetLat, targetLon, FeedbackData.latitude, FeedbackData.longtitude))  + radians(GetYaw())), 1, -1, 0, 180)); 
  //Serial.print(GetYaw());Serial.print("/");Serial.print(gps.location.lat());Serial.print("/");Serial.print(gps.location.lng());Serial.print("/");Serial.print(GetBearing(targetLat, targetLon, FeedbackData.latitude, FeedbackData.longtitude));Serial.print("/");Serial.println(gps.satellites.value());
  Serial0.print(GetYaw());Serial0.print("/");Serial0.print(temp_event.temperature);Serial0.print("/");Serial0.print(pressure_event.pressure);Serial0.print("/");Serial0.print(humidity_event.relative_humidity);Serial0.print("/");Serial0.print(MapToFloat(sin(0 + radians(GetYaw())), -1, 1, 0, 180));Serial0.print("/");Serial0.print(GetBearing(targetLat, targetLon, FeedbackData.latitude, FeedbackData.longtitude));//Serial0.print("/");Serial0.println(gps.satellites.value());
  Serial0.print("/");Serial0.print(gps.location.lat(), 6);Serial0.print("/");Serial0.println(gps.location.lng(), 6);
  delay(100);

  //SetServos(GetYaw());

    

}
