#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>
#include <cmath>
#include <Adafruit_BME280.h>
#include <AceRoutine.h>

using namespace ace_routine;

//=BME280=Temp=Pressure=Humidity
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

//=XBEE3=radio=
#define Xbee_Baud 57600

#define MAX_MESSAGE_LENGTH 50 // Maximum length of the message

char message[MAX_MESSAGE_LENGTH]; // Array to store the received message
int messageIndex = 0;


bool isSending = false;
bool isReceiving = true;

//=OpenLog=SD=
#define SD_RX 7
#define SD_TX 8

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


float targetLat = 54.67744326669359; //54.67836624970636;
float targetLon = 25.26362502137699;//25.263374017806708;

float currentLat = 54.67847666414606;
float currentLon = 25.263313505862428;

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
  float humidity;
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
    delay(1000);
    Serial.println(" ");
    Serial.println("Arduino Nano Esp32-S3 booted successfully!");
    Serial.println("Starting all systems...");
    InitSD();
    InitRadio();
    InitIMU();
    InitGPS();
    InitServos();
    InitBME();
    delay(1000);
    Serial.println("Begin!");

}
void InitSD(){
  Serial2.begin(57600, SERIAL_8N1, SD_RX, SD_TX);
  while (!Serial2) {
    Serial.println("Warning! Waiting for SD...");
  }
  Serial.println("SD Port open! Logging data!...");
}

void InitServos(){
  servoX.write(posX);
  servoY.write(posY);
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

void InitBME(){
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1) delay(10);
  }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();
  Serial.println("BME sensors init() success!");
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

int count = 0;

COROUTINE(transmit) {
  COROUTINE_LOOP(){
    //Serial0.print(GetYaw());Serial0.print("/");Serial0.print(temp_event.temperature);Serial0.print("/");Serial0.print(pressure_event.pressure);Serial0.print("/");Serial0.print(humidity_event.relative_humidity);Serial0.print("/");Serial0.print(MapToFloat(sin(0 + radians(GetYaw())), -1, 1, 0, 180));Serial0.print("/");Serial0.print(degrees(GetBearing(targetLat, targetLon, currentLat, currentLon)));//Serial0.print("/");Serial0.println(gps.satellites.value());
    //Serial0.print("/");Serial0.print(gps.location.lat(), 6);Serial0.print("/");Serial0.print(gps.location.lng(), 6);Serial0.print("/X::");Serial0.print(MapToFloat(sin(radians(GetBearing(targetLat, targetLon, currentLat, currentLon)) + radians(GetYaw())), 1, -1, 0, 180));Serial0.print("/Y::");Serial0.println(MapToFloat(cos(radians(GetBearing(targetLat, targetLon, currentLat, currentLon)) + radians(GetYaw())), 1, -1, 0, 180));
    if(isSending){
      Serial0.print(FeedbackData.yaw);Serial0.print("/"); Serial0.println(FeedbackData.temperature);
      Serial0.flush();
      
    }


    COROUTINE_DELAY(60);  
  }

}

COROUTINE(getIMUReadings){
  COROUTINE_LOOP(){
    FeedbackData.yaw = GetYaw();
    FeedbackData.pitch = GetPitch();
    FeedbackData.roll = GetRoll();
    COROUTINE_DELAY(400);
  }
}

COROUTINE(getBMEReadings){
  COROUTINE_LOOP(){
    sensors_event_t temp_event, pressure_event, humidity_event;

    GetBMEData(&temp_event, &pressure_event, &humidity_event);
    
    FeedbackData.temperature = temp_event.temperature;
    FeedbackData.pressure = pressure_event.pressure;
    FeedbackData.humidity = humidity_event.relative_humidity;
    COROUTINE_DELAY(400);
  }
}

COROUTINE(logToSD){
  COROUTINE_LOOP(){
    Serial2.println("Implement");
    Serial2.flush();
    COROUTINE_DELAY(500);
  }
}


COROUTINE(fetchData){
  COROUTINE_LOOP(){
    while (Serial0.available() > 0) {

      char incomingByte = Serial0.read();

      if (incomingByte == '\n') {
        if (strcmp(message, "#?#") == 0) { // Check if the received message is "#?#"
              // Do something here if the received message is "#?#"
              isSending = true;
              Serial0.println("#!#2#");
              Serial.println("Sent start msg!");
            }
        
        message[messageIndex] = '\0';
        Serial.print("Received message: ");
        Serial.println(message);
        Serial0.flush();
        messageIndex = 0;
        
      } 
      else {

        if (messageIndex < MAX_MESSAGE_LENGTH - 1) {
          message[messageIndex] = incomingByte;
          messageIndex++;
        }
        else{
          Serial.println("Overflow!");
        }
      }
    }
    COROUTINE_DELAY(300);
    
  }
}


void loop() {

  transmit.runCoroutine();
  getIMUReadings.runCoroutine();
  getBMEReadings.runCoroutine();
  logToSD.runCoroutine();
  fetchData.runCoroutine();
//Serial.println(count);
  /*while (Serial1.available() > 0){
    gps.encode(Serial1.read());
  }

  if (gps.location.isValid())
  {
    FeedbackData.latitude = gps.location.lat();
    FeedbackData.longtitude = gps.location.lng();
  }

  sensors_event_t temp_event, pressure_event, humidity_event;

  GetBMEData(&temp_event, &pressure_event, &humidity_event);

  FeedbackData.yaw = GetYaw();
  FeedbackData.roll = GetRoll();
  FeedbackData.pitch = GetPitch();
  FeedbackData.pressure = pressure_event.pressure;

  //Serial.print("X: "); Serial.println(sin(GetBearing(targetLat, targetLon, FeedbackData.latitude, FeedbackData.longtitude) + radians(GetYaw())));
  //Serial.print("Y: "); Serial.println(cos(GetBearing(targetLat, targetLon, FeedbackData.latitude, FeedbackData.longtitude) + radians(GetYaw())));

  // servoX.write(MapToFloat(sin(radians(GetBearing(targetLat, targetLon, FeedbackData.latitude, FeedbackData.longtitude)) + radians(GetYaw())), 1, -1, 0, 180));
  // servoY.write(MapToFloat(cos(radians(GetBearing(targetLat, targetLon, FeedbackData.latitude, FeedbackData.longtitude))  + radians(GetYaw())), 1, -1, 0, 180)); 

  

  //Serial.print(GetYaw());Serial.print("/");Serial.print(gps.location.lat());Serial.print("/");Serial.print(gps.location.lng());Serial.print("/");Serial.print(GetBearing(targetLat, targetLon, FeedbackData.latitude, FeedbackData.longtitude));Serial.print("/");Serial.println(gps.satellites.value());
  Serial0.println("Alive from main loop!");
  // if (Serial0.available() > 0) {
  //   // read the incoming byte:
  //   incomingByte = Serial0.read();

  //   // say what you got:
  //   Serial.print("I received: ");
  //   Serial.println(incomingByte, DEC);
  // }
  if(isSending){
    Serial0.print(GetYaw());Serial0.print("/");Serial0.print(temp_event.temperature);Serial0.print("/");Serial0.print(pressure_event.pressure);Serial0.print("/");Serial0.print(humidity_event.relative_humidity);Serial0.print("/");Serial0.print(MapToFloat(sin(0 + radians(GetYaw())), -1, 1, 0, 180));Serial0.print("/");Serial0.print(degrees(GetBearing(targetLat, targetLon, currentLat, currentLon)));//Serial0.print("/");Serial0.println(gps.satellites.value());
    Serial0.print("/");Serial0.print(gps.location.lat(), 6);Serial0.print("/");Serial0.print(gps.location.lng(), 6);Serial0.print("/X::");Serial0.print(MapToFloat(sin(radians(GetBearing(targetLat, targetLon, currentLat, currentLon)) + radians(GetYaw())), 1, -1, 0, 180));Serial0.print("/Y::");Serial0.println(MapToFloat(cos(radians(GetBearing(targetLat, targetLon, currentLat, currentLon)) + radians(GetYaw())), 1, -1, 0, 180));
    
    Serial2.print(count); Log("->", false); Serial2.print(GetYaw());Log("/", false);Serial2.print(temp_event.temperature); Log("/", false); Serial2.print(pressure_event.pressure); Log("/", false); Serial2.print(humidity_event.relative_humidity); Log("/", false); Serial2.print(MapToFloat(sin(0 + radians(GetYaw())), -1, 1, 0, 180)); Log("/", false); Serial2.print(degrees(GetBearing(targetLat, targetLon, currentLat, currentLon)));
    Log("/", false); Serial2.print(gps.location.lat(), 6); Log("/", false); Serial2.print(gps.location.lng(), 6); Serial2.print("/X::");Serial2.print(MapToFloat(sin(radians(GetBearing(targetLat, targetLon, currentLat, currentLon)) + radians(GetYaw())), 1, -1, 0, 180));Serial2.print("/Y::");Serial2.println(MapToFloat(cos(radians(GetBearing(targetLat, targetLon, currentLat, currentLon)) + radians(GetYaw())), 1, -1, 0, 180));
    
    delay(100);
  }
  delay(30);
  

  //SetServos(GetYaw());
  */
}

void Log(String a, bool send){
  Serial2.print(a);
  if (send) {
    Serial0.print(a);
  }

}
