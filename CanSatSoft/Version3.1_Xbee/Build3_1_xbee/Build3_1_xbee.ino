#include <Adafruit_BNO08x.h>
#include <sh2.h>
#include <sh2_SensorValue.h>
#include <sh2_err.h>
#include <sh2_hal.h>
#include <sh2_util.h>
#include <shtp.h>

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
#include <SparkFun_AS7331.h>


using namespace ace_routine;

//=BME280=Temp=Pressure=Humidity
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
float zeroPointAltitude = 0;
float atmPressure = 1024;

float verticalSpeed = 0;
float horizontalSpeed = 0;

//buzzer=pin=6
#define buzzerPin 6 
bool isBuzzing = false;

SfeAS7331ArdI2C myUVSensor;

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
bool controllerMode = false;
float servoXYArr[2];

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

int cnt = 0;

struct SEND_DATA
{
  double longtitude;
  double latitude;
  float sat_cnt;
  float seconds;
  float minutes;

  float temperature;
  float pressure;
  float humidity;
  float altitude;

  float UVA;
  float UVB;
  float UVC;

  float gs;

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
    // while(!Serial){
    //   delay(10);
    // }
    Serial.println(" ");
    Serial.println("Arduino Nano Esp32-S3 booted successfully!");
    Serial.println("Starting all systems...");
    InitSD();
    InitRadio();
    InitIMU();
    InitGPS();
    InitServos();
    InitBME();
    InitUV();
    pinMode(buzzerPin, OUTPUT);
    delay(1000);
    Serial.println("Begin!");

}

void InitUV(){

    if(myUVSensor.begin() == false) {
    Serial.println("UV Sensor failed to begin.");
    Serial.println("Halting...");
    while(1);
  }

  Serial.println("UV Sensor started!.");

  if(myUVSensor.prepareMeasurement(MEAS_MODE_CMD) == false) {
    Serial.println("Sensor did not get set properly.");
    Serial.println("Halting...");
    while(1);
  }

  Serial.println("UV Set mode to command.");
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
  zeroPointAltitude = bme.readAltitude(atmPressure);
  Serial.println("BME sensors init() success!");
}

float GetBearing(float lat1, float lon1, float lat2, float lon2){

    delta_lon = lon1 - lon2;
    delta_lat = lat1 - lat2;

    bearing = atan2(delta_lon, delta_lat);

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

float GetGs(){
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
    return sensorValue.un.gravity.z;
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

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double earthRadius = 6371000.0; // Radius of the Earth in meters
  double dLat = toRadians(lat2 - lat1);
  double dLon = toRadians(lon2 - lon1);
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(toRadians(lat1)) * cos(toRadians(lat2)) *
             sin(dLon / 2.0) * sin(dLon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return earthRadius * c;
}

double toRadians(double degree) {
  return degree * M_PI / 180.0;
}

COROUTINE(transmit) {
  COROUTINE_LOOP(){
    //Serial0.print(GetYaw());Serial0.print("/");Serial0.print(temp_event.temperature);Serial0.print("/");Serial0.print(pressure_event.pressure);Serial0.print("/");Serial0.print(humidity_event.relative_humidity);Serial0.print("/");Serial0.print(MapToFloat(sin(0 + radians(GetYaw())), -1, 1, 0, 180));Serial0.print("/");Serial0.print(degrees(GetBearing(targetLat, targetLon, currentLat, currentLon)));//Serial0.print("/");Serial0.println(gps.satellites.value());
    //Serial0.print("/");Serial0.print(gps.location.lat(), 6);Serial0.print("/");Serial0.print(gps.location.lng(), 6);Serial0.print("/X::");Serial0.print(MapToFloat(sin(radians(GetBearing(targetLat, targetLon, currentLat, currentLon)) + radians(GetYaw())), 1, -1, 0, 180));Serial0.print("/Y::");Serial0.println(MapToFloat(cos(radians(GetBearing(targetLat, targetLon, currentLat, currentLon)) + radians(GetYaw())), 1, -1, 0, 180));
    if(isSending){
      Serial0.print("#D#");
      Serial0.print(FeedbackData.latitude, 6);
      Serial0.print("#"); 
      Serial0.print(FeedbackData.longtitude, 6);
      Serial0.print("#"); 
      Serial0.print(FeedbackData.yaw);
      Serial0.print("#"); 
      Serial0.print(FeedbackData.pitch);
      Serial0.print("#"); 
      Serial0.print(FeedbackData.roll);
      Serial0.print("#");
      Serial0.print(FeedbackData.temperature);
      Serial0.print("#"); 
      Serial0.print(FeedbackData.humidity);
      Serial0.print("#"); 
      Serial0.print(FeedbackData.pressure);
      Serial0.print("#"); 
      Serial0.print(FeedbackData.altitude);
      Serial0.print("#"); 
      Serial0.print(FeedbackData.altitude - zeroPointAltitude);
      Serial0.print("#");
      Serial0.print(FeedbackData.UVA); 
      Serial0.print("#");
      Serial0.print(FeedbackData.UVB);
      Serial0.print("#");
      Serial0.print(FeedbackData.UVC); 
      Serial0.print("#");
      Serial0.print(verticalSpeed); 
      Serial0.print("#"); 
      Serial0.print(horizontalSpeed); 
      Serial0.print("#");
      Serial0.print(FeedbackData.gs); 
      Serial0.print("#");
      Serial0.print("10.0"); 
      Serial0.println("#");
      Serial0.flush();
    }


    COROUTINE_DELAY(150);  
  }

}

COROUTINE(getGPS){
  COROUTINE_LOOP(){
    
    while (Serial1.available() > 0){
      gps.encode(Serial1.read());
    }

    if (gps.location.isValid())
    {
      FeedbackData.latitude = gps.location.lat();
      FeedbackData.longtitude = gps.location.lng();
    }
    COROUTINE_DELAY(200);
  }
}

COROUTINE(getIMUReadings){
  COROUTINE_LOOP(){
    FeedbackData.yaw = GetYaw();
    FeedbackData.pitch = GetPitch();
    FeedbackData.roll = GetRoll();
    FeedbackData.gs = GetGs();
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
    FeedbackData.altitude = bme.readAltitude(atmPressure);
    
    COROUTINE_DELAY(400);
  }
}

COROUTINE(logToSD){
  COROUTINE_LOOP(){
    Serial2.print("#D#");
    Serial2.print(FeedbackData.UVA); 
    Serial2.print("#"); 
    Serial2.print(FeedbackData.latitude, 6); 
    Serial2.print("#"); 
    Serial2.print(FeedbackData.longtitude, 6); 
    Serial2.print("#"); 
    Serial2.print(FeedbackData.yaw);
    Serial2.print("#"); 
    Serial2.print(FeedbackData.pitch);
    Serial2.print("#"); 
    Serial2.print(FeedbackData.roll);
    Serial2.print("#");
    Serial2.print(FeedbackData.temperature);
    Serial2.print("#");
    Serial2.print(FeedbackData.humidity);
    Serial2.print("#"); 
    Serial2.print(FeedbackData.pressure); 
    Serial2.print("#"); 
    Serial2.print(horizontalSpeed); 
    Serial2.print("#"); 
    Serial2.print(verticalSpeed); 
    Serial2.print("#"); 
    Serial2.print("10.0"); 
    Serial2.println("#");
    Serial2.flush();
    COROUTINE_DELAY(500);
  }
}

COROUTINE(buzz){
  COROUTINE_LOOP(){
    if (isBuzzing) {
        digitalWrite(buzzerPin, HIGH);
        COROUTINE_DELAY(1000);
        digitalWrite(buzzerPin, LOW);
        COROUTINE_DELAY(1000);
    }
    else {
      digitalWrite(buzzerPin, LOW);
    }

    COROUTINE_DELAY(2000);
  }
}

double lastAltitude = 0.0;
double lastLat = 0.0000000;
double lastLon = 0.0000000;
double changeLat = 0;
double changeLon = 0;
double squaredChangeLat = 0;
double squaredChangeLon = 0;

// double x1 = 0.000;
// double y1 = 0.000;

// double x2 = 0.000;
// double y2 = 0.000;

double cLat = 0;
double cLon = 0;
int counter = 0;
double radius = 6371.0;

COROUTINE(getSpeed){

  COROUTINE_LOOP() {

    Serial.println("RESET");
    float errorV = 0;
    double errorH = 0;
    lastAltitude = FeedbackData.altitude;
    
    lastLat = FeedbackData.latitude;
    lastLon = FeedbackData.longtitude;



    

    COROUTINE_DELAY(8000);


    // x2 = radius * cos(lastLat) * cos(lastLon);
    // y2 = radius * cos(lastLat) * sin(lastLon);

    // changeLat = x2 - x1;
    // // changeLon = y2 - y1;

    // errorH = std::sqrt(std::pow(changeLat, 2) + std::pow(changeLon, 2));

    // Serial.println(FeedbackData.latitude, 6);
    // Serial.println(lastLat, 6);
    // changeLat = FeedbackData.latitude - lastLat;
    // changeLon = FeedbackData.longtitude - lastLon;
    // Serial.println(changeLat, 6);
    
    // squaredChangeLat = changeLat * changeLat;
    // squaredChangeLat = changeLat * changeLat;
    // Serial.print("Squared: "); Serial.println(squaredChangeLat, 13);
    // Serial.println(" ");
    errorH = calculateDistance(FeedbackData.latitude, FeedbackData.longtitude, lastLat, lastLon);
    //errorH = std::sqrt(squaredChangeLat + squaredChangeLon);



    // changeLat = FeedbackData.latitude - lastLat;
    // changeLon = FeedbackData.longtitude - lastLon;
    // squaredChangeLat = changeLat * changeLat;
    // squaredChangeLon = changeLon * changeLon;
    // Serial.print("Powered: "); Serial.println(squaredChangeLat, 6);
    // errorH = std::sqrt(squaredChangeLat + squaredChangeLon);
    // errorV = FeedbackData.altitude - lastAltitude;
    // // Update vel
    // Serial.println(std::pow(FeedbackData.latitude - lastLat, 2), 6);
    // Serial.println(lastLat, 6);
    // Serial.println(FeedbackData.latitude, 6);
    // verticalSpeed = errorV;
    // horizontalSpeed = std::abs(errorH);
    Serial.print("Finished: "); Serial.println(errorH, 12); Serial.print(" "); Serial.println(FeedbackData.latitude);
  }
}

COROUTINE(controlServos){
  COROUTINE_LOOP(){

    if (controllerMode) {
      //Full manual
      servoX.write(MapToFloat(servoXYArr[0], -1, 1, 0, 180));
      servoY.write(MapToFloat(servoXYArr[1], -1, 1, 0, 180));
      Serial.println("Controller mode!!");
      Serial.println(servoXYArr[0]);
      Serial.println(servoXYArr[1]);
    }
    else {
      servoX.write(90);
      servoY.write(90);
      //Serial.println("GPS mode!!");
    }
    
          


    COROUTINE_DELAY(100);
  }
}

COROUTINE(getUV){
  COROUTINE_LOOP(){
    if(kSTkErrOk != myUVSensor.setStartState(true))
      Serial.println("Error starting reading!");
    
    // Wait for a bit longer than the conversion time.
    COROUTINE_DELAY(2+myUVSensor.getConversionTimeMillis());

    // Read UV values.
    if(kSTkErrOk != myUVSensor.readAllUV())
      Serial.println("Error reading UV.");
      
    FeedbackData.UVA = myUVSensor.getUVA();

    FeedbackData.UVB = myUVSensor.getUVB();

    FeedbackData.UVC = myUVSensor.getUVC();

    COROUTINE_DELAY(2000);
  }
}


COROUTINE(fetchData){
  COROUTINE_LOOP(){
    while (Serial0.available() > 0) {

      //Gautos žinutės apdorojimas
      String message = Serial0.readStringUntil('\n');
      Serial.print("Got: ");
      Serial.println(message);
      if(message.charAt(0) == '#'){

        //Autorizuojamas žemės stoties prisijungimas
        if (message.charAt(1) == '?') {
          Serial0.println("#!#4#");
          Serial.println("Sync success!");
        }
        //Pradedam/sustabdom duomenų siuntimą
        else if(message.charAt(1) == 'A'){
          isSending = true;
          Serial.println("Start send!");
        }
        else if(message.charAt(1) == 'S'){
          isSending = false;
          //Serial0.println("#S!#");
          Serial.println("Stop send!");
        }
        //Nuskaitomi: valdymo rėžimas ir valdymo pulto duomenys 
        else if(message.charAt(1) == 'C'){
          controllerMode = true;
          int startIndex = message.indexOf('#') + 3;
          int endIndex = message.indexOf('#', startIndex);

          servoXYArr[0]=message.substring(startIndex, endIndex).toFloat();

          startIndex = endIndex + 1;

          endIndex = message.indexOf('#', startIndex);

          servoXYArr[1]=message.substring(startIndex, endIndex).toFloat();
          Serial.println("Controller mode!");
        }
        else if (message.charAt(1) == 'G') {
          controllerMode = false;
          //Serial.println("GPS mode!");
        }
        //Pradedame/sustabdome pypsėjimą
        else if (message.charAt(1) == 'B') {
          isBuzzing = true;
          //Serial.println("Buzzing!");
        }
        else if (message.charAt(1) == 'N') {
          isBuzzing = false;
          //Serial.println("NOT Buzzing!");
        }
      }
    }
    //Palaukiame(Šis delay neužrakina pagrindinio Thread), nes
    //nenorime užtvindyti radijo modulio Buffer'io
    COROUTINE_DELAY(160);
  }
}


void loop() {

  //Užduotys vadinasi Coroutines
  transmit.runCoroutine();
  getIMUReadings.runCoroutine();
  getBMEReadings.runCoroutine();
  logToSD.runCoroutine();
  fetchData.runCoroutine();
  controlServos.runCoroutine();
  buzz.runCoroutine();
  getUV.runCoroutine();
  getGPS.runCoroutine();
  getSpeed.runCoroutine();

}

void Log(String a, bool send){
  Serial2.print(a);
  if (send) {
    Serial0.print(a);
  }

}
