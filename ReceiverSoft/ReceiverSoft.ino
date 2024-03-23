#include <SoftwareSerial.h>
#include "EBYTE.h"

#define PIN_RX 2
#define PIN_TX 3
#define PIN_M0 4
#define PIN_M1 5
#define PIN_AX 6

int communicating = 0;


struct DATA_REC {
  int Count;

};
struct DATA_SEND {
  int Cooldown;

};

int Chan;
DATA_REC MyData_receive;
DATA_SEND MyData_send;
unsigned long Last;

SoftwareSerial ESerial(PIN_RX, PIN_TX);
EBYTE Transceiver(&ESerial, PIN_M0, PIN_M1, PIN_AX);

void setup() {
  Serial.begin(9600);
  ESerial.begin(9600);
  Serial.println("Starting Reader");
  Transceiver.init();

  Transceiver.PrintParameters();
}

int cooldown = 9999;

void loop() {
  // String receivedData = ""; // Variable to store received data

  // // Read data from serial until "#?#\n" is received
  // while (!receivedData.endsWith("#?#\n")) {
  //   while (Serial.available()) {
  //     char c = Serial.read();
  //     receivedData += c;
  //   }
  // }

  // // Find the index of "?" character
  // int questionMarkIndex = receivedData.indexOf('?');

  // // Check if "?" character is found
  // if (questionMarkIndex != -1) {
  //   // Send "Hello world" back to Serial
  //   Serial.println("#!#2#100#50#\n");
  // }
  if (ESerial.available()) {

    Transceiver.GetStruct(&MyData_receive, sizeof(MyData_receive));

    // dump out what was just received
    Serial.print("Count: "); Serial.println(int(MyData_receive.Count));
    // Serial.print("Bits: "); Serial.println(MyData.Bits);
    // Serial.print("Volts: "); Serial.println(MyData.Volts);
    // if you got data, update the checker
    Last = millis();

  }
  else {
    // if the time checker is over some prescribed amount
    // let the user know there is no incoming data
    if ((millis() - Last) > 1000) {
      Serial.println("Searching: ");
      Last = millis();
    }
    else{
      //Serial.println("Wait..");
    }

  }
  ESerial.stopListening();
  delay(300);
  // if (ESerial.availableForWrite()) {

  cooldown -=1;
  MyData_send.Cooldown = cooldown;
  Transceiver.SendStruct(&MyData_send, sizeof(MyData_send));
  Serial.print("Sent: "); Serial.println(MyData_send.Cooldown);
    
  //}
  ESerial.listen();
  delay(300);

}
