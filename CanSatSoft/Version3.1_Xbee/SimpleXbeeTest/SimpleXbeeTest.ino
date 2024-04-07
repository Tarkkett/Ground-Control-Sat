void setup() {
  Serial.begin(115200);
  Serial0.begin(57600);

}

void loop() {
  Serial0.println("OK!");
  delay(100);

}
