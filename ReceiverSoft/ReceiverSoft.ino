void setup() {
  Serial.begin(115200); // Initialize serial communication
}

void loop() {
  String receivedData = ""; // Variable to store received data

  // Read data from serial until "#?#\n" is received
  while (!receivedData.endsWith("#?#\n")) {
    while (Serial.available()) {
      char c = Serial.read();
      receivedData += c;
    }
  }

  // Find the index of "?" character
  int questionMarkIndex = receivedData.indexOf('?');

  // Check if "?" character is found
  if (questionMarkIndex != -1) {
    // Send "Hello world" back to Serial
    Serial.println("#!#2#100#50#\n");
  }
}
