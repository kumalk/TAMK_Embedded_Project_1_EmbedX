#include <Wire.h>

// Define I2C address for the CMPS14 compass module
#define CMPS14_ADDRESS 0x60

void setup() {
  Wire.begin();             // Start I2C communication
  Serial.begin(9600);       // Start Serial Monitor
  Serial.println("CMPS14 simple test");
}

void loop() {
  // Start communication with the CMPS14
  Wire.beginTransmission(CMPS14_ADDRESS);  // Talk to device at address 0x60
  Wire.write(1);                           // Access register 1 (bearing high byte)
  Wire.endTransmission(false);             // Send the address, but keep connection open

  // Request 1 byte from the CMPS14
  Wire.requestFrom(CMPS14_ADDRESS, 1, true);  // Read 1 byte, then release the bus

  if (Wire.available() >= 1) {  // Check if data is available
    byte raw = Wire.read();     // Read one byte from register 1
    Serial.print("Raw bearing high byte: ");
    Serial.println(raw);
  }

  delay(500);  // Wait before next reading
}
