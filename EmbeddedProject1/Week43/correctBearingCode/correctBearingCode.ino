#include <Wire.h>

// Define I2C address for the CMPS14 compass module
#define CMPS14_ADDRESS 0x60

// --- Function to read and process the 16-bit bearing ---
float readBearing16Bit() {
  
  // 1. Set the starting register to 2 (16-bit High Byte)
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(2);
  Wire.endTransmission(); // Stop and release the bus

  // 2. Request 2 bytes (Register 2, then Register 3)
  if (Wire.requestFrom(CMPS14_ADDRESS, 2) == 2) {
    
    // Read the bytes in the order they are received (High then Low)
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    
    // Combine the two bytes into a 16-bit integer (raw value 0-3599)
    // We explicitly cast to int/unsigned int for proper bit shifting
    int rawBearing = ((unsigned int)highByte << 8) | lowByte;
    
    // Convert the raw value (tenths of a degree) to a float in degrees
    return (float)rawBearing / 10.0;
  }
  
  // Return 0.0 on read failure
  return 0.0;
}

void setup() {
  Wire.begin();             // Start I2C communication
  Serial.begin(9600);       // Start Serial Monitor
  Serial.println("CMPS14 Bearing Readout");
}

void loop() {
  float bearingDegrees = readBearing16Bit();

  // Print the final result
  Serial.print("Bearing: ");
  Serial.print(bearingDegrees, 1); // Print with 1 decimal place
  Serial.println(" deg");

  delay(500); // Wait before next reading
}