#include "i2c_scanner.h"

// Inspiration from https://learn.adafruit.com/scanning-i2c-addresses/arduino
void i2cScanner() {
  byte error, address;

  int nDevices;

  Serial.println("I2C Scanner");
  nDevices = 0;
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
      nDevices++;
    }
  }

  if(nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("...done\n");
}
