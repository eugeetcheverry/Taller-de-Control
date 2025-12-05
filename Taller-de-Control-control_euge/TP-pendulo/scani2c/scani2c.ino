#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nEscaneando I2C...");
  byte count = 0;
  for (byte address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Dispositivo I2C encontrado en 0x");
      Serial.println(address, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("No se detectó ningún dispositivo I2C.");
}

void loop() {}
