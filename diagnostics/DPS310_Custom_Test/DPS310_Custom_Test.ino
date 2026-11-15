#include <Arduino.h>
#include <Wire.h>
#include "DPS310_Simple.h"

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

DPS310_Simple dps;

void setup() {
  Serial.begin(115200);
  delay(2500); // Wait for serial monitor

  Serial.println("\n============================================================");
  Serial.println("         CUSTOM DPS310 DIRECT DRIVER TEST (ESP32-S3)        ");
  Serial.println("============================================================");

  // Initialize Wire at 40 kHz (matches your working scanner)
  Serial.printf("Initializing I2C on SDA: %d, SCL: %d at 40 kHz... ", I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 40000);
  Wire.setTimeOut(100);
  Serial.println("Done.");

  // Initialize DPS310 using the custom driver
  Serial.print("Connecting to DPS310... ");
  if (dps.begin(Wire, 0x7)) {
    Serial.printf("SUCCESS! Connected at I2C address 0x%02X (Product ID: 0x%02X)\n", 
                  dps.getAddress(), dps.getProductID());
    Serial.println("Streaming live calibrated pressure and temperature at 10 Hz...\n");
  } else {
    Serial.println("FAILED to initialize DPS310!");
    Serial.println("Diagnostic Checklist:");
    Serial.println(" 1. Check if GND or 3.3V jumper has a loose crimp.");
    Serial.println(" 2. Make sure BNO085 is unplugged to isolate the bus.");
    Serial.println(" 3. Check if address is 0x76 instead of 0x77.");
  }
}

void loop() {
  if (dps.isReady()) {
    float pressure = 0.0f;
    float temperature = 0.0f;

    if (dps.readSensor(pressure, temperature)) {
      float altitude = dps.calculateAltitude(pressure);
      Serial.printf("[DPS310] Pressure: %7.2f hPa | Temp: %5.1f C | Est Alt: %6.1f m\n",
                    pressure, temperature, altitude);
    } else {
      Serial.println("[DPS310] Read error / timeout");
    }
  }
  delay(100); // 10 Hz
}
