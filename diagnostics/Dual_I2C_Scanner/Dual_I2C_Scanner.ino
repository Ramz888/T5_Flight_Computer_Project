#include <Arduino.h>
#include <Wire.h>

#define PRIMARY_SDA   8
#define PRIMARY_SCL   9
#define SECONDARY_SDA 40
#define SECONDARY_SCL 39

const char* identifyDevice(uint8_t addr, const char* busName) {
  if (addr == 0x4B || addr == 0x4A) return "BNO085 (9-DoF IMU)";
  if (addr == 0x77) {
    if (strcmp(busName, "Wire (Primary)") == 0) return "DPS310 Barometer (Primary bus)";
    else return "MS5611 Barometer (Secondary bus)";
  }
  if (addr == 0x76) return "DPS310 / BMP280 alternate address";
  if (addr == 0x1E) return "HMC5883L Compass";
  return "Unknown Device";
}

void scanBus(TwoWire &wirePort, const char* busName, int sda, int scl) {
  Serial.printf("\nScanning %s [SDA: %d, SCL: %d] at 40 kHz...\n", busName, sda, scl);
  int devicesFound = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    wirePort.beginTransmission(addr);
    byte error = wirePort.endTransmission();

    if (error == 0) {
      Serial.printf("  -> [FOUND] 0x%02X (%3d) : %s\n", addr, addr, identifyDevice(addr, busName));
      devicesFound++;
    } else if (error == 4) {
      Serial.printf("  -> [ERROR] Bus error on address 0x%02X\n", addr);
    }
  }

  if (devicesFound == 0) {
    Serial.println("  -> [EMPTY] No I2C devices responded on this bus.");
  } else {
    Serial.printf("  -> Total devices found on %s: %d\n", busName, devicesFound);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n========================================================");
  Serial.println("       ESP32-S3 DUAL I2C BUS HIGH-RELIABILITY SCANNER   ");
  Serial.println("========================================================");

  Wire.begin(PRIMARY_SDA, PRIMARY_SCL, 40000);
  Wire.setTimeOut(100);

  Wire1.begin(SECONDARY_SDA, SECONDARY_SCL, 40000);
  Wire1.setTimeOut(100);
}

void loop() {
  scanBus(Wire, "Wire (Primary)", PRIMARY_SDA, PRIMARY_SCL);
  scanBus(Wire1, "Wire1 (Secondary)", SECONDARY_SDA, SECONDARY_SCL);

  Serial.println("\n--------------------------------------------------------");
  Serial.println("Scan complete. Rescanning in 5 seconds...");
  delay(5000);
}
