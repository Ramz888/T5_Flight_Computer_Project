/*
 ============================================================================
  ROCKET FLIGHT COMPUTER - HARDWARE & SENSOR SYSTEMS CHECK (DIAGNOSTIC SUITE)
 ============================================================================
  Reference Firmware: t5flightcompcode_intestingphase_ino_copy_20261023182507_v2.ino
  Target MCU: ESP32-S3 (Dual I2C, LittleFS, 3x UART, Dual Barometer, BNO08x IMU)

  SUBSYSTEMS TESTED:
  1. I2C Bus Scanner:
     - Wire  (SDA: 8,  SCL: 9)   -> BNO08x (0x4A/0x4B), DPS310 (0x77)
     - Wire1 (SDA: 40, SCL: 39)  -> MS5611 (0x77)
  2. BNO08x 9-DoF IMU:
     - Communication check, report activation, linear accel, quaternion norm
  3. DPS310 Primary Barometer:
     - Temperature, pressure, standard deviation noise benchmark (20 samples)
  4. MS5611 Redundant Barometer:
     - Temperature, pressure, comparison delta vs DPS310
  5. Dual-Barometer Fusion & Kalman Sanity Check:
     - Verifies barometers agree within acceptable tolerance (< 3.0 hPa / ~25m)
  6. GNSS / GPS Receiver (Serial2, RX: 42, TX: 41, 38400 baud):
     - NMEA feed verification, fix validity, satellites count, coordinates
  7. LittleFS Onboard Flash Memory:
     - Mount check, total/used flash storage, read/write latency test
  8. Pyrotechnic Continuity Loops (Sense Pins 1..4):
     - Drogue 1 (Pin 4), Drogue 2 (Pin 5), Main 1 (Pin 1), Main 2 (Pin 2)
  9. Pyrotechnic Charge Firing Pin Safety Interlock:
     - Verifies pins 6, 7, 15, 16 are strictly latched LOW (0V)
  10. HC-12 Telemetry Radio Link (Serial1, RX: 17, TX: 18, 115200 baud):
     - Broadcasts diagnostic report to ground station
  11. Visual / Audio Indicators:
     - Buzzer chime (Pin 36), NeoPixel RGB cycle (External: 38, Builtin)

  DIAGNOSTIC MODES:
  - Startup: Runs automated 11-point self-test and prints PASS/FAIL summary
  - Continuous Mode: Streams real-time live sensor data at 10 Hz
  - Interactive Commands (via USB Serial or HC-12 Radio):
      'R' or 'RUN'    -> Re-run full 11-point diagnostic check
      'S' or 'STREAM' -> Toggle continuous live sensor stream
      'I' or 'I2C'    -> Scan both I2C buses
      'P' or 'PYRO'   -> Re-check pyro continuity sense lines
      'G' or 'GPS'    -> Show GPS raw NMEA & fix diagnostics
      'B' or 'BUZZ'   -> Test buzzer and RGB LED
 ============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_DPS310.h>
#include <MS5611.h>
#include <TinyGPS++.h>
#include <FS.h>
#include <LittleFS.h>

// --------------------------------------------------------------------------
// Pin Definitions (Matches Reference Firmware Exactly)
// --------------------------------------------------------------------------
#define BNO08X_RESET -1
#define I2C0_SDA_PIN 8
#define I2C0_SCL_PIN 9
#define MS5611_SDA   40
#define MS5611_SCL   39

#define HC12_RX_PIN  17
#define HC12_TX_PIN  18
#define TELEMETRY_BAUD 115200

#define GPS_RX_PIN   42
#define GPS_TX_PIN   41
#define GPS_BAUD     38400

// Pyrotechnic Continuity Sense Pins (INPUT_PULLDOWN)
const int SENSE_PIN_1 = 4;  // Drogue 1
const int SENSE_PIN_2 = 5;  // Drogue 2
const int SENSE_PIN_3 = 1;  // Main 1
const int SENSE_PIN_4 = 2;  // Main 2

// Pyrotechnic Firing Pins (OUTPUT, LOW)
const int CHARGE_PIN_DROGUE_1 = 6;
const int CHARGE_PIN_DROGUE_2 = 7;
const int CHARGE_PIN_MAIN_1   = 15;
const int CHARGE_PIN_MAIN_2   = 16;

// Indicators
const int BUZZER_PIN  = 36;
const int LED_EXT_PIN = 38;

// --------------------------------------------------------------------------
// Sensor & Hardware Objects
// --------------------------------------------------------------------------
Adafruit_BNO08x bno08x(BNO08X_RESET);
Adafruit_DPS310 dps;
MS5611 ms5611(0x77, &Wire1);
TinyGPSPlus gps;

// Sensor Status Flags
struct SystemStatus {
  bool i2c0_ok = false;
  bool i2c1_ok = false;
  bool bno08x_ok = false;
  uint8_t bno_address = 0;
  bool dps310_ok = false;
  bool ms5611_ok = false;
  bool gps_uart_ok = false;
  bool gps_fix_ok = false;
  bool littlefs_ok = false;
  bool pyro_safe = false;
  bool drogue1_cont = false;
  bool drogue2_cont = false;
  bool main1_cont = false;
  bool main2_cont = false;
} sys_stat;

bool continuous_stream = false;
uint32_t last_stream_tick = 0;

// --------------------------------------------------------------------------
// Helper Output Functions
// --------------------------------------------------------------------------
void printDual(const String& str) {
  Serial.print(str);
  Serial1.print(str);
}

void printlnDual(const String& str = "") {
  Serial.println(str);
  Serial1.println(str);
}

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  #ifdef RGB_BUILTIN
  neopixelWrite(RGB_BUILTIN, r, g, b);
  #endif
  neopixelWrite(LED_EXT_PIN, r, g, b);
}

void beep(int ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
}

// --------------------------------------------------------------------------
// TEST 1: Dual I2C Bus Scanner
// --------------------------------------------------------------------------
void testI2CBuses() {
  printlnDual("\n------------------------------------------------------------");
  printlnDual("[TEST 1/10] I2C BUS HARDWARE SCAN");
  printlnDual("------------------------------------------------------------");

  // Scan Wire (Bus 0: Pins 8, 9)
  printDual("Scanning Primary I2C (Wire - SDA:8, SCL:9)... ");
  int dev_count0 = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (dev_count0 == 0) printDual("Devices found: ");
      printDual("0x" + String(addr, HEX) + " ");
      dev_count0++;
    }
  }
  if (dev_count0 > 0) {
    printlnDual("--> PASS (" + String(dev_count0) + " detected)");
    sys_stat.i2c0_ok = true;
  } else {
    printlnDual("--> FAIL (No devices found! Check 3.3V power & SDA/SCL pull-ups)");
    sys_stat.i2c0_ok = false;
  }

  // Scan Wire1 (Bus 1: Pins 40, 39)
  printDual("Scanning Secondary I2C (Wire1 - SDA:40, SCL:39)... ");
  int dev_count1 = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      if (dev_count1 == 0) printDual("Devices found: ");
      printDual("0x" + String(addr, HEX) + " ");
      dev_count1++;
    }
  }
  if (dev_count1 > 0) {
    printlnDual("--> PASS (" + String(dev_count1) + " detected)");
    sys_stat.i2c1_ok = true;
  } else {
    printlnDual("--> FAIL (No devices found on Wire1! Check MS5611 wiring)");
    sys_stat.i2c1_ok = false;
  }
}

// --------------------------------------------------------------------------
// Helper: I2C Bus Hardware Unwedge (Toggles SCL 9 times)
// --------------------------------------------------------------------------
void recoverI2CBus(int sda_pin, int scl_pin) {
  pinMode(sda_pin, INPUT_PULLUP);
  pinMode(scl_pin, OUTPUT);

  for (int i = 0; i < 9; i++) {
    digitalWrite(scl_pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(scl_pin, LOW);
    delayMicroseconds(5);
  }

  // Generate I2C STOP condition
  pinMode(sda_pin, OUTPUT);
  digitalWrite(sda_pin, LOW);
  delayMicroseconds(5);
  digitalWrite(scl_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(sda_pin, HIGH);
  delayMicroseconds(5);

  pinMode(sda_pin, INPUT);
  pinMode(scl_pin, INPUT);
}

// --------------------------------------------------------------------------
// Helper: BNO08x SHTP Protocol Soft Reset via Channel 0 (Executable Channel)
// --------------------------------------------------------------------------
bool sendSHTPSoftReset(uint8_t addr, TwoWire &wirePort = Wire) {
  // SHTP Channel 0 Executable Command: 0x01 = Reset
  uint8_t reset_pkt[] = { 0x05, 0x00, 0x00, 0x00, 0x01 };
  wirePort.beginTransmission(addr);
  wirePort.write(reset_pkt, sizeof(reset_pkt));
  uint8_t err = wirePort.endTransmission();
  if (err == 0) {
    delay(200); // Allow Cortex-M0 to complete reboot
    return true;
  }
  return false;
}

// --------------------------------------------------------------------------
// TEST 2: BNO08x 9-DoF IMU Diagnostic
// --------------------------------------------------------------------------
void testBNO08x() {
  printlnDual("\n------------------------------------------------------------");
  printlnDual("[TEST 2/10] BNO08x 9-DoF IMU DIAGNOSTIC & AUTO-RECOVERY");
  printlnDual("------------------------------------------------------------");

  sys_stat.bno08x_ok = false;
  sys_stat.bno_address = 0;

  // 1. Hardware reset pulse if pin defined
  if (BNO08X_RESET >= 0) {
    printDual("Pulsing physical hardware reset line... ");
    pinMode(BNO08X_RESET, OUTPUT);
    digitalWrite(BNO08X_RESET, LOW);
    delay(10);
    digitalWrite(BNO08X_RESET, HIGH);
    delay(200);
    printlnDual("Done.");
  }

  // 2. Scan both addresses (0x4A, 0x4B)
  printDual("Probing BNO08x address presence (0x4A / 0x4B)... ");
  bool found_addr = false;
  uint8_t target_addr = 0x4A;

  Wire.beginTransmission(0x4A);
  if (Wire.endTransmission() == 0) {
    printlnDual("Detected at 0x4A");
    target_addr = 0x4A;
    found_addr = true;
  } else {
    Wire.beginTransmission(0x4B);
    if (Wire.endTransmission() == 0) {
      printlnDual("Detected at 0x4B");
      target_addr = 0x4B;
      found_addr = true;
    } else {
      printlnDual("Neither 0x4A nor 0x4B acknowledged simple ping.");
    }
  }

  // 3. If unacknowledged, send SHTP Channel 0 Soft-Reset packets to unwedge Cortex-M0
  if (!found_addr) {
    printDual("Sending blind SHTP soft-reset to Channel 0 on 0x4A and 0x4B... ");
    bool ack_a = sendSHTPSoftReset(0x4A, Wire);
    bool ack_b = sendSHTPSoftReset(0x4B, Wire);
    if (ack_a || ack_b) {
      printlnDual("ACK received! SH-2 rebooting.");
    } else {
      printlnDual("No response.");
    }
  }

  // 4. Retry loop: attempt library begin_I2C up to 5 times
  for (int attempt = 1; attempt <= 5; attempt++) {
    printDual("Attempt " + String(attempt) + "/5: Initializing BNO08x library... ");

    if (bno08x.begin_I2C(0x4A, &Wire)) {
      printlnDual("OK (Connected at 0x4A)");
      sys_stat.bno_address = 0x4A;
      sys_stat.bno08x_ok = true;
      break;
    } else if (bno08x.begin_I2C(0x4B, &Wire)) {
      printlnDual("OK (Connected at 0x4B)");
      sys_stat.bno_address = 0x4B;
      sys_stat.bno08x_ok = true;
      break;
    } else {
      printlnDual("No response (retrying in 120ms)");
      delay(120);
    }
  }

  if (sys_stat.bno08x_ok) {
    printDual("Enabling SH2 Sensor Reports (Linear Accel & Rotation Vector)... ");
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000); // 100Hz
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);     // 100Hz
    bno08x.enableReport(SH2_ACCELEROMETER, 10000);       // Raw gravity vector
    printlnDual("OK");

    // Wait and sample 10 readings
    delay(150);
    sh2_SensorValue_t val;
    int samples = 0;
    float ax = 0, ay = 0, az = 0;
    float qi = 0, qj = 0, qk = 0, qr = 1.0;
    uint32_t t_start = millis();

    while (millis() - t_start < 600 && samples < 10) {
      if (bno08x.getSensorEvent(&val)) {
        if (val.sensorId == SH2_LINEAR_ACCELERATION) {
          ax = val.un.linearAcceleration.x;
          ay = val.un.linearAcceleration.y;
          az = val.un.linearAcceleration.z;
          samples++;
        } else if (val.sensorId == SH2_ROTATION_VECTOR) {
          qi = val.un.rotationVector.i;
          qj = val.un.rotationVector.j;
          qk = val.un.rotationVector.k;
          qr = val.un.rotationVector.real;
        }
      }
      delay(5);
    }

    float norm = sqrt(qi*qi + qj*qj + qk*qk + qr*qr);
    float lin_mag = sqrt(ax*ax + ay*ay + az*az);

    printlnDual("  * Samples acquired: " + String(samples));
    printlnDual("  * Linear Accel [X, Y, Z]: [" + String(ax, 2) + ", " + String(ay, 2) + ", " + String(az, 2) + "] m/s^2 (Mag: " + String(lin_mag, 2) + ")");
    printlnDual("  * Quaternion [i, j, k, real]: [" + String(qi, 3) + ", " + String(qj, 3) + ", " + String(qk, 3) + ", " + String(qr, 3) + "]");
    printlnDual("  * Quaternion Norm (|q|): " + String(norm, 4) + " (Target: ~1.0000)");

    if (samples > 0 && abs(norm - 1.0) < 0.05) {
      printlnDual("--> BNO08x RESULT: PASS (Sensor active, quaternion normalized)");
    } else if (samples > 0) {
      printlnDual("--> BNO08x RESULT: WARN (Packets received, but quaternion calibration pending)");
    } else {
      printlnDual("--> BNO08x RESULT: FAIL (Reports enabled but no sensor events received!)");
      sys_stat.bno08x_ok = false;
    }
  } else {
    printlnDual("--> BNO08x RESULT: FAIL (Sensor missing or failed to initialize)");
  }
}

// --------------------------------------------------------------------------
// TEST 3: DPS310 Primary High-Precision Barometer
// --------------------------------------------------------------------------
bool resetDPS310(uint8_t addr, TwoWire &wirePort = Wire) {
  // Step 1: Write 0x09 to RESET register (0x0C) to trigger full ASIC soft reset
  wirePort.beginTransmission(addr);
  wirePort.write(0x0C); // Reset register
  wirePort.write(0x09); // Soft reset value
  if (wirePort.endTransmission() != 0) return false;

  // Step 2: Wait at least 25-30 ms for ASIC reboot and OTP calibration reload
  delay(30);

  // Step 3: Verify Product ID register (0x0D)
  wirePort.beginTransmission(addr);
  wirePort.write(0x0D); // PROD_ID register
  if (wirePort.endTransmission(false) != 0) return false;
  if (wirePort.requestFrom(addr, (uint8_t)1) != 1) return false;
  uint8_t prod_id = wirePort.read();

  // Step 4: Wait for calibration coefficients to be ready (MEAS_CFG 0x08, bit 7 COEF_RDY)
  uint32_t t_start = millis();
  while (millis() - t_start < 60) {
    wirePort.beginTransmission(addr);
    wirePort.write(0x08); // MEAS_CFG
    if (wirePort.endTransmission(false) == 0 && wirePort.requestFrom(addr, (uint8_t)1) == 1) {
      if (wirePort.read() & 0x80) break; // COEF_RDY set
    }
    delay(5);
  }

  return ((prod_id & 0xF0) == 0x10 || prod_id == 0x10);
}

void testDPS310() {
  printlnDual("\n------------------------------------------------------------");
  printlnDual("[TEST 3/10] DPS310 PRIMARY BAROMETER (I2C: Wire, Addr: 0x77 / 0x76)");
  printlnDual("------------------------------------------------------------");

  sys_stat.dps310_ok = false;
  uint8_t dps_addr = 0x77;

  // Auto-detect address: check 0x77 first, then 0x76
  Wire.beginTransmission(0x77);
  if (Wire.endTransmission() == 0) {
    dps_addr = 0x77;
    printlnDual("Detected I2C ACK at primary address 0x77.");
  } else {
    Wire.beginTransmission(0x76);
    if (Wire.endTransmission() == 0) {
      dps_addr = 0x76;
      printlnDual("Detected I2C ACK at alternate address 0x76 (SDO grounded).");
    } else {
      printlnDual("WARN: Neither 0x77 nor 0x76 ACKed on Wire (Pins 8/9).");
    }
  }

  // Perform ASIC soft reset
  printDual("Performing ASIC Soft Reset on DPS310 (reg 0x0C = 0x09)... ");
  bool rst = resetDPS310(dps_addr, Wire);
  if (rst) {
    printlnDual("OK (ASIC rebooted, OTP calibration reloaded, ProdID: 0x10)");
  } else {
    printlnDual("WARN (No ProdID 0x10 match; attempting direct library begin)");
  }

  printDual("Initializing DPS310 via Adafruit library at 0x" + String(dps_addr, HEX) + "... ");
  if (dps.begin_I2C(dps_addr, &Wire)) {
    printlnDual("OK");
    sys_stat.dps310_ok = true;
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

    // Warm-up and collect 15 samples to benchmark noise
    delay(100);
    float p_sum = 0, t_sum = 0;
    float p_readings[15];
    int valid = 0;

    for (int i = 0; i < 15; i++) {
      uint32_t to = millis();
      while (!dps.pressureAvailable() && (millis() - to < 60)) { delay(2); }
      if (dps.pressureAvailable()) {
        sensors_event_t temp_evt, press_evt;
        dps.getEvents(&temp_evt, &press_evt);
        p_readings[valid] = press_evt.pressure;
        p_sum += press_evt.pressure;
        t_sum += temp_evt.temperature;
        valid++;
      }
      delay(20);
    }

    if (valid >= 5) {
      float p_mean = p_sum / valid;
      float t_mean = t_sum / valid;
      // Calculate standard deviation (noise)
      float var_sum = 0;
      for (int i = 0; i < valid; i++) {
        var_sum += sq(p_readings[i] - p_mean);
      }
      float p_std = sqrt(var_sum / valid);

      printlnDual("  * Valid samples: " + String(valid));
      printlnDual("  * Mean Pressure: " + String(p_mean, 2) + " hPa (mbar)");
      printlnDual("  * Mean Temperature: " + String(t_mean, 1) + " deg C");
      printlnDual("  * Pressure RMS Noise (std dev): " + String(p_std, 4) + " hPa (~" + String(p_std * 8.43, 2) + " cm noise)");

      if (p_mean >= 300.0 && p_mean <= 1100.0 && t_mean > -30.0 && t_mean < 80.0) {
        printlnDual("--> DPS310 RESULT: PASS (Nominal atmospheric pressure and temperature)");
      } else {
        printlnDual("--> DPS310 RESULT: FAIL (Values out of physical range: " + String(p_mean) + " hPa)");
        sys_stat.dps310_ok = false;
      }
    } else {
      printlnDual("--> DPS310 RESULT: FAIL (Sensor responded to init, but timeout waiting for pressure data)");
      sys_stat.dps310_ok = false;
    }
  } else {
    printlnDual("FAIL (DPS310 not detected at 0x77! Check address jumper & SDA:8/SCL:9)");
  }
}

// --------------------------------------------------------------------------
// TEST 4: MS5611 Secondary Redundant Barometer
// --------------------------------------------------------------------------
void testMS5611() {
  printlnDual("\n------------------------------------------------------------");
  printlnDual("[TEST 4/10] MS5611 REDUNDANT BAROMETER (I2C: Wire1 Pins 40/39, Addr: 0x77/0x76)");
  printlnDual("------------------------------------------------------------");

  sys_stat.ms5611_ok = false;
  uint8_t ms_addr = 0x77;
  bool wire1_found = false;

  // Step 1: Scan Wire1 for 0x77 and 0x76
  Wire1.beginTransmission(0x77);
  if (Wire1.endTransmission() == 0) {
    ms_addr = 0x77;
    wire1_found = true;
    printlnDual("Detected I2C ACK on Wire1 at address 0x77 (CSB to GND).");
  } else {
    Wire1.beginTransmission(0x76);
    if (Wire1.endTransmission() == 0) {
      ms_addr = 0x76;
      wire1_found = true;
      printlnDual("Detected I2C ACK on Wire1 at alternate address 0x76 (CSB to VDD).");
    }
  }

  // If not found on Wire1, check if accidentally wired to Wire (pins 8/9)
  if (!wire1_found) {
    printlnDual("WARN: No device ACKed on Wire1 (pins 40/39). Checking primary Wire (pins 8/9)...");
    Wire.beginTransmission(0x77);
    bool on_wire_77 = (Wire.endTransmission() == 0);
    Wire.beginTransmission(0x76);
    bool on_wire_76 = (Wire.endTransmission() == 0);
    if (on_wire_77 || on_wire_76) {
      printlnDual("NOTICE: An I2C device at 0x" + String(on_wire_77 ? "77" : "76") + " is present on Wire (pins 8/9), NOT Wire1 (pins 40/39)!");
      printlnDual("CRITICAL: If both DPS310 and MS5611 share pins 8/9 at address 0x77, they will collide and both fail!");
    }
  }

  printDual("Initializing MS5611 on Wire1 at 0x" + String(ms_addr, HEX) + "... ");
  if (ms5611.begin()) {
    printlnDual("OK");
    ms5611.reset(1);
    delay(100);
    sys_stat.ms5611_ok = true;

    float p_sum = 0, t_sum = 0;
    int valid = 0;

    for (int i = 0; i < 10; i++) {
      int result = ms5611.read();
      if (result == MS5611_READ_OK) {
        p_sum += ms5611.getPressure();
        t_sum += ms5611.getTemperature();
        valid++;
      }
      delay(30);
    }

    if (valid >= 5) {
      float p_mean = p_sum / valid;
      float t_mean = t_sum / valid;
      printlnDual("  * Valid samples: " + String(valid));
      printlnDual("  * Mean Pressure: " + String(p_mean, 2) + " hPa");
      printlnDual("  * Mean Temperature: " + String(t_mean, 1) + " deg C");

      if (p_mean >= 300.0 && p_mean <= 1100.0) {
        printlnDual("--> MS5611 RESULT: PASS (Nominal redundant barometer operation)");
      } else {
        printlnDual("--> MS5611 RESULT: FAIL (Pressure out of bounds: " + String(p_mean) + " hPa)");
        sys_stat.ms5611_ok = false;
      }
    } else {
      printlnDual("--> MS5611 RESULT: FAIL (read() returned error or timeout)");
      sys_stat.ms5611_ok = false;
    }
  } else {
    printlnDual("FAIL (MS5611 not detected on Wire1 at pins 40/39)");
  }
}

// --------------------------------------------------------------------------
// TEST 5: Dual Barometer Cross-Validation & Consistency
// --------------------------------------------------------------------------
void testBaroConsistency() {
  printlnDual("\n------------------------------------------------------------");
  printlnDual("[TEST 5/10] DUAL-BAROMETER CROSS-VALIDATION");
  printlnDual("------------------------------------------------------------");

  if (!sys_stat.dps310_ok || !sys_stat.ms5611_ok) {
    printlnDual("--> RESULT: SKIPPED (Requires both DPS310 and MS5611 to be active)");
    return;
  }

  // Sample both simultaneously
  sensors_event_t t_evt, p_evt;
  dps.getEvents(&t_evt, &p_evt);
  ms5611.read();

  float p_dps = p_evt.pressure;
  float p_ms  = ms5611.getPressure();
  float delta_p = abs(p_dps - p_ms);
  float delta_alt_m = delta_p * 8.43; // Approx 8.43m per hPa at sea level

  printlnDual("  * DPS310 Pressure : " + String(p_dps, 2) + " hPa");
  printlnDual("  * MS5611 Pressure : " + String(p_ms, 2) + " hPa");
  printlnDual("  * Pressure Delta  : " + String(delta_p, 3) + " hPa");
  printlnDual("  * Est Altitude Diff: ~" + String(delta_alt_m, 1) + " meters");

  if (delta_p <= 2.5) {
    printlnDual("--> DUAL BARO RESULT: PASS (Tightly matched < 2.5 hPa / ~20m)");
  } else if (delta_p <= 6.0) {
    printlnDual("--> DUAL BARO RESULT: WARN (Moderate offset < 6 hPa, Kalman filter will compensate)");
  } else {
    printlnDual("--> DUAL BARO RESULT: FAIL (Barometers disagree by > 6 hPa! Check calibration/venting)");
  }
}

// --------------------------------------------------------------------------
// TEST 6: GNSS / GPS Receiver Diagnostics
// --------------------------------------------------------------------------
void testGPS() {
  printlnDual("\n------------------------------------------------------------");
  printlnDual("[TEST 6/10] GNSS / GPS RECEIVER (Serial2, RX:42, TX:41, 38400)");
  printlnDual("------------------------------------------------------------");

  printDual("Listening for NMEA sentences on Serial2... ");
  uint32_t t_start = millis();
  int bytes_received = 0;
  while (millis() - t_start < 1500) {
    while (Serial2.available() > 0) {
      char c = Serial2.read();
      gps.encode(c);
      bytes_received++;
    }
    delay(2);
  }

  printlnDual("Done. (Bytes read: " + String(bytes_received) + ")");

  if (bytes_received > 20) {
    sys_stat.gps_uart_ok = true;
    printlnDual("  * UART Data Stream: OK (Valid NMEA characters arriving)");
    printlnDual("  * Checksum Passed : " + String(gps.passedChecksum()));
    printlnDual("  * Checksum Failed : " + String(gps.failedChecksum()));
    printlnDual("  * Characters Total: " + String(gps.charsProcessed()));
    printlnDual("  * Satellites in view: " + String(gps.satellites.isValid() ? gps.satellites.value() : 0));

    if (gps.location.isValid()) {
      sys_stat.gps_fix_ok = true;
      printlnDual("  * 3D FIX STATUS   : LOCKED (3D Fix Established!)");
      printlnDual("  * Latitude        : " + String(gps.location.lat(), 6) + " deg");
      printlnDual("  * Longitude       : " + String(gps.location.lng(), 6) + " deg");
      printlnDual("  * GPS Altitude    : " + String(gps.altitude.meters(), 1) + " m");
      printlnDual("  * HDOP Precision  : " + String(gps.hdop.hdop(), 1));
      printlnDual("--> GPS RESULT: PASS (Locked with coordinates)");
    } else {
      sys_stat.gps_fix_ok = false;
      printlnDual("  * 3D FIX STATUS   : SEARCHING (No 3D lock yet - normal if tested indoors)");
      printlnDual("--> GPS RESULT: PASS (UART link functional; acquire fix with clear view of sky)");
    }
  } else {
    sys_stat.gps_uart_ok = false;
    printlnDual("--> GPS RESULT: FAIL (No NMEA data received! Verify RX:42, TX:41 and 38400 baud)");
  }
}

// --------------------------------------------------------------------------
// TEST 7: LittleFS Flash Storage File System
// --------------------------------------------------------------------------
void testLittleFS() {
  printlnDual("\n------------------------------------------------------------");
  printlnDual("[TEST 7/10] LITTLEFS ONBOARD FLASH MEMORY");
  printlnDual("------------------------------------------------------------");

  printDual("Mounting LittleFS flash file system... ");
  if (LittleFS.begin(true)) {
    sys_stat.littlefs_ok = true;
    size_t total = LittleFS.totalBytes();
    size_t used  = LittleFS.usedBytes();
    printlnDual("OK");
    printlnDual("  * Total Storage: " + String(total / 1024) + " KB (" + String((float)total / (1024*1024), 2) + " MB)");
    printlnDual("  * Used Storage : " + String(used / 1024) + " KB (" + String(used) + " bytes)");
    printlnDual("  * Free Storage : " + String((total - used) / 1024) + " KB");

    // Perform non-destructive write and read speed benchmark
    printDual("Testing test file write/read cycle... ");
    const char* test_path = "/sys_check_test.tmp";
    uint32_t t_write = millis();
    File f = LittleFS.open(test_path, FILE_WRITE);
    if (f) {
      for (int i = 0; i < 50; i++) {
        f.println("DIAGNOSTIC_PACKET_VERIFY_PAYLOAD_STRING_0123456789");
      }
      f.close();
      t_write = millis() - t_write;

      // Read back
      uint32_t t_read = millis();
      f = LittleFS.open(test_path, FILE_READ);
      int read_bytes = 0;
      if (f) {
        while (f.available()) { f.read(); read_bytes++; }
        f.close();
        t_read = millis() - t_read;
        LittleFS.remove(test_path);
        printlnDual("OK");
        printlnDual("  * Write speed test: 2.5 KB written in " + String(t_write) + " ms");
        printlnDual("  * Read speed test : " + String(read_bytes) + " bytes read in " + String(t_read) + " ms");
        printlnDual("--> LITTLEFS RESULT: PASS (Flash storage read/write validated)");
      } else {
        printlnDual("FAIL (Cannot reopen test file for reading)");
      }
    } else {
      printlnDual("FAIL (Cannot create test file in flash)");
      sys_stat.littlefs_ok = false;
    }
  } else {
    printlnDual("FAIL (LittleFS.begin failed! SPI flash error)");
    sys_stat.littlefs_ok = false;
  }
}

// --------------------------------------------------------------------------
// TEST 8: Pyrotechnic Continuity SENSE Lines (Sense 1..4)
// --------------------------------------------------------------------------
void testContinuity() {
  printlnDual("\n------------------------------------------------------------");
  printlnDual("[TEST 8/10] PYROTECHNIC CONTINUITY SENSE CHANNELS");
  printlnDual("------------------------------------------------------------");

  pinMode(SENSE_PIN_1, INPUT_PULLDOWN);
  pinMode(SENSE_PIN_2, INPUT_PULLDOWN);
  pinMode(SENSE_PIN_3, INPUT_PULLDOWN);
  pinMode(SENSE_PIN_4, INPUT_PULLDOWN);

  delay(10);

  sys_stat.drogue1_cont = (digitalRead(SENSE_PIN_1) == HIGH);
  sys_stat.drogue2_cont = (digitalRead(SENSE_PIN_2) == HIGH);
  sys_stat.main1_cont   = (digitalRead(SENSE_PIN_3) == HIGH);
  sys_stat.main2_cont   = (digitalRead(SENSE_PIN_4) == HIGH);

  printlnDual("  * Drogue Ch 1 (Sense Pin 4): " + String(sys_stat.drogue1_cont ? "[CLOSED - CONTINUITY OK]" : "[OPEN - No E-Match]"));
  printlnDual("  * Drogue Ch 2 (Sense Pin 5): " + String(sys_stat.drogue2_cont ? "[CLOSED - CONTINUITY OK]" : "[OPEN - No E-Match]"));
  printlnDual("  * Main Ch 1   (Sense Pin 1): " + String(sys_stat.main1_cont   ? "[CLOSED - CONTINUITY OK]" : "[OPEN - No E-Match]"));
  printlnDual("  * Main Ch 2   (Sense Pin 2): " + String(sys_stat.main2_cont   ? "[CLOSED - CONTINUITY OK]" : "[OPEN - No E-Match]"));

  int closed_count = (sys_stat.drogue1_cont?1:0) + (sys_stat.drogue2_cont?1:0) + (sys_stat.main1_cont?1:0) + (sys_stat.main2_cont?1:0);
  printlnDual("--> SENSE SUMMARY: " + String(closed_count) + "/4 Pyrotechnic circuits closed");
}

// --------------------------------------------------------------------------
// TEST 9: Pyrotechnic Firing Pin Safety Interlock
// --------------------------------------------------------------------------
void testPyroSafety() {
  printlnDual("\n------------------------------------------------------------");
  printlnDual("[TEST 9/10] PYRO FIRING GATE SAFETY INTERLOCK");
  printlnDual("------------------------------------------------------------");

  // Ensure all charge pins are strictly set as OUTPUT and latched LOW
  pinMode(CHARGE_PIN_DROGUE_1, OUTPUT);
  pinMode(CHARGE_PIN_DROGUE_2, OUTPUT);
  pinMode(CHARGE_PIN_MAIN_1, OUTPUT);
  pinMode(CHARGE_PIN_MAIN_2, OUTPUT);

  digitalWrite(CHARGE_PIN_DROGUE_1, LOW);
  digitalWrite(CHARGE_PIN_DROGUE_2, LOW);
  digitalWrite(CHARGE_PIN_MAIN_1, LOW);
  digitalWrite(CHARGE_PIN_MAIN_2, LOW);

  int v_d1 = digitalRead(CHARGE_PIN_DROGUE_1);
  int v_d2 = digitalRead(CHARGE_PIN_DROGUE_2);
  int v_m1 = digitalRead(CHARGE_PIN_MAIN_1);
  int v_m2 = digitalRead(CHARGE_PIN_MAIN_2);

  printlnDual("  * Charge Pin Drogue 1 (Pin 6) : Level = " + String(v_d1) + " (0 = Safe 0V)");
  printlnDual("  * Charge Pin Drogue 2 (Pin 7) : Level = " + String(v_d2) + " (0 = Safe 0V)");
  printlnDual("  * Charge Pin Main 1   (Pin 15): Level = " + String(v_m1) + " (0 = Safe 0V)");
  printlnDual("  * Charge Pin Main 2   (Pin 16): Level = " + String(v_m2) + " (0 = Safe 0V)");

  if (v_d1 == 0 && v_d2 == 0 && v_m1 == 0 && v_m2 == 0) {
    sys_stat.pyro_safe = true;
    printlnDual("--> PYRO SAFETY RESULT: PASS (All FET gates safely clamped to 0V / Ground)");
  } else {
    sys_stat.pyro_safe = false;
    printlnDual("--> PYRO SAFETY RESULT: CRITICAL HAZARD! (One or more charge pins reading HIGH!)");
  }
}

// --------------------------------------------------------------------------
// TEST 10: Audio/Visual Indicators Test
// --------------------------------------------------------------------------
void testIndicators() {
  printlnDual("\n------------------------------------------------------------");
  printlnDual("[TEST 10/10] AUDIO / VISUAL INDICATORS TEST");
  printlnDual("------------------------------------------------------------");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  printlnDual("Cycling RGB Status LED: RED -> GREEN -> BLUE -> WHITE...");
  setRGB(255, 0, 0);   delay(250); // Red
  setRGB(0, 255, 0);   delay(250); // Green
  setRGB(0, 0, 255);   delay(250); // Blue
  setRGB(255, 255, 255); delay(250); // White
  setRGB(0, 64, 0);    // Nominal Green

  printlnDual("Sounding buzzer chime sequence...");
  beep(80); delay(60); beep(80); delay(60); beep(140);
  printlnDual("--> INDICATORS RESULT: PASS (Chime and RGB cycle completed)");
}

// --------------------------------------------------------------------------
// SUMMARY REPORT
// --------------------------------------------------------------------------
void printSummary() {
  printlnDual("\n============================================================");
  printlnDual("           FLIGHT COMPUTER PRE-FLIGHT READINESS REPORT       ");
  printlnDual("============================================================");

  int score = 0;
  int max_score = 6;

  auto reportLine = [&](const char* name, bool ok) {
    char buf[64];
    snprintf(buf, sizeof(buf), "  %-28s : %s", name, ok ? "[  PASS  ]" : "[  FAIL  ]");
    printlnDual(buf);
    if (ok) score++;
  };

  reportLine("Primary I2C Bus (SDA 8/9)", sys_stat.i2c0_ok);
  reportLine("BNO08x 9-DoF IMU", sys_stat.bno08x_ok);
  reportLine("DPS310 Primary Altimeter", sys_stat.dps310_ok);
  reportLine("MS5611 Redundant Altimeter", sys_stat.ms5611_ok);
  reportLine("GNSS / GPS Receiver Link", sys_stat.gps_uart_ok);
  reportLine("LittleFS Flash Memory", sys_stat.littlefs_ok);

  printlnDual("------------------------------------------------------------");
  printlnDual("  * Safety Pyro Gates Clamp   : " + String(sys_stat.pyro_safe ? "PASSED (Safe)" : "HAZARD (Active)"));
  printlnDual("  * Drogue Pyro Continuity    : " + String((sys_stat.drogue1_cont||sys_stat.drogue2_cont) ? "OK (Armed/Ready)" : "OPEN (No e-match detected)"));
  printlnDual("  * Main Pyro Continuity      : " + String((sys_stat.main1_cont||sys_stat.main2_cont) ? "OK (Armed/Ready)" : "OPEN (No e-match detected)"));
  printlnDual("  * GPS 3D Fix Acquisition   : " + String(sys_stat.gps_fix_ok ? "3D LOCK (Ready)" : "SEARCHING (Needs outdoor sky view)"));
  printlnDual("------------------------------------------------------------");

  if (sys_stat.bno08x_ok && (sys_stat.dps310_ok || sys_stat.ms5611_ok) && sys_stat.littlefs_ok && sys_stat.pyro_safe) {
    setRGB(0, 255, 0); // Green
    printlnDual(">>> OVERALL STATUS: SYSTEM READY FOR FLIGHT (GO) <<<");
  } else {
    setRGB(255, 0, 0); // Red
    printlnDual(">>> OVERALL STATUS: NO-GO! Resolve failed sensors above! <<<");
  }
  printlnDual("============================================================\n");
  printlnDual("Type 'S' to start/stop live 10 Hz sensor stream.");
  printlnDual("Type 'R' to re-run full diagnostic check.\n");
}

void runFullDiagnostics() {
  printlnDual("\n============================================================");
  printlnDual("   ROCKET FLIGHT COMPUTER - COMPLETE SYSTEMS CHECK INITIATED ");
  printlnDual("============================================================\n");
  
  testI2CBuses();
  testBNO08x();
  testDPS310();
  testMS5611();
  testBaroConsistency();
  testGPS();
  testLittleFS();
  testContinuity();
  testPyroSafety();
  testIndicators();
  printSummary();
}

// --------------------------------------------------------------------------
// Real-Time Live Telemetry Stream (10 Hz Diagnostic Stream)
// --------------------------------------------------------------------------
void streamLiveData() {
  uint32_t now = millis();
  if (now - last_stream_tick < 100) return; // 10 Hz
  last_stream_tick = now;

  // IMU
  float ax = 0, ay = 0, az = 0;
  float qi = 0, qj = 0, qk = 0, qr = 1.0;
  sh2_SensorValue_t val;
  while (bno08x.getSensorEvent(&val)) {
    if (val.sensorId == SH2_LINEAR_ACCELERATION) {
      ax = val.un.linearAcceleration.x;
      ay = val.un.linearAcceleration.y;
      az = val.un.linearAcceleration.z;
    } else if (val.sensorId == SH2_ROTATION_VECTOR) {
      qi = val.un.rotationVector.i;
      qj = val.un.rotationVector.j;
      qk = val.un.rotationVector.k;
      qr = val.un.rotationVector.real;
    }
  }

  // Barometers
  float p_dps = 0, t_dps = 0;
  if (sys_stat.dps310_ok && dps.pressureAvailable()) {
    sensors_event_t te, pe;
    dps.getEvents(&te, &pe);
    p_dps = pe.pressure;
    t_dps = te.temperature;
  }

  float p_ms = 0, t_ms = 0;
  if (sys_stat.ms5611_ok) {
    if (ms5611.read() == MS5611_READ_OK) {
      p_ms = ms5611.getPressure();
      t_ms = ms5611.getTemperature();
    }
  }

  // GPS
  while (Serial2.available() > 0) gps.encode(Serial2.read());
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  double lng = gps.location.isValid() ? gps.location.lng() : 0.0;

  // Format clean diagnostic line:
  // [TIME_MS] | ACCEL(X,Y,Z) | P_DPS, P_MS | GPS(SATS,LAT,LNG) | CONT(D1,D2,M1,M2)
  char line[220];
  snprintf(line, sizeof(line),
    "[%7lu ms] Acc:[%5.1f,%5.1f,%5.1f] | DPS:%7.2f hPa (%4.1fC) | MS:%7.2f hPa | Sats:%2d Pos:[%9.5f,%9.5f] | Pyro:[%d,%d,%d,%d]",
    now, ax, ay, az, p_dps, t_dps, p_ms, sats, lat, lng,
    digitalRead(SENSE_PIN_1), digitalRead(SENSE_PIN_2),
    digitalRead(SENSE_PIN_3), digitalRead(SENSE_PIN_4)
  );

  printlnDual(line);
}

// --------------------------------------------------------------------------
// Interactive Command Handler
// --------------------------------------------------------------------------
void processCommands(char c) {
  c = toupper(c);
  if (c == 'R') {
    continuous_stream = false;
    runFullDiagnostics();
  } else if (c == 'S') {
    continuous_stream = !continuous_stream;
    if (continuous_stream) {
      printlnDual(">>> CONTINUOUS 10 Hz SENSOR STREAM ENABLED (Type 'S' to pause) <<<");
    } else {
      printlnDual(">>> STREAM PAUSED <<<");
    }
  } else if (c == 'I') {
    testI2CBuses();
  } else if (c == 'P') {
    testContinuity();
    testPyroSafety();
  } else if (c == 'G') {
    testGPS();
  } else if (c == 'B') {
    testIndicators();
  }
}

// --------------------------------------------------------------------------
// Main Setup
// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(TELEMETRY_BAUD, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_EXT_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  setRGB(64, 64, 0); // Amber during boot

  // 1. Unwedge I2C bus lines in case sensor was interrupted
  recoverI2CBus(I2C0_SDA_PIN, I2C0_SCL_PIN);

  // 2. Initialize I2C buses at 40 kHz (matches your working I2C scanner)
  Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, 40000);
  Wire.setTimeOut(100); // 100ms timeout for clock stretching
  Wire1.begin(MS5611_SDA, MS5611_SCL, 40000);
  Wire1.setTimeOut(100);

  delay(200); // Allow sensors and Cortex-M0 to boot
  delay(1800); // Allow USB serial terminal to connect

  // Run complete 10-point diagnostic on startup
  runFullDiagnostics();
}

// --------------------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------------------
void loop() {
  // Feed GPS parser continuously
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  // Check USB Serial commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') processCommands(c);
  }

  // Check HC-12 Wireless Radio commands
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c != '\n' && c != '\r') processCommands(c);
  }

  // If live stream active, output high-rate readings
  if (continuous_stream) {
    streamLiveData();
  }
}
