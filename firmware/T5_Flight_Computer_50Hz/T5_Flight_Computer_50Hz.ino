#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_DPS310.h>
#include <MS5611.h>
#include <TinyGPS++.h>
#include <FS.h>
#include <LittleFS.h>

/**
 * ============================================================================
 *   T5 FLIGHT COMPUTER FIRMWARE - 50 Hz HIGH-RATE BINARY LOGGING
 * ============================================================================
 *   Target MCU: ESP32-S3
 *   Features:
 *   - 50 Hz State Machine & Raw Sensor Acquisition (20 ms loop)
 *   - Typed Binary High-Density Logging (IMU/Baro 0x01, GPS 0x02, Events 0x03)
 *   - 512-Byte RAM Chunk Flash Buffering (eliminates SPI write latency spikes)
 *   - 5-Second Circular Pre-Flight RAM Buffer on PAD
 *   - Live 4 Hz Binary Radio Telemetry (HC-12 / Serial1)
 * ============================================================================
 */

// --- Logging & Timing Configuration ---
#define TARGET_LOOP_HZ 50
const uint32_t STATE_TICK_INTERVAL_MS = 1000 / TARGET_LOOP_HZ; // 20 ms
const long IMU_REPORT_INTERVAL_US = 20000;                     // 50 Hz

#define BNO08X_RESET -1
#define MS5611_SDA 40
#define MS5611_SCL 39

// --- Telemetry & GPS Configuration ---
#define HC12_RX_PIN 17
#define HC12_TX_PIN 18
#define TELEMETRY_BAUD 115200

// Radio broadcast transmission rate (Hz)
// Adjust based on your RF transceiver & over-the-air data rate:
//   - LoRa (SX1276 / RFM95 / EBYTE): 1.0 to 2.0 Hz (narrowband, long-range)
//   - HC-12 (default FU3 mode):      2.0 to 5.0 Hz
//   - High-Speed RF (RFD900 / XBee): 10.0 to 20.0 Hz
float telemetry_broadcast_rate_hz = 4.0; // Default 4 Hz
uint32_t telemetry_interval_ms = (uint32_t)(1000.0 / telemetry_broadcast_rate_hz);

#define GPS_RX_PIN 42
#define GPS_TX_PIN 41
#define GPS_BAUD 38400

// --- Hardware Pins (Deployment & Indicators) ---
const int SENSE_PIN_1 = 4;
const int SENSE_PIN_2 = 5;
const int SENSE_PIN_3 = 1;
const int SENSE_PIN_4 = 2;

const int CHARGE_PIN_DROGUE_1 = 6;
const int CHARGE_PIN_DROGUE_2 = 7;
const int CHARGE_PIN_MAIN_1 = 15;
const int CHARGE_PIN_MAIN_2 = 16;

const int BUZZER_PIN = 36;
const int LED_EXT_PIN = 38;

// --- Flight Constraints ---
const uint32_t MACH_DELAY_MS = 1000;     
const uint32_t EJECTION_HOLD_TIME = 5000; 
const uint32_t REDUNDANT_CHARGE_DELAY_MS = 1000; 
const float MAIN_CHUTE_ALTITUDE_M = 500.0; 
const float LAUNCH_FAILSAFE_ALT_M = 100.0; 
const float LAUNCH_CONFIRM_ALT_M = 5.0;    
const uint32_t VALIDATION_TIMEOUT_MS = 500; 
const uint32_t DROGUE_FAILSAFE_TIMEOUT_MS = 30000; 

// --- Binary Log Packet Definitions ---
#define LOG_TYPE_IMU_BARO 0x01
#define LOG_TYPE_GPS      0x02
#define LOG_TYPE_EVENT    0x03

#pragma pack(push, 1)
// Record Type 0x01: Logged at 50 Hz (42 bytes)
struct ImuBaroRecord {
  uint8_t  type;         // 0x01
  uint32_t timestamp_ms; // System uptime in ms
  uint8_t  state;        // Flight state enum (0-9)
  int16_t  accel_x;      // Calibrated m/s^2 * 100 (range +-327 m/s^2)
  int16_t  accel_y;
  int16_t  accel_z;
  int16_t  gyro_x;       // Calibrated rad/s * 1000
  int16_t  gyro_y;
  int16_t  gyro_z;
  int16_t  mag_x;        // Calibrated uT * 10
  int16_t  mag_y;
  int16_t  mag_z;
  int32_t  dps_press;    // Raw pressure in Pa
  int16_t  dps_temp;     // Raw temperature in deg C * 100
  int32_t  ms_press;     // Raw pressure in Pa
  int16_t  ms_temp;      // Raw temperature in deg C * 100
  float    fused_alt;    // Kalman AGL altitude in meters
  uint8_t  continuity;   // (drogue_count * 10) + main_count
  uint8_t  pyro_status;  // Bit 0: D1, Bit 1: D2, Bit 2: M1, Bit 3: M2
};

// Record Type 0x02: Logged ONLY when GPS yields new fix (~5-10 Hz) (23 bytes)
struct GpsRecord {
  uint8_t  type;         // 0x02
  uint32_t timestamp_ms; // System uptime in ms
  int32_t  lat;          // Decimal deg * 10,000,000
  int32_t  lng;          // Decimal deg * 10,000,000
  int32_t  alt_cm;       // Altitude MSL in cm
  uint16_t speed_cms;    // Ground speed in cm/s
  uint16_t heading_cd;   // Course in centidegrees (deg * 100)
  uint8_t  sats;         // Satellites count
  uint8_t  fix_valid;    // 1 if fix valid, 0 otherwise
};

// Record Type 0x03: Logged on state transitions & events (8 bytes)
struct EventRecord {
  uint8_t  type;         // 0x03
  uint32_t timestamp_ms; // System uptime in ms
  uint8_t  event_id;     // 1: TRANSITION, 2: DROGUE_FIRE, 3: MAIN_FIRE, 4: APOGEE
  uint8_t  param1;       // New state / detail
  uint8_t  param2;       // Old state / extra
};

// 26-byte live telemetry packet broadcasted over radio (HC-12)
struct TelemetryPacket {
  uint16_t sync_word;  // 0xAA55
  uint32_t timestamp;  // Milliseconds
  uint8_t  state;      // FlightState enum (0-9)
  float    accel;      // Linear acceleration magnitude (m/s^2)
  float    altitude;   // Kalman-fused altitude AGL (meters)
  uint8_t  continuity; // (drogue_count * 10) + main_count
  int32_t  lat;        // Latitude * 10,000,000
  int32_t  lng;        // Longitude * 10,000,000
  uint8_t  sats;       // Satellite count
  uint8_t  checksum;   // XOR checksum over payload
};
#pragma pack(pop)

TelemetryPacket current_packet;
const char* log_file_path = "/flight_log.bin";

// --- RAM Buffering Management ---
// 5 seconds pre-flight ring buffer at 50 Hz = 250 records (~10.5 KB)
#define PRE_FLIGHT_RECORDS 250
ImuBaroRecord preFlightRing[PRE_FLIGHT_RECORDS];
uint16_t ringHead = 0;
bool ringWrapped = false;

// 512-byte page write chunk buffer to prevent LittleFS write latency spikes
#define WRITE_CHUNK_SIZE 512
uint8_t chunkBuffer[WRITE_CHUNK_SIZE];
size_t chunkIndex = 0;

// --- Sensor Objects ---
Adafruit_BNO08x bno08x(BNO08X_RESET);
Adafruit_DPS310 dps;
TinyGPSPlus gps;
MS5611 ms5611(0x77, &Wire1); 

// --- Variables ---
sh2_SensorValue_t sensorValue;

// Raw & Calibrated Sensor Readings
float raw_accel_x = 0.0, raw_accel_y = 0.0, raw_accel_z = 0.0;
float raw_gyro_x = 0.0, raw_gyro_y = 0.0, raw_gyro_z = 0.0;
float raw_mag_x = 0.0, raw_mag_y = 0.0, raw_mag_z = 0.0;
float lin_accel_x = 0.0, lin_accel_y = 0.0, lin_accel_z = 0.0;
float debug_true_accel_ms2 = 0.0;
float quat_i = 0.0, quat_j = 0.0, quat_k = 0.0, quat_real = 1.0;

float dps310_ref_pressure = 0.0;
float ms5611_ref_pressure = 0.0;
float current_dps_pressure = 0.0;
float current_dps_temp = 0.0;
float current_ms_pressure = 0.0;
float current_ms_temp = 0.0;
float smoothed_altitude = 0.0;
float max_altitude = 0.0;

bool bno08x_active = false;
bool dps310_active = false;
bool ms5611_active = false;
bool fs_operational = false;
bool test_baro_mode = false; 

uint32_t burnout_timestamp = 0; 
uint32_t state_timer = 0; 
uint32_t last_tx_tick = 0;
uint32_t last_state_tick = 0;

// --- State Machine ---
enum FlightState { 
  STATE_SYSTEM_ERROR = 0,
  STATE_PAD, 
  STATE_LAUNCH_VALIDATION, 
  STATE_BOOST, 
  STATE_COAST, 
  STATE_APOGEE_SEARCH, 
  STATE_FIRE_DROGUE,      
  STATE_DROGUE_DESCENT,   
  STATE_FIRE_MAIN,        
  STATE_MAIN_DESCENT 
};
FlightState rocket_state = STATE_SYSTEM_ERROR;

const char* state_names[] = { 
  "STATE_SYSTEM_ERROR", "STATE_PAD", "STATE_LAUNCH_VALIDATION", 
  "STATE_BOOST", "STATE_COAST", "STATE_APOGEE_SEARCH", 
  "STATE_FIRE_DROGUE", "STATE_DROGUE_DESCENT", 
  "STATE_FIRE_MAIN", "STATE_MAIN_DESCENT" 
};

uint8_t calculateChecksum(uint8_t* data, size_t length) {
  uint8_t chk = 0;
  for (size_t i = 0; i < length; i++) chk ^= data[i];
  return chk;
}

// --- Kalman Filter Class ---
class DualBaroKalman {
private:
  float est_alt;
  float est_p;
  float q; 

public:
  DualBaroKalman(float initial_alt, float process_noise) {
    est_alt = initial_alt;
    est_p = 1.0;
    q = process_noise;
  }
  void setAltitude(float alt) { est_alt = alt; }
  void predict() { est_p += q; }
  void update(float measurement, float r_noise) {
    float k = est_p / (est_p + r_noise);
    est_alt = est_alt + k * (measurement - est_alt);
    est_p = (1.0 - k) * est_p;
  }
  float getAltitude() { return est_alt; }
};

DualBaroKalman altFilter(0.0, 0.1);

// --- Helpers ---
void setLED(uint8_t r, uint8_t g, uint8_t b) {
  #ifdef RGB_BUILTIN
  neopixelWrite(RGB_BUILTIN, r, g, b);
  #endif
  neopixelWrite(LED_EXT_PIN, r, g, b);
}

void broadcastMsg(const char* msg) {
  Serial.println(msg);
  Serial1.println(msg);
}

uint8_t getPyroStatusMask() {
  uint8_t mask = 0;
  if (digitalRead(CHARGE_PIN_DROGUE_1) == HIGH) mask |= (1 << 0);
  if (digitalRead(CHARGE_PIN_DROGUE_2) == HIGH) mask |= (1 << 1);
  if (digitalRead(CHARGE_PIN_MAIN_1) == HIGH)   mask |= (1 << 2);
  if (digitalRead(CHARGE_PIN_MAIN_2) == HIGH)   mask |= (1 << 3);
  return mask;
}

// --- Binary Flash Logging Engine ---
void flushChunkBuffer() {
  if (!fs_operational || chunkIndex == 0) return;
  File file = LittleFS.open(log_file_path, FILE_APPEND);
  if (file) {
    file.write(chunkBuffer, chunkIndex);
    file.close();
  }
  chunkIndex = 0;
}

void writeBinaryData(const void* data, size_t length) {
  if (!fs_operational) return;
  const uint8_t* ptr = (const uint8_t*)data;
  while (length > 0) {
    size_t space = WRITE_CHUNK_SIZE - chunkIndex;
    size_t toCopy = (length < space) ? length : space;
    memcpy(chunkBuffer + chunkIndex, ptr, toCopy);
    chunkIndex += toCopy;
    ptr += toCopy;
    length -= toCopy;

    if (chunkIndex >= WRITE_CHUNK_SIZE) {
      flushChunkBuffer();
    }
  }
}

void pushPreFlightRing(const ImuBaroRecord* rec) {
  preFlightRing[ringHead] = *rec;
  ringHead++;
  if (ringHead >= PRE_FLIGHT_RECORDS) {
    ringHead = 0;
    ringWrapped = true;
  }
}

void flushPreFlightBuffer() {
  if (!fs_operational) return;
  uint16_t count = ringWrapped ? PRE_FLIGHT_RECORDS : ringHead;
  for (uint16_t i = 0; i < count; i++) {
    uint16_t idx = ringWrapped ? ((ringHead + i) % PRE_FLIGHT_RECORDS) : i;
    writeBinaryData(&preFlightRing[idx], sizeof(ImuBaroRecord));
  }
  ringWrapped = false;
  ringHead = 0;
}

void logBinaryEvent(uint8_t event_id, uint8_t param1 = 0, uint8_t param2 = 0) {
  EventRecord ev;
  ev.type = LOG_TYPE_EVENT;
  ev.timestamp_ms = millis();
  ev.event_id = event_id;
  ev.param1 = param1;
  ev.param2 = param2;

  if (rocket_state == STATE_PAD) {
    // Also record event directly
    writeBinaryData(&ev, sizeof(EventRecord));
  } else {
    writeBinaryData(&ev, sizeof(EventRecord));
  }
}

void clearLog() {
  flushChunkBuffer();
  if (LittleFS.remove(log_file_path)) {
    broadcastMsg("LOG_CLEARED");
  } else {
    broadcastMsg("LOG_CLEAR_FAILED");
  }
}

void dumpBinaryLog() {
  flushChunkBuffer();
  if (!fs_operational) { broadcastMsg("FS_ERROR"); return; }
  File file = LittleFS.open(log_file_path, FILE_READ);
  if (!file) { broadcastMsg("FILE_ERROR"); return; }

  size_t totalBytes = file.size();
  char headerMsg[64];
  snprintf(headerMsg, sizeof(headerMsg), "=== BINARY LOG START: %u ===", (unsigned int)totalBytes);
  broadcastMsg(headerMsg);
  delay(30);

  uint8_t buffer[256];
  while (file.available()) {
    size_t n = file.read(buffer, sizeof(buffer));
    if (n > 0) {
      Serial.write(buffer, n);
      Serial1.write(buffer, n);
    }
    delay(1);
  }
  file.close();
  delay(30);
  broadcastMsg("\n=== BINARY LOG END ===");
}

void zeroAltitude() {
  if (rocket_state != STATE_PAD) {
    broadcastMsg("ERROR: Can only zero altitude in STATE_PAD!");
    return;
  }
  if (current_dps_pressure > 0) dps310_ref_pressure = current_dps_pressure;
  if (current_ms_pressure > 0) ms5611_ref_pressure = current_ms_pressure;
  altFilter.setAltitude(0.0);
  smoothed_altitude = 0.0;
  max_altitude = 0.0;
  broadcastMsg("ALTITUDE ZEROED TO CURRENT PAD PRESSURE.");
}

void testEjectionCharge(String chargeType) {
  if (rocket_state != STATE_PAD) {
    broadcastMsg("ERROR: Cannot test charges while active flight state!");
    return;
  }

  broadcastMsg("WARNING: CLEAR THE AREA. FIRING IN 3 SECONDS.");
  for (int i = 3; i > 0; i--) {
    char countdownMsg[32];
    snprintf(countdownMsg, sizeof(countdownMsg), "T-MINUS %d...", i);
    broadcastMsg(countdownMsg);
    digitalWrite(BUZZER_PIN, HIGH);
    setLED(255, 0, 0); 
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    setLED(0, 0, 0);   
    delay(500);
  }

  if (chargeType == "DROGUE") {
    broadcastMsg("EVENT: FIRING TEST DROGUE CHARGES!");
    digitalWrite(CHARGE_PIN_DROGUE_1, HIGH);
    digitalWrite(CHARGE_PIN_DROGUE_2, HIGH);
    logBinaryEvent(2, 1, 1);
    delay(1000); 
    digitalWrite(CHARGE_PIN_DROGUE_1, LOW);
    digitalWrite(CHARGE_PIN_DROGUE_2, LOW);
    broadcastMsg("EVENT: DROGUE TEST COMPLETE.");
  } 
  else if (chargeType == "MAIN") {
    broadcastMsg("EVENT: FIRING TEST MAIN CHARGES!");
    digitalWrite(CHARGE_PIN_MAIN_1, HIGH);
    digitalWrite(CHARGE_PIN_MAIN_2, HIGH);
    logBinaryEvent(3, 1, 1);
    delay(1000); 
    digitalWrite(CHARGE_PIN_MAIN_1, LOW);
    digitalWrite(CHARGE_PIN_MAIN_2, LOW);
    broadcastMsg("EVENT: MAIN TEST COMPLETE.");
  }
  setLED(0, 64, 0); 
}

void toggleTestMode() {
  test_baro_mode = !test_baro_mode;
  if (test_baro_mode) {
    broadcastMsg("TEST_BARO: ENABLED. Altimeter free, sim launch ready.");
  } else {
    broadcastMsg("TEST_BARO: DISABLED. Normal pad ops resumed.");
    rocket_state = STATE_PAD;
    zeroAltitude();
    setLED(0, 64, 0); 
  }
}

void handleCommand(String& cmd) {
  cmd.toUpperCase();
  if (cmd.indexOf("REBOOT") >= 0 || cmd.indexOf("RESET") >= 0) ESP.restart();
  else if (cmd.indexOf("READ_BIN") >= 0 || cmd.indexOf("DUMP_BIN") >= 0 || cmd.indexOf("READ") >= 0) dumpBinaryLog();
  else if (cmd.indexOf("CLEAR") >= 0) clearLog();
  else if (cmd.indexOf("ZERO_ALT") >= 0) zeroAltitude();
  else if (cmd.indexOf("TEST_DROGUE") >= 0) testEjectionCharge("DROGUE");
  else if (cmd.indexOf("TEST_MAIN") >= 0) testEjectionCharge("MAIN");
  else if (cmd.indexOf("TEST_BARO") >= 0) toggleTestMode();
  else if (cmd.startsWith("SET_TX_RATE") || cmd.startsWith("TX_RATE")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx > 0) {
      float new_rate = cmd.substring(spaceIdx + 1).toFloat();
      if (new_rate >= 0.5f && new_rate <= 50.0f) {
        telemetry_broadcast_rate_hz = new_rate;
        telemetry_interval_ms = (uint32_t)(1000.0f / telemetry_broadcast_rate_hz);
        char msg[64];
        snprintf(msg, sizeof(msg), "TELEMETRY RATE UPDATED: %.1f Hz (%u ms)", telemetry_broadcast_rate_hz, (unsigned int)telemetry_interval_ms);
        broadcastMsg(msg);
      } else {
        broadcastMsg("ERR: Rate must be between 0.5 and 50.0 Hz");
      }
    }
  }
}

void checkSerialCommands() {
  static String usb_cmd = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (usb_cmd.length() > 0) {
        handleCommand(usb_cmd);
        usb_cmd = "";
      }
    } else { usb_cmd += c; }
  }

  static String tel_cmd = "";
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') {
      if (tel_cmd.length() > 0) {
        handleCommand(tel_cmd);
        tel_cmd = "";
      }
    } else { tel_cmd += c; }
  }
}

void setReports(long report_interval) {
  bno08x.enableReport(SH2_ACCELEROMETER, report_interval);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, report_interval);
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, report_interval);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, report_interval);
  bno08x.enableReport(SH2_ROTATION_VECTOR, report_interval);
}

float calculateAltitudeAGL(float current_pressure, float reference_pressure) {
  if (reference_pressure == 0) return 0.0;
  return 44330.0 * (1.0 - pow(current_pressure / reference_pressure, 0.1903));
}

void changeState(FlightState new_state) {
  // If moving from PAD to active flight, flush 5-sec preflight RAM buffer
  if ((rocket_state == STATE_PAD && new_state == STATE_LAUNCH_VALIDATION) || 
      (rocket_state == STATE_PAD && new_state == STATE_BOOST)) {
    flushPreFlightBuffer();
  }

  logBinaryEvent(1, (uint8_t)new_state, (uint8_t)rocket_state);

  rocket_state = new_state;
  state_timer = millis();
  char msg[64];
  snprintf(msg, sizeof(msg), "TRANSITION: %s", state_names[new_state]);
  broadcastMsg(msg);
  flushChunkBuffer();
}

// --- Setup ---
void setup(void) {
  Serial.begin(115200);
  Serial1.begin(TELEMETRY_BAUD, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(1500);

  broadcastMsg("\n================================================");
  broadcastMsg("    T5 FLIGHT COMPUTER - 50Hz BINARY LOGGING   ");
  broadcastMsg("================================================");

  setLED(64, 64, 0); 

  if (LittleFS.begin(true)) { 
    fs_operational = true;
    broadcastMsg("[OK] LittleFS initialized. Flash binary logger ready.");
  } else {
    broadcastMsg("[WARN] LittleFS mount failed.");
  }
  
  broadcastMsg("Initializing Fast I2C buses (400kHz)...");
  Wire.begin(8, 9, 400000); 
  Wire.setTimeOut(100);
  Wire1.begin(MS5611_SDA, MS5611_SCL, 400000); 
  Wire1.setTimeOut(100);
  delay(500);
  
  broadcastMsg("Probing BNO085 9-DoF IMU...");
  if (bno08x.begin_I2C(0x4B, &Wire) || bno08x.begin_I2C(0x4A, &Wire)) {
    bno08x_active = true;
    delay(50);
    setReports(IMU_REPORT_INTERVAL_US);
    broadcastMsg("[OK] BNO085 online (50Hz reports enabled)!");
  } else {
    broadcastMsg("[WARN] BNO085 not found on Wire (0x4B / 0x4A).");
  }

  broadcastMsg("Probing DPS310 primary barometer...");
  if (dps.begin_I2C(0x77, &Wire) || dps.begin_I2C(0x76, &Wire)) { 
    dps310_active = true;
    dps.setMode(DPS310_IDLE);
    delay(10);
    // 50Hz mode: 64Hz sampling with 8x oversampling
    dps.configurePressure(DPS310_64HZ, DPS310_8SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_8SAMPLES);
    dps.setMode(DPS310_CONT_PRESTEMP);
    broadcastMsg("[OK] DPS310 online (64Hz / 8x oversampled)!");
  } else {
    broadcastMsg("[WARN] DPS310 not found on Wire (0x77).");
  }

  broadcastMsg("Probing MS5611 redundant barometer...");
  if (ms5611.begin()) {
    ms5611_active = true;
    ms5611.reset(1);
    broadcastMsg("[OK] MS5611 online on Wire1!");
  } else {
    broadcastMsg("[WARN] MS5611 not found on Wire1.");
  }

  delay(1000); 

  if (dps310_active || ms5611_active) {
    broadcastMsg("Sampling pad pressure baseline (10 samples)...");
    float dps_sum = 0, ms_sum = 0;
    int valid_dps_samples = 0, valid_ms_samples = 0;
    
    for (int i = 0; i < 10; i++) {
      if (dps310_active) {
        uint32_t timeout = millis();
        while (!dps.pressureAvailable() && (millis() - timeout < 50)) { delay(2); }
        if (dps.pressureAvailable()) {
          sensors_event_t temp_event, pressure_event;
          dps.getEvents(&temp_event, &pressure_event);
          dps_sum += pressure_event.pressure;
          valid_dps_samples++;
        }
      }
      if (ms5611_active) {
        ms5611.read(); 
        float p = ms5611.getPressure();
        if (p > 300.0 && p < 1200.0) {
          ms_sum += p;
          valid_ms_samples++;
        }
      }
      delay(50); 
    }
    dps310_ref_pressure = (valid_dps_samples > 0) ? (dps_sum / valid_dps_samples) : 1013.25;
    ms5611_ref_pressure = (valid_ms_samples > 0) ? (ms_sum / valid_ms_samples) : 1013.25;
    char refMsg[80];
    snprintf(refMsg, sizeof(refMsg), "Pad Baseline: DPS=%.2f hPa, MS=%.2f hPa",
             dps310_ref_pressure, ms5611_ref_pressure);
    broadcastMsg(refMsg);
  }

  if (bno08x_active && (dps310_active || ms5611_active)) {
    rocket_state = STATE_PAD;
    setLED(0, 64, 0); 
    broadcastMsg("ALL SENSORS NOMINAL. 50Hz SYSTEM READY.");
  } else {
    rocket_state = STATE_SYSTEM_ERROR;
    broadcastMsg("CRITICAL: SENSOR INITIALIZATION FAILED.");
  }
  
  pinMode(SENSE_PIN_1, INPUT_PULLDOWN);
  pinMode(SENSE_PIN_2, INPUT_PULLDOWN);
  pinMode(SENSE_PIN_3, INPUT_PULLDOWN);
  pinMode(SENSE_PIN_4, INPUT_PULLDOWN);

  pinMode(CHARGE_PIN_DROGUE_1, OUTPUT);
  pinMode(CHARGE_PIN_DROGUE_2, OUTPUT);
  pinMode(CHARGE_PIN_MAIN_1, OUTPUT);
  pinMode(CHARGE_PIN_MAIN_2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  digitalWrite(CHARGE_PIN_DROGUE_1, LOW); 
  digitalWrite(CHARGE_PIN_DROGUE_2, LOW); 
  digitalWrite(CHARGE_PIN_MAIN_1, LOW); 
  digitalWrite(CHARGE_PIN_MAIN_2, LOW); 
  digitalWrite(BUZZER_PIN, LOW);
}

// --- Main Loop ---
void loop() {
  while (Serial2.available() > 0) gps.encode(Serial2.read());
  checkSerialCommands();
  uint32_t now = millis();

  if (rocket_state == STATE_SYSTEM_ERROR) {
    static uint32_t error_tick = 0;
    static bool toggle = false;
    if (now - error_tick >= 1000) {
      error_tick = now;
      toggle = !toggle;
      if (toggle) { setLED(255, 0, 0); digitalWrite(BUZZER_PIN, HIGH); } 
      else { setLED(0, 0, 0); digitalWrite(BUZZER_PIN, LOW); }
      String errorMsg = "ERROR - Missing: ";
      if (!bno08x_active) errorMsg += "[BNO08x] ";
      if (!dps310_active) errorMsg += "[DPS310] ";
      if (!ms5611_active) errorMsg += "[MS5611]";
      broadcastMsg(errorMsg.c_str());
    }
    return;
  }

  // --- Fetch BNO085 Sensor Events ---
  if (bno08x_active) {
    static uint32_t last_reset_reconfig = 0;
    if (bno08x.wasReset() && (now - last_reset_reconfig > 1000)) {
      last_reset_reconfig = now;
      delay(10);
      setReports(IMU_REPORT_INTERVAL_US);
    }
    uint8_t event_count = 0;
    while (bno08x.getSensorEvent(&sensorValue) && (event_count++ < 10)) {
      if (sensorValue.sensorId == SH2_ACCELEROMETER) {
        raw_accel_x = sensorValue.un.accelerometer.x;
        raw_accel_y = sensorValue.un.accelerometer.y;
        raw_accel_z = sensorValue.un.accelerometer.z;
      } else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        raw_gyro_x = sensorValue.un.gyroscope.x;
        raw_gyro_y = sensorValue.un.gyroscope.y;
        raw_gyro_z = sensorValue.un.gyroscope.z;
      } else if (sensorValue.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
        raw_mag_x = sensorValue.un.magneticField.x;
        raw_mag_y = sensorValue.un.magneticField.y;
        raw_mag_z = sensorValue.un.magneticField.z;
      } else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        lin_accel_x = sensorValue.un.linearAcceleration.x;
        lin_accel_y = sensorValue.un.linearAcceleration.y;
        lin_accel_z = sensorValue.un.linearAcceleration.z;
        debug_true_accel_ms2 = sqrt(sq(lin_accel_x) + sq(lin_accel_y) + sq(lin_accel_z));
      } else if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        quat_i = sensorValue.un.rotationVector.i;
        quat_j = sensorValue.un.rotationVector.j;
        quat_k = sensorValue.un.rotationVector.k;
        quat_real = sensorValue.un.rotationVector.real;
      }
    }
  }

  // --- 50 Hz State Machine & Logging Tick ---
  if (now - last_state_tick >= STATE_TICK_INTERVAL_MS) {
    last_state_tick = now;
    static float dps_alt = 0.0, ms_alt = 0.0;

    // Read DPS310 Barometer
    if (dps310_active) {
      if (dps.temperatureAvailable() && dps.pressureAvailable()) {
        sensors_event_t temp_event, pressure_event;
        dps.getEvents(&temp_event, &pressure_event);
        if (pressure_event.pressure >= 300.0 && pressure_event.pressure <= 1200.0) {
          current_dps_pressure = pressure_event.pressure;
          current_dps_temp = temp_event.temperature;
          dps_alt = calculateAltitudeAGL(current_dps_pressure, dps310_ref_pressure);
        }
      }
    }

    // Read MS5611 Barometer
    if (ms5611_active) {
      if (ms5611.read() == MS5611_READ_OK) {
        float new_ms_pressure = ms5611.getPressure();
        if (new_ms_pressure > 0) {
          current_ms_pressure = new_ms_pressure;
          current_ms_temp = ms5611.getTemperature();
          ms_alt = calculateAltitudeAGL(current_ms_pressure, ms5611_ref_pressure);
        }
      }
    }

    // Update Kalman Filter
    altFilter.predict();
    if (ms5611_active && current_ms_pressure > 0) altFilter.update(ms_alt, 5.0); 
    if (dps310_active && current_dps_pressure > 0) altFilter.update(dps_alt, 1.0); 
    smoothed_altitude = altFilter.getAltitude();

    // Continuity sense values
    int drogue_count = (digitalRead(SENSE_PIN_1) == HIGH ? 1 : 0) + (digitalRead(SENSE_PIN_2) == HIGH ? 1 : 0);
    int main_count   = (digitalRead(SENSE_PIN_3) == HIGH ? 1 : 0) + (digitalRead(SENSE_PIN_4) == HIGH ? 1 : 0);

    // Build 50 Hz Binary Record
    ImuBaroRecord rec;
    rec.type = LOG_TYPE_IMU_BARO;
    rec.timestamp_ms = now;
    rec.state = (uint8_t)rocket_state;
    rec.accel_x = (int16_t)(raw_accel_x * 100.0f);
    rec.accel_y = (int16_t)(raw_accel_y * 100.0f);
    rec.accel_z = (int16_t)(raw_accel_z * 100.0f);
    rec.gyro_x  = (int16_t)(raw_gyro_x * 1000.0f);
    rec.gyro_y  = (int16_t)(raw_gyro_y * 1000.0f);
    rec.gyro_z  = (int16_t)(raw_gyro_z * 1000.0f);
    rec.mag_x   = (int16_t)(raw_mag_x * 10.0f);
    rec.mag_y   = (int16_t)(raw_mag_y * 10.0f);
    rec.mag_z   = (int16_t)(raw_mag_z * 10.0f);
    rec.dps_press = (int32_t)(current_dps_pressure * 100.0f);
    rec.dps_temp  = (int16_t)(current_dps_temp * 100.0f);
    rec.ms_press  = (int32_t)(current_ms_pressure * 100.0f);
    rec.ms_temp   = (int16_t)(current_ms_temp * 100.0f);
    rec.fused_alt = smoothed_altitude;
    rec.continuity = (uint8_t)((drogue_count * 10) + main_count);
    rec.pyro_status = getPyroStatusMask();

    // Write to RAM buffer or Flash
    if (rocket_state == STATE_PAD) {
      pushPreFlightRing(&rec);
    } else if (rocket_state == STATE_MAIN_DESCENT) {
      static uint32_t last_landed_log = 0;
      if (now - last_landed_log >= 1000) { 
        last_landed_log = now;
        writeBinaryData(&rec, sizeof(rec));
        flushChunkBuffer();
      }
    } else {
      writeBinaryData(&rec, sizeof(rec));
    }

    // Check GPS Update - ONLY log if new GPS fix data arrived
    if (gps.location.isUpdated()) {
      GpsRecord gRec;
      gRec.type = LOG_TYPE_GPS;
      gRec.timestamp_ms = now;
      gRec.lat = (int32_t)(gps.location.lat() * 10000000.0);
      gRec.lng = (int32_t)(gps.location.lng() * 10000000.0);
      gRec.alt_cm = (int32_t)(gps.altitude.meters() * 100.0);
      gRec.speed_cms = (uint16_t)(gps.speed.mps() * 100.0);
      gRec.heading_cd = (uint16_t)(gps.course.deg() * 100.0);
      gRec.sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
      gRec.fix_valid = gps.location.isValid() ? 1 : 0;

      if (rocket_state != STATE_PAD) {
        writeBinaryData(&gRec, sizeof(gRec));
      }
    }

    // --- State Machine Transitions ---
    switch (rocket_state) {
      case STATE_PAD: {
        if (test_baro_mode && smoothed_altitude >= LAUNCH_CONFIRM_ALT_M) {
          changeState(STATE_BOOST);
        } else {
          static uint32_t accel_start = 0;
          if (debug_true_accel_ms2 > 15.0) { 
            if (accel_start == 0) accel_start = millis();
            else if (millis() - accel_start > 100) { changeState(STATE_LAUNCH_VALIDATION); setLED(64, 32, 0); }
          } else { accel_start = 0; }
        }
        break;
      }
      case STATE_LAUNCH_VALIDATION:
        if (smoothed_altitude >= LAUNCH_CONFIRM_ALT_M) { changeState(STATE_BOOST); setLED(0, 64, 0); } 
        else if (millis() - state_timer > VALIDATION_TIMEOUT_MS) { changeState(STATE_PAD); setLED(64, 0, 0); }
        break;
      case STATE_BOOST:
        if (test_baro_mode) {
          if (millis() - state_timer > 1000) { burnout_timestamp = millis(); changeState(STATE_COAST); }
        } else {
          if (debug_true_accel_ms2 < 2.0) { burnout_timestamp = millis(); changeState(STATE_COAST); }
        }
        break;
      case STATE_COAST:
        if (millis() - state_timer > MACH_DELAY_MS) {
          max_altitude = smoothed_altitude;
          changeState(STATE_APOGEE_SEARCH);
          setLED(0, 0, 64); 
        }
        break;
      case STATE_APOGEE_SEARCH: { 
        if (smoothed_altitude > max_altitude) max_altitude = smoothed_altitude;
        static uint8_t apogee_confidence_ticks = 0;
        if (smoothed_altitude < (max_altitude - 2.0)) apogee_confidence_ticks++;
        else apogee_confidence_ticks = 0; 
        
        bool baro_apogee = (smoothed_altitude > LAUNCH_FAILSAFE_ALT_M) && (apogee_confidence_ticks >= 5);
        bool timer_failsafe = (millis() - burnout_timestamp > DROGUE_FAILSAFE_TIMEOUT_MS) && (smoothed_altitude > LAUNCH_FAILSAFE_ALT_M);

        if (baro_apogee || timer_failsafe) {
          changeState(STATE_FIRE_DROGUE);
          digitalWrite(CHARGE_PIN_DROGUE_1, HIGH); 
          logBinaryEvent(2, 1, 0);
          setLED(64, 64, 64); 
        }
        break;
      } 
      case STATE_FIRE_DROGUE:
        if (millis() - state_timer > EJECTION_HOLD_TIME) { 
          digitalWrite(CHARGE_PIN_DROGUE_1, LOW); 
          digitalWrite(CHARGE_PIN_DROGUE_2, LOW);
          changeState(STATE_DROGUE_DESCENT);
        } else if (millis() - state_timer > REDUNDANT_CHARGE_DELAY_MS) {
          digitalWrite(CHARGE_PIN_DROGUE_2, HIGH); 
          logBinaryEvent(2, 2, 0);
        }
        break;
      case STATE_DROGUE_DESCENT:
        if ((smoothed_altitude <= MAIN_CHUTE_ALTITUDE_M) && (max_altitude - smoothed_altitude > 50.0)) {
          changeState(STATE_FIRE_MAIN);
          digitalWrite(CHARGE_PIN_MAIN_1, HIGH); 
          logBinaryEvent(3, 1, 0);
        }
        break;
      case STATE_FIRE_MAIN:
        if (millis() - state_timer > EJECTION_HOLD_TIME) { 
          digitalWrite(CHARGE_PIN_MAIN_1, LOW); 
          digitalWrite(CHARGE_PIN_MAIN_2, LOW);
          changeState(STATE_MAIN_DESCENT);
          setLED(64, 0, 64); 
        } else if (millis() - state_timer > REDUNDANT_CHARGE_DELAY_MS) {
          digitalWrite(CHARGE_PIN_MAIN_2, HIGH); 
          logBinaryEvent(3, 2, 0);
        }
        break;
      case STATE_MAIN_DESCENT: break;
      case STATE_SYSTEM_ERROR: break;
    }
  }

  if (test_baro_mode) {
    static uint32_t blink_tick = 0;
    static bool blink_state = false;
    if (now - blink_tick >= 500) {
      blink_tick = now;
      blink_state = !blink_state;
      if (blink_state) setLED(128, 64, 0); 
      else setLED(0, 0, 0);               
    }
  }

  // --- Telemetry Output (Configurable Broadcast Rate Radio & USB) ---
  if (now - last_tx_tick >= telemetry_interval_ms) {
    last_tx_tick = now;
    
    int drogue_count = (digitalRead(SENSE_PIN_1) == HIGH ? 1 : 0) + (digitalRead(SENSE_PIN_2) == HIGH ? 1 : 0);
    int main_count   = (digitalRead(SENSE_PIN_3) == HIGH ? 1 : 0) + (digitalRead(SENSE_PIN_4) == HIGH ? 1 : 0);
    int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

    // Send 26-byte binary packet over HC-12 Radio (Serial1)
    current_packet.sync_word = 0xAA55;
    current_packet.timestamp = now;
    current_packet.state = (uint8_t)rocket_state;
    current_packet.accel = debug_true_accel_ms2;
    current_packet.altitude = smoothed_altitude;
    current_packet.continuity = (drogue_count * 10) + main_count;
    
    if (gps.location.isValid()) {
      current_packet.lat = (int32_t)(gps.location.lat() * 10000000.0);
      current_packet.lng = (int32_t)(gps.location.lng() * 10000000.0);
      current_packet.sats = sats;
    } else {
      current_packet.lat = 0;
      current_packet.lng = 0;
      current_packet.sats = 0;
    }

    uint8_t* payload_start = ((uint8_t*)&current_packet) + 2;
    size_t payload_length = sizeof(current_packet) - 3;
    current_packet.checksum = calculateChecksum(payload_start, payload_length);
    
    Serial1.write((uint8_t*)&current_packet, sizeof(current_packet));

    // Also send human-readable USB telemetry string
    char usb_buf[128];
    if (gps.location.isValid()) {
      snprintf(usb_buf, sizeof(usb_buf), 
               "%lu,%d,%.2f,%.2f,%d%d,%.6f,%.6f,%d",
               now, rocket_state, debug_true_accel_ms2, smoothed_altitude,
               drogue_count, main_count, gps.location.lat(), gps.location.lng(), sats);
    } else {
      snprintf(usb_buf, sizeof(usb_buf), 
               "%lu,%d,%.2f,%.2f,%d%d,,,0",
               now, rocket_state, debug_true_accel_ms2, smoothed_altitude,
               drogue_count, main_count);
    }
    Serial.println(usb_buf);
  }
}
