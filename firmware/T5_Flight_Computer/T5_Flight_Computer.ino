#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_DPS310.h>
#include <MS5611.h>
#include <TinyGPS++.h>
#include <FS.h>
#include <LittleFS.h>

// --- Configuration ---
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

// --- Memory Management ---
#define PRE_FLIGHT_BUFFER_SIZE 20 // 5 seconds at 4Hz
#define TELEMETRY_MAX_LEN 200     // Expanded for quaternion fields
char preFlightBuffer[PRE_FLIGHT_BUFFER_SIZE][TELEMETRY_MAX_LEN];
int bufferIndex = 0;
bool bufferWrapped = false;

// --- Objects ---
Adafruit_BNO08x bno08x(BNO08X_RESET);
Adafruit_DPS310 dps;
TinyGPSPlus gps;
MS5611 ms5611(0x77, &Wire1); 

// --- Variables ---
sh2_SensorValue_t sensorValue;
long reportIntervalUs = 10000; // 100Hz

float accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
float debug_true_accel_ms2 = 0.0;
float quat_i = 0.0, quat_j = 0.0, quat_k = 0.0, quat_real = 1.0;

float dps310_ref_pressure = 0.0;
float ms5611_ref_pressure = 0.0;
float current_dps_pressure = 0.0;
float current_ms_pressure = 0.0;
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
  STATE_SYSTEM_ERROR,
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

const char* log_file_path = "/flight_log.csv";

// --- Binary Telemetry Struct ---
#pragma pack(push, 1)
struct TelemetryPacket {
  uint16_t sync_word;  
  uint32_t timestamp;  
  uint8_t  state;      
  float    accel;      
  float    altitude;   
  uint8_t  continuity; 
  int32_t  lat;        
  int32_t  lng;        
  uint8_t  sats;       
  uint8_t  checksum;   
};
#pragma pack(pop)

TelemetryPacket current_packet;

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

// --- Helper Functions ---
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

void clearLog() {
  if (LittleFS.remove(log_file_path)) broadcastMsg("LOG_CLEARED");
  else broadcastMsg("LOG_CLEAR_FAILED");
}

void dumpLog() {
  if (!fs_operational) { broadcastMsg("FS_ERROR"); return; }
  File file = LittleFS.open(log_file_path, FILE_READ);
  if (!file) { broadcastMsg("FILE_ERROR"); return; }
  broadcastMsg("--- LOG DUMP START ---");
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim(); 
    if (line.length() > 0) broadcastMsg(line.c_str());
    delay(2); 
  }
  broadcastMsg("--- LOG DUMP END ---");
  file.close();
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
    delay(1000); 
    digitalWrite(CHARGE_PIN_DROGUE_1, LOW);
    digitalWrite(CHARGE_PIN_DROGUE_2, LOW);
    broadcastMsg("EVENT: DROGUE TEST COMPLETE.");
  } 
  else if (chargeType == "MAIN") {
    broadcastMsg("EVENT: FIRING TEST MAIN CHARGES!");
    digitalWrite(CHARGE_PIN_MAIN_1, HIGH);
    digitalWrite(CHARGE_PIN_MAIN_2, HIGH);
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
  else if (cmd.indexOf("READ") >= 0) dumpLog();
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
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, report_interval);
  bno08x.enableReport(SH2_ROTATION_VECTOR, report_interval);
}

float calculateAltitudeAGL(float current_pressure, float reference_pressure) {
  if (reference_pressure == 0) return 0.0;
  return 44330.0 * (1.0 - pow(current_pressure / reference_pressure, 0.1903));
}

void logTelemetry(const char* data) {
  if (!fs_operational) return;
  File file = LittleFS.open(log_file_path, FILE_APPEND);
  if (file) {
    file.println(data);
    file.close(); 
  }
}

void logTelemetryBuffer(const char* data) {
  strncpy(preFlightBuffer[bufferIndex], data, TELEMETRY_MAX_LEN - 1);
  preFlightBuffer[bufferIndex][TELEMETRY_MAX_LEN - 1] = '\0';
  bufferIndex++;
  if (bufferIndex >= PRE_FLIGHT_BUFFER_SIZE) {
    bufferIndex = 0;
    bufferWrapped = true;
  }
}

void flushTelemetryBuffer() {
  if (!fs_operational) return;
  File file = LittleFS.open(log_file_path, FILE_APPEND);
  if (file) {
    int count = bufferWrapped ? PRE_FLIGHT_BUFFER_SIZE : bufferIndex;
    for (int i = 0; i < count; i++) {
      int idx = bufferWrapped ? ((bufferIndex + i) % PRE_FLIGHT_BUFFER_SIZE) : i;
      file.println(preFlightBuffer[idx]);
    }
    file.close();
  }
  bufferWrapped = false;
  bufferIndex = 0;
}

void changeState(FlightState new_state) {
  if ((rocket_state == STATE_PAD && new_state == STATE_LAUNCH_VALIDATION) || 
      (rocket_state == STATE_PAD && new_state == STATE_BOOST)) {
    flushTelemetryBuffer();
  }

  rocket_state = new_state;
  state_timer = millis();
  char msg[64];
  snprintf(msg, sizeof(msg), "TRANSITION: %s", state_names[new_state]);
  broadcastMsg(msg);
  logTelemetry(msg);
}

// --- Setup ---
void setup(void) {
  // 1. Initialize all Serial ports FIRST so all boot & diagnostic logs are visible
  Serial.begin(115200);
  Serial1.begin(TELEMETRY_BAUD, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(1500); // Allow USB Serial Monitor to open

  broadcastMsg("\n================================================");
  broadcastMsg("        T5 FLIGHT COMPUTER BOOT SEQUENCE        ");
  broadcastMsg("================================================");

  setLED(64, 64, 0); 

  if (LittleFS.begin(true)) { 
    fs_operational = true;
    File file = LittleFS.open(log_file_path, FILE_APPEND);
    if (file) {
      file.println("\n--- NEW BOOT ---");
      file.println("Time_ms,State,AccelTotal,AltFused,Continuity,Lat,Lng,Sats,QuatI,QuatJ,QuatK,QuatReal");
      file.close();
    }
    broadcastMsg("[OK] LittleFS initialized.");
  } else {
    broadcastMsg("[WARN] LittleFS mount failed.");
  }
  
  broadcastMsg("Initializing I2C buses...");
  Wire.begin(8, 9, 40000); 
  Wire.setTimeOut(100);
  Wire1.begin(MS5611_SDA, MS5611_SCL, 40000); 
  Wire1.setTimeOut(100);
  delay(500);
  
  broadcastMsg("Probing BNO085...");
  if (bno08x.begin_I2C(0x4B, &Wire) || bno08x.begin_I2C(0x4A, &Wire)) {
    bno08x_active = true;
    delay(50);
    setReports(reportIntervalUs);
    broadcastMsg("[OK] BNO085 online!");
  } else {
    broadcastMsg("[WARN] BNO085 not found on Wire (0x4B / 0x4A).");
  }

  broadcastMsg("Probing DPS310...");
  if (dps.begin_I2C(0x77, &Wire) || dps.begin_I2C(0x76, &Wire)) { 
    dps310_active = true;
    // CRITICAL: Put DPS310 into Standby BEFORE modifying sample rates or oversampling!
    dps.setMode(DPS310_IDLE);
    delay(10);
    dps.configurePressure(DPS310_16HZ, DPS310_16SAMPLES);
    dps.configureTemperature(DPS310_16HZ, DPS310_16SAMPLES);
    dps.setMode(DPS310_CONT_PRESTEMP);
    broadcastMsg("[OK] DPS310 online & calibrated!");
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
    snprintf(refMsg, sizeof(refMsg), "Pad Baseline: DPS=%.2f hPa (n=%d), MS=%.2f hPa (n=%d)",
             dps310_ref_pressure, valid_dps_samples, ms5611_ref_pressure, valid_ms_samples);
    broadcastMsg(refMsg);
  }

  if (bno08x_active && (dps310_active || ms5611_active)) {
    rocket_state = STATE_PAD;
    setLED(0, 64, 0); 
    broadcastMsg("ALL SENSORS NOMINAL. SYSTEM READY.");
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

  if (bno08x_active) {
    static uint32_t last_reset_reconfig = 0;
    if (bno08x.wasReset() && (now - last_reset_reconfig > 1000)) {
      last_reset_reconfig = now;
      delay(10);
      setReports(reportIntervalUs);
    }
    uint8_t event_count = 0;
    while (bno08x.getSensorEvent(&sensorValue) && (event_count++ < 5)) {
      if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        accel_x = sensorValue.un.linearAcceleration.x;
        accel_y = sensorValue.un.linearAcceleration.y;
        accel_z = sensorValue.un.linearAcceleration.z;
        debug_true_accel_ms2 = sqrt(sq(accel_x) + sq(accel_y) + sq(accel_z));
      } else if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        quat_i = sensorValue.un.rotationVector.i;
        quat_j = sensorValue.un.rotationVector.j;
        quat_k = sensorValue.un.rotationVector.k;
        quat_real = sensorValue.un.rotationVector.real;
      }
    }
  }

  if (now - last_state_tick >= 20) {
    last_state_tick = now;
    static float dps_alt = 0.0, ms_alt = 0.0;

    if (dps310_active) {
      if (dps.temperatureAvailable() && dps.pressureAvailable()) {
        sensors_event_t temp_event, pressure_event;
        dps.getEvents(&temp_event, &pressure_event);
        if (pressure_event.pressure >= 300.0 && pressure_event.pressure <= 1200.0) {
          current_dps_pressure = pressure_event.pressure;
          dps_alt = calculateAltitudeAGL(current_dps_pressure, dps310_ref_pressure);
        }
      }
    }
    if (ms5611_active) {
      ms5611.read();
      float new_ms_pressure = ms5611.getPressure();
      if (new_ms_pressure > 0) {
        current_ms_pressure = new_ms_pressure;
        ms_alt = calculateAltitudeAGL(current_ms_pressure, ms5611_ref_pressure);
      }
    }

    altFilter.predict();
    if (ms5611_active && current_ms_pressure > 0) altFilter.update(ms_alt, 5.0); 
    if (dps310_active && current_dps_pressure > 0) altFilter.update(dps_alt, 1.0); 

    // Output live Kalman-filtered altitude in all states (including STATE_PAD)
    smoothed_altitude = altFilter.getAltitude();

    // STATE MACHINE
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
        }
        break;
      case STATE_DROGUE_DESCENT:
        if ((smoothed_altitude <= MAIN_CHUTE_ALTITUDE_M) && (max_altitude - smoothed_altitude > 50.0)) {
          changeState(STATE_FIRE_MAIN);
          digitalWrite(CHARGE_PIN_MAIN_1, HIGH); 
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

    // 1. Send Binary Struct over Radio (Serial1)
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

    // 2. Format CSV for USB debugging and LittleFS local logging
    char telemetry_buffer[TELEMETRY_MAX_LEN];
    if (gps.location.isValid()) {
      snprintf(telemetry_buffer, sizeof(telemetry_buffer), 
               "%lu,%d,%.2f,%.2f,%d%d,%.6f,%.6f,%d,%.4f,%.4f,%.4f,%.4f",
               now, rocket_state, debug_true_accel_ms2, smoothed_altitude,
               drogue_count, main_count, gps.location.lat(), gps.location.lng(), sats,
               quat_i, quat_j, quat_k, quat_real);
    } else {
      snprintf(telemetry_buffer, sizeof(telemetry_buffer), 
               "%lu,%d,%.2f,%.2f,%d%d,,,0,%.4f,%.4f,%.4f,%.4f",
               now, rocket_state, debug_true_accel_ms2, smoothed_altitude,
               drogue_count, main_count,
               quat_i, quat_j, quat_k, quat_real);
    }

    Serial.println(telemetry_buffer); // Print to USB (human-readable)

    // Save to local LittleFS flash memory
    if (rocket_state == STATE_PAD) {
      logTelemetryBuffer(telemetry_buffer);
    } else if (rocket_state == STATE_MAIN_DESCENT) {
      static uint32_t last_landed_log = 0;
      if (now - last_landed_log >= 1000) { 
        last_landed_log = now;
        logTelemetry(telemetry_buffer);
      }
    } else {
      logTelemetry(telemetry_buffer);
    }
  }
}