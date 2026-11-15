# T5 Telemetry & Data Logging Specification

## 1. Serial USB Telemetry (4 Hz CSV Stream)

The USB port outputs a comma-separated values (CSV) string designed for ground station terminals and flight log graphing:

```text
Time_ms,State,AccelTotal,AltFused,Continuity,Lat,Lng,Sats,QuatI,QuatJ,QuatK,QuatReal
```

### Example Record:
```text
7268,1,0.00,0.17,00,14.599512,120.984219,8,0.0000,0.0000,0.0000,1.0000
```

### Field Descriptions:
1. **`Time_ms`** (uint32): Milliseconds elapsed since flight computer power-on.
2. **`State`** (uint8): Current rocket state machine enum:
   * `0`: `STATE_SYSTEM_ERROR`
   * `1`: `STATE_PAD`
   * `2`: `STATE_LAUNCH_VALIDATION`
   * `3`: `STATE_BOOST`
   * `4`: `STATE_COAST`
   * `5`: `STATE_APOGEE_SEARCH`
   * `6`: `STATE_FIRE_DROGUE`
   * `7`: `STATE_DROGUE_DESCENT`
   * `8`: `STATE_FIRE_MAIN`
   * `9`: `STATE_MAIN_DESCENT`
3. **`AccelTotal`** (float, 2 dec): Total linear acceleration magnitude in $\text{m/s}^2$ (gravity vector subtracted by BNO085 sensor fusion).
4. **`AltFused`** (float, 2 dec): Fused Above-Ground-Level (AGL) altitude in meters from the Dual Barometer Kalman Filter.
5. **`Continuity`** (2-digit string): First digit = operational drogue charges ($0-2$), Second digit = operational main charges ($0-2$).
6. **`Lat`** (float, 6 dec): Latitude in decimal degrees (empty if no GPS fix).
7. **`Lng`** (float, 6 dec): Longitude in decimal degrees (empty if no GPS fix).
8. **`Sats`** (uint8): Tracked GPS satellite count.
9. **`QuatI, QuatJ, QuatK, QuatReal`** (float, 4 dec): Absolute 3D orientation quaternion from BNO085.

---

## 2. Radio Telemetry (Serial1 HC-12 Binary Packet)

To maximize RF transmission speed and prevent transmission drops, telemetry over HC-12 is serialized into a packed binary struct:

```cpp
#pragma pack(push, 1)
struct TelemetryPacket {
  uint16_t sync_word;   // 0xAA55
  uint32_t timestamp;   // Milliseconds
  uint8_t  state;       // FlightState enum (0-9)
  float    accel;       // Linear acceleration magnitude (m/s^2)
  float    altitude;    // Kalman-fused altitude AGL (meters)
  uint8_t  continuity;  // (drogue_count * 10) + main_count
  int32_t  lat;         // Latitude * 10,000,000 (scaled integer)
  int32_t  lng;         // Longitude * 10,000,000 (scaled integer)
  uint8_t  sats;        // Satellite count
  uint8_t  checksum;    // XOR checksum over payload
};
#pragma pack(pop)
```

---

## 3. Ground Station Command Console

Commands can be transmitted into either USB `Serial` or HC-12 `Serial1` terminated with newline (`\n`):

| Command | Action |
| :--- | :--- |
| `DUMP_BIN` / `READ_BIN` | Streams the high-rate binary flight log (`/flight_log.bin`) framed with byte length |
| `READ` | Streams the binary flight log over serial (compatible with Ground Station) |
| `CLEAR` | Wipes flash memory and clears `/flight_log.bin` |
| `ZERO_ALT` | Re-zeros barometric altitude to current pad barometric pressure |
| `SET_TX_RATE <hz>` | Dynamically adjust live radio broadcast rate (e.g. `SET_TX_RATE 2` or `SET_TX_RATE 10`) |
| `TEST_BARO` | Toggles barometer simulation test mode (permits simulated flight testing on the bench) |
| `TEST_DROGUE`| Arm and fire drogue test pyro (3-second audible warning before pulse) |
| `TEST_MAIN` | Arm and fire main test pyro (3-second audible warning before pulse) |
| `REBOOT` / `RESET` | Performs software reset of the ESP32-S3 |

---

## 4. High-Rate Local Flash Binary Protocol (50 Hz & 100 Hz)

To capture full raw dynamics without bottlenecking MCU loop execution or wasting flash storage, data is recorded into `/flight_log.bin` using **Typed Packet Framing**:

### Packet Type `0x01`: High-Rate IMU & Dual Barometer (42 bytes, 50 Hz or 100 Hz)
```cpp
#pragma pack(push, 1)
struct ImuBaroRecord {
  uint8_t  type;         // 0x01
  uint32_t timestamp_ms; // System uptime in ms
  uint8_t  state;        // Flight state enum (0-9)
  int16_t  accel_x;      // Calibrated m/s^2 * 100
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
  uint8_t  pyro_status;  // Active charge bitmask (D1, D2, M1, M2)
};
#pragma pack(pop)
```

### Packet Type `0x02`: GNSS / GPS Update (23 bytes, logged only when new fix arrives)
```cpp
#pragma pack(push, 1)
struct GpsRecord {
  uint8_t  type;         // 0x02
  uint32_t timestamp_ms; // System uptime in ms
  int32_t  lat;          // Decimal deg * 10,000,000
  int32_t  lng;          // Decimal deg * 10,000,000
  int32_t  alt_cm;       // Altitude MSL in cm
  uint16_t speed_cms;    // Ground speed in cm/s
  uint16_t heading_cd;   // Course in centidegrees (deg * 100)
  uint8_t  sats;         // Satellites count
  uint8_t  fix_valid;    // 1 if valid, 0 otherwise
};
#pragma pack(pop)
```

### Packet Type `0x03`: Flight Events & Transitions (8 bytes, event-driven)
```cpp
#pragma pack(push, 1)
struct EventRecord {
  uint8_t  type;         // 0x03
  uint32_t timestamp_ms; // System uptime in ms
  uint8_t  event_id;     // 1: TRANSITION, 2: DROGUE_FIRE, 3: MAIN_FIRE, 4: APOGEE
  uint8_t  param1;       // New state / detail
  uint8_t  param2;       // Old state / extra
};
#pragma pack(pop)
```

### RAM Buffering Pipeline
* **512-Byte Write Buffer:** Records pack into a 512-byte RAM buffer before writing to LittleFS in blocks, eliminating individual flash write stalls.
* **Pre-Flight Pad Buffer:** The last 5 seconds on the pad cycle inside a circular RAM buffer (250 records @ 50 Hz, 500 records @ 100 Hz). Upon launch validation, the buffer flushes to flash immediately.
