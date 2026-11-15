# T5 Dual-Redundant Flight Computer Project

An advanced, dual-redundant model rocketry flight computer firmware and diagnostic suite developed for the **ESP32-S3**.

---

## Project Structure

```text
T5_Flight_Computer_Project/
│
├── README.md                                   <-- Project Overview & Quick Start
│
├── firmware/
│   └── T5_Flight_Computer/
│       ├── T5_Flight_Computer.ino              <-- Primary Flight Software
│       └── DPS310_Simple.h                     <-- Zero-dependency robust DPS310 driver
│
├── diagnostics/
│   ├── FlightComputer_Systems_Check/
│   │   └── FlightComputer_Systems_Check.ino    <-- Full hardware & continuity self-test
│   ├── DPS310_Custom_Test/
│   │   ├── DPS310_Custom_Test.ino              <-- Standalone bench test for DPS310
│   │   └── DPS310_Simple.h
│   └── Dual_I2C_Scanner/
│       └── Dual_I2C_Scanner.ino                <-- Probes Wire (8,9) and Wire1 (40,39)
│
└── docs/
    ├── PINOUT_AND_WIRING.md                    <-- Complete hardware & pin mapping
    └── TELEMETRY_PROTOCOL.md                   <-- CSV & binary telemetry packet specs
```

---

## Key Hardware Features & Defenses

1. **Dual Isolated I2C Buses**:
   * **`Wire` (GPIO 8, 9 @ 40 kHz)**: BNO085 (9-DoF IMU at `0x4B`) + DPS310 (Barometer at `0x77`).
   * **`Wire1` (GPIO 40, 39 @ 40 kHz)**: MS5611 (Redundant Barometer at `0x77`).
   * Eliminates single-bus lockups and allows two devices with address `0x77` to coexist simultaneously.

2. **Dual-Barometer Kalman Filter**:
   * Fuses DPS310 and MS5611 altitude estimates with adaptive sensor noise weighting.
   * If either barometer glitches or disconnects, the Kalman filter smoothly transitions to the surviving sensor.

3. **Non-Blocking Fault Tolerance**:
   * All I2C read and write operations feature hardware-enforced timeouts.
   * The BNO085 event loop is rate-limited and bounded to prevent SHTP protocol lockups.
   * DPS310 safely transitions to `Standby` mode before modifying sampling rates or oversampling, preventing ADC saturation lockups (`0x7FFFFF`).

4. **Dual Ejection Pyros with Continuity Sensing**:
   * Dual drogue (Pins 6, 7) and dual main (Pins 15, 16) ejection channels.
   * 4-channel continuity sense pins (Pins 4, 5, 1, 2) monitored in real time at 4 Hz.

5. **Flash Memory Blackbox**:
   * High-speed flight logging via onboard LittleFS flash memory (`/flight_log.csv`).
   * Circular pre-flight FIFO buffer preserves the 5 seconds immediately preceding motor ignition.

---

## Quick Start / How to Upload

### 1. Opening in Arduino IDE
To open any sketch in the Arduino IDE, open the `.ino` file directly from its respective directory:
* **Primary Flight Code**: `firmware/T5_Flight_Computer/T5_Flight_Computer.ino`
* **Bench Hardware Test**: `diagnostics/FlightComputer_Systems_Check/FlightComputer_Systems_Check.ino`
* **Dual I2C Scanner**: `diagnostics/Dual_I2C_Scanner/Dual_I2C_Scanner.ino`

### 2. Required Arduino Libraries
* `Adafruit BNO08x`
* `Adafruit DPS310` (or the included header-only `DPS310_Simple.h`)
* `MS5611` (by Rob Tillaart)
* `TinyGPSPlus`
* `LittleFS` (built into ESP32 core)

### 3. Board Settings (ESP32-S3)
* **Board**: `ESP32S3 Dev Module` (or your specific S3 board)
* **USB CDC On Boot**: `Enabled` (Crucial for native USB serial logs)
* **Flash Size**: `4MB` or `8MB` (matching your module)
* **Upload Speed**: `921600`
