# T5 Flight Computer Pinout & Hardware Architecture

## Microcontroller
* **MCU**: ESP32-S3 (Dual-core Xtensa LX7 @ 240 MHz, 512KB SRAM, USB-OTG Native)
* **Bus Speed**: 40 kHz (I2C Low-Speed High-Noise-Immunity Mode)

---

## 1. I2C Bus Topology (Dual-Bus Isolation)

The flight computer separates redundant sensors across two distinct hardware I2C peripheral controllers to eliminate single-point bus lockups.

| Bus | SDA Pin | SCL Pin | Connected Devices | Default Address |
| :--- | :--- | :--- | :--- | :--- |
| **Primary I2C (`Wire`)** | **GPIO 8** | **GPIO 9** | • BNO085 (9-DoF IMU)<br>• DPS310 (Primary Barometer) | `0x4B` (or `0x4A`)<br>`0x77` (or `0x76`) |
| **Secondary I2C (`Wire1`)** | **GPIO 40** | **GPIO 39** | • MS5611 (Redundant Barometer) | `0x77` |

> [!NOTE]
> `0x77` is shared between DPS310 and MS5611, but because they are physically routed to separate buses (`Wire` and `Wire1`), there is **no address collision**.

---

## 2. Serial & Communication Interfaces

| Peripheral | RX Pin | TX Pin | Baud Rate | Function |
| :--- | :--- | :--- | :--- | :--- |
| **USB CDC (`Serial`)** | Native USB | Native USB | 115200 | Live CSV Telemetry, Diagnostic Logs, Command Console |
| **Radio UART (`Serial1`)** | **GPIO 17** | **GPIO 18** | 115200 | HC-12 Long-Range 433MHz Telemetry Radio (Binary Struct) |
| **GPS UART (`Serial2`)** | **GPIO 42** | **GPIO 41** | 38400 | NMEA GPS Receiver (TinyGPSPlus Parser) |

---

## 3. Ejection Pyro Charges & Continuity Sensing

All pyro channels use dual redundant MOSFET firing channels.

| Signal | GPIO Pin | Type | Notes |
| :--- | :--- | :--- | :--- |
| **Drogue Fire 1** | **GPIO 6** | Digital Output | Primary drogue ejection charge |
| **Drogue Fire 2** | **GPIO 7** | Digital Output | Redundant drogue backup charge (fires +1.0s) |
| **Main Fire 1** | **GPIO 15** | Digital Output | Primary main parachute ejection charge |
| **Main Fire 2** | **GPIO 16** | Digital Output | Redundant main backup charge (fires +1.0s) |
| **Drogue Sense 1** | **GPIO 4** | Digital Input | Continuity sense circuit (`INPUT_PULLDOWN`) |
| **Drogue Sense 2** | **GPIO 5** | Digital Input | Continuity sense circuit (`INPUT_PULLDOWN`) |
| **Main Sense 1** | **GPIO 1** | Digital Input | Continuity sense circuit (`INPUT_PULLDOWN`) |
| **Main Sense 2** | **GPIO 2** | Digital Input | Continuity sense circuit (`INPUT_PULLDOWN`) |

---

## 4. Status Indicators

| Indicator | GPIO Pin | Type | Notes |
| :--- | :--- | :--- | :--- |
| **Audible Buzzer** | **GPIO 36** | Digital Output | Pad ready tone, error beeps, recovery locator |
| **External Status LED**| **GPIO 38** | NeoPixel / RGB | Multi-color state transitions & error alerts |
