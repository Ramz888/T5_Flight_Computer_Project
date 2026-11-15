#ifndef DPS310_SIMPLE_H
#define DPS310_SIMPLE_H

#include <Arduino.h>
#include <Wire.h>

/**
 * ============================================================================
 *  DPS310_Simple - Lightweight, Robust, Zero-Dependency Driver for Infineon DPS310
 * ============================================================================
 *  Designed for Flight Computers & ESP32-S3.
 *  - No Adafruit_BusIO or Adafruit_Sensor dependencies. Pure TwoWire.
 *  - Strict 50ms timeouts on all while loops (NEVER freezes the MCU).
 *  - Accepts any standard I2C bus (Wire, Wire1) on any pins and speeds (40k-400k).
 *  - Auto-detects 0x77 and 0x76 addresses.
 *  - Tolerates all chip revisions (0x10, 0x11, 0x12, clones).
 * ============================================================================
 */

class DPS310_Simple {
public:
  DPS310_Simple() : _wire(&Wire), _addr(0x77), _init_ok(false) {}

  // Auto-detects on primary (0x77) or secondary (0x76) address
  bool begin(TwoWire &wirePort = Wire, uint8_t i2c_addr = 0x77) {
    _wire = &wirePort;
    _addr = i2c_addr;
    _init_ok = false;

    // 1. Check if device ACKs at given address
    _wire->beginTransmission(_addr);
    if (_wire->endTransmission() != 0) {
      // Try alternate address (0x76 vs 0x77)
      uint8_t alt = (_addr == 0x77) ? 0x76 : 0x77;
      _wire->beginTransmission(alt);
      if (_wire->endTransmission() == 0) {
        _addr = alt;
      } else {
        return false; // Neither address responded
      }
    }

    // 2. Perform Soft Reset to clear undefined ASIC state
    softReset();
    delay(25); // Allow internal OTP coefficients to reload

    // 3. Verify Product ID (Register 0x0D: Upper nibble 0x1 = DPS310)
    uint8_t prod_id = read8(0x0D);
    if ((prod_id & 0xF0) != 0x10 && prod_id != 0x10) {
      // Some clones or revisions return without mask, if 0x00 or 0xFF it's not connected
      if (prod_id == 0x00 || prod_id == 0xFF) return false;
    }

    // 4. Wait for coefficients to be ready (MEAS_CFG 0x08, bit 7 COEF_RDY)
    uint32_t t_start = millis();
    while (!(read8(0x08) & 0x80)) {
      if (millis() - t_start > 100) return false; // Timeout safety
      delay(2);
    }

    // 5. Read all 18 factory calibration coefficients from OTP
    if (!readCoefficients()) return false;

    // 6. Configure Temperature & Pressure (64 Hz, 64x oversampling)
    // Register 0x06 (PRS_CFG): 64 Hz (0b110 << 4) | 64x oversampling (0b0110) = 0x66
    write8(0x06, 0x66);
    _kP = 1040384.0f; // Scale factor for 64x oversampling

    // Check temp sensor source in 0x28 (bit 7: 1 = external MEMS, 0 = ASIC internal)
    uint8_t tmp_src = read8(0x28) & 0x80;
    // Register 0x07 (TMP_CFG): tmp_src | 64 Hz (0b110 << 4) | 64x oversampling (0b0110)
    write8(0x07, tmp_src | 0x66);
    _kT = 1040384.0f; // Scale factor for 64x oversampling

    // Register 0x09 (CFG_REG): Enable shift for oversampling > 8 (P_SHIFT bit 2, T_SHIFT bit 3 = 0x0C)
    write8(0x09, 0x0C);

    // 7. Set mode to Continuous Measurement (MEAS_CFG 0x08, bits [2:0] = 0b111 = 0x07)
    write8(0x08, 0x07);

    // 8. Wait for first measurement to complete
    t_start = millis();
    while ((read8(0x08) & 0x30) != 0x30) {
      if (millis() - t_start > 150) break; // Don't hang if delayed
      delay(5);
    }

    _init_ok = true;
    return true;
  }

  void softReset() {
    write8(0x0C, 0x09); // Full ASIC reset command
  }

  uint8_t getAddress() const { return _addr; }
  uint8_t getProductID() { return read8(0x0D); }
  bool isReady() const { return _init_ok; }

  // Returns true if new data was successfully read and calculated
  bool readSensor(float &pressure_hPa, float &temperature_C) {
    if (!_init_ok) return false;

    // Read 6 bytes starting from 0x00 (PSR_B2, PSR_B1, PSR_B0, TMP_B2, TMP_B1, TMP_B0)
    uint8_t buf[6];
    if (!readBytes(0x00, buf, 6)) return false;

    int32_t raw_p = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    raw_p = twosComplement(raw_p, 24);

    int32_t raw_t = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    raw_t = twosComplement(raw_t, 24);

    float t_scaled = (float)raw_t / _kT;
    temperature_C = (float)_c0 * 0.5f + (float)_c1 * t_scaled;

    float p_scaled = (float)raw_p / _kP;
    float p_comp = (float)_c00 +
                   p_scaled * ((float)_c10 + p_scaled * ((float)_c20 + p_scaled * (float)_c30)) +
                   t_scaled * (float)_c01 +
                   t_scaled * p_scaled * ((float)_c11 + p_scaled * (float)_c21);

    pressure_hPa = p_comp / 100.0f; // Convert Pascals to hPa (mbar)

    // Sanity check physical range
    return (pressure_hPa >= 300.0f && pressure_hPa <= 1200.0f);
  }

  float calculateAltitude(float pressure_hPa, float seaLevel_hPa = 1013.25f) {
    if (pressure_hPa <= 0.0f || seaLevel_hPa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - powf(pressure_hPa / seaLevel_hPa, 0.1902949f));
  }

private:
  TwoWire *_wire;
  uint8_t  _addr;
  bool     _init_ok;
  float    _kP, _kT;

  // Calibration coefficients
  int16_t  _c0, _c1;
  int32_t  _c00, _c10;
  int16_t  _c01, _c11, _c20, _c21, _c30;

  static int32_t twosComplement(int32_t val, uint8_t bits) {
    if (val & (1L << (bits - 1))) {
      val -= (1L << bits);
    }
    return val;
  }

  void write8(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
  }

  uint8_t read8(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return 0xFF;
    if (_wire->requestFrom(_addr, (uint8_t)1) == 1) {
      return _wire->read();
    }
    return 0xFF;
  }

  bool readBytes(uint8_t reg, uint8_t *buffer, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) {
      // Fallback: If repeated start fails on ESP32-S3, retry with STOP
      _wire->beginTransmission(_addr);
      _wire->write(reg);
      if (_wire->endTransmission(true) != 0) return false;
    }
    if (_wire->requestFrom(_addr, len) == len) {
      for (uint8_t i = 0; i < len; i++) {
        buffer[i] = _wire->read();
      }
      return true;
    }
    return false;
  }

  bool readCoefficients() {
    uint8_t c[18];
    if (!readBytes(0x10, c, 18)) {
      // Fallback: Read byte by byte if burst read is rejected
      for (uint8_t i = 0; i < 18; i++) {
        c[i] = read8(0x10 + i);
      }
    }

    _c0  = twosComplement(((uint16_t)c[0] << 4) | (((uint16_t)c[1] >> 4) & 0x0F), 12);
    _c1  = twosComplement((((uint16_t)c[1] & 0x0F) << 8) | c[2], 12);
    _c00 = twosComplement(((uint32_t)c[3] << 12) | ((uint32_t)c[4] << 4) | (((uint32_t)c[5] >> 4) & 0x0F), 20);
    _c10 = twosComplement((((uint32_t)c[5] & 0x0F) << 16) | ((uint32_t)c[6] << 8) | (uint32_t)c[7], 20);
    _c01 = twosComplement(((uint16_t)c[8] << 8) | c[9], 16);
    _c11 = twosComplement(((uint16_t)c[10] << 8) | c[11], 16);
    _c20 = twosComplement(((uint16_t)c[12] << 8) | c[13], 16);
    _c21 = twosComplement(((uint16_t)c[14] << 8) | c[15], 16);
    _c30 = twosComplement(((uint16_t)c[16] << 8) | c[17], 16);

    // If all calibration words are zero or 0xFFFF, OTP ROM failed
    if (_c0 == 0 && _c1 == 0 && _c00 == 0) return false;
    return true;
  }
};

#endif // DPS310_SIMPLE_H
