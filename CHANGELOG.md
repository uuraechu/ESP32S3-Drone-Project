# Changelog

All notable changes to the ESP32-S3 Quadcopter MK2 firmware are recorded here.

## [Unreleased]

### Pending
- First stable hover flight — validate Kp=1.5
- PITCH_TRIM / ROLL_TRIM — set at first hover
- ICM-42688-P IMU driver port (SPI, replaces Adafruit MPU6050)
- Set `LOOP_INTERVAL_MS 1` after ICM-42688 ported — verify 1kHz via dashboard dt readout
- MMC5983MA magnetometer driver port (SPI, replaces HMC5883L)
- BMP388 barometer driver port (BMP388_DEV library, SPI, replaces BMP280)
- Re-run magnetometer calibration after hardware swap
- Update D filter Hz after loop rate increase (attitude axes: raise toward 50–100Hz)
- MAG calibration improvement (current offsets are estimated)
- FAA Remote ID — external broadcast module required (software-only ESP32 BLE insufficient for ASTM F3411-22a)
- PROP_GYRO_COEFF and YAW_THROTTLE_COEFF tuning after Kp is stable (currently 0.0)
- Wind rejection validation — increase Kp in 0.3 steps if sluggish

### Frame rebuild pending
- Move battery below base plate to lower center of mass to motor plane
- After rebuild: update `offsetZ` in `readSensors()`
- After rebuild: re-run IMU calibration and update GYRO/ACCEL offsets
- After rebuild: re-tune PITCH_TRIM / ROLL_TRIM

---

## [MK2 Initial Build]

### Added
- ESP32-S3-WROOM-1 flight controller, replacing previous MCU
- DSHOT300 motor protocol via ESP32 RMT peripheral (one channel per motor)
- Dual-core task architecture: flight loop on Core 1, iBUS / GPS / WiFi on Core 0
- 200Hz flight loop (`LOOP_INTERVAL_MS 5`)
- 1D Kalman filter per attitude axis (roll, pitch, yaw)
- Complementary filter for altitude fusion (accelerometer + barometer)
- iBUS receiver parsing on UART1
- NEO-6M GPS on UART2 at 9600 baud, 20Hz update task
- MPU6050 IMU (I2C) with hard-iron calibrated offsets
- HMC5883L magnetometer (I2C) with hard-iron offsets
- BMP280 barometer (I2C)
- WiFi AP telemetry dashboard at `http://192.168.4.1`
  - `/data` JSON endpoint (10Hz armed, 1Hz disarmed)
  - `/pid/get` and `/pid/set` live tuning endpoints (blocked while armed)
  - `/status` and `/debug/on` / `/debug/off` endpoints
  - RC panel showing all 8 channels
  - PID tab with 16 fields (Kp/Ki/Kd/DHZ × 4 axes)
- Altitude hold with 0.5s ramp-in/ramp-out, 5m minimum engagement height
- Failsafe: 500ms timeout → 5s hover → 0.3 m/s descent
- Battery auto-detection: >13.5V = 4S (1500mAh), ≤13.5V = 3S (2200mAh)
- Per-cell voltage thresholds: warning 3.4V, cutoff 3.1V, recovery 3.2V
- Quadratic ADC voltage calibration polynomial
- Buzzer arming/warning tones
- LED indicators: ARMED, LOW_BAT, ALT_HOLD, FAILSAFE
- Software trim: PITCH_TRIM and ROLL_TRIM (degrees)
- Cross-axis coupling stubs: PROP_GYRO_COEFF and YAW_THROTTLE_COEFF (0.0, disabled)
- Anti-windup conditional integration in PID
- Derivative-on-measurement PID (no setpoint kick)
- EMA low-pass filter on D term (configurable cutoff Hz per axis)
- Motor mixer for X-config with correct roll/pitch/yaw signs
- Battery cutoff override on CH7 (SWC)
- Assisted descent rate on CH8 (SWD, 3-position: off / 0.3 m/s / 0.6 m/s)
- `dataMutex`, `rcMutex`, `gpsMutex` for cross-core safety
- Calibration constants for Cookeville TN magnetic declination (−5.5°)
