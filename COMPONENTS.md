# Components

Full bill of materials for the ESP32-S3 Quadcopter MK2.

## Flight Controller

| Component | Part | Status | Notes |
|-----------|------|--------|-------|
| MCU | ESP32-S3-WROOM-1 | Installed | Dual-core 240MHz, hardware FPU, WiFi/BT |

## Sensors (Current)

| Component | Part | Interface | Status |
|-----------|------|-----------|--------|
| IMU | MPU6050 (gyro + accel) | I2C (SCL=6, SDA=7) | Installed — being replaced |
| Magnetometer | HMC5883L | I2C (shared bus) | Installed — being replaced |
| Barometer | BMP280 | I2C (shared bus) | Installed — being replaced |
| GPS | u-blox NEO-6M | UART2 (RX=10, TX=9) | Installed, 5Hz output |

## Sensors (Ordered / Planned Replacements)

| Component | Part | Replaces | Interface | Key Improvement |
|-----------|------|----------|-----------|-----------------|
| IMU | ICM-42688-P | MPU6050 | SPI 24MHz | ~10µs read vs ~400µs I2C; enables 1kHz loop |
| Magnetometer | MMC5983MA | HMC5883L | SPI | ±0.5° heading, 0.4mG noise, 1000Hz ODR, 18-bit |
| Barometer | BMP388 | BMP280 | SPI 4MHz | ~4× lower noise RMS, built-in FIFO, BMP388_DEV library |

> Alternative: **Sysrox 10-DOF board** (ICM-42688 + MMC5983MA + LPS22HB) — all three sensors on one SPI board.

## Propulsion

| Component | Part | Qty | Notes |
|-----------|------|-----|-------|
| ESC | 4-in-1 or individual (DSHOT300) | 4 | DSHOT300 via RMT; 2kΩ pullup to 3V3 on each signal line |
| Motor | RS2205 2300KV | 4 | M1/M4 CCW, M2/M3 CW |

## Motor Configuration (X-frame)

| Motor | Position | Spin | Roll | Pitch | Yaw |
|-------|----------|------|------|-------|-----|
| M1 (ESC1, pin 1) | Front-Right | CCW | −roll | +pitch | −yaw |
| M2 (ESC2, pin 2) | Rear-Right  | CW  | −roll | −pitch | +yaw |
| M3 (ESC3, pin 4) | Front-Left  | CW  | +roll | +pitch | +yaw |
| M4 (ESC4, pin 5) | Rear-Left   | CCW | +roll | −pitch | −yaw |

## Receiver

| Component | Part | Interface | Notes |
|-----------|------|-----------|-------|
| Receiver | iBUS-compatible (FlySky) | UART1 (RX=12) | 14 channels, 115200 baud |

## Power

| Component | Part | Notes |
|-----------|------|-------|
| Battery | 4S 1500mAh LiPo | Auto-detected at boot (>13.5V) |
| Battery | 3S 2200mAh LiPo | Auto-detected at boot (≤13.5V) |
| TWR | 5.69:1 | At 4S with RS2205 2300KV |

### Battery Thresholds

| Threshold | Per Cell | 3S Pack | 4S Pack |
|-----------|----------|---------|---------|
| Warning | 3.4V | 10.2V | 13.6V |
| Cutoff | 3.1V | 9.3V | 12.4V |
| Recovery | 3.2V | 9.6V | 12.8V |

## Indicators

| Component | Pin | Function |
|-----------|-----|----------|
| LED | 13 | Armed |
| LED | 14 | Low battery |
| LED | 15 | Altitude hold active |
| LED | 17 | Failsafe active |
| Buzzer | 16 | Arming tones, warnings (LEDC channel 0) |

## ADC Inputs

| Signal | Pin | Notes |
|--------|-----|-------|
| Battery voltage | 8 | ADC1 channel; quadratic calibration polynomial |
| Current sense | 3 | ADC1 channel; `CURR_SCALE` 116.5 A/V |

> ADC2 is not used — avoid it on ESP32-S3 when WiFi is active.

## Libraries

| Library | Purpose |
|---------|---------|
| Adafruit MPU6050 | IMU driver |
| Adafruit HMC5883 Unified | Magnetometer driver |
| Adafruit BMP280 | Barometer driver |
| Adafruit Unified Sensor | Abstraction layer |
| TinyGPS++ | NMEA sentence parser |
| ESP32 WebServer | WiFi telemetry server |
| ESP32 RMT (driver/rmt.h) | DSHOT300 bit-banging |
