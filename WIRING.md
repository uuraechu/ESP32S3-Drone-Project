# Wiring

All pin references are GPIO numbers on the ESP32-S3-WROOM-1.

## Pin Summary

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 1 | ESC1 (M1 Front-Right CCW) | Output | DSHOT300 via RMT ch0; 2kΩ pull-up to 3V3 |
| 2 | ESC2 (M2 Rear-Right CW) | Output | DSHOT300 via RMT ch1; 2kΩ pull-up to 3V3 |
| 3 | Current sense ADC | Input | ADC1 — do not use ADC2 |
| 4 | ESC3 (M3 Front-Left CW) | Output | DSHOT300 via RMT ch2; 2kΩ pull-up to 3V3 |
| 5 | ESC4 (M4 Rear-Left CCW) | Output | DSHOT300 via RMT ch3; 2kΩ pull-up to 3V3 |
| 6 | I2C SCL | Bidirectional | Shared bus: MPU6050, HMC5883L, BMP280 |
| 7 | I2C SDA | Bidirectional | Shared bus: MPU6050, HMC5883L, BMP280 |
| 8 | Battery voltage ADC | Input | ADC1; voltage divider → quadratic polynomial |
| 9 | GPS TX (to ESP32 RX) | Input | UART2 |
| 10 | GPS RX (from ESP32 TX) | Output | UART2 |
| 12 | iBUS RX | Input | UART1; TX not connected |
| 13 | LED Armed | Output | Active high |
| 14 | LED Low Battery | Output | Active high |
| 15 | LED Altitude Hold | Output | Active high |
| 16 | Buzzer | Output | LEDC channel 0, 10-bit resolution |
| 17 | LED Failsafe | Output | Active high |

## DSHOT300 — ESC Signal Lines

Each motor signal line requires a **2kΩ pull-up resistor to 3.3V**.

```
ESP32-S3 GPIO 1 ──[2kΩ]── 3V3
               └────────── ESC1 signal (M1 Front-Right)

ESP32-S3 GPIO 2 ──[2kΩ]── 3V3
               └────────── ESC2 signal (M2 Rear-Right)

ESP32-S3 GPIO 4 ──[2kΩ]── 3V3
               └────────── ESC3 signal (M3 Front-Left)

ESP32-S3 GPIO 5 ──[2kΩ]── 3V3
               └────────── ESC4 signal (M4 Rear-Left)
```

DSHOT300 bit timing (80MHz RMT clock, 12.5ns/tick):

| Bit | High | Low |
|-----|------|-----|
| 1 | 200 ticks (2500ns) | 66 ticks (833ns) |
| 0 | 100 ticks (1250ns) | 133 ticks (1666ns) |
| Frame pause | — | 2667 ticks (~33µs) |

Throttle range: 48 (min armed) to 1800 (ceiling). Hardware max is 2047.

## I2C Bus — Sensors

All three sensors share the same I2C bus. Use 4.7kΩ pull-ups on SCL and SDA to 3.3V (or rely on breakout board pull-ups if present).

```
ESP32-S3 GPIO 6 (SCL) ──── MPU6050 SCL
                       ──── HMC5883L SCL
                       ──── BMP280 SCL

ESP32-S3 GPIO 7 (SDA) ──── MPU6050 SDA
                       ──── HMC5883L SDA
                       ──── BMP280 SDA
```

| Device | I2C Address | Notes |
|--------|-------------|-------|
| MPU6050 | 0x68 | AD0 pin low (default) |
| HMC5883L | 0x1E | Fixed address |
| BMP280 | 0x76 | SDO low (default); 0x77 if SDO high |

## iBUS Receiver — UART1

Only the RX line is connected. TX is unused.

```
Receiver iBUS out ──── ESP32-S3 GPIO 12 (UART1 RX)
```

Baud: 115200. Packet: 32 bytes, 14 channels.

## GPS — UART2

```
NEO-6M TX ──── ESP32-S3 GPIO 10 (UART2 RX)
NEO-6M RX ──── ESP32-S3 GPIO 9  (UART2 TX)
```

Baud: 9600. Recommended: configure module to 5Hz update rate.

## Battery ADC

A resistor voltage divider scales the pack voltage down to the ESP32-S3 ADC input range (0–3.3V).

```
Battery+ ──[R1]──┬──[R2]── GND
                 └──────── GPIO 8 (ADC1)
```

The firmware applies a quadratic correction polynomial:

```
vbat = VDIV_A × adc² + VDIV_B × adc + VDIV_C
```

Current calibration (Cookeville TN bench):

| Constant | Value |
|----------|-------|
| VDIV_A | −0.79516 |
| VDIV_B | 8.87512 |
| VDIV_C | −2.36039 |

Recalibrate by measuring actual voltage at 6–8 points across the full battery range and fitting a polynomial.

## Current Sense — UART ADC

```
Current sensor output ──── GPIO 3 (ADC1)
```

Scale: `CURR_SCALE` = 116.5 A/V. Zero offset: `CURR_OFFSET_A` = 0.0 (adjust if idle reads non-zero).

> Do not use any ADC2 pins while WiFi is active on ESP32-S3.

## Indicators

All LEDs: active high (LED anode → GPIO via current-limiting resistor → LED cathode → GND).

```
GPIO 13 ──[330Ω]── LED (Armed)       ── GND
GPIO 14 ──[330Ω]── LED (Low Battery) ── GND
GPIO 15 ──[330Ω]── LED (Alt Hold)    ── GND
GPIO 17 ──[330Ω]── LED (Failsafe)    ── GND
GPIO 16 ──────────── Buzzer (+)      ── GND
```

Buzzer is driven by LEDC channel 0 at a configurable frequency.

## Body Frame Convention

```
          Forward (+Y)
              ↑
  M3 (CW) ───┼─── M1 (CCW)
  Front-Left  │   Front-Right
              │
  M4 (CCW) ──┼─── M2 (CW)
  Rear-Left   │   Rear-Right
              │
```

- X = Right
- Y = Forward
- Z = Up

IMU and magnetometer are mounted in the same orientation — no software remapping needed.

## Planned SPI Wiring (Post Sensor Upgrade)

When ICM-42688-P, MMC5983MA, and BMP388 are installed, they will share the SPI bus. Assign a dedicated CS pin per device.

| Device | SPI Speed | Suggested CS |
|--------|-----------|--------------|
| ICM-42688-P | 24MHz | TBD |
| MMC5983MA | Standard | TBD |
| BMP388 | 4MHz | TBD |

I2C bus (GPIO 6/7) will be unused after the sensor upgrade.
