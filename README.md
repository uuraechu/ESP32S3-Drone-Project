# ESP32-S3 Quadcopter MK2

Custom quadcopter flight controller built on the ESP32-S3-WROOM-1. Runs Arduino-style C++ firmware with a 200Hz flight loop, DSHOT300 ESC protocol, iBUS receiver, WiFi telemetry dashboard, altitude hold, and GPS.

## Quick Start

### Prerequisites

Install the following Arduino libraries via Library Manager:

- `Adafruit MPU6050`
- `Adafruit HMC5883 Unified`
- `Adafruit BMP280`
- `Adafruit Unified Sensor`
- `TinyGPS++`

Board: **ESP32S3 Dev Module** (Arduino-ESP32 core 2.x or later)

### Flash

1. Open `Drone_Control_MK2/Drone_Control_MK2.ino` in Arduino IDE.
2. Select board: **ESP32S3 Dev Module**.
3. Set partition scheme to **Huge APP** (for WiFi + flight stack).
4. Flash.

### WiFi Telemetry

Connect to AP `QuadTelemetry` (password: `flysafe123`), then open `http://192.168.4.1` in a browser.

> Change the SSID and password in the firmware before any public use.

## Project Structure

```
ESP32S3-Drone-Project/
├── Drone_Control_MK2/
│   └── Drone_Control_MK2.ino   # Main firmware (single-file)
├── CLAUDE.md                    # Project context and architecture notes
├── README.md
├── CHANGELOG.md
├── COMPONENTS.md
├── TROUBLESHOOTING.md
└── WIRING.md
```

## Architecture Overview

| Core | Tasks |
|------|-------|
| Core 1 | Flight loop @ 200Hz — sensors, Kalman fusion, PIDs, motor mixing |
| Core 0 | iBUSTask (receiver parse), GPSTask (20Hz), WiFiTask (100Hz) |

**Sensor fusion:** 1D Kalman filter per attitude axis. Altitude uses a complementary filter (accelerometer vertical + barometer).

**Motor protocol:** DSHOT300 via ESP32 RMT peripheral — one RMT channel per motor.

## PID Tuning Order

1. Roll/Pitch Kp — hover in calm air, step ±0.3
2. Yaw Kp — match roll/pitch Kp, step +0.3
3. Roll/Pitch Ki — only if persistent drift (check trim first)
4. Roll/Pitch Kd — only after Kp/Ki stable
5. Altitude hold PID — only after manual flight is stable
6. Cross-axis coupling coefficients — last, from 0

See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for symptom-to-fix quick reference.

## Channel Map

| CH | Stick / Switch | Low (<1200) | High (>1700) |
|----|----------------|-------------|--------------|
| 1 | Roll | Left | Right |
| 2 | Pitch | Nose down | Nose up |
| 3 | Throttle | Zero | Full |
| 4 | Yaw | Left | Right |
| 5 | ARM (SWA) | Disarmed | Armed |
| 6 | Alt Hold (SWB) | Manual | Hold active |
| 7 | Bat Override (SWC) | Cutoff active | Override ON |
| 8 | Descent (SWD 3-pos) | Off | 0.3–0.6 m/s |

## Current Status

- Hardware assembled and bench-tested
- First stable hover flight pending
- PITCH_TRIM / ROLL_TRIM to be set at first hover
- ICM-42688-P IMU ordered (replaces MPU6050 — enables 1kHz loop)

See [CHANGELOG.md](CHANGELOG.md) for version history and [COMPONENTS.md](COMPONENTS.md) for full hardware BOM.
