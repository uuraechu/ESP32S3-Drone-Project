# Troubleshooting

## Flight Behavior

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Oscillation at hover | Kp too high | Reduce Kp −0.3 |
| Sluggish wind response | Kp too low | Increase Kp +0.3 |
| Slow drift at hover | Mechanical offset or Ki too low | Check PITCH_TRIM / ROLL_TRIM first; if still drifting, add Ki +0.1 |
| Slow oscillation that builds over time | Ki too high | Reduce Ki −0.05 |
| High-frequency motor buzz | Kd derivative amplifying noise | Reduce D filter Hz −5 |
| Altitude bounces during hold | ALT_INNOV_WEIGHT too high | Reduce `ALT_INNOV_WEIGHT` to 0.04 |
| Altitude hold won't engage | Below minimum height or throttle | Must be above 5m AND throttle >10% |
| Throttle snap when engaging/disengaging alt hold | Ramp transition not working | Check `altHoldEngaging` / `altHoldDisengaging` state flags |
| Yaw inputs cause roll/pitch coupling | Gyroscopic precession | Tune `PROP_GYRO_COEFF` from 0 in 0.001 steps |
| Altitude dips during fast yaw spins | Yaw-to-throttle coupling | Tune `YAW_THROTTLE_COEFF` from 0 in 0.001 steps |

## Arming / Safety

| Symptom | Fix |
|---------|-----|
| Drone won't arm | CH5 (SWA) must be in high position (>1700). Verify receiver is bound and iBUS packets are being received. |
| Drone disarms immediately after arming | Check failsafe — receiver signal may be dropping. Verify `FAILSAFE_TIMEOUT_MS` (500ms). |
| Failsafe triggered unexpectedly | Signal lost for >500ms and confirmed over 8 consecutive loops. Check receiver range and binding. |
| Battery cutoff preventing motor spin | CH7 low = cutoff active. Flip SWC to override OR charge battery above 3.2V/cell recovery threshold. |

## Sensors

| Symptom | Fix |
|---------|-----|
| Roll/pitch non-zero at rest on flat surface | Re-run IMU calibration; update `GYRO_OFFSET_*` and `ACCEL_OFFSET_*` constants |
| Yaw drifts or jumps | Magnetometer interference; move metal/ESCs further from sensor. Re-run hard-iron calibration and update `MAG_OFFSET_*` |
| Altitude reads wrong or drifts at rest | BMP280 barometer offset; let it warm up. `baroOffset` is zeroed at boot — drone must be stationary at startup |
| GPS not getting fix | Clear sky view required. `gpsSatellites` < 4 means no fix. `gpsAge` shows milliseconds since last valid sentence |
| IMU readings are wrong after remounting | Recalibrate — offsets are position-sensitive. Re-run calibration tool and update constants |

## WiFi / Telemetry

| Symptom | Fix |
|---------|-----|
| Can't connect to `QuadTelemetry` | Wait ~5s after boot for AP to start. Verify SSID/password in firmware. |
| Dashboard shows stale data | `/data` polls at 10Hz when armed, 1Hz when disarmed. Check browser console for fetch errors. |
| PID fields greyed out | Drone is armed. Disarm to edit PID values. |
| `/pid/set` returns error | Drone is armed or JSON body malformed. Disarm first. |
| Dashboard `dt` reads much higher than 5ms | Flight loop is overrunning. Check for I2C hangs or task starvation on Core 1. |

## Compilation / Upload

| Symptom | Fix |
|---------|-----|
| Sketch too large | Set partition scheme to **Huge APP** in Arduino IDE board settings |
| `originalAltKi` undeclared in loop() | Must be declared immediately after the `pidAlt` constructor — C++ global init is top-to-bottom |
| RMT / DSHOT compile error | Ensure `driver/rmt.h` is included and you are using Arduino-ESP32 core 2.x |
| MPU6050 not found at startup | Check I2C wiring (SCL=6, SDA=7). I2C address is 0x68. Confirm 3.3V power to sensor. |
| BMP280 not found at startup | Same I2C bus. Default address is 0x76. If 0x77, SDO pin is high — check breakout board. |

## Battery / Power

| Symptom | Fix |
|---------|-----|
| Voltage reads low/incorrect at rest | Recalibrate ADC polynomial (`VDIV_A`, `VDIV_B`, `VDIV_C`) by measuring actual voltage at 6–8 points |
| Cell count detected wrong at boot | Check actual pack voltage. Threshold is 13.5V: above = 4S, below = 3S |
| Current reads non-zero at idle | Adjust `CURR_OFFSET_A` to zero out the idle reading |
| Rapid 600Hz buzzer while flying | Battery override (CH7) is active with low battery — land immediately |

## Hardware Notes

- **DSHOT signal lines** require a 2kΩ pull-up resistor to 3.3V on each ESC signal pin.
- **ADC2** is not used. Do not route any analog signals to ADC2 pins — it conflicts with WiFi on ESP32-S3.
- **I2C bus** is shared between MPU6050, HMC5883L, and BMP280. Address conflicts will prevent one or more sensors from initialising.
- **After frame rebuild**: update `offsetZ` in `readSensors()`, re-run IMU calibration, re-tune PITCH_TRIM / ROLL_TRIM.
- **After sensor swap** (ICM-42688 / MMC5983MA / BMP388): re-run full calibration and update all offset constants.
