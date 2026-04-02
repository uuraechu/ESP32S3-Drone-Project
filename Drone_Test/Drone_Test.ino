#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <driver/rmt.h>

// ────────────────────────────────────────────────────────────────
// PIN DEFINITIONS
// ────────────────────────────────────────────────────────────────
#define I2C_SCL 6
#define I2C_SDA 7
#define IBUS_RX_PIN 12
#define IBUS_TX_PIN -1
#define GPS_RX_PIN 10
#define GPS_TX_PIN 9
#define GPS_BAUD 9600
#define IBUS_BAUD 115200

#define ESC1_PIN 1 // Front-right CCW (ESC pad 1)
#define ESC2_PIN 2 // Rear-right CW (ESC pad 2)
#define ESC3_PIN 4 // Front-left CW (ESC pad 3)
#define ESC4_PIN 5 // Rear-left CCW (ESC pad 4)

#define BAT_ADC_PIN 8
#define CURR_ADC_PIN 3
#define BUZZER_PIN 16
#define LED_ARMED 13
#define LED_LOWBAT 14
#define LED_ALTHOLD 15
#define LED_FAILSAFE 17

#define BUZZER_LEDC_CHANNEL 0
#define BUZZER_LEDC_RES 10

// ────────────────────────────────────────────────────────────────
// VOLTAGE / CURRENT CONSTANTS — match flight firmware exactly
// ────────────────────────────────────────────────────────────────
#define VDIV_A -0.79516f // quadratic coefficient — fill after calibration
#define VDIV_B 8.87512f // linear coefficient — fill after calibration
#define VDIV_C -2.36039f // offset — fill after calibration
#define CURR_SCALE 116.5f
#define CURR_OFFSET_A 0.0f
#define CELL_DETECT_THRESHOLD 13.5f // above = 4S, below = 3S

// ────────────────────────────────────────────────────────────────
// DSHOT300 CONSTANTS
// ────────────────────────────────────────────────────────────────
#define DSHOT_RMT_CLK_DIV 1
#define DSHOT_T1H_TICKS 200
#define DSHOT_T1L_TICKS 66
#define DSHOT_T0H_TICKS 100
#define DSHOT_T0L_TICKS 133
#define DSHOT_PAUSE_TICKS 2667
#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047
#define DSHOT_CMD_STOP 0
#define DSHOT_10PCT 248

#define RMT_CH_M1 RMT_CHANNEL_0 // Front-right CCW
#define RMT_CH_M2 RMT_CHANNEL_1 // Rear-right CW
#define RMT_CH_M3 RMT_CHANNEL_2 // Front-left CW
#define RMT_CH_M4 RMT_CHANNEL_3 // Rear-left CCW

// ────────────────────────────────────────────────────────────────
// iBUS CONSTANTS
// ────────────────────────────────────────────────────────────────
#define IBUS_PACKET_LEN 32
#define IBUS_HEADER1 0x20
#define IBUS_HEADER2 0x40
#define IBUS_NUM_CHANNELS 14

// ────────────────────────────────────────────────────────────────
// OBJECTS
// ────────────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
TinyGPSPlus gps;
HardwareSerial iBusSerial(1);
HardwareSerial gpsSerial(2);

static rmt_item32_t dshotItems[4][17];

// ────────────────────────────────────────────────────────────────
// TEST STATE
// ────────────────────────────────────────────────────────────────
enum class TestStage {
  SENSORS,
  VOLTAGE_CURRENT,
  BUZZER,
  LEDS,
  RECEIVER,
  ESC_CONFIRM,
  ESC_ARMING,
  ESC_RUNNING,
  DONE
};

TestStage stage = TestStage::SENSORS;
unsigned long stageStart = 0;
unsigned long lastPrint = 0;
unsigned long lastDshot = 0;
int batteryCells = 3; // set during setup

// ────────────────────────────────────────────────────────────────
// BUZZER HELPER
// ────────────────────────────────────────────────────────────────

void buzzerTone(int freq, int duration_ms) {
  ledcSetup(BUZZER_LEDC_CHANNEL, freq, BUZZER_LEDC_RES);
  ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
  ledcWrite(BUZZER_LEDC_CHANNEL, 512);  // 50% duty
  delay(duration_ms);
  ledcWrite(BUZZER_LEDC_CHANNEL, 0);
  ledcDetachPin(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
}

// ────────────────────────────────────────────────────────────────
// ADC HELPERS
// ────────────────────────────────────────────────────────────────
float adcToVbat(float adcRaw) {
  float v = adcRaw * (3.3f / 4095.0f);  // convert to ADC input voltage first
  return VDIV_A * v * v + VDIV_B * v + VDIV_C;
}

float readVbat() {
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(BAT_ADC_PIN);
  return adcToVbat(sum / 16.0f);
}

void detectBatteryCells() {
  // Average 32 reads for stable resting voltage
  long sum = 0;
  for (int i = 0; i < 32; i++) {
    sum += analogRead(BAT_ADC_PIN);
    delay(5);
  }
  float vbat = adcToVbat(sum / 32.0f);
  batteryCells = (vbat >= CELL_DETECT_THRESHOLD) ? 4 : 3;
  Serial.printf("[BAT]  Resting voltage: %.2fV — detected %dS\n",
                vbat, batteryCells);
}

float readCurrentAmps() {
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(CURR_ADC_PIN);
  float v = (sum / 16.0f) * (3.3f / 4095.0f);
  return constrain(v * CURR_SCALE - CURR_OFFSET_A, 0.0f, 200.0f);
}

// ────────────────────────────────────────────────────────────────
// DSHOT HELPERS
// ────────────────────────────────────────────────────────────────
uint8_t dshotCRC(uint16_t value) {
  return ((value ^ (value >> 4) ^ (value >> 8)) & 0x0F);
}

void buildDshotPacket(rmt_item32_t *items, uint16_t value, bool telem) {
  uint16_t packet = (value << 1) | (telem ? 1 : 0);
  uint8_t crc = dshotCRC(packet);
  packet = (packet << 4) | crc;
  for (int i = 0; i < 16; i++) {
    bool bit = (packet >> (15 - i)) & 0x01;
    items[i].duration0 = bit ? DSHOT_T1H_TICKS : DSHOT_T0H_TICKS;
    items[i].level0 = 1;
    items[i].duration1 = bit ? DSHOT_T1L_TICKS : DSHOT_T0L_TICKS;
    items[i].level1 = 0;
  }
  items[16].duration0 = DSHOT_PAUSE_TICKS;
  items[16].level0 = 0;
  items[16].duration1 = 0;
  items[16].level1 = 0;
}

void initDshotTxChannel(rmt_channel_t ch, int pin) {
  rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX((gpio_num_t)pin, ch);
  cfg.clk_div = DSHOT_RMT_CLK_DIV;
  rmt_config(&cfg);
  rmt_driver_install(ch, 0, 0);
}

void dshotSend(rmt_channel_t ch, int chIdx, uint16_t value, bool telem) {
  if (value != 0 && value < DSHOT_THROTTLE_MIN) value = DSHOT_THROTTLE_MIN;
  if (value > DSHOT_THROTTLE_MAX) value = DSHOT_THROTTLE_MAX;
  buildDshotPacket(dshotItems[chIdx], value, telem);
  rmt_write_items(ch, dshotItems[chIdx], 17, false);
}

void dshotSendAll(uint16_t val, bool telem) {
  dshotSend(RMT_CH_M1, 0, val, telem);
  dshotSend(RMT_CH_M2, 1, val, telem);
  dshotSend(RMT_CH_M3, 2, val, telem);
  dshotSend(RMT_CH_M4, 3, val, telem);
  rmt_wait_tx_done(RMT_CH_M1, pdMS_TO_TICKS(2));
  rmt_wait_tx_done(RMT_CH_M2, pdMS_TO_TICKS(2));
  rmt_wait_tx_done(RMT_CH_M3, pdMS_TO_TICKS(2));
  rmt_wait_tx_done(RMT_CH_M4, pdMS_TO_TICKS(2));
}

// ────────────────────────────────────────────────────────────────
// iBUS PARSER
// ────────────────────────────────────────────────────────────────
bool parseIBus(uint16_t channels[14]) {
  if (iBusSerial.available() < IBUS_PACKET_LEN) return false;
  uint8_t buf[IBUS_PACKET_LEN];
  while (iBusSerial.available() >= IBUS_PACKET_LEN) {
    if (iBusSerial.peek() == IBUS_HEADER1) {
      iBusSerial.readBytes(buf, IBUS_PACKET_LEN);
      if (buf[1] != IBUS_HEADER2) continue;
      uint16_t checksum = 0xFFFF;
      for (int i = 0; i < 30; i++) checksum -= buf[i];
      if (checksum != (buf[30] | (buf[31] << 8))) continue;
      for (int i = 0; i < IBUS_NUM_CHANNELS; i++)
        channels[i] = constrain(buf[2 + i*2] | (buf[3 + i*2] << 8), 1000, 2000);
      return true;
    } else {
      iBusSerial.read();
    }
  }
  return false;
}

// ────────────────────────────────────────────────────────────────
// SETUP
// ────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n========================================");
  Serial.println("  DRONE HARDWARE TEST");
  Serial.println("========================================\n");

  // Pins
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.print("Battery... ");
  detectBatteryCells();
  pinMode(BAT_ADC_PIN, INPUT);
  pinMode(CURR_ADC_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_ARMED, OUTPUT); digitalWrite(LED_ARMED, LOW);
  pinMode(LED_LOWBAT, OUTPUT); digitalWrite(LED_LOWBAT, LOW);
  pinMode(LED_ALTHOLD, OUTPUT); digitalWrite(LED_ALTHOLD, LOW);
  pinMode(LED_FAILSAFE, OUTPUT); digitalWrite(LED_FAILSAFE, LOW);

  // Raw I2C scan — no library needed
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Serial.println("Scanning I2C...");
  for (int addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Device found at 0x%02X\n", addr);
    }
  }
  Serial.println("Scan done.");

  // ── MPU6050 ──────────────────────────────────────────────────
  Serial.print("MPU6050...  ");
  if (!mpu.begin()) {
    Serial.println("FAIL — check SDA/SCL wiring");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    Serial.println("OK");
  }

  // ── BMP280 ───────────────────────────────────────────────────
  Serial.print("BMP280...   ");
  if (!bmp.begin(0x76)) {
    Serial.println("FAIL — check address (0x76) and wiring");
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_63);
    Serial.println("OK");
  }

  // ── HMC5883L ─────────────────────────────────────────────────
  Serial.print("HMC5883L... ");
  if (!mag.begin()) {
    Serial.println("FAIL — check address (0x1E) and wiring");
  } else {
    Serial.println("OK");
  }

  // ── GPS ──────────────────────────────────────────────────────
  Serial.print("GPS UART2... ");
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("OK");

  // ── iBUS ─────────────────────────────────────────────────────
  Serial.print("iBUS UART1... ");
  iBusSerial.begin(IBUS_BAUD, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);
  Serial.println("OK");

  // ── DSHOT ────────────────────────────────────────────────────
  Serial.print("DSHOT RMT... ");
  initDshotTxChannel(RMT_CH_M1, ESC1_PIN);
  initDshotTxChannel(RMT_CH_M2, ESC2_PIN);
  initDshotTxChannel(RMT_CH_M3, ESC3_PIN);
  initDshotTxChannel(RMT_CH_M4, ESC4_PIN);
  Serial.println("OK");

  Serial.println("\n----------------------------------------");
  Serial.println("STAGE 1 — SENSORS (10 seconds)");
  Serial.println("  Accel Z ~9.81 m/s² flat");
  Serial.println("  Baro altitude non-zero, temp ~room temp");
  Serial.println("  Mag heading 0-360°");
  Serial.println("  GPS waiting for fix outdoors");
  Serial.println("----------------------------------------\n");

  stageStart = millis();
  stage = TestStage::SENSORS;
}

// ────────────────────────────────────────────────────────────────
// LOOP
// ────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  // Keep ESC signal alive at all times — stops disarming between stages
  if (now - lastDshot >= 3) {
    lastDshot = now;
    if (stage != TestStage::ESC_RUNNING) {
      dshotSendAll(DSHOT_CMD_STOP, false);
    }
  }

  // ── STAGE 1 — SENSORS ────────────────────────────────────────
  if (stage == TestStage::SENSORS) {
    if (now - lastPrint >= 1000) {
      lastPrint = now;

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      Serial.printf("[MPU]  Accel: X=%6.3f Y=%6.3f Z=%6.3f m/s²   "
                    "Gyro: X=%6.3f Y=%6.3f Z=%6.3f rad/s\n",
                    a.acceleration.x, a.acceleration.y, a.acceleration.z,
                    g.gyro.x, g.gyro.y, g.gyro.z);

      Serial.printf("[BARO] Alt: %.2f m   Temp: %.1f °C\n",
                    bmp.readAltitude(1013.25f), bmp.readTemperature());

      sensors_event_t magEvent;
      mag.getEvent(&magEvent);
      float heading = atan2f(-magEvent.magnetic.y, magEvent.magnetic.x) * (180.0f / M_PI);
      if (heading < 0.0f) heading += 360.0f;
      Serial.printf("[MAG]  X=%6.2f Y=%6.2f Z=%6.2f uT   Heading: %.1f°\n",
                    magEvent.magnetic.x, magEvent.magnetic.y,
                    magEvent.magnetic.z, heading);

      if (gps.location.isValid()) {
        Serial.printf("[GPS]  Lat: %.6f  Lng: %.6f  Alt: %.1f m  "
                      "Sats: %u  Speed: %.1f km/h\n",
                      gps.location.lat(), gps.location.lng(),
                      gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
                      gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0,
                      gps.speed.isValid() ? gps.speed.kmph() : 0.0f);
      } else {
        Serial.printf("[GPS]  No fix — chars: %lu  sentences: %lu  failed: %lu\n",
                      gps.charsProcessed(), gps.sentencesWithFix(),
                      gps.failedChecksum());
      }
      Serial.println();
    }

    if (now - stageStart >= 10000) {
      stage = TestStage::VOLTAGE_CURRENT;
      stageStart = now;
      Serial.println("----------------------------------------");
      Serial.println("STAGE 2 — VOLTAGE & CURRENT (5 seconds)");
      Serial.println("  Connect battery now if not already.");
      Serial.println("  Voltage should match multimeter reading.");
      Serial.println("  Cell count should match your battery.");
      Serial.println("  Current should read ~0 A at idle.");
      Serial.println("----------------------------------------\n");
    }
  }

  // ── STAGE 2 — VOLTAGE & CURRENT ──────────────────────────────
  else if (stage == TestStage::VOLTAGE_CURRENT) {
    if (now - lastPrint >= 500) {
      lastPrint = now;
      float vbat = readVbat();
      float amps = readCurrentAmps();
      float watts = vbat * amps;
      Serial.printf("[BAT]  Voltage: %.2f V (%dS)   Current: %.1f A   Power: %.1f W\n",
              vbat, batteryCells, amps, watts);
    }

    if (now - stageStart >= 5000) {
      stage = TestStage::BUZZER;
      stageStart = now;
      Serial.println("\n----------------------------------------");
      Serial.println("STAGE 3 — BUZZER");
      Serial.println("  Listen for 4 beeps.");
      Serial.println("----------------------------------------\n");
    }
  }

  // ── STAGE 3 — BUZZER ─────────────────────────────────────────
  else if (stage == TestStage::BUZZER) {
    // Four beeps at different frequencies to confirm tone() works
    static bool buzzerDone = false;
    if (!buzzerDone) {
      buzzerDone = true;
      int freqs[] = {800, 1200, 1600, 2000};
      for (int i = 0; i < 4; i++) {
        buzzerTone(freqs[i], 200);
        delay(150);  // pause between beeps
      }
      Serial.println("[BUZ]  4 beeps done — did you hear them?");
      Serial.println("       If silent: check passive buzzer polarity and wiring to IO16.");
    }

    if (now - stageStart >= 3000) {
      buzzerDone = false;
      stage = TestStage::LEDS;
      stageStart = now;
      Serial.println("\n----------------------------------------");
      Serial.println("STAGE 4 — LEDs (4 seconds)");
      Serial.println("  Each LED lights in sequence.");
      Serial.println("  ARMED → LOWBAT → ALTHOLD → FAILSAFE");
      Serial.println("----------------------------------------\n");
    }
  }

  // ── STAGE 4 — LEDs ───────────────────────────────────────────
  else if (stage == TestStage::LEDS) {
    unsigned long elapsed = now - stageStart;

    // Each LED on for 1 second, then all on briefly, then all off
    digitalWrite(LED_ARMED, elapsed < 1000 ? HIGH : LOW);
    digitalWrite(LED_LOWBAT, (elapsed >= 1000 && elapsed < 2000) ? HIGH : LOW);
    digitalWrite(LED_ALTHOLD, (elapsed >= 2000 && elapsed < 3000) ? HIGH : LOW);
    digitalWrite(LED_FAILSAFE, (elapsed >= 3000 && elapsed < 4000) ? HIGH : LOW);

    if (now - lastPrint >= 1000) {
      lastPrint = now;
      const char* names[] = {"ARMED (IO13)", "LOWBAT (IO14)",
                              "ALTHOLD (IO15)", "FAILSAFE (IO17)"};
      int idx = constrain((int)(elapsed / 1000), 0, 3);
      Serial.printf("[LED]  %s — on\n", names[idx]);
    }

    if (elapsed >= 4000) {
      digitalWrite(LED_ARMED, LOW);
      digitalWrite(LED_LOWBAT, LOW);
      digitalWrite(LED_ALTHOLD, LOW);
      digitalWrite(LED_FAILSAFE, LOW);
      stage = TestStage::RECEIVER;
      stageStart = now;
      Serial.println("\n----------------------------------------");
      Serial.println("STAGE 5 — RECEIVER (10 seconds)");
      Serial.println("  Turn on transmitter and move sticks.");
      Serial.println("  CH1=Roll CH2=Pitch CH3=Throttle CH4=Yaw");
      Serial.println("  CH5=Arm CH6=AltHold CH7=BatOverride");
      Serial.println("  All channels should range 1000-2000.");
      Serial.println("----------------------------------------\n");
    }
  }

  // ── STAGE 5 — RECEIVER ───────────────────────────────────────
  else if (stage == TestStage::RECEIVER) {
    uint16_t channels[14];
    if (parseIBus(channels)) {
      if (now - lastPrint >= 200) {
        lastPrint = now;
        Serial.printf("[IBUS] CH1:%4u  CH2:%4u  CH3:%4u  CH4:%4u  "
                      "CH5:%4u  CH6:%4u  CH7:%4u\n",
                      channels[0], channels[1], channels[2],
                      channels[3], channels[4], channels[5], channels[6]);
      }               // ← closes lastPrint if
    } else {          // ← now correctly attaches to parseIBus if
      if (now - lastPrint >= 2000) {
        lastPrint = now;
        Serial.println("[IBUS] No packets — check iBUS pin and transmitter binding");
      }
    }

    if (now - stageStart >= 10000) {
      stage = TestStage::ESC_CONFIRM;
      stageStart = now;
      Serial.println("\n----------------------------------------");
      Serial.println("STAGE 6 — ESC / MOTORS");
      Serial.println("  !! REMOVE ALL PROPELLERS NOW !!");
      Serial.println("  Motors will spin at 10% throttle.");
      Serial.println("  Motors spin individually: FR → RR → FL → RL");
      Serial.println("  then all four together.");
      Serial.println("  Type 'go' in Serial Monitor to continue.");
      Serial.println("----------------------------------------\n");
    }
  }

  // ── STAGE 6 — ESC CONFIRM ────────────────────────────────────
  else if (stage == TestStage::ESC_CONFIRM) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.equalsIgnoreCase("go")) {
        stage = TestStage::ESC_ARMING;
        stageStart = now;
        Serial.println("Arming ESCs — sending stop frames for 2 seconds...");
      } else {
        Serial.println("Type 'go' to proceed.");
      }
    }
  }

  // ── STAGE 7 — ESC ARMING ─────────────────────────────────────
  else if (stage == TestStage::ESC_ARMING) {
    if (now - stageStart >= 2000) {
      stage = TestStage::ESC_RUNNING;
      stageStart = now;
      Serial.println("Armed. Spinning at 10% for 5 seconds...\n");
    }
  }

  // ── STAGE 8 — ESC RUNNING ────────────────────────────────────
  else if (stage == TestStage::ESC_RUNNING) {
    static int motorPhase = -1;  // -1 = not started
    static unsigned long phaseStart = 0;

    // Motor sequence: 1=FR, 2=RR, 3=FL, 4=RL, 5=all four
    struct MotorDef {
      rmt_channel_t ch;
      int idx;
      const char* label;
    };
    static const MotorDef motors[] = {
      { RMT_CH_M1, 0, "M1 — Front-right CCW (ESC pad 1)" },
      { RMT_CH_M2, 1, "M2 — Rear-right CW   (ESC pad 2)" },
      { RMT_CH_M3, 2, "M3 — Front-left CW   (ESC pad 3)" },
      { RMT_CH_M4, 3, "M4 — Rear-left CCW   (ESC pad 4)" },
    };

    if (motorPhase == -1) {
      motorPhase = 0;
      phaseStart = now;
      Serial.printf("\n[ESC]  Spinning %s for 2 seconds...\n", motors[0].label);
    }

    // Keep currently selected motor spinning
    if (motorPhase < 4) {
      uint16_t vals[4] = {DSHOT_CMD_STOP, DSHOT_CMD_STOP, DSHOT_CMD_STOP, DSHOT_CMD_STOP};
      vals[motors[motorPhase].idx] = DSHOT_10PCT;
      dshotSend(RMT_CH_M1, 0, vals[0], false);
      dshotSend(RMT_CH_M2, 1, vals[1], false);
      dshotSend(RMT_CH_M3, 2, vals[2], false);
      dshotSend(RMT_CH_M4, 3, vals[3], false);
      rmt_wait_tx_done(RMT_CH_M1, pdMS_TO_TICKS(2));
      rmt_wait_tx_done(RMT_CH_M2, pdMS_TO_TICKS(2));
      rmt_wait_tx_done(RMT_CH_M3, pdMS_TO_TICKS(2));
      rmt_wait_tx_done(RMT_CH_M4, pdMS_TO_TICKS(2));

      // Print current during spin
      if (now - lastPrint >= 500) {
        lastPrint = now;
        float vbat = readVbat();
        float amps = readCurrentAmps();
        Serial.printf("[BAT]  %.2f V   %.1f A   %.1f W\n",
                      vbat, amps, vbat * amps);
      }

      // Advance to next motor after 2 seconds
      if (now - phaseStart >= 2000) {
        dshotSendAll(DSHOT_CMD_STOP, false);
        delay(200); // brief gap so motor stops before next starts
        motorPhase++;
        phaseStart = now;
        if (motorPhase < 4) {
          Serial.printf("\n[ESC]  Spinning %s for 2 seconds...\n",
                        motors[motorPhase].label);
        } else {
          Serial.println("\n[ESC]  All four together for 3 seconds...");
        }
      }
    } else {
      // All four together
      dshotSendAll(DSHOT_10PCT, false);

      if (now - lastPrint >= 500) {
        lastPrint = now;
        float vbat = readVbat();
        float amps = readCurrentAmps();
        Serial.printf("[BAT]  %.2f V   %.1f A   %.1f W\n",
                      vbat, amps, vbat * amps);
      }

      if (now - phaseStart >= 3000) {
        dshotSendAll(DSHOT_CMD_STOP, false);
        digitalWrite(LED_ARMED, LOW);
        motorPhase = -1;  // reset for safety
        stage = TestStage::DONE;
        Serial.println("\n========================================");
        Serial.println("  TEST COMPLETE");
        Serial.println("  Each motor should have spun in order:");
        Serial.println("  FR → RR → FL → RL → all four");
        Serial.println("  Current during all-four should be > 2A.");
        Serial.println("========================================");
      }
    }
  }
}
