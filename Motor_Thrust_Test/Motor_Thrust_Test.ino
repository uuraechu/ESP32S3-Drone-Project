#include <driver/rmt.h>

// ────────────────────────────────────────────────────────────────
// PIN DEFINITIONS
// ────────────────────────────────────────────────────────────────
#define ESC1_PIN 1 // Front-right CCW (ESC pad 1)
#define ESC2_PIN 2
#define ESC3_PIN 4
#define ESC4_PIN 5
#define IBUS_RX_PIN 12
#define IBUS_TX_PIN -1
#define BUZZER_PIN 16

// ────────────────────────────────────────────────────────────────
// DSHOT300 CONSTANTS
// ────────────────────────────────────────────────────────────────
#define DSHOT_RMT_CLK_DIV 1
#define DSHOT_T1H_TICKS 200
#define DSHOT_T1L_TICKS 66
#define DSHOT_T0H_TICKS 100
#define DSHOT_T0L_TICKS 133
#define DSHOT_PAUSE_TICKS 2667
#define DSHOT_MIN 48
#define DSHOT_MAX 2047
#define DSHOT_CMD_STOP 0

#define RMT_CH_M1 RMT_CHANNEL_0
#define RMT_CH_M2 RMT_CHANNEL_1
#define RMT_CH_M3 RMT_CHANNEL_2
#define RMT_CH_M4 RMT_CHANNEL_3

// ────────────────────────────────────────────────────────────────
// iBUS CONSTANTS
// ────────────────────────────────────────────────────────────────
#define IBUS_BAUD 115200
#define IBUS_PACKET_LEN 32
#define IBUS_HEADER1 0x20
#define IBUS_HEADER2 0x40
#define IBUS_NUM_CHANNELS 14

#define CH_THROTTLE 2 // CH3 — index 2
#define CH_ARM 4 // CH5 — index 4

// ────────────────────────────────────────────────────────────────
// BUZZER
// ────────────────────────────────────────────────────────────────
#define BUZZER_LEDC_CHANNEL 0
#define BUZZER_LEDC_RES 10

struct BuzzerState {
  bool active = false;
  int beepsRemaining = 0;
  int frequency = 0;
  int duration_ms = 0;
  int pause_ms = 150;
  bool isTone = false;
  unsigned long lastToggle = 0;
  bool continuous = false;
};

BuzzerState buzzer;

void buzzerAlert(int count, int duration_ms, int freq, int pause_ms) {
  ledcWrite(BUZZER_LEDC_CHANNEL, 0);
  ledcDetachPin(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
  buzzer.active = true;
  buzzer.beepsRemaining = count;
  buzzer.frequency = freq;
  buzzer.duration_ms = duration_ms;
  buzzer.pause_ms = pause_ms;
  buzzer.continuous = false;
  buzzer.isTone = false;
  buzzer.lastToggle = millis();
}

void buzzerStop() {
  ledcWrite(BUZZER_LEDC_CHANNEL, 0);
  ledcDetachPin(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
  buzzer.active = false;
  buzzer.isTone = false;
  buzzer.beepsRemaining = 0;
  buzzer.continuous = false;
}

void updateBuzzer() {
  if (!buzzer.active) return;
  unsigned long now = millis();
  unsigned long elapsed = now - buzzer.lastToggle;

  if (!buzzer.isTone) {
    if (elapsed >= (unsigned long)buzzer.pause_ms) {
      ledcSetup(BUZZER_LEDC_CHANNEL, buzzer.frequency, BUZZER_LEDC_RES);
      ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
      ledcWrite(BUZZER_LEDC_CHANNEL, 512);
      buzzer.isTone = true;
      buzzer.lastToggle = now;
    }
  } else {
    if (elapsed >= (unsigned long)buzzer.duration_ms) {
      ledcWrite(BUZZER_LEDC_CHANNEL, 0);
      ledcDetachPin(BUZZER_PIN);
      digitalWrite(BUZZER_PIN, LOW);
      buzzer.isTone = false;
      buzzer.lastToggle = now;
      if (!buzzer.continuous && buzzer.beepsRemaining > 0) {
        buzzer.beepsRemaining--;
        if (buzzer.beepsRemaining <= 0) buzzer.active = false;
      }
    }
  }
}

// ────────────────────────────────────────────────────────────────
// GLOBALS
// ────────────────────────────────────────────────────────────────
#define SIGNAL_TIMEOUT_MS 500

HardwareSerial iBusSerial(1);
static rmt_item32_t dshotItems1[17];
static rmt_item32_t dshotItems2[17];
static rmt_item32_t dshotItems3[17];
static rmt_item32_t dshotItems4[17];

bool armed = false;
unsigned long lastDshot = 0;
unsigned long lastPrint = 0;
unsigned long lastValidSignal = 0;
uint16_t dshotVal = DSHOT_CMD_STOP;

// ────────────────────────────────────────────────────────────────
// DSHOT
// ────────────────────────────────────────────────────────────────
uint8_t dshotCRC(uint16_t value) {
  return ((value ^ (value >> 4) ^ (value >> 8)) & 0x0F);
}

void buildDshotPacket(rmt_item32_t *items, uint16_t value) {
  uint16_t packet = (value << 1); // telemetry bit = 0
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

// Replace dshotSend() and initDshot() with:

void dshotSendRaw(rmt_channel_t ch, rmt_item32_t *items, uint16_t value) {
  if (value != 0) {
    if (value < DSHOT_MIN) value = DSHOT_MIN;
    if (value > DSHOT_MAX) value = DSHOT_MAX;
  }
  buildDshotPacket(items, value);
  rmt_write_items(ch, items, 17, false);
}

void dshotSendAll(uint16_t val) {
  dshotSendRaw(RMT_CH_M1, dshotItems1, val);
  dshotSendRaw(RMT_CH_M2, dshotItems2, val);
  dshotSendRaw(RMT_CH_M3, dshotItems3, val);
  dshotSendRaw(RMT_CH_M4, dshotItems4, val);
  rmt_wait_tx_done(RMT_CH_M1, pdMS_TO_TICKS(2));
  rmt_wait_tx_done(RMT_CH_M2, pdMS_TO_TICKS(2));
  rmt_wait_tx_done(RMT_CH_M3, pdMS_TO_TICKS(2));
  rmt_wait_tx_done(RMT_CH_M4, pdMS_TO_TICKS(2));
}

void initDshot() {
  rmt_config_t cfg1 = RMT_DEFAULT_CONFIG_TX((gpio_num_t)ESC1_PIN, RMT_CH_M1);
  cfg1.clk_div = DSHOT_RMT_CLK_DIV; rmt_config(&cfg1);
  rmt_driver_install(RMT_CH_M1, 0, 0);

  rmt_config_t cfg2 = RMT_DEFAULT_CONFIG_TX((gpio_num_t)ESC2_PIN, RMT_CH_M2);
  cfg2.clk_div = DSHOT_RMT_CLK_DIV; rmt_config(&cfg2);
  rmt_driver_install(RMT_CH_M2, 0, 0);

  rmt_config_t cfg3 = RMT_DEFAULT_CONFIG_TX((gpio_num_t)ESC3_PIN, RMT_CH_M3);
  cfg3.clk_div = DSHOT_RMT_CLK_DIV; rmt_config(&cfg3);
  rmt_driver_install(RMT_CH_M3, 0, 0);

  rmt_config_t cfg4 = RMT_DEFAULT_CONFIG_TX((gpio_num_t)ESC4_PIN, RMT_CH_M4);
  cfg4.clk_div = DSHOT_RMT_CLK_DIV; rmt_config(&cfg4);
  rmt_driver_install(RMT_CH_M4, 0, 0);

  Serial.print("Arming ESC...");
  unsigned long t = millis();
  while (millis() - t < 1000) {
    dshotSendAll(DSHOT_CMD_STOP);
    delay(10);
  }
  Serial.println(" done.");
  buzzerAlert(1, 200, 1800, 0);
}

// ────────────────────────────────────────────────────────────────
// iBUS
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

  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZER_LEDC_CHANNEL, 1000, BUZZER_LEDC_RES);
  ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
  ledcWrite(BUZZER_LEDC_CHANNEL, 0); // silent
  digitalWrite(BUZZER_PIN, LOW);

  iBusSerial.begin(IBUS_BAUD, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);

  Serial.println("\n========================================");
  Serial.println("  THRUST TEST — ALL MOTORS");
  Serial.println("  !! REMOVE ALL PROPELLERS !!");
  Serial.println("  CH3 = Throttle  CH5 = Arm switch");
  Serial.println("  Arm switch HIGH to arm, LOW to stop.");
  Serial.println("========================================\n");

  initDshot();
  lastValidSignal = millis();
}

// ────────────────────────────────────────────────────────────────
// LOOP
// ────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  updateBuzzer();

  uint16_t channels[14];
  bool gotPacket = parseIBus(channels);
  static bool lastArmState = false;
  static unsigned long lastArmChange = 0;

  if (gotPacket) {
    lastValidSignal = now;
    bool armSwitchHigh = (channels[CH_ARM] > 1500);
    uint16_t pwmThrottle = channels[CH_THROTTLE];

    // Arm / disarm transitions
    if (armSwitchHigh != lastArmState && now - lastArmChange >= 500) {
      lastArmChange = now;
      lastArmState = armSwitchHigh;
      if (armSwitchHigh && pwmThrottle < 1100) {
        armed = true;
        Serial.println("[ARM]  ARMED — throttle active");
        buzzerAlert(2, 150, 1400, 100);
      } else if (armSwitchHigh) {
        Serial.println("[ARM]  Arm rejected — lower throttle first");
        buzzerAlert(3, 100, 800, 80);
      } else {
        armed = false;
        Serial.println("[ARM]  DISARMED — motor stopped");
        buzzerAlert(1, 400, 600, 0);
      }
    }

    // Update dshotVal from latest packet
    if (!armed) {
      dshotVal = DSHOT_CMD_STOP;
    } else if (pwmThrottle < 1050) {
      dshotVal = DSHOT_CMD_STOP;
    } else {
      dshotVal = (uint16_t)map(pwmThrottle, 1000, 2000, DSHOT_MIN, DSHOT_MAX);
      dshotVal = constrain(dshotVal, DSHOT_MIN, DSHOT_MAX);
    }

    // Print status at 5Hz
    if (now - lastPrint >= 200) {
      lastPrint = now;
      Serial.printf("[THR]  PWM:%4u  DSHOT:%4u  ARM_SW:%4u  Armed:%s\n",
                    channels[CH_THROTTLE], dshotVal,
                    channels[CH_ARM], armed ? "YES" : "NO ");
    }
  }

  // Signal loss failsafe — independent of gotPacket
  if (armed && (now - lastValidSignal > SIGNAL_TIMEOUT_MS)) {
    armed = false;
    lastArmState = true; // keep as true so transition only fires after switch goes LOW then HIGH
    dshotVal = DSHOT_CMD_STOP;
    Serial.println("[FAIL] No receiver signal — motor stopped");
    Serial.println("[FAIL] Toggle arm switch to re-arm after signal recovery");
    buzzerAlert(1, 500, 800, 0);
  }

  // Send last known value continuously — never interrupted by STOP
  if (now - lastDshot >= 3) {
    lastDshot = now;
    dshotSendAll(dshotVal);
  }
}
