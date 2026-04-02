#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>
#include <driver/rmt.h>

// ────────────────────────────────────────────────────────────────
// CONFIGURATION
// ────────────────────────────────────────────────────────────────
#define WIFI_SSID "IMU-Calibrate"
#define WIFI_PASSWORD "calibrate123"

#define I2C_SDA 7
#define I2C_SCL 6

#define ESC1_PIN 1
#define ESC2_PIN 2
#define ESC3_PIN 4
#define ESC4_PIN 5

#define GRAVITY_CON 9.81f

// DSHOT 20–30% throttle range
// DSHOT range 48–2047 → 20% = ~448, 30% = ~648
#define MOTOR_DSHOT_IDLE 448 // 20% — lower bound
#define MOTOR_DSHOT_WARM 648 // 30% — upper bound (used as default)

#define DSHOT_RMT_CLK_DIV 1
#define DSHOT_T1H_TICKS 200
#define DSHOT_T1L_TICKS 66
#define DSHOT_T0H_TICKS 100
#define DSHOT_T0L_TICKS 133
#define DSHOT_PAUSE_TICKS 2667
#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047
#define DSHOT_CMD_STOP 0

#define RMT_CH_M1 RMT_CHANNEL_0
#define RMT_CH_M2 RMT_CHANNEL_1
#define RMT_CH_M3 RMT_CHANNEL_2
#define RMT_CH_M4 RMT_CHANNEL_3

// Calibration parameters
#define CAL_NUM_RUNS 3
#define CAL_SAMPLES_PER_RUN 500

// ────────────────────────────────────────────────────────────────
// GLOBALS
// ────────────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;
WebServer server(80);

static rmt_item32_t dshotItems[4][17];

// Motor state
volatile bool motorsSpinning = false;
volatile uint16_t motorThrottle = MOTOR_DSHOT_WARM;
unsigned long lastDshot = 0;

// Calibration state
enum class CalState { IDLE, RUNNING, DONE, FAILED };
volatile CalState calState = CalState::IDLE;
volatile int calRun = 0;
volatile int calSample = 0;

// Running accumulators
double sumGX = 0, sumGY = 0, sumGZ = 0;
double sumAX = 0, sumAY = 0, sumAZ = 0;
double runGX[CAL_NUM_RUNS], runGY[CAL_NUM_RUNS], runGZ[CAL_NUM_RUNS];
double runAX[CAL_NUM_RUNS], runAY[CAL_NUM_RUNS], runAZ[CAL_NUM_RUNS];

// Final results
float resGyroX = 0, resGyroY = 0, resGyroZ = 0;
float resAccelX = 0, resAccelY = 0, resAccelZ = 0;
float resStdDev_gyro = 0, resStdDev_accel = 0;
bool resultsReady = false;

// Live sensor readings (updated during calibration)
float liveGX = 0, liveGY = 0, liveGZ = 0;
float liveAX = 0, liveAY = 0, liveAZ = 0;
float liveTemp = 0;

// ────────────────────────────────────────────────────────────────
// DSHOT
// ────────────────────────────────────────────────────────────────
uint8_t dshotCRC(uint16_t v) {
  return ((v ^ (v >> 4) ^ (v >> 8)) & 0x0F);
}

void buildDshotPacket(rmt_item32_t *items, uint16_t value) {
  uint16_t packet = (value << 1);
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

void dshotSend(rmt_channel_t ch, int idx, uint16_t val) {
  if (val != 0) {
    if (val < DSHOT_THROTTLE_MIN) val = DSHOT_THROTTLE_MIN;
    if (val > DSHOT_THROTTLE_MAX) val = DSHOT_THROTTLE_MAX;
  }
  buildDshotPacket(dshotItems[idx], val);
  rmt_write_items(ch, dshotItems[idx], 17, false);
}

void dshotWriteAll(uint16_t val) {
  dshotSend(RMT_CH_M1, 0, val);
  dshotSend(RMT_CH_M2, 1, val);
  dshotSend(RMT_CH_M3, 2, val);
  dshotSend(RMT_CH_M4, 3, val);
  rmt_wait_tx_done(RMT_CH_M1, pdMS_TO_TICKS(2));
  rmt_wait_tx_done(RMT_CH_M2, pdMS_TO_TICKS(2));
  rmt_wait_tx_done(RMT_CH_M3, pdMS_TO_TICKS(2));
  rmt_wait_tx_done(RMT_CH_M4, pdMS_TO_TICKS(2));
}

static void initDshotChannel(rmt_channel_t ch, int pin) {
  rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX((gpio_num_t)pin, ch);
  cfg.clk_div = DSHOT_RMT_CLK_DIV;
  rmt_config(&cfg);
  rmt_driver_install(ch, 0, 0);
}

// ────────────────────────────────────────────────────────────────
// CALIBRATION TASK (Core 0)
// ────────────────────────────────────────────────────────────────
void calibrationTask(void*) {
  calState = CalState::RUNNING;
  calRun = 0;
  calSample = 0;
  resultsReady = false;

  memset(runGX, 0, sizeof(runGX));
  memset(runGY, 0, sizeof(runGY));
  memset(runGZ, 0, sizeof(runGZ));
  memset(runAX, 0, sizeof(runAX));
  memset(runAY, 0, sizeof(runAY));
  memset(runAZ, 0, sizeof(runAZ));

  for (int run = 0; run < CAL_NUM_RUNS; run++) {
    calRun = run + 1;
    sumGX = sumGY = sumGZ = 0;
    sumAX = sumAY = sumAZ = 0;

    for (int s = 0; s < CAL_SAMPLES_PER_RUN; s++) {
      if (calState != CalState::RUNNING) goto done; // cancelled

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      sumGX += g.gyro.x;
      sumGY += g.gyro.y;
      sumGZ += g.gyro.z;
      sumAX += a.acceleration.x;
      sumAY += a.acceleration.y;
      sumAZ += a.acceleration.z;

      // Update live readings
      liveGX = g.gyro.x;
      liveGY = g.gyro.y;
      liveGZ = g.gyro.z;
      liveAX = a.acceleration.x;
      liveAY = a.acceleration.y;
      liveAZ = a.acceleration.z;
      liveTemp = temp.temperature;

      calSample = s + 1;
      vTaskDelay(1); // yield — ~500ms per run
    }

    runGX[run] = sumGX / CAL_SAMPLES_PER_RUN;
    runGY[run] = sumGY / CAL_SAMPLES_PER_RUN;
    runGZ[run] = sumGZ / CAL_SAMPLES_PER_RUN;
    runAX[run] = sumAX / CAL_SAMPLES_PER_RUN;
    runAY[run] = sumAY / CAL_SAMPLES_PER_RUN;
    runAZ[run] = sumAZ / CAL_SAMPLES_PER_RUN;
  }

  // Average across runs
  {
    double gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
    for (int r = 0; r < CAL_NUM_RUNS; r++) {
      gx += runGX[r]; gy += runGY[r]; gz += runGZ[r];
      ax += runAX[r]; ay += runAY[r]; az += runAZ[r];
    }
    resGyroX = (float)(gx / CAL_NUM_RUNS);
    resGyroY = (float)(gy / CAL_NUM_RUNS);
    resGyroZ = (float)(gz / CAL_NUM_RUNS);
    resAccelX = (float)(ax / CAL_NUM_RUNS);
    resAccelY = (float)(ay / CAL_NUM_RUNS);
    resAccelZ = (float)(az / CAL_NUM_RUNS) - GRAVITY_CON;

    // Gyro std dev across runs (measure of stability)
    double vg = 0;
    for (int r = 0; r < CAL_NUM_RUNS; r++) {
      double d = runGX[r] - resGyroX;
      vg += d * d;
    }
    resStdDev_gyro = (float)sqrt(vg / CAL_NUM_RUNS);

    // Accel std dev across runs
    double va = 0;
    for (int r = 0; r < CAL_NUM_RUNS; r++) {
      double d = runAZ[r] - (resAccelZ + GRAVITY_CON);
      va += d * d;
    }
    resStdDev_accel = (float)sqrt(va / CAL_NUM_RUNS);
  }

  resultsReady = true;
  calState = CalState::DONE;
  done:
  vTaskDelete(NULL);
}

// ────────────────────────────────────────────────────────────────
// HTTP HANDLERS
// ────────────────────────────────────────────────────────────────
void handleData() {
  char buf[512];
  const char* stateStr = "idle";
  if (calState == CalState::RUNNING) stateStr = "running";
  else if (calState == CalState::DONE) stateStr = "done";
  else if (calState == CalState::FAILED) stateStr = "failed";

  int totalSamples = CAL_NUM_RUNS * CAL_SAMPLES_PER_RUN;
  int doneSamples = (calRun - 1) * CAL_SAMPLES_PER_RUN + calSample;
  int pct = (calState == CalState::RUNNING && totalSamples > 0)
            ? (doneSamples * 100 / totalSamples) : 0;
  if (calState == CalState::DONE) pct = 100;

  snprintf(buf, sizeof(buf),
    "{"
    "\"state\":\"%s\","
    "\"run\":%d,\"runTotal\":%d,"
    "\"sample\":%d,\"sampleTotal\":%d,"
    "\"pct\":%d,"
    "\"motorsOn\":%s,"
    "\"throttle\":%d,"
    "\"gx\":%.6f,\"gy\":%.6f,\"gz\":%.6f,"
    "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
    "\"temp\":%.1f,"
    "\"resultsReady\":%s,"
    "\"rGyroX\":%.6f,\"rGyroY\":%.6f,\"rGyroZ\":%.6f,"
    "\"rAccelX\":%.6f,\"rAccelY\":%.6f,\"rAccelZ\":%.6f,"
    "\"stdGyro\":%.6f,\"stdAccel\":%.6f"
    "}",
    stateStr,
    (int)calRun, CAL_NUM_RUNS,
    (int)calSample, CAL_SAMPLES_PER_RUN,
    pct,
    motorsSpinning ? "true" : "false",
    (int)motorThrottle,
    liveGX, liveGY, liveGZ,
    liveAX, liveAY, liveAZ,
    liveTemp,
    resultsReady ? "true" : "false",
    resGyroX, resGyroY, resGyroZ,
    resAccelX, resAccelY, resAccelZ,
    resStdDev_gyro, resStdDev_accel
  );

  server.send(200, "application/json", buf);
}

void handleMotorOn() {
  if (calState == CalState::RUNNING) {
    server.send(400, "application/json", "{\"error\":\"calibration running\"}");
    return;
  }
  motorsSpinning = true;
  server.send(200, "application/json", "{\"motors\":true}");
}

void handleMotorOff() {
  motorsSpinning = false;
  dshotWriteAll(DSHOT_CMD_STOP);
  server.send(200, "application/json", "{\"motors\":false}");
}

void handleThrottle() {
  if (server.hasArg("v")) {
    int v = server.arg("v").toInt();
    motorThrottle = (uint16_t)constrain(v, MOTOR_DSHOT_IDLE, MOTOR_DSHOT_WARM);
  }
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"throttle\":%d}", (int)motorThrottle);
  server.send(200, "application/json", buf);
}

void handleCalStart() {
  if (calState == CalState::RUNNING) {
    server.send(400, "application/json", "{\"error\":\"already running\"}");
    return;
  }
  calRun = 0; calSample = 0;
  xTaskCreatePinnedToCore(calibrationTask, "CalTask", 4096, nullptr, 1, nullptr, 0);
  server.send(200, "application/json", "{\"started\":true}");
}

void handleCalStop() {
  if (calState == CalState::RUNNING) {
    calState = CalState::IDLE;
  }
  server.send(200, "application/json", "{\"stopped\":true}");
}

void handleRoot() {
  server.send(200, "text/html", R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>IMU CALIBRATION</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap');
:root {
  --bg:#080c10; --bg2:#0d1520; --bg3:#111d2e;
  --border:#1a3a5c; --accent:#00d4ff; --green:#00ff88;
  --yellow:#ffcc00; --red:#ff3344; --dim:#3a5a7a;
  --text:#c8e0f0; --mono:'Share Tech Mono',monospace;
  --sans:'Rajdhani',sans-serif;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:var(--sans);min-height:100vh;padding:16px}
body::before{content:'';position:fixed;inset:0;background:repeating-linear-gradient(0deg,transparent,transparent 2px,rgba(0,0,0,0.06) 2px,rgba(0,0,0,0.06) 4px);pointer-events:none;z-index:9999}
h1{font-family:var(--mono);color:var(--accent);letter-spacing:4px;font-size:18px;margin-bottom:4px}
.sub{font-family:var(--mono);font-size:11px;color:var(--dim);letter-spacing:2px;margin-bottom:20px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;max-width:900px}
@media(max-width:600px){.grid{grid-template-columns:1fr}}
.panel{background:var(--bg2);border:1px solid var(--border);border-radius:3px;padding:16px;position:relative;overflow:hidden}
.panel::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;background:linear-gradient(90deg,var(--accent),transparent)}
.panel-title{font-family:var(--mono);font-size:10px;letter-spacing:3px;color:var(--dim);text-transform:uppercase;margin-bottom:14px}
.btn{display:inline-block;padding:10px 22px;border-radius:2px;font-family:var(--mono);font-size:12px;letter-spacing:2px;font-weight:700;cursor:pointer;border:none;transition:all .15s;text-transform:uppercase}
.btn-green{background:rgba(0,255,136,.1);color:var(--green);border:1px solid var(--green)}
.btn-green:hover{background:rgba(0,255,136,.2)}
.btn-green:active{background:rgba(0,255,136,.3)}
.btn-red{background:rgba(255,51,68,.1);color:var(--red);border:1px solid var(--red)}
.btn-red:hover{background:rgba(255,51,68,.2)}
.btn-blue{background:rgba(0,212,255,.1);color:var(--accent);border:1px solid var(--accent)}
.btn-blue:hover{background:rgba(0,212,255,.2)}
.btn:disabled{opacity:.35;cursor:not-allowed}
.btn-row{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:12px}
.stat-row{display:flex;justify-content:space-between;align-items:baseline;padding:5px 0;border-bottom:1px solid var(--bg3)}
.stat-row:last-child{border-bottom:none}
.stat-key{font-family:var(--mono);font-size:10px;color:var(--dim);letter-spacing:1px}
.stat-val{font-family:var(--mono);font-size:13px;color:var(--text)}
.stat-val.good{color:var(--green)}
.stat-val.warn{color:var(--yellow)}
.status-pill{display:inline-block;padding:3px 12px;border-radius:2px;font-family:var(--mono);font-size:11px;font-weight:700;letter-spacing:1px;margin-bottom:14px}
.pill-idle{color:var(--dim);border:1px solid var(--dim)}
.pill-running{color:var(--yellow);border:1px solid var(--yellow);animation:blink .8s infinite}
.pill-done{color:var(--green);border:1px solid var(--green)}
@keyframes blink{50%{opacity:.4}}
.progress-bg{background:var(--bg3);border:1px solid var(--border);height:10px;border-radius:2px;overflow:hidden;margin:10px 0}
.progress-fill{height:100%;background:linear-gradient(90deg,var(--accent),var(--green));transition:width .3s;border-radius:2px}
.results-box{background:var(--bg3);border:1px solid var(--border);border-radius:2px;padding:12px;margin-top:12px;font-family:var(--mono);font-size:12px;line-height:1.7;color:var(--green);white-space:pre;overflow-x:auto;display:none}
.warn-box{background:rgba(255,204,0,.06);border:1px solid rgba(255,204,0,.3);border-radius:2px;padding:10px 12px;margin-top:10px;font-family:var(--mono);font-size:11px;color:var(--yellow);line-height:1.6}
.throttle-row{display:flex;align-items:center;gap:12px;margin-bottom:10px}
.throttle-label{font-family:var(--mono);font-size:10px;color:var(--dim);letter-spacing:1px;white-space:nowrap}
input[type=range]{flex:1;accent-color:var(--accent)}
#throttle-val{font-family:var(--mono);font-size:13px;color:var(--accent);width:50px;text-align:right}
.full{grid-column:1/-1}
</style>
</head>
<body>
<h1>IMU CALIBRATION</h1>
<div class="sub">MPU6050 — ESP32-S3 N16R8 — GYRO & ACCEL OFFSETS</div>
<div class="grid">

  <!-- MOTORS -->
  <div class="panel">
    <div class="panel-title">MOTOR WARM-UP</div>
    <div class="warn-box">⚠ REMOVE ALL PROPELLERS before starting motors.</div>
    <br>
    <div class="throttle-row">
      <span class="throttle-label">THROTTLE</span>
      <input type="range" id="throttle-slider" min="448" max="648" value="648" oninput="onThrottle(this.value)">
      <span id="throttle-val">648</span>
    </div>
    <div class="btn-row">
      <button class="btn btn-green" id="btn-motors-on"  onclick="motorStart()">START MOTORS</button>
      <button class="btn btn-red"   id="btn-motors-off" onclick="motorStop()" disabled>STOP MOTORS</button>
    </div>
    <div class="stat-row"><span class="stat-key">STATUS</span><span class="stat-val" id="motor-status">STOPPED</span></div>
    <div class="stat-row"><span class="stat-key">DSHOT VALUE</span><span class="stat-val" id="motor-dshot">—</span></div>
    <div class="stat-row"><span class="stat-key">THROTTLE %</span><span class="stat-val" id="motor-pct">—</span></div>
    <div class="stat-row"><span class="stat-key">IMU TEMP</span><span class="stat-val" id="imu-temp">—</span></div>
  </div>

  <!-- LIVE READINGS -->
  <div class="panel">
    <div class="panel-title">LIVE SENSOR READINGS</div>
    <div class="stat-row"><span class="stat-key">GYRO X (rad/s)</span><span class="stat-val" id="lv-gx">—</span></div>
    <div class="stat-row"><span class="stat-key">GYRO Y (rad/s)</span><span class="stat-val" id="lv-gy">—</span></div>
    <div class="stat-row"><span class="stat-key">GYRO Z (rad/s)</span><span class="stat-val" id="lv-gz">—</span></div>
    <div class="stat-row"><span class="stat-key">ACCEL X (m/s²)</span><span class="stat-val" id="lv-ax">—</span></div>
    <div class="stat-row"><span class="stat-key">ACCEL Y (m/s²)</span><span class="stat-val" id="lv-ay">—</span></div>
    <div class="stat-row"><span class="stat-key">ACCEL Z (m/s²)</span><span class="stat-val" id="lv-az">—</span></div>
  </div>

  <!-- CALIBRATION CONTROL -->
  <div class="panel full">
    <div class="panel-title">CALIBRATION</div>
    <div id="cal-pill" class="status-pill pill-idle">IDLE</div>
    <div class="btn-row">
      <button class="btn btn-blue" id="btn-cal-start" onclick="calStart()">START CALIBRATION</button>
      <button class="btn btn-red"  id="btn-cal-stop"  onclick="calStop()" disabled>CANCEL</button>
    </div>
    <div class="stat-row"><span class="stat-key">PROGRESS</span><span class="stat-val" id="cal-pct">—</span></div>
    <div class="progress-bg"><div class="progress-fill" id="cal-bar" style="width:0%"></div></div>
    <div class="stat-row"><span class="stat-key">RUN</span><span class="stat-val" id="cal-run">—</span></div>
    <div class="stat-row"><span class="stat-key">SAMPLES THIS RUN</span><span class="stat-val" id="cal-sample">—</span></div>
    <div class="warn-box">
      Keep drone FLAT and STILL during calibration.<br>
      Motors may spin at 20-30% — this is intentional to capture thermal offsets.<br>
      3 runs x 500 samples ≈ 1.5 seconds total.
    </div>
    <div id="results-box" class="results-box"></div>
    <div class="btn-row" style="margin-top:10px">
      <button class="btn btn-blue" id="btn-copy" onclick="copyResults()" style="display:none">COPY DEFINES</button>
    </div>
  </div>

</div>

<script>
const DSHOT_MIN = 48, DSHOT_MAX = 2047;

function dshotPct(v) {
  return ((v - DSHOT_MIN) / (DSHOT_MAX - DSHOT_MIN) * 100).toFixed(1);
}

async function motorStart() {
  await fetch('/motor/on');
}
async function motorStop() {
  await fetch('/motor/off');
}
async function calStart() {
  await fetch('/cal/start');
}
async function calStop() {
  await fetch('/cal/stop');
}

function onThrottle(v) {
  document.getElementById('throttle-val').textContent = v;
  fetch('/motor/throttle?v=' + v);
}

let lastResults = '';

async function poll() {
  try {
    const r = await fetch('/data');
    const d = await r.json();

    // Motors
    const mon = d.motorsOn;
    document.getElementById('motor-status').textContent = mon ? 'SPINNING' : 'STOPPED';
    document.getElementById('motor-status').style.color = mon ? 'var(--green)' : 'var(--dim)';
    document.getElementById('motor-dshot').textContent = mon ? d.throttle : '—';
    document.getElementById('motor-pct').textContent   = mon ? dshotPct(d.throttle) + '%' : '—';
    document.getElementById('imu-temp').textContent    = d.temp.toFixed(1) + ' °C';
    document.getElementById('btn-motors-on').disabled  = mon;
    document.getElementById('btn-motors-off').disabled = !mon;

    // Slider sync
    document.getElementById('throttle-slider').value = d.throttle;
    document.getElementById('throttle-val').textContent = d.throttle;

    // Live readings
    document.getElementById('lv-gx').textContent = d.gx.toFixed(6);
    document.getElementById('lv-gy').textContent = d.gy.toFixed(6);
    document.getElementById('lv-gz').textContent = d.gz.toFixed(6);
    document.getElementById('lv-ax').textContent = d.ax.toFixed(4);
    document.getElementById('lv-ay').textContent = d.ay.toFixed(4);
    document.getElementById('lv-az').textContent = d.az.toFixed(4);

    // Calibration
    const state = d.state;
    const pill = document.getElementById('cal-pill');
    if (state === 'running') {
      pill.textContent = 'RUNNING'; pill.className = 'status-pill pill-running';
      document.getElementById('btn-cal-start').disabled = true;
      document.getElementById('btn-cal-stop').disabled  = false;
    } else if (state === 'done') {
      pill.textContent = 'DONE'; pill.className = 'status-pill pill-done';
      document.getElementById('btn-cal-start').disabled = false;
      document.getElementById('btn-cal-stop').disabled  = true;
    } else {
      pill.textContent = 'IDLE'; pill.className = 'status-pill pill-idle';
      document.getElementById('btn-cal-start').disabled = false;
      document.getElementById('btn-cal-stop').disabled  = true;
    }

    document.getElementById('cal-pct').textContent    = d.pct + '%';
    document.getElementById('cal-bar').style.width    = d.pct + '%';
    document.getElementById('cal-run').textContent    = state === 'idle' ? '—' : d.run + ' / ' + d.runTotal;
    document.getElementById('cal-sample').textContent = state === 'idle' ? '—' : d.sample + ' / ' + d.sampleTotal;

    // Results
    if (d.resultsReady) {
      const gyroOk  = d.stdGyro  < 0.001;
      const accelOk = d.stdAccel < 0.01;
      const lines = [
        '// IMU Calibration Results',
        '// ' + CAL_NUM_RUNS + ' runs x ' + CAL_SAMPLES_PER_RUN + ' samples',
        '// Gyro std dev: ' + d.stdGyro.toFixed(6) + ' rad/s  ' + (gyroOk ? '✓ STABLE' : '⚠ CHECK MOUNTING'),
        '// Accel std dev: ' + d.stdAccel.toFixed(6) + ' m/s²  ' + (accelOk ? '✓ STABLE' : '⚠ CHECK MOUNTING'),
        '',
        '#define GYRO_OFFSET_X  ' + d.rGyroX.toFixed(6) + 'f',
        '#define GYRO_OFFSET_Y  ' + d.rGyroY.toFixed(6) + 'f',
        '#define GYRO_OFFSET_Z  ' + d.rGyroZ.toFixed(6) + 'f',
        '',
        '#define ACCEL_OFFSET_X ' + d.rAccelX.toFixed(6) + 'f',
        '#define ACCEL_OFFSET_Y ' + d.rAccelY.toFixed(6) + 'f',
        '#define ACCEL_OFFSET_Z ' + d.rAccelZ.toFixed(6) + 'f  // raw mean - gravity',
      ].join('\n');

      const box = document.getElementById('results-box');
      box.textContent = lines;
      box.style.display = 'block';
      document.getElementById('btn-copy').style.display = '';
      lastResults = lines;
    }

  } catch(e) {}
  setTimeout(poll, 200);
}

const CAL_NUM_RUNS = 3, CAL_SAMPLES_PER_RUN = 500;

function copyResults() {
  navigator.clipboard.writeText(lastResults).catch(() => {
    const ta = document.createElement('textarea');
    ta.value = lastResults;
    document.body.appendChild(ta);
    ta.select();
    document.execCommand('copy');
    document.body.removeChild(ta);
  });
  const btn = document.getElementById('btn-copy');
  btn.textContent = 'COPIED ✓';
  setTimeout(() => btn.textContent = 'COPY DEFINES', 2000);
}

poll();
</script>
</body>
</html>
)rawhtml");
}

// ────────────────────────────────────────────────────────────────
// SETUP
// ────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Serial.print("MPU6050... ");
  if (!mpu.begin()) {
    Serial.println("FAIL — check wiring");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  Serial.println("OK");

  // Init DSHOT — arm ESCs with 1 second of stop frames
  Serial.print("Arming ESCs... ");
  initDshotChannel(RMT_CH_M1, ESC1_PIN);
  initDshotChannel(RMT_CH_M2, ESC2_PIN);
  initDshotChannel(RMT_CH_M3, ESC3_PIN);
  initDshotChannel(RMT_CH_M4, ESC4_PIN);
  unsigned long armT = millis();
  while (millis() - armT < 1000) {
    dshotWriteAll(DSHOT_CMD_STOP);
    delay(10);
  }
  Serial.println("done");

  // WiFi AP
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 1, 0, 1);
  WiFi.setTxPower(WIFI_POWER_2dBm);
  Serial.print("WiFi AP: "); Serial.println(WIFI_SSID);
  Serial.print("Open: http://"); Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/motor/on", handleMotorOn);
  server.on("/motor/off", handleMotorOff);
  server.on("/motor/throttle", handleThrottle);
  server.on("/cal/start", handleCalStart);
  server.on("/cal/stop", handleCalStop);
  server.begin();

  Serial.println("Ready. Connect to WiFi and open browser.");
}

// ────────────────────────────────────────────────────────────────
// LOOP
// ────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();

  unsigned long now = millis();
  if (now - lastDshot >= 3) {
    lastDshot = now;
    dshotWriteAll(motorsSpinning ? motorThrottle : DSHOT_CMD_STOP);
  }
}
