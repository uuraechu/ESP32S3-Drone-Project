#include <math.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>
#include <driver/rmt.h>
#include <TinyGPS++.h>

// ────────────────────────────────────────────────────────────────
// PIN DEFINITIONS
// ────────────────────────────────────────────────────────────────
// DSHOT motor outputs — each needs 2kΩ pullup to 3V3
#define ESC1_PIN 1 // Front-right (M1 CCW)
#define ESC2_PIN 2 // Rear-right (M2 CW)
#define ESC3_PIN 4 // Front-left (M3 CW)
#define ESC4_PIN 5 // Rear-left (M4 CCW)

// iBUS receiver — UART1 RX only
#define IBUS_RX_PIN 12
#define IBUS_TX_PIN -1 // not used

// GPS UART2 pins
#define GPS_RX_PIN 10
#define GPS_TX_PIN 9
#define GPS_BAUD 9600

// Magnetic declination for your location (degrees)
// Find yours at https://www.magnetic-declination.com
// Positive = East, Negative = West
#define MAG_DECLINATION -5.5f // Cookeville TN — negative = west

// I2C — shared bus (MPU6050 + BMP280)
#define I2C_SCL 6
#define I2C_SDA 7

// Peripherals
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
// DSHOT CONSTANTS
// ────────────────────────────────────────────────────────────────
// DSHOT300 timing at 80MHz RMT clock (12.5ns per tick)
// Bit period = 3333ns = 266.7 ticks
// T1H = 2500ns = 200 ticks T1L = 833ns = 66 ticks
// T0H = 1250ns = 100 ticks T0L = 1666ns = 133 ticks
#define DSHOT_RMT_CLK_DIV 1
#define DSHOT_T1H_TICKS 200
#define DSHOT_T1L_TICKS 66
#define DSHOT_T0H_TICKS 100
#define DSHOT_T0L_TICKS 133
#define DSHOT_PAUSE_TICKS 2667 // ~33µs pause between frames

#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047
#define DSHOT_CMD_STOP 0

// RMT channels — one per motor
#define RMT_CH_M1 RMT_CHANNEL_0 // Front-right CCW
#define RMT_CH_M2 RMT_CHANNEL_1 // Rear-right CW
#define RMT_CH_M3 RMT_CHANNEL_2 // Front-left CW
#define RMT_CH_M4 RMT_CHANNEL_3 // Rear-left CCW

// ────────────────────────────────────────────────────────────────
// iBUS CONSTANTS
// ────────────────────────────────────────────────────────────────
#define IBUS_BAUD 115200
#define IBUS_PACKET_LEN 32
#define IBUS_HEADER1 0x20
#define IBUS_HEADER2 0x40
#define IBUS_NUM_CHANNELS 14

// ────────────────────────────────────────────────────────────────
// FLIGHT CONSTANTS
// ────────────────────────────────────────────────────────────────
// Quadratic ADC calibration: vbat = VDIV_A*adc² + VDIV_B*adc + VDIV_C
// Calibrate by measuring actual voltage at 6-8 points across full range
// then use polynomial regression to find coefficients
#define VDIV_A -0.79516f // quadratic coefficient — fill after calibration
#define VDIV_B 8.87512f // linear coefficient — fill after calibration
#define VDIV_C -2.36039f // offset — fill after calibration

// Per-cell thresholds — valid for both 3S and 4S
#define CELL_VOLT_CUTOFF 3.1f // per-cell cutoff (3S=9.3V 4S=12.4V)
#define CELL_VOLT_WARNING 3.4f // per-cell warning (3S=10.2V 4S=13.6V)

// Auto-detected at boot — do not set manually
#define CELL_DETECT_THRESHOLD 13.5f // above = 4S, below = 3S
                                    // 3S full = 12.6V, 4S empty = 12.8V

// Update to match your pack
#define BATTERY_CAPACITY_MAH_3S 2200.0f // 3S pack capacity
#define BATTERY_CAPACITY_MAH_4S 1500.0f // 4S pack capacity
#define CELL_COUNT_3S 3
#define CELL_COUNT_4S 4
#define CURR_SCALE 116.5f
#define CURR_ESTIMATE_THRESHOLD_A 1.0f
#define CURR_OFFSET_A 0.0f // zero-current ADC offset in amps — adjust if reads non-zero at idle
#define FAILSAFE_TIMEOUT_MS 500
#define FAILSAFE_HOVER_SEC 5 // seconds to hover before descending
#define ALT_HOLD_MIN_HEIGHT_M 5.0f // minimum height to engage altitude hold
#define ALT_HOLD_MIN_THROTTLE 100.0f // minimum rcThrottle to engage altitude hold
#define ALT_HOLD_RAMP_SEC 0.5f // seconds to ramp throttle on transition
#define BAT_OVERRIDE_CHANNEL 6 // CH7 — battery cutoff override switch
#define DESCENT_CHANNEL 7 // CH8 — index 7
#define GRAVITY_CON 9.81f
#define LOOP_INTERVAL_MS 5
#define ROLL_MAX_MIN 45.0f
#define PITCH_MAX_MIN 45.0f
#define YAW_RATE_MAX_MIN 220.0f
#define ALT_BARO_WEIGHT 0.08f // barometer blend into height (0.02–0.15)
                              // lower = smoother but slower baro correction
                              // raise if height drifts, lower if it bounces
#define ALT_INNOV_WEIGHT 0.08f // innovation blend into velocity (same range)
                               // this × (1/dt) is the effective velocity Kp
                               // at 5ms dt: 0.08 × 200 = Kp 16
                               // reduce to 0.02 if altitude bounces on first flight
#define ALT_DEADBAND_M 0.10f // ±10cm — no correction within this band
                             // increase if throttle jitters at hover
                             // decrease if hold feels too loose

// Gyroscopic precession compensation
// PROP_GYRO_COEFF — tune from 0, increase in 0.001 steps until yaw no longer
// induces roll/pitch. Typically 0.002–0.010 depending on prop inertia.
#define PROP_GYRO_COEFF 0.0f // set to 0 to disable, tune after Kp is stable

// Yaw-to-throttle coupling compensation
// YAW_THROTTLE_COEFF — tune from 0, increase in 0.001 steps.
// Eliminates altitude dip during fast yaw spins.
#define YAW_THROTTLE_COEFF 0.0f // disable until Kp is stable

// ── USER CONFIGURATION (WiFi AP settings) ───────────────────────
#define WIFI_SSID "QuadTelemetry" // ← change before public use
#define WIFI_PASSWORD "flysafe123" // ← change before public use

// ── HARD IMU OFFSETS ──────────────────────────────────
// Generated by calibration tool
// 3 runs x 500 samples | Gyro dev: 0.000140 Accel dev: 0.002320
// Remeasure if IMU is remounted, replaced, or readings
// show consistent non-zero pitch/roll at rest.
// Fill section after calibration
// Gyro units: rad/s Accel units: m/s²
#define GYRO_OFFSET_X -0.035158f
#define GYRO_OFFSET_Y 0.008582f
#define GYRO_OFFSET_Z -0.001296f

#define ACCEL_OFFSET_X 0.298911f
#define ACCEL_OFFSET_Y 0.150439f
#define ACCEL_OFFSET_Z 0.468533f // raw mean - gravity

// Magnetometer hard iron calibration offsets
// Leave at 0 until you run calibration routine below
#define MAG_OFFSET_X -15.9091f
#define MAG_OFFSET_Y -12.3636f
#define MAG_OFFSET_Z 6.1224f

// ── SOFTWARE TRIM ────────────────────────────────────────────────
// Compensates for fixed mechanical offsets (battery position etc.)
// Adjust PITCH_TRIM in 0.5° steps until hover is level.
// Positive PITCH_TRIM = nose-up correction (forward battery)
#define PITCH_TRIM 0.0f // degrees
#define ROLL_TRIM 0.0f // degrees

// Motor min/max in DSHOT units
#define DSHOT_MIN DSHOT_THROTTLE_MIN // 48
#define DSHOT_MAX 1800 // leave headroom below 2047

// ────────────────────────────────────────────────────────────────
// GLOBAL STATE VARIABLES
// ────────────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
TinyGPSPlus gps;
WebServer server(80);
HardwareSerial iBusSerial(1); // UART1 for iBUS
HardwareSerial gpsSerial(2); // UART2

// Magnetometer
float magHeading = 0.0f; // degrees, 0=North, 90=East
bool magValid = false;

// GPS
float gpsLat = 0.0f;
float gpsLng = 0.0f;
float gpsAltitude = 0.0f;
float gpsSpeed = 0.0f; // km/h
float gpsCourse = 0.0f; // degrees from North
uint8_t gpsSatellites = 0;
bool gpsFixed = false;
unsigned long gpsAge = 0;

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float baroOffset = 0.0f;

float roll = 0, pitch = 0, yawRate = 0;
float height = 0, velocity = 0;
float gyroRollRate = 0, gyroPitchRate = 0, gyroYawRate = 0;
float accelPitch = 0, accelRoll = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float baroAlt = 0;

unsigned long prevTime = 0;
unsigned long lastValidSignalTime = 0;

// ── iBUS shared RC values ─────────────────────────────────────────
// Written by Core 0 iBUS task, read by Core 1 flight loop
volatile uint16_t sharedPwmRoll = 1500;
volatile uint16_t sharedPwmPitch = 1500;
volatile uint16_t sharedPwmThrottle = 1000;
volatile uint16_t sharedPwmYaw = 1500;
volatile uint16_t sharedPwmArm = 1000;
volatile uint16_t sharedPwmAux = 1000;
volatile uint16_t sharedPwmBatOverride = 1000;
volatile uint16_t sharedPwmDescent = 1000;
volatile unsigned long sharedLastValidSignal = 0;

uint16_t pwmRoll = 1500, pwmPitch = 1500, pwmThrottle = 1000;
uint16_t pwmYaw = 1500, pwmArm = 1000, pwmAux = 1000;
uint16_t pwmBatOverride = 1000, pwmDescent = 1000;
float rcRoll = 0, rcPitch = 0, rcYawRate = 0, rcThrottle = 0;

// ── Motor outputs in DSHOT units ─────────────────────────────────
volatile uint16_t m1 = DSHOT_MIN, m2 = DSHOT_MIN;
volatile uint16_t m3 = DSHOT_MIN, m4 = DSHOT_MIN;

const float seaLevel = 1013.25f;
const float GYRO_SCALE = (180.0f / M_PI); // rad/s → deg/s
volatile int batteryCells = 3; // overwritten at boot by initBatteryState()

// RMT item buffers — 16 bits + 1 pause item per motor
static rmt_item32_t dshotItems[4][17];

struct BuzzerState {
  bool active = false;
  int beepsRemaining = 0;
  int frequency = 0;
  int duration_ms = 0;
  int pause_ms = 150;
  bool isTone = false;
  unsigned long lastToggle = 0;
  bool continuous = false;
  uint8_t priority = 0;
};

BuzzerState buzzer;

// Mutex for shared data between cores:
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t rcMutex;
SemaphoreHandle_t gpsMutex;

// Runtime debug toggle — controlled via HTTP /debug endpoint
volatile bool debugOutputEnabled = false; // off by default for flight

// Volatile — written outside dataMutex, read inside it in handleData()
// volatile prevents compiler reordering across the mutex barrier
volatile float vbat = 0.0f;
volatile float currentAmps = 0.0f;
volatile float powerWatts = 0.0f; // instantaneous power draw (W)
volatile float consumedMah = 0.0f; // mAh used since boot
volatile float remainingMah = 0.0f; // mAh remaining
volatile float flightTimeSec = 0.0f; // estimated seconds remaining at current draw
volatile float temperature = 0.0f;
volatile bool armed = false;
volatile bool altitudeHoldEnabled = false;
volatile bool receiverFailsafeActive = false;
volatile bool lowBatCutoff = false;
volatile bool lowBatWarning = false;
bool batCutoffOverrideActive = false;
bool controlledDescentActive = false;
float descentRate = 0.0f;
unsigned long failsafeHoverStart = 0;
float altHoldThrottleRamp = 0.0f;
unsigned long altHoldTransitionStart = 0;
bool altHoldEngaging = false;
bool altHoldDisengaging = false;

// ── Boot orientation check ───────────────────────────────────────
enum class BootOrientation {
  LEVEL, // within ±15° on both axes — safe to fly
  TILTED, // 15°–60° — warn, recoverable without reboot
  ON_SIDE, // >60° — seriously misoriented
  INVERTED // accelZ negative — upside down
};

volatile bool orientationClearToFly = false;

// ────────────────────────────────────────────────────────────────
// PID CONTROLLER
// Gains are on a 0-10 scale: Kp=5 → half output at full-scale error
//                             Kp=10 → full output at full-scale error
// Input is normalised to [-1,+1] via inputMax before gains are applied.
// Output is scaled to real units via outMax after clamping to [-1,+1].
// Derivative is on measurement to avoid kick on setpoint changes.
// ────────────────────────────────────────────────────────────────
#define PID_GAIN_SCALE 10.0f

struct PIDController {
  float kp, ki, kd;
  float inputMax;
  float outMax;

  float *input;
  float *setpoint;
  float *output;

  float integrator;
  float lastInput;
  float filteredDTerm;
  float dAlpha;
  bool firstRun;

  PIDController(float *in, float *sp, float *out,
                float Kp, float Ki, float Kd,
                float inMax, float maxOut,
                float filterHz = 20.0f)
    : kp(Kp), ki(Ki), kd(Kd), inputMax(inMax), outMax(maxOut),
      input(in), setpoint(sp), output(out),
      integrator(0.0f), lastInput(0.0f),
      filteredDTerm(0.0f), dAlpha(0.39f),
      firstRun(true) {
    setDFilter(filterHz);
  }

  void setGains(float Kp, float Ki, float Kd) { kp = Kp; ki = Ki; kd = Kd; }

  void setDFilter(float cutoffHz) {
    const float dt = LOOP_INTERVAL_MS / 1000.0f;
    float rc = 1.0f / (2.0f * M_PI * cutoffHz);
    dAlpha = dt / (rc + dt);
  }

  void reset() {
    integrator = 0.0f;
    filteredDTerm = 0.0f;
    firstRun = true;
  }

  void compute(float dt) {
    if (dt < 0.0001f) dt = 0.005f;

    float normIn = *input / inputMax;
    float normSp = *setpoint / inputMax;
    float error = normSp - normIn;

    float pTerm = (kp / PID_GAIN_SCALE) * error;

    // Derivative on measurement with low-pass filter
    float dTerm = 0.0f;
    if (!firstRun) {
      float rawDTerm = -(kd / PID_GAIN_SCALE) * (normIn - lastInput) / dt;
      filteredDTerm = dAlpha * rawDTerm + (1.0f - dAlpha) * filteredDTerm;
      dTerm = filteredDTerm;
    }
    firstRun = false;
    lastInput = normIn;

    // Anti-windup — conditional integration
    float preIntegrator = integrator;
    integrator += (ki / PID_GAIN_SCALE) * error * dt;
    integrator = fmaxf(-1.0f, fminf(1.0f, integrator));

    float unsaturated = pTerm + integrator + dTerm;
    if ((unsaturated >  1.0f && error > 0.0f) ||
        (unsaturated < -1.0f && error < 0.0f)) {
      integrator = preIntegrator;
    }

    float normalised = fmaxf(-1.0f, fminf(1.0f, pTerm + integrator + dTerm));
    *output = normalised * outMax;
  }
};

float rollSetpoint = 0, rollInput, rollOutput;
float pitchSetpoint = 0, pitchInput, pitchOutput;
float yawRateSetpoint = 0, yawRateInput, yawRateOutput;
float altSetpoint = 0.5f;
float altInput, altOutput;

// Gains on 0-10 scale: 5 = half output at full-scale error, 10 = full output
// inputMax: physical range of input  |  outMax: output in DSHOT units
PIDController pidRoll (&rollInput, &rollSetpoint, &rollOutput, 1.5f, 0.0f, 0.0f, ROLL_MAX_MIN, 500.0f, 20.0f);
PIDController pidPitch (&pitchInput, &pitchSetpoint, &pitchOutput, 1.5f, 0.0f, 0.0f, PITCH_MAX_MIN, 500.0f, 20.0f);
PIDController pidYawRate (&yawRateInput, &yawRateSetpoint, &yawRateOutput, 1.5f, 0.0f, 0.0f, YAW_RATE_MAX_MIN, 360.0f, 20.0f);
PIDController pidAlt (&altInput, &altSetpoint, &altOutput, 2.0f, 0.15f, 1.0f, 2.0f, 300.0f, 10.0f);
float originalAltKi = pidAlt.ki;

// ────────────────────────────────────────────────────────────────
// 1D KALMAN FILTER
// Tracks angle + gyro bias per axis independently.
// ────────────────────────────────────────────────────────────────

struct KalmanFilter {
  // State
  float angle = 0.0f; // estimated angle (degrees)
  float bias = 0.0f; // estimated gyro bias (deg/s)

  // Error covariance matrix (2x2 stored as 4 floats)
  float P00 = 1.0f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 1.0f;

  // Tuning — adjust these to change filter behaviour
  float Q_angle = 0.001f; // process noise — gyro model trust
                          // higher = follows gyro faster, more noise
  float Q_bias  = 0.003f; // process noise — how fast bias can change
                          // higher = bias corrects faster
  float R_measure = 0.03f; // measurement noise — accelerometer trust
                           // higher = smoother but slower accel correction

  // Call every loop iteration.
  // gyroRate: raw gyro reading for this axis in deg/s (with hard offset applied)
  // accelAngle: angle estimated from accelerometer in degrees
  // dt: loop time in seconds
  // Returns: filtered angle estimate
  float update(float gyroRate, float accelAngle, float dt) {
    // ── PREDICTION STEP ─────────────────────────────────────────
    // Project state forward using gyro (corrected for estimated bias)
    float rate = gyroRate - bias;
    angle += rate * dt;

    // Project error covariance forward
    // P = A*P*A^T + Q
    // A = [[1, -dt], [0, 1]]
    P00 += dt * (dt*P11 - P01 - P10 + Q_angle);
    P01 -= dt * P11;
    P10 -= dt * P11;
    P11 += Q_bias * dt;

    // ── UPDATE STEP ──────────────────────────────────────────────
    // Innovation — difference between accel measurement and prediction
    float innovation = accelAngle - angle;

    // Innovation covariance
    float S = P00 + R_measure;

    // Kalman gain — optimal blending coefficient (recomputed every step)
    float K0 = P00 / S; // gain for angle correction
    float K1 = P10 / S; // gain for bias correction

    // Update state estimate
    angle += K0 * innovation;
    bias += K1 * innovation;

    // Update error covariance
    float P00_saved = P00;
    float P01_saved = P01;
    P00 -= K0 * P00_saved;
    P01 -= K0 * P01_saved;
    P10 -= K1 * P00_saved;
    P11 -= K1 * P01_saved;

    return angle;
  }

  // Seed the filter with a known starting angle (call at boot and on disarm)
  void setAngle(float a) { angle = a; }
};

// ── Three filter instances — one per axis ────────────────────────
KalmanFilter kalmanRoll;
KalmanFilter kalmanPitch;
KalmanFilter kalmanYaw;

// Global dt (updated every loop iteration)
float global_dt = 0.0f;

// ── FORWARD DECLARATIONS ─────────────────────────────────────────
void handleRoot();
void handleData();
void calibrateBarometer();
BootOrientation checkBootOrientation(float &initialPitch, float &initialRoll);
void reEstimateAttitude(int numSamples = 20);
float lipoCellVoltageToSoc(float cellVolts);
float adcToVbat(float adcRaw);
void buzzerTone(int freq, int duration_ms);
void buzzerAlert(int count, int duration_ms, int freq = 1200, int pause_ms = 150, uint8_t priority = 0);
void buzzerAlertContinuous(int freq = 800, int duration_ms = 200, int pause_ms = 100, uint8_t priority = 0);
void buzzerStop();
void updateBuzzer();
void stopMotors();
void dshotWriteAll(uint16_t m1v, uint16_t m2v, uint16_t m3v, uint16_t m4v);
void dshotSendValue(rmt_channel_t ch, uint16_t value, bool requestTelemetry);
uint8_t dshotCRC(uint16_t value);
void ensureGPS5Hz();
void applyGPS5Hz();

// ────────────────────────────────────────────────────────────────
// SETUP
// ────────────────────────────────────────────────────────────────
// One-time hardware and software initialisation sequence.
// Order is significant — sensors must be ready before calibration,
// DSHOT must be initialised before battery state (buzzer used in both),
// and HTTP/tasks must start last since they depend on all prior state.
// ────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(2000);

  // I2C on custom pins
  Wire.begin(I2C_SDA, I2C_SCL);

  // iBUS receiver on UART1
  iBusSerial.begin(IBUS_BAUD, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);

  initPins();
  kalmanYaw.R_measure = 0.1f; // trust magnetometer heading moderately
  kalmanYaw.Q_angle = 0.001f; // gyro model trust
  kalmanYaw.Q_bias = 0.003f; // bias correction rate
  kalmanYaw.setAngle(0.0f); // will be updated after first mag read

  initSensors();
  calibrateAllSensors();
  initDshot();
  initBatteryState();
  initPIDs();
  initHTTPTelemetry();

  Serial.println("Quad ready. Connect to WiFi: QuadTelemetry / flysafe123");
  Serial.println("Open browser at IP shown below for live telemetry.");
  Serial.println("Bind FS-i6. CH6 = altitude hold.");
  delay(2000);

  unsigned long startTime = millis();
  lastValidSignalTime = startTime;
  prevTime = startTime;
}

// ────────────────────────────────────────────────────────────────
// MAIN LOOP
// ────────────────────────────────────────────────────────────────
// Main flight loop running on Core 1 at LOOP_INTERVAL_MS cadence.
// When disarmed: updates receiver, failsafe, battery, buzzer, LEDs
// and continually re-estimates attitude for clean filter state on
// next arm. When armed: takes dataMutex, reads sensors, runs attitude
// and altitude fusion, runs PIDs, releases mutex, then mixes and
// writes motors. Elapsed time is measured before sensor reads so
// sensor latency cancels out of dt calculation exactly.
// ────────────────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();
  unsigned long elapsed = now - prevTime;
  if (elapsed < LOOP_INTERVAL_MS) return;
  prevTime += LOOP_INTERVAL_MS;

  readReceiver();
  checkReceiverFailsafe();
  updateArmingAndHoldMode();
  checkBatteryVoltage();
  updateBuzzer();

  if (!armed) {
    static unsigned long lastReEstimate = 0;
    if (millis() - lastReEstimate >= 500) {
      reEstimateAttitude();
      lastReEstimate = millis();
    }
    stopMotors();
    updateLEDs();
    return;
  }

  // elapsed is the true loop period — sensor read time cancels out exactly
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  readSensors();
  global_dt = elapsed / 1000.0f;

  rollSetpoint = rcRoll + ROLL_TRIM;
  pitchSetpoint = -rcPitch + PITCH_TRIM; // negative: forward stick = nose down - PITCH_TRIM corrects fixed battery offset
  yawRateSetpoint = rcYawRate;

  float gyroComp = rcYawRate * PROP_GYRO_COEFF;
  rollSetpoint += gyroComp;
  pitchSetpoint += gyroComp;

  if (controlledDescentActive) {
    if (!altitudeHoldEnabled) {
      pidAlt.ki = originalAltKi;
      pidAlt.reset();
      altSetpoint = height;
      altitudeHoldEnabled = true;
    }
    altSetpoint -= descentRate * (global_dt > 0.0f ? global_dt : 0.005f);
    if (altSetpoint < 0.0f) altSetpoint = 0.0f;
  } else if (!altitudeHoldEnabled) {
    const float track_alpha = 0.01f;
    altSetpoint = track_alpha * height + (1.0f - track_alpha) * altSetpoint;
  }

  float cr, cp, sr, sp;
  fuseAttitude(global_dt, cr, cp, sr, sp);
  fuseAltitude(global_dt, cr, cp, sr, sp);
  runPIDs();
  // Bias altOutput upward proportional to yaw rate magnitude
  if (altitudeHoldEnabled) altOutput += fabsf(rcYawRate) * YAW_THROTTLE_COEFF;
  xSemaphoreGive(dataMutex);

  mixAndWriteMotors();
  updateLEDs();
  debugOutput();
}

// ────────────────────────────────────────────────────────────────
// INITIALIZATION FUNCTIONS
// ────────────────────────────────────────────────────────────────

// ────────────────────────────────────────────────────────────────
// initPins()
// Configures all GPIO pins to their correct modes and safe initial
// states. Sets buzzer and all LEDs LOW. Configures ADC for 12-bit
// resolution with 11dB attenuation (0-3.3V input range) on battery
// and current sense pins. Must be called before any ADC reads or
// peripheral output.
// ────────────────────────────────────────────────────────────────
void initPins() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(LED_ARMED, OUTPUT);
  pinMode(LED_LOWBAT, OUTPUT);
  pinMode(LED_ALTHOLD, OUTPUT);
  pinMode(LED_FAILSAFE, OUTPUT);
  digitalWrite(LED_ARMED, LOW);
  digitalWrite(LED_LOWBAT, LOW);
  digitalWrite(LED_ALTHOLD, LOW);
  digitalWrite(LED_FAILSAFE, LOW);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(BAT_ADC_PIN, INPUT);
  pinMode(CURR_ADC_PIN, INPUT);
}

// ────────────────────────────────────────────────────────────────
// initSensors()
// Initialises all I2C and UART sensors in sequence:
//   MPU6050 — 8G accel range, 500 deg/s gyro, 44Hz DLPF
//   HMC5883L — default settings via Adafruit driver
//   GPS NEO-6M — UART2 at GPS_BAUD, then rate configured via ensureGPS5Hz()
//   BMP280 — normal mode, x16 pressure oversample, x16 IIR filter, 63ms standby
// I2C clock raised to 400kHz after all sensors are initialised.
// Halts boot with Serial error if any sensor fails to respond.
// ────────────────────────────────────────────────────────────────
void initSensors() {
  if (!mpu.begin()) {
    Serial.println("MPU6050 failed!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!mag.begin()) {
    Serial.println("HMC5883L failed!");
    while (1) delay(10);
  }
  Serial.println("HMC5883L ready.");

  // GPS on UART2
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(500);
  ensureGPS5Hz();
  Serial.println("GPS UART2 ready — waiting for fix...");

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 failed!");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_63);

  Wire.setClock(400000);
}

// ────────────────────────────────────────────────────────────────
// ensureGPS5Hz()
// Checks current GPS measurement rate by sending a UBX-CFG-RATE poll
// request and parsing the response with an explicit state machine.
// Flushes the GPS UART buffer before polling to avoid parsing through
// NMEA sentence noise. If measRate is already 200ms (5Hz) no action
// is taken — avoids unnecessary flash writes. If rate differs or poll
// times out after 500ms, calls applyGPS5Hz() to configure and save.
// Called once during initSensors() after GPS UART is initialised.
// ────────────────────────────────────────────────────────────────
void ensureGPS5Hz() {
  // Flush incoming buffer before polling — avoids parsing through NMEA noise
  delay(20);
  while (gpsSerial.available()) gpsSerial.read();

  // Poll current CFG-RATE
  uint8_t poll[] = {
    0xB5, 0x62,
    0x06, 0x08,
    0x00, 0x00,
    0x0E, 0x30
  };
  for (int i = 0; i < (int)sizeof(poll); i++) gpsSerial.write(poll[i]);

  // Parse response using explicit state machine
  enum State { SYNC1, SYNC2, CLASS, ID, LEN_L, LEN_H, PAYLOAD, DONE };
  State state = SYNC1;

  uint8_t payload[6];
  int payloadIdx = 0;
  int payloadLen = 0;

  unsigned long timeout = millis() + 500;

  while (millis() < timeout && state != DONE) {
    if (!gpsSerial.available()) continue;
    uint8_t b = gpsSerial.read();

    switch (state) {
      case SYNC1:  if (b == 0xB5) state = SYNC2; break;
      case SYNC2:  state = (b == 0x62) ? CLASS : SYNC1; break;
      case CLASS:  state = (b == 0x06) ? ID    : SYNC1; break;
      case ID:     state = (b == 0x08) ? LEN_L : SYNC1; break;
      case LEN_L:  payloadLen = b; state = LEN_H; break;
      case LEN_H:
        payloadLen |= (b << 8);
        payloadIdx = 0;
        state = (payloadLen == 6) ? PAYLOAD : SYNC1;
        break;
      case PAYLOAD:
        if (payloadIdx < 6) payload[payloadIdx++] = b;
        if (payloadIdx >= 6) state = DONE;
        break;
      default: break;
    }
  }

  if (state != DONE) {
    Serial.println("GPS rate poll timeout — applying config anyway");
    applyGPS5Hz();
    return;
  }

  uint16_t measRate = payload[0] | (payload[1] << 8);
  Serial.printf("GPS current rate: %dms (%dHz)\n",
                measRate, measRate > 0 ? 1000 / measRate : 0);

  if (measRate == 200) {
    Serial.println("GPS already at 5Hz — no update needed");
    return;
  }

  Serial.println("GPS rate incorrect — updating to 5Hz and saving");
  applyGPS5Hz();
}

// ────────────────────────────────────────────────────────────────
// applyGPS5Hz()
// Sends UBX-CFG-RATE command to set GPS measurement rate to 200ms
// (5Hz). Waits up to 300ms for UBX-ACK-ACK (05 01) or UBX-ACK-NAK
// (05 00) response. On NAK or timeout: logs error and returns without
// saving. On ACK: sends UBX-CFG-CFG save command to write the new
// rate to NEO-6M non-volatile flash so it persists across power cycles.
// Flash save is conditional on confirmed ACK to avoid unnecessary
// wear. After the first successful save ensureGPS5Hz() will read
// 200ms and skip this function entirely on subsequent boots.
// ────────────────────────────────────────────────────────────────
void applyGPS5Hz() {
  uint8_t cfg[] = {
    0xB5, 0x62,
    0x06, 0x08,
    0x06, 0x00,
    0xC8, 0x00,
    0x01, 0x00,
    0x01, 0x00,
    0xDE, 0x6A
  };
  for (int i = 0; i < (int)sizeof(cfg); i++) gpsSerial.write(cfg[i]);

  // Wait for ACK-ACK (05 01) or ACK-NAK (05 00)
  unsigned long t = millis() + 300;
  bool ackFound = false;
  uint8_t prev = 0, curr = 0;
  while (millis() < t) {
    if (gpsSerial.available()) {
      prev = curr;
      curr = gpsSerial.read();
      if (prev == 0x05 && curr == 0x01) { ackFound = true; break; }
      if (prev == 0x05 && curr == 0x00) {
        Serial.println("GPS CFG-RATE NAK — module rejected config");
        return;
      }
    }
  }
  if (!ackFound) {
    Serial.println("GPS CFG-RATE ACK timeout — config may not have applied");
    return;
  }

  // Save to flash only if config was accepted
  uint8_t save[] = {
    0xB5, 0x62,
    0x06, 0x09,
    0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x17,
    0x31, 0xBF
  };
  for (int i = 0; i < (int)sizeof(save); i++) gpsSerial.write(save[i]);
  delay(100);

  Serial.println("GPS updated to 5Hz and saved to flash");
}

// ────────────────────────────────────────────────────────────────
// initBatteryState()
// Measures resting battery voltage at boot using 32 averaged ADC
// reads for stability. Auto-detects cell count by comparing against
// CELL_DETECT_THRESHOLD (13.5V): above = 4S, below = 3S. Derives
// initial state of charge from per-cell voltage via lipoCellVoltageToSoc()
// and sets remainingMah accordingly. Prints full battery report to Serial.
// Must be called after initPins() (ADC) and after adcToVbat() is valid
// (i.e. VDIV_A/B/C calibrated).
// ────────────────────────────────────────────────────────────────
void initBatteryState() {
  // Sample resting voltage — 32 averaged reads for stability
  long sum = 0;
  for (int i = 0; i < 32; i++) {
    int s = 0;
    for (int j = 0; j < 4; j++) s += analogRead(BAT_ADC_PIN);
    sum += s / 4;
    delay(5);
  }
  float restingVbat = adcToVbat(sum / 32.0f);

  // Auto-detect cell count from resting voltage
  float batteryCapacityMah;
  if (restingVbat >= CELL_DETECT_THRESHOLD) {
    batteryCells = CELL_COUNT_4S;
    batteryCapacityMah = BATTERY_CAPACITY_MAH_4S;
    Serial.println("Battery detected: 4S");
  } else {
    batteryCells = CELL_COUNT_3S;
    batteryCapacityMah = BATTERY_CAPACITY_MAH_3S;
    Serial.println("Battery detected: 3S");
  }

  float cellVolts = restingVbat / (float)batteryCells;
  float soc = lipoCellVoltageToSoc(cellVolts);

  consumedMah = 0.0f;
  remainingMah = batteryCapacityMah * soc;

  Serial.printf("Battery init — Resting: %.2fV  Cells: %dS  Cell: %.2fV  "
                "SoC: %.0f%%  Capacity: %.0f mAh  Starting: %.0f mAh\n",
                restingVbat, batteryCells, cellVolts,
                soc * 100.0f, batteryCapacityMah, remainingMah);
}

// ────────────────────────────────────────────────────────────────
// initDshot()
// Initialises all four RMT TX channels for DSHOT300 output at
// DSHOT_RMT_CLK_DIV=1 (80MHz, 12.5ns per tick). Sends DSHOT_CMD_STOP
// (value 0) for 1000ms to satisfy Bluejay v0.19.2 arming requirement.
// Beeps buzzer at 1800Hz on completion. Must be called before any
// motor commands are issued.
// ────────────────────────────────────────────────────────────────
void initDshot() {
  // Initialise TX channels (DSHOT output)
  initDshotTxChannel(RMT_CH_M1, ESC1_PIN);
  initDshotTxChannel(RMT_CH_M2, ESC2_PIN);
  initDshotTxChannel(RMT_CH_M3, ESC3_PIN);
  initDshotTxChannel(RMT_CH_M4, ESC4_PIN);

  // Arm ESCs — send stop command for 300ms minimum (Bluejay requirement)
  Serial.print("Arming ESCs via DSHOT...");
  unsigned long armStart = millis();
  while (millis() - armStart < 1000) {
    dshotWriteAll(DSHOT_CMD_STOP, DSHOT_CMD_STOP,
                  DSHOT_CMD_STOP, DSHOT_CMD_STOP);
    delay(10);
  }
  Serial.println(" done.");
  buzzerTone(1800, 200);
}

// ────────────────────────────────────────────────────────────────
// initPIDs()
// Resets all four PID controller integrators and firstRun flags to
// their initial states. Ensures clean integrator state on first arm
// regardless of boot sequence timing.
// ────────────────────────────────────────────────────────────────
void initPIDs() {
  pidRoll.reset();
  pidPitch.reset();
  pidYawRate.reset();
  pidAlt.reset();
}

// ────────────────────────────────────────────────────────────────
// initHTTPTelemetry()
// Starts WiFi in soft-AP mode with WIFI_SSID/WIFI_PASSWORD credentials.
// Registers HTTP routes: / (dashboard), /data (JSON telemetry),
// /debug/on, /debug/off, /debug (runtime debug toggle).
// Creates three FreeRTOS mutexes: dataMutex, rcMutex, gpsMutex.
// Spawns three pinned tasks on Core 0:
//   iBUSTask  — parses iBUS frames from UART1, writes shared RC values under rcMutex
//   GPSTask   — calls processGPS() at 20Hz to drain GPS UART and update globals
//   WiFiTask  — calls server.handleClient() continuously
// Must be called after all hardware is initialised.
// ────────────────────────────────────────────────────────────────
void initHTTPTelemetry() {
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 1, 0, 1); // channel 1, visible, max 1 client
  WiFi.setTxPower(WIFI_POWER_2dBm); // minimum power — reduces interference radius ~20x
  IPAddress IP = WiFi.softAPIP();

  Serial.println("WiFi AP started");
  Serial.print("SSID: "); Serial.println(WIFI_SSID);
  Serial.print("Password: "); Serial.println(WIFI_PASSWORD);
  Serial.print("Open browser at: http://"); Serial.println(IP);

  server.on("/", handleRoot);
  server.on("/data", handleData);

  // Debug toggle endpoints
  server.on("/debug/on", []() {
    debugOutputEnabled = true;
    server.send(200, "application/json", "{\"debug\":true}");
    Serial.println("Debug output ENABLED via HTTP");
  });
  server.on("/debug/off", []() {
    debugOutputEnabled = false;
    server.send(200, "application/json", "{\"debug\":false}");
    Serial.println("Debug output DISABLED via HTTP");
  });
  server.on("/debug", []() {
    char buf[32];
    snprintf(buf, sizeof(buf), "{\"debug\":%s}", debugOutputEnabled ? "true" : "false");
    server.send(200, "application/json", buf);
  });
  server.on("/status", []() {
    char buf[32];
    snprintf(buf, sizeof(buf), "{\"armed\":%s}", armed ? "true" : "false");
    server.send(200, "application/json", buf);
  });
  server.on("/pid/get", []() {
    char buf[384];
    auto calcHz = [](float alpha) {
      return 1.0f / (2.0f * M_PI * (1.0f / alpha - 1.0f) * LOOP_INTERVAL_MS / 1000.0f);
    };
    snprintf(buf, sizeof(buf),
      "{\"rollKp\":%.2f,\"rollKi\":%.2f,\"rollKd\":%.2f,\"rollDHz\":%.1f,"
      "\"pitchKp\":%.2f,\"pitchKi\":%.2f,\"pitchKd\":%.2f,\"pitchDHz\":%.1f,"
      "\"yawKp\":%.2f,\"yawKi\":%.2f,\"yawKd\":%.2f,\"yawDHz\":%.1f,"
      "\"altKp\":%.2f,\"altKi\":%.2f,\"altKd\":%.2f,\"altDHz\":%.1f}",
      pidRoll.kp,    pidRoll.ki,    pidRoll.kd,    calcHz(pidRoll.dAlpha),
      pidPitch.kp,   pidPitch.ki,   pidPitch.kd,   calcHz(pidPitch.dAlpha),
      pidYawRate.kp, pidYawRate.ki, pidYawRate.kd, calcHz(pidYawRate.dAlpha),
      pidAlt.kp,     pidAlt.ki,     pidAlt.kd,     calcHz(pidAlt.dAlpha));
    server.send(200, "application/json", buf);
  });
  server.on("/pid/set", []() {
    if (armed) {
      server.send(403, "application/json", "{\"error\":\"disarm first\"}");
      return;
    }
    auto getArg = [&](const char* name, float fallback) {
      return server.hasArg(name) ? server.arg(name).toFloat() : fallback;
    };
    pidRoll.setGains(
      getArg("rollKp",  pidRoll.kp),
      getArg("rollKi",  pidRoll.ki),
      getArg("rollKd",  pidRoll.kd));
    pidPitch.setGains(
      getArg("pitchKp", pidPitch.kp),
      getArg("pitchKi", pidPitch.ki),
      getArg("pitchKd", pidPitch.kd));
    pidYawRate.setGains(
      getArg("yawKp",   pidYawRate.kp),
      getArg("yawKi",   pidYawRate.ki),
      getArg("yawKd",   pidYawRate.kd));
    pidAlt.setGains(
      getArg("altKp",   pidAlt.kp),
      getArg("altKi",   pidAlt.ki),
      getArg("altKd",   pidAlt.kd));
    pidRoll.setDFilter(   getArg("rollDHz",  20.0f));
    pidPitch.setDFilter(  getArg("pitchDHz", 20.0f));
    pidYawRate.setDFilter(getArg("yawDHz",   20.0f));
    pidAlt.setDFilter(    getArg("altDHz",   10.0f));
    originalAltKi = pidAlt.ki;
    pidRoll.reset();
    pidPitch.reset();
    pidYawRate.reset();
    pidAlt.reset();
    Serial.printf("PID updated — Roll:%.2f/%.2f/%.2f  Pitch:%.2f/%.2f/%.2f  "
                  "Yaw:%.2f/%.2f/%.2f  Alt:%.2f/%.2f/%.2f\n",
                  pidRoll.kp,    pidRoll.ki,    pidRoll.kd,
                  pidPitch.kp,   pidPitch.ki,   pidPitch.kd,
                  pidYawRate.kp, pidYawRate.ki, pidYawRate.kd,
                  pidAlt.kp,     pidAlt.ki,     pidAlt.kd);
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.begin();
  Serial.println("HTTP telemetry server started.");

  dataMutex = xSemaphoreCreateMutex();
  rcMutex = xSemaphoreCreateMutex();
  gpsMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    [](void*) {
      uint16_t channels[14];
      while (1) {
        if (parseIBusPacket(iBusSerial, channels)) {
          xSemaphoreTake(rcMutex, portMAX_DELAY);
          sharedPwmRoll = channels[0];
          sharedPwmPitch = channels[1];
          sharedPwmThrottle = channels[2];
          sharedPwmYaw = channels[3];
          sharedPwmArm = channels[4];
          sharedPwmAux = channels[5];
          sharedPwmBatOverride = channels[BAT_OVERRIDE_CHANNEL];
          sharedPwmDescent = channels[DESCENT_CHANNEL];
          sharedLastValidSignal = millis();
          xSemaphoreGive(rcMutex);
        }
        vTaskDelay(1);
      }
    },
    "iBUSTask", 3072, nullptr, 2, nullptr, 0
  );

  xTaskCreatePinnedToCore(
    [](void*) {
      while (1) {
        processGPS();
        vTaskDelay(50); // 20Hz polling — sufficient margin over 5Hz GPS update rate
      }
    },
    "GPSTask", 4096, nullptr, 1, nullptr, 0
  );

  xTaskCreatePinnedToCore(
    [](void*) {
      while (1) {
        server.handleClient();
        vTaskDelay(10); // 100Hz service rate — more than sufficient
      }
    },
    "WiFiTask", 10240, nullptr, 1, nullptr, 0
  );
}

// ────────────────────────────────────────────────────────────────
// calibrateAllSensors()
// Applies hard-coded IMU offsets from compile-time defines to runtime
// offset variables. Calls checkBootOrientation() to measure initial
// tilt and seeds Kalman filters with the result — eliminates the
// ~1.25s convergence delay that would otherwise occur if the filter
// started at 0°. Calls calibrateBarometer() to measure ground-level
// pressure reference. Warns via Serial if orientation is not level.
// ────────────────────────────────────────────────────────────────
void calibrateAllSensors() {
  // Apply hard offsets — no runtime measurement needed
  gyroOffsetX = GYRO_OFFSET_X;
  gyroOffsetY = GYRO_OFFSET_Y;
  gyroOffsetZ = GYRO_OFFSET_Z;
  accelOffsetX = ACCEL_OFFSET_X;
  accelOffsetY = ACCEL_OFFSET_Y;
  accelOffsetZ = ACCEL_OFFSET_Z;

  Serial.println("Hard IMU offsets applied.");
  Serial.printf("Gyro offsets: X=%.6f Y=%.6f Z=%.6f rad/s\n",
                gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.printf("Accel offsets: X=%.6f Y=%.6f Z=%.6f m/s²\n",
                accelOffsetX, accelOffsetY, accelOffsetZ);

  // Check orientation and seed filter with measured angles
  float initialPitch = 0.0f;
  float initialRoll = 0.0f;
  BootOrientation orientation = checkBootOrientation(initialPitch, initialRoll);

  // Seed complementary filter — eliminates ~1.25s convergence delay at boot
  pitch = initialPitch;
  roll = initialRoll;
  kalmanRoll.setAngle(initialRoll);
  kalmanPitch.setAngle(initialPitch);
  kalmanYaw.setAngle(0.0f);
  Serial.printf("Filter seeded — Pitch: %.2f° Roll: %.2f°\n", pitch, roll);

  // Barometer always calibrated at runtime — pressure varies with weather
  calibrateBarometer();
  Serial.printf("Baro offset: %.2f m\n", baroOffset);

  if (orientation != BootOrientation::LEVEL) {
    Serial.println("WARNING: Drone not level at boot — arming blocked until level.");
  }
}

// ────────────────────────────────────────────────────────────────
// CALIBRATION FUNCTIONS
// ────────────────────────────────────────────────────────────────

// ────────────────────────────────────────────────────────────────
// calibrateBarometer()
// Averages 100 barometer altitude readings at 10ms intervals to
// establish baroOffset — the ground-level pressure altitude at boot
// location. All subsequent baroAlt readings subtract this offset so
// height is relative to launch point. Beeps at 1800Hz on completion.
// Pressure varies with weather so this must run at runtime, not
// compile time.
// ────────────────────────────────────────────────────────────────
void calibrateBarometer() {
  const int numSamples = 100;
  float sumAlt = 0.0f;

  Serial.print("Calibrating barometer... ");

  for (int i = 0; i < numSamples; i++) {
    float alt = bmp.readAltitude(seaLevel);
    sumAlt += alt;
    delay(10);
  }

  baroOffset = sumAlt / numSamples;
  buzzerTone(1800, 200);
}

// ────────────────────────────────────────────────────────────────
// checkBootOrientation()
// Averages 100 accelerometer readings to determine drone orientation
// at boot. Classifies as LEVEL (<15°), TILTED (15-60°), ON_SIDE (>60°),
// or INVERTED (accelZ < 0). Sets orientationClearToFly = true only
// for LEVEL. Seeds initialPitch and initialRoll output parameters
// for use by calibrateAllSensors() to prime the Kalman filters.
// Emits distinct buzzer patterns per orientation state.
// Returns BootOrientation enum value.
// ────────────────────────────────────────────────────────────────
BootOrientation checkBootOrientation(float &initialPitch, float &initialRoll) {
  const int numSamples = 100;
  float sumAx = 0, sumAy = 0, sumAz = 0;

  Serial.println("Checking boot orientation...");

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumAx += a.acceleration.x - accelOffsetX;
    sumAy += a.acceleration.y - accelOffsetY;
    sumAz += a.acceleration.z - accelOffsetZ;
    delay(5);
  }

  float ax = sumAx / numSamples;
  float ay = sumAy / numSamples;
  float az = sumAz / numSamples;

  // Same formula as readSensors() — consistent with filter
  initialPitch = atan2f(ay, az) * GYRO_SCALE;
  initialRoll = -atan2f(ax, sqrtf(ay * ay + az * az)) * GYRO_SCALE;
  float tiltAngle = sqrtf(initialPitch * initialPitch + initialRoll * initialRoll);

  Serial.printf("Boot orientation — Pitch: %.2f° Roll: %.2f° Tilt: %.2f°\n",
                initialPitch, initialRoll, tiltAngle);

  BootOrientation result;

  if (az < 0.0f) {
    result = BootOrientation::INVERTED;
    Serial.println("ORIENTATION: INVERTED — flip drone right side up before arming");
  } else if (tiltAngle > 60.0f) {
    result = BootOrientation::ON_SIDE;
    Serial.printf("ORIENTATION: ON SIDE (%.1f°) — place drone flat before arming\n",
                  tiltAngle);
  } else if (tiltAngle > 15.0f) {
    result = BootOrientation::TILTED;
    Serial.printf("ORIENTATION: TILTED (%.1f°) — level the drone before arming\n",
                  tiltAngle);
  } else {
    result = BootOrientation::LEVEL;
    Serial.printf("ORIENTATION: LEVEL (%.1f°) — OK to fly\n", tiltAngle);
    orientationClearToFly = true;
  }

  switch (result) {
    case BootOrientation::LEVEL:
      buzzerTone(2000, 150);
      delay(200);
      buzzerTone(2000, 150);
      break;
    case BootOrientation::TILTED:
      for (int i=0;i<3;i++) {
        buzzerTone(1200, 200);
        delay(250);
      }
      break;
    case BootOrientation::ON_SIDE:
      for (int i=0;i<4;i++) {
        buzzerTone(800, 300);
        delay(350);
      }
      break;
    case BootOrientation::INVERTED:
      buzzerTone(600, 1500); delay(1600); break;
  }

  return result;
}

// ────────────────────────────────────────────────────────────────
// reEstimateAttitude()
// Re-measures attitude from accelerometer using 20 averaged samples
// and reseeds the roll and pitch Kalman filters. Called on every
// disarm event and periodically while disarmed (500ms interval) to
// keep filter state consistent with physical reality before next arm.
// Skips reseeding if tilt > 60° — a crashed or sideways drone would
// seed an invalid angle and destabilise the next flight. Does not
// reseed yaw — magnetometer heading reference must be preserved.
// ────────────────────────────────────────────────────────────────
void reEstimateAttitude(int numSamples) {
  float sumAx = 0, sumAy = 0, sumAz = 0;

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumAx += a.acceleration.x - accelOffsetX;
    sumAy += a.acceleration.y - accelOffsetY;
    sumAz += a.acceleration.z - accelOffsetZ;
  }

  float ax = sumAx / numSamples;
  float ay = sumAy / numSamples;
  float az = sumAz / numSamples;

  // Same formula as readSensors() and checkBootOrientation()
  float newPitch = atan2f(ay, az) * GYRO_SCALE;
  float newRoll = -atan2f(ax, sqrtf(ay * ay + az * az)) * GYRO_SCALE;

  // Only reseed if drone is reasonably level — if it disarmed mid-flip
  // or landed on its side the accel reading is valid but seeding a
  // 90° angle into the filter would cause problems on next arm
  float tilt = sqrtf(newPitch * newPitch + newRoll * newRoll);
  if (tilt < 60.0f) {
    pitch = newPitch;
    roll = newRoll;
    kalmanRoll.setAngle(roll);
    kalmanPitch.setAngle(pitch);
    // do not reseed yaw — heading reference would be lost
    Serial.printf("Attitude re-estimated — Pitch: %.2f° Roll: %.2f°\n", pitch, roll);
  } else {
    Serial.printf("Disarm re-estimate skipped — tilt too large (%.1f°) [Pitch: %.2f° Roll: %.2f°]\n", tilt, pitch, roll);
  }
}

// ────────────────────────────────────────────────────────────────
// CORE FLIGHT FUNCTIONS
// ────────────────────────────────────────────────────────────────

// ────────────────────────────────────────────────────────────────
// readReceiver()
// Copies shared iBUS channel values into local variables under rcMutex
// to prevent race with Core 0 iBUSTask. During receiver failsafe,
// freezes all stick inputs to safe defaults (roll/pitch/yaw = 0,
// throttle = 500) and returns early — altitude hold PID then has
// full symmetric authority around the fixed 50% base throttle.
// Outside failsafe, converts raw PWM values (1000-2000) to physical
// units:
//   rcRoll/rcPitch  — degrees, constrained to ±angleLimit
//   rcYawRate       — deg/s, negated so stick-right = nose-right
//   rcThrottle      — 0-1000 units
//   batCutoffOverrideActive — true when pwmBatOverride > 1500
//   controlledDescentActive / descentRate — set from 3-position
//     CH8 switch: low=off, mid=0.3 m/s, high=0.6 m/s
// angleLimit narrows to 25° in altitude hold to preserve thrust
// headroom and altitude estimate reliability.
// ────────────────────────────────────────────────────────────────
void readReceiver() {
  xSemaphoreTake(rcMutex, portMAX_DELAY);
  pwmRoll = sharedPwmRoll;
  pwmPitch = sharedPwmPitch;
  pwmThrottle = sharedPwmThrottle;
  pwmYaw = sharedPwmYaw;
  pwmArm = sharedPwmArm;
  pwmAux = sharedPwmAux;
  pwmBatOverride = sharedPwmBatOverride;
  pwmDescent = sharedPwmDescent;
  lastValidSignalTime = sharedLastValidSignal;
  xSemaphoreGive(rcMutex);

  if (receiverFailsafeActive) {
    rcRoll = 0.0f;
    rcPitch = 0.0f;
    rcYawRate = 0.0f;
    rcThrottle = 500.0f;
    return;
  }

  batCutoffOverrideActive = (pwmBatOverride > 1500);

  if (pwmDescent < 1200) {
    controlledDescentActive = false;
    descentRate = 0.0f;
  } else if (pwmDescent < 1700) {
    controlledDescentActive = true;
    descentRate = 0.3f; // mid — slow descent
  } else {
    controlledDescentActive = true;
    descentRate = 0.6f; // high — faster descent
  }

  // Tighter angle limit in altitude hold — preserves thrust headroom and
  // altitude estimate reliability. cos(25°)=0.906 vs cos(45°)=0.707.
  float angleLimit = altitudeHoldEnabled ? 25.0f : ROLL_MAX_MIN;

  rcRoll = constrain((pwmRoll - 1500) / 500.0f * angleLimit, -angleLimit, angleLimit);
  rcPitch = constrain((pwmPitch - 1500) / 500.0f * angleLimit, -angleLimit, angleLimit);
  rcYawRate = constrain((pwmYaw - 1500) / 500.0f * -YAW_RATE_MAX_MIN, -YAW_RATE_MAX_MIN, YAW_RATE_MAX_MIN); // negated — stick right = nose right
  rcThrottle = constrain((pwmThrottle - 1000) / 1000.0f * 1000.0f, 0.0f, 1000.0f);
}

// ────────────────────────────────────────────────────────────────
// readSensors()
// Reads one sample from MPU6050 and BMP280. Applies hard-iron gyro
// and accelerometer offsets. Computes centripetal acceleration
// correction for IMU-to-CoM offset (currently zero — update offsetX/Y/Z
// if IMU is not at centre of mass). Derives accelPitch and accelRoll
// using atan2 for use as Kalman measurement inputs. Reads barometer
// altitude and subtracts baroOffset to give height above launch point.
// Must be called inside dataMutex.
// ────────────────────────────────────────────────────────────────
void readSensors() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Body frame: X = right, Y = forward, Z = up
  // Right-hand rule conventions:
  // Positive pitch = nose up (rotation about +X)
  // Positive roll = right side down (rotation about +Y)
  // Positive yaw = right to left (rotation about +Z)

  gyroPitchRate = g.gyro.x - gyroOffsetX; // RHR about +X: +ve = nose up
  gyroRollRate = g.gyro.y - gyroOffsetY; // RHR about +Y: +ve = right side down
  gyroYawRate = g.gyro.z - gyroOffsetZ; // RHR about +Z: +ve = right to left

  accelX = a.acceleration.x - accelOffsetX; // -g at 90° right-side-down
  accelY = a.acceleration.y - accelOffsetY; // +g at 90° nose-up
  accelZ = a.acceleration.z - accelOffsetZ; // +g when flat

  const float offsetX = (0.0f / 1000.0f); // 0 mm right CoM
  const float offsetY = (0.0f / 1000.0f); // 0 mm forward CoM
  const float offsetZ = (0.0f / 1000.0f); // 0 mm above CoM

  float correctedAccelX = accelX + (gyroRollRate * gyroRollRate * offsetX);
  float correctedAccelY = accelY + (gyroPitchRate * gyroPitchRate * offsetY);
  float correctedAccelZ = accelZ + (gyroPitchRate * gyroPitchRate * offsetZ
                                  + gyroRollRate * gyroRollRate * offsetZ);

  // +ve = nose up: atan2(+g, 0) = +90° at 90° nose-up ✓
  accelPitch = atan2f(correctedAccelY, correctedAccelZ) * GYRO_SCALE;

  // +ve = right side down: -atan2(-g, 0) = +90° at 90° right-side-down ✓
  accelRoll = -atan2f(correctedAccelX,
                sqrtf(correctedAccelY * correctedAccelY
                    + correctedAccelZ * correctedAccelZ)) * GYRO_SCALE;

  accelX = correctedAccelX;
  accelY = correctedAccelY;
  accelZ = correctedAccelZ;

  static unsigned long lastBaroRead = 0;
  unsigned long nowBaro = millis();
  if (nowBaro - lastBaroRead >= 64) {
    baroAlt = bmp.readAltitude(seaLevel) - baroOffset;
    temperature = bmp.readTemperature();
    lastBaroRead = nowBaro;
  }
}

// ────────────────────────────────────────────────────────────────
// readMagnetometer()
// Reads HMC5883L and applies hard-iron calibration offsets
// (MAG_OFFSET_X/Y/Z). Performs tilt compensation using current
// Kalman roll and pitch estimates to rotate the magnetic field
// vector into the horizontal plane before computing heading.
// Applies MAG_DECLINATION correction and wraps result to 0-360°.
// Sets magValid = true on every successful read. Called every loop
// iteration from fuseAttitude() — cheap at 400kHz I2C.
// ────────────────────────────────────────────────────────────────
void readMagnetometer() {
  sensors_event_t event;
  mag.getEvent(&event);

  // Apply hard iron calibration offsets
  float mx = event.magnetic.x - MAG_OFFSET_X;
  float my = event.magnetic.y - MAG_OFFSET_Y;
  float mz = event.magnetic.z - MAG_OFFSET_Z;

  // Tilt compensation using current roll and pitch from Kalman filter
  // This corrects for the magnetometer being tilted with the drone
  float rollRad = roll * DEG_TO_RAD;
  float pitchRad = pitch * DEG_TO_RAD;

  float cr = cosf(rollRad);
  float sr = sinf(rollRad);
  float cp = cosf(pitchRad);
  float sp = sinf(pitchRad);

  // Rotate magnetic field vector into horizontal plane
  float Xh = mx * cp + my * sr * sp + mz * cr * sp;
  float Yh = my * cr - mz * sr;

  // Compute heading
  float heading = atan2f(-Yh, Xh) * GYRO_SCALE; // degrees

  // Apply declination correction
  heading += MAG_DECLINATION;

  // Wrap to 0-360
  if (heading < 0.0f) heading += 360.0f;
  if (heading > 360.0f) heading -= 360.0f;

  magHeading = heading;
  magValid = true;
}

// ────────────────────────────────────────────────────────────────
// processGPS()
// Drains all available bytes from GPS UART into TinyGPS++ parser.
// Copies valid fix data (position, altitude, speed, course, satellites,
// age) into GPS globals under gpsMutex to prevent race with handleData()
// on Core 1. Sets gpsFixed = false if location is not valid.
// Called by GPSTask on Core 0 at 20Hz — sufficient margin over the
// 5Hz GPS update rate configured via ensureGPS5Hz().
// ────────────────────────────────────────────────────────────────
void processGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  xSemaphoreTake(gpsMutex, portMAX_DELAY);
  if (gps.location.isValid()) {
    gpsFixed = true;
    gpsLat = (float)gps.location.lat();
    gpsLng = (float)gps.location.lng();
    gpsAge = gps.location.age();
  } else {
    gpsFixed = false;
  }

  if (gps.altitude.isValid())
    gpsAltitude = (float)gps.altitude.meters();

  if (gps.speed.isValid())
    gpsSpeed = (float)gps.speed.kmph();

  if (gps.course.isValid())
    gpsCourse = (float)gps.course.deg();

  if (gps.satellites.isValid())
    gpsSatellites = (uint8_t)gps.satellites.value();
  xSemaphoreGive(gpsMutex);
}

// ────────────────────────────────────────────────────────────────
// FLIGHT LOGIC FUNCTIONS
// ────────────────────────────────────────────────────────────────

// ────────────────────────────────────────────────────────────────
// checkReceiverFailsafe()
// Monitors time since last valid iBUS packet against FAILSAFE_TIMEOUT_MS.
// Uses confirmation counters (8 loops to trigger, 10 loops to clear)
// to prevent false triggers from single missed packets.
// On confirmed timeout:
//   - Sets receiverFailsafeActive and records failsafeHoverStart
//   - Enables altitude hold at current height (with ramp) so the
//     drone hovers in place rather than dropping
//   - After FAILSAFE_HOVER_SEC seconds, triggers controlled descent
//     at 0.3 m/s (or 0.6 m/s if low battery warning is active)
//   - Fires 4-beep alert at priority 1
// On confirmed signal recovery:
//   - Clears receiverFailsafeActive, controlledDescentActive,
//     failsafeHoverStart, and descentRate
//   - Disengages altitude hold via ramp transition back to manual
//     throttle so the pilot regains control smoothly
// Must be called every loop before arming logic.
// ────────────────────────────────────────────────────────────────
void checkReceiverFailsafe() {
  static int timeoutCounter = 0;
  static int goodSignalCounter = 0;
  const int TIMEOUT_CONFIRM_LOOPS = 8;
  const int GOOD_CONFIRM_LOOPS = 10;

  unsigned long now = millis();

  if (now - lastValidSignalTime > FAILSAFE_TIMEOUT_MS) {
    if (timeoutCounter < TIMEOUT_CONFIRM_LOOPS) timeoutCounter++;
    goodSignalCounter = 0;

    if (timeoutCounter >= TIMEOUT_CONFIRM_LOOPS && !receiverFailsafeActive) {
      receiverFailsafeActive = true;
      failsafeHoverStart = millis();
      Serial.println("FAILSAFE: Signal lost — hovering in place");

      if (!altitudeHoldEnabled) {
        pidAlt.ki = originalAltKi;
        pidAlt.reset();
        altSetpoint = height;
        altitudeHoldEnabled = true;
        altHoldEngaging = true;
        altHoldThrottleRamp = 0.0f;
        altHoldTransitionStart = millis();
      }

      buzzerAlert(4, 250, 1200, 150, 1);
    }

    if (receiverFailsafeActive && !controlledDescentActive) {
      if (millis() - failsafeHoverStart >= FAILSAFE_HOVER_SEC * 1000UL) {
        controlledDescentActive = true;
        descentRate = lowBatWarning ? 0.6f : 0.3f;
        Serial.printf("FAILSAFE: Hover timeout — descending at %.1f m/s\n", descentRate);
      }
    }

  } else {
    timeoutCounter = 0;
    if (goodSignalCounter < GOOD_CONFIRM_LOOPS) goodSignalCounter++;

    if (goodSignalCounter >= GOOD_CONFIRM_LOOPS && receiverFailsafeActive) {
      receiverFailsafeActive = false;
      controlledDescentActive = false;
      descentRate = 0.0f;
      failsafeHoverStart = 0;
      altitudeHoldEnabled = false;
      altHoldDisengaging = true;
      altHoldEngaging = false;
      altHoldThrottleRamp = 1.0f;
      altHoldTransitionStart = millis();
      pidAlt.ki = 0.0f;
      Serial.println("Signal recovered — failsafe cleared");
    }
  }
}

// ────────────────────────────────────────────────────────────────
// lipoCellVoltageToSoc()
// Converts a per-cell resting voltage (V) to state of charge (0.0-1.0)
// using piecewise linear interpolation of a standard LiPo discharge
// curve. Clamps to table bounds at 4.20V (100%) and 3.10V (0%).
// Input must be per-cell voltage — divide pack voltage by batteryCells
// before calling. Valid for 3S and 4S packs using the same curve.
// Returns float in range [0.0, 1.0].
// ────────────────────────────────────────────────────────────────
float lipoCellVoltageToSoc(float cellVolts) {
  // Piecewise linear interpolation of standard LiPo discharge curve
  // Points: {cell_voltage, soc_fraction}
  static const float curve[][2] = {
    {4.20f, 1.00f},
    {4.10f, 0.90f},
    {4.00f, 0.80f},
    {3.90f, 0.65f},
    {3.80f, 0.50f},
    {3.70f, 0.35f},
    {3.60f, 0.20f},
    {3.50f, 0.10f},
    {3.40f, 0.05f},
    {3.10f, 0.00f},
  };
  const int n = sizeof(curve) / sizeof(curve[0]);

  // Clamp to table bounds
  if (cellVolts >= curve[0][0]) return curve[0][1];
  if (cellVolts <= curve[n-1][0]) return curve[n-1][1];

  // Find bracketing segment and interpolate
  for (int i = 0; i < n - 1; i++) {
    if (cellVolts <= curve[i][0] && cellVolts > curve[i+1][0]) {
      float t = (cellVolts - curve[i+1][0]) / (curve[i][0] - curve[i+1][0]);
      return curve[i+1][1] + t * (curve[i][1] - curve[i+1][1]);
    }
  }
  return 0.0f;
}

// ────────────────────────────────────────────────────────────────
// adcToVbat()
// Converts raw 12-bit ADC count to battery pack voltage using a
// quadratic polynomial calibrated to compensate for both the voltage
// divider ratio and ESP32 ADC non-linearity. Input adcRaw is first
// converted to ADC input voltage (0-3.3V) then evaluated through:
//   vbat = VDIV_A * v² + VDIV_B * v + VDIV_C
// Coefficients must be derived from polynomial regression against
// measured voltage pairs using the standalone ADC calibration sketch.
// Valid across the full 3S (9-12.6V) and 4S (12-16.8V) range when
// calibrated with a 47kΩ + 10kΩ voltage divider.
// ────────────────────────────────────────────────────────────────
float adcToVbat(float adcRaw) {
  float v = adcRaw * (3.3f / 4095.0f); // convert to ADC input voltage first
  return VDIV_A * v * v + VDIV_B * v + VDIV_C;
}

// ────────────────────────────────────────────────────────────────
// checkBatteryVoltage()
// Non-blocking battery monitoring called every loop iteration.
// Accumulates 16 ADC samples at 2ms intervals for both voltage and
// current sense channels, then on each complete batch:
//   - Converts averaged ADC to vbat via adcToVbat()
//   - Computes currentAmps, powerWatts, consumedMah, remainingMah
//   - Estimates remaining flight time at current draw
// Implements three-tier battery protection:
//   WARNING  — fires 2-beep alert once at CELL_VOLT_WARNING (3.4V/cell)
//   CUTOFF   — disarms, stops motors, continuous 800Hz alert at CELL_VOLT_CUTOFF (3.1V/cell)
//              suppressed if batCutoffOverrideActive — fires rapid 600Hz alert instead
//   RECOVERY — clears cutoff latch if voltage recovers above cutoff + 0.1V/cell
// overrideBuzzerActive static prevents the override buzzer from being
// restarted on every sample batch while voltage remains below cutoff.
// ────────────────────────────────────────────────────────────────
void checkBatteryVoltage() {
  static unsigned long lastSampleTime = 0;
  static long sum = 0;
  static long currSum = 0;
  static int sampleCount = 0;
  static bool overrideBuzzerActive = false;
  const int samples = 16;

  unsigned long now = millis();

  if (now - lastSampleTime >= 2) {
    lastSampleTime += 2; // phase-locked to 2ms grid
    int s = 0;
    for (int i = 0; i < 4; i++) s += analogRead(BAT_ADC_PIN);
    sum += s / 4;

    int cs = 0;
    for (int i = 0; i < 4; i++) cs += analogRead(CURR_ADC_PIN);
    currSum += cs / 4;

    sampleCount++;
  }

  if (sampleCount >= samples) {
    vbat = adcToVbat(sum / (float)samples);

    float currVoltage = (currSum / (float)samples) * (3.3f / 4095.0f);
    currentAmps = constrain(currVoltage * CURR_SCALE - CURR_OFFSET_A,
                            0.0f, 200.0f);
    powerWatts = vbat * currentAmps;

    const float dt_hours = (2.0f * samples) / 3600000.0f;
    consumedMah += currentAmps * 1000.0f * dt_hours;
    static float remainingMahAtBoot = -1.0f;
    if (remainingMahAtBoot < 0.0f) remainingMahAtBoot = remainingMah;
    remainingMah = constrain(remainingMahAtBoot - consumedMah,
                             0.0f, remainingMahAtBoot);

    if (currentAmps >= CURR_ESTIMATE_THRESHOLD_A) {
      flightTimeSec = (remainingMah / 1000.0f) * 3600.0f / currentAmps;
    } else {
      flightTimeSec = -1.0f;
    }

    sum = 0; currSum = 0; sampleCount = 0;

    // Dynamic thresholds from detected cell count
    float cutoffVoltage = CELL_VOLT_CUTOFF * (float)batteryCells;
    float warningVoltage = CELL_VOLT_WARNING * (float)batteryCells;
    float recoverVoltage = (CELL_VOLT_CUTOFF + 0.1f) * (float)batteryCells;

    if (vbat < warningVoltage && !lowBatWarning) {
      lowBatWarning = true;
      Serial.printf("LOW BATTERY WARNING! VBat = %.2fV (%dS)\n",
                    vbat, batteryCells);
      buzzerAlert(2, 200, 1000, 150, 2);
    } else if (vbat >= warningVoltage) {
      lowBatWarning = false;
    }

    if (vbat < cutoffVoltage && !lowBatCutoff) {
      if (batCutoffOverrideActive) {
        if (!overrideBuzzerActive) {
          Serial.printf("LOW VOLTAGE OVERRIDE ACTIVE! VBat = %.2fV (%dS) - "
                        "Cutoff suppressed\n", vbat, batteryCells);
          buzzerAlertContinuous(600, 100, 100, 2);
          overrideBuzzerActive = true;
        }
      } else {
        if (overrideBuzzerActive) {
          // Override was just switched off — stop the override beep
          buzzerStop();
          overrideBuzzerActive = false;
        }
        lowBatCutoff = true;
        armed = false;
        altitudeHoldEnabled = false;
        stopMotors();
        reEstimateAttitude();
        Serial.printf("LOW VOLTAGE CUTOFF! VBat = %.2fV (%dS) - LATCHED\n",
                      vbat, batteryCells);
        buzzerStop();
        buzzerAlertContinuous(800, 250, 100, 2);
      }
    } else if (lowBatCutoff && vbat > recoverVoltage) {
      lowBatCutoff = false;
      lowBatWarning = false;
      overrideBuzzerActive = false;
      Serial.printf("Battery recovered: VBat = %.2fV - cutoff cleared\n", vbat);
      buzzerStop();
    }
  }
}

// ────────────────────────────────────────────────────────────────
// updateArmingAndHoldMode()
// Manages arming state machine, altitude hold toggle, and throttle
// ramp transitions each loop.
//
// Altitude hold engagement (CH6/pwmAux) is blocked if height is
// below ALT_HOLD_MIN_HEIGHT_M (5m) or rcThrottle is below
// ALT_HOLD_MIN_THROTTLE (100) — prevents engaging hold on the
// ground or at near-zero throttle. On valid engage: resets pidAlt,
// sets altSetpoint from current height, starts ramp-in transition.
// On disengage: starts ramp-out transition back to manual throttle.
// Ramp state (altHoldEngaging/Disengaging, altHoldThrottleRamp) is
// advanced each loop over ALT_HOLD_RAMP_SEC (0.5s).
//
// Controlled descent cleanup: if CH8 descent is released and CH6
// hold is not active, altitude hold is disabled.
//
// Arming conditions: arm switch high, throttle < 200, no battery
// cutoff (or cutoff overridden), orientation clear to fly.
// forceDisarm fires on arm switch low UNLESS receiver failsafe is
// active (failsafe hover must not be interrupted by arm switch state)
// or on battery cutoff without override. On disarm: resets all PID
// integrators, clears ramp state, stops motors, re-estimates attitude.
//
// Orientation re-check runs at 100ms intervals while
// orientationClearToFly is false, allowing correction without reboot.
// ────────────────────────────────────────────────────────────────
void updateArmingAndHoldMode() {
  if (!controlledDescentActive && altitudeHoldEnabled && !(pwmAux > 1500)) {
    pidAlt.ki = 0.0f;
    Serial.println("Alt hold cleanup — zeroing Ki, ramp-out will follow");
  }

  bool auxHigh = (pwmAux > 1500);
  bool armSwitchHigh = (pwmArm > 1500);
  bool throttleSafe = (rcThrottle < 200);

  // Re-check orientation each loop until cleared —
  // allows correction after boot without requiring reboot
  if (!orientationClearToFly) {
    static unsigned long lastOrientCheck = 0;
    unsigned long now_oc = millis();

    if (now_oc - lastOrientCheck >= 100) {
      lastOrientCheck = now_oc;

      const int numSamples = 5;
      float sumAx = 0, sumAy = 0, sumAz = 0;

      for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sumAx += a.acceleration.x - accelOffsetX;
        sumAy += a.acceleration.y - accelOffsetY;
        sumAz += a.acceleration.z - accelOffsetZ;
      }

      float ax = sumAx / numSamples;
      float ay = sumAy / numSamples;
      float az = sumAz / numSamples;

      float detectedPitch = atan2f(ay, az) * GYRO_SCALE;
      float detectedRoll = -atan2f(ax, sqrtf(ay * ay + az * az)) * GYRO_SCALE;
      float tiltAngle = sqrtf(detectedPitch * detectedPitch + detectedRoll * detectedRoll);

      if (az > 0.0f && tiltAngle <= 15.0f) {
        orientationClearToFly = true;
        pitch = detectedPitch;
        roll = detectedRoll;
        Serial.println("Orientation now level — arming enabled");
        buzzerTone(2000, 150);
        delay(200);
        buzzerTone(2000, 150);
      }
    }
  }

  // Altitude hold toggle (CH6 — independent of arming)
  if (auxHigh != altitudeHoldEnabled) {
    if (auxHigh) {
      if (height < ALT_HOLD_MIN_HEIGHT_M) {
        Serial.printf("Altitude hold BLOCKED — height %.2fm below %.0fm minimum\n",
                      height, ALT_HOLD_MIN_HEIGHT_M);
        buzzerAlert(2, 100, 800, 100, 0);
      } else if (rcThrottle < ALT_HOLD_MIN_THROTTLE) {
        Serial.println("Altitude hold BLOCKED — throttle too low");
        buzzerAlert(2, 100, 800, 100, 0);
      } else {
        pidAlt.ki = originalAltKi;
        pidAlt.reset();
        altSetpoint = height;
        altHoldEngaging = true;
        altHoldDisengaging = false;
        altHoldThrottleRamp = 0.0f;
        altHoldTransitionStart = millis();
        altitudeHoldEnabled = true;
        Serial.println("Altitude hold ENABLED — ramping in");
        buzzerAlert(1, 200, 1400, 50, 0);
      }
    } else {
      pidAlt.ki = 0.0f;
      altHoldDisengaging = true;
      altHoldEngaging = false;
      altHoldThrottleRamp = 1.0f;
      altHoldTransitionStart = millis();
      altitudeHoldEnabled = false;
      Serial.println("Altitude hold DISABLED — throttle stick now active, centre before disengaging");
    }
  }

  // Advance ramp
  unsigned long rampMs = (unsigned long)(ALT_HOLD_RAMP_SEC * 1000.0f);
  if (altHoldEngaging) {
    altHoldThrottleRamp = constrain(
      (float)(millis() - altHoldTransitionStart) / (float)rampMs,
      0.0f, 1.0f);
    if (altHoldThrottleRamp >= 1.0f) {
      altHoldEngaging = false;
      Serial.println("Altitude hold ramp complete");
    }
  }
  if (altHoldDisengaging) {
    altHoldThrottleRamp = constrain(
      1.0f - (float)(millis() - altHoldTransitionStart) / (float)rampMs,
      0.0f, 1.0f);
    if (altHoldThrottleRamp <= 0.0f) {
      altHoldDisengaging = false;
      Serial.println("Altitude hold disengage ramp complete");
    }
  }

  // Arming — requires orientation clear, throttle low, no battery cutoff
  bool canArm = armSwitchHigh && throttleSafe
              && (!lowBatCutoff || batCutoffOverrideActive)
              && orientationClearToFly;
  bool forceDisarm = (!armSwitchHigh && !receiverFailsafeActive)
                      || (lowBatCutoff && !batCutoffOverrideActive);

  if (canArm && !armed) {
    armed = true;
    Serial.println("ARMED - Motors active");
    buzzerAlert(1, 300, 1200, 50, 0);
    digitalWrite(LED_ARMED, HIGH);
  }

  if (forceDisarm && armed) {
    armed = false;
    altitudeHoldEnabled = false;
    altHoldEngaging = false;
    altHoldDisengaging = false;
    altHoldThrottleRamp = 0.0f;
    pidAlt.ki = 0.0f;
    pidRoll.reset();
    pidPitch.reset();
    pidYawRate.reset();
    pidAlt.reset();
    stopMotors();
    reEstimateAttitude();
    Serial.println("DISARMED - Motors stopped");
    buzzerAlert(1, 500, 600, 50, 0);
    digitalWrite(LED_ARMED, LOW);
  }
}

// ────────────────────────────────────────────────────────────────
// fuseAttitude()
// Fuses gyro and accelerometer through independent 1D Kalman filters
// for roll and pitch. Reads magnetometer and fuses heading into yaw
// Kalman filter with heading unwrapping to handle 359°/0° discontinuity.
// When magValid is false, yaw filter runs gyro-only (prediction step
// only, no measurement update). Exposes raw gyroYawDeg as yawRate for
// the yaw rate PID — the Kalman yaw angle is used only for telemetry
// and heading reference. Outputs cr, cp, sr, sp (cos/sin of roll and
// pitch) for use by fuseAltitude().
// ────────────────────────────────────────────────────────────────
void fuseAttitude(float dt, float &cr_out, float &cp_out, float &sr_out, float &sp_out) {
  float gyroRollDeg = gyroRollRate * GYRO_SCALE;
  float gyroPitchDeg = gyroPitchRate * GYRO_SCALE;
  float gyroYawDeg = gyroYawRate * GYRO_SCALE;

  // Roll and pitch — Kalman fuses gyro + accelerometer
  roll = kalmanRoll.update(gyroRollDeg, accelRoll, dt);
  pitch = kalmanPitch.update(gyroPitchDeg, accelPitch, dt);

  // Read magnetometer every iteration — cheap I2C read at 400kHz
  static unsigned long lastMagRead = 0;
  unsigned long nowMag = millis();
  if (nowMag - lastMagRead >= 50) { // 20Hz — above HMC5883L 15Hz output
    readMagnetometer();
    lastMagRead = nowMag;
  }

  // Yaw — Kalman fuses gyro + magnetometer heading
  // gyroYawDeg is the rate — Kalman integrates it and corrects with mag
  if (magValid) {
    // Unwrap magnetometer heading to avoid discontinuity at 0/360
    float currentAngle = kalmanYaw.angle;
    float rawHeading = magHeading;

    // Bring rawHeading within 180 degrees of current estimate
    // to handle the 359->0 wraparound
    while (rawHeading - currentAngle > 180.0f) rawHeading -= 360.0f;
    while (rawHeading - currentAngle < -180.0f) rawHeading += 360.0f;

    kalmanYaw.update(gyroYawDeg, rawHeading, dt);
  } else {
    // No valid mag reading — gyro integration only this iteration
    kalmanYaw.update(gyroYawDeg, kalmanYaw.angle, dt);
  }

  // Expose yaw for PID and telemetry
  yawRate = gyroYawDeg; // raw rate for yaw rate PID — unchanged

  // Trig for altitude fusion
  float cr = cosf(roll * DEG_TO_RAD);
  float cp = cosf(pitch * DEG_TO_RAD);
  float sr = sinf(roll * DEG_TO_RAD);
  float sp = sinf(pitch * DEG_TO_RAD);
  cr_out = cr; cp_out = cp; sr_out = sr; sp_out = sp;
}

// ────────────────────────────────────────────────────────────────
// fuseAltitude()
// Estimates height and vertical velocity by fusing accelerometer
// vertical acceleration with barometer altitude. Rotates body-frame
// accelerometer readings into world-frame vertical using the rotation
// matrix world-up row [-sr*cp, sp, cr*cp] derived from current
// Kalman roll and pitch. Subtracts gravity to get net vertical
// acceleration. Fuses with barometer using complementary filter:
//   velocity — weighted blend of integrated accel and baro innovation
//   height   — weighted blend of integrated velocity and raw baro
// ALT_INNOV_WEIGHT and ALT_BARO_WEIGHT are tunable. Reduce
// ALT_INNOV_WEIGHT to 0.02 if altitude bounces on first flight.
// ────────────────────────────────────────────────────────────────
void fuseAltitude(float dt, float cr, float cp, float sr, float sp) {
  // R = Rx(pitch) · Ry(roll)
  // World-up row of R: [-sr*cp,  sp,  cr*cp]
  //
  // Verified:
  // Flat stationary:          0       + 0   + g    - g = 0 ✓
  // 90° roll right-side-down: -(-g)*1 + 0   + 0    - g = 0 ✓
  // 90° nose-up:              0       + g*1 + 0    - g = 0 ✓
  // Flat +1m/s² up:           0       + 0   +(g+1) - g = 1 ✓

  float R_world_up[3] = {
    -sr * cp, // body X component of world-up
     sp,      // body Y component of world-up
     cr * cp  // body Z component of world-up
  };

  float worldZ = R_world_up[0] * accelX
               + R_world_up[1] * accelY
               + R_world_up[2] * accelZ
               - GRAVITY_CON;

  float innovation = baroAlt - height;
  velocity = (1.0f - ALT_INNOV_WEIGHT) * (velocity + worldZ * dt) + ALT_INNOV_WEIGHT * (innovation / dt);
  height = (1.0f - ALT_BARO_WEIGHT) * (height + velocity * dt) + ALT_BARO_WEIGHT * baroAlt;
}

// ────────────────────────────────────────────────────────────────
// runPIDs()
// Assigns sensor readings to PID inputs and calls compute() on all
// four controllers with the current global_dt. Altitude deadband
// suppresses correction within ALT_DEADBAND_M (±10cm) by presenting
// the setpoint as the input — producing zero error — rather than
// zeroing the output directly, which preserves integrator state.
// Altitude PID Ki is set to zero outside altitude hold mode via
// updateArmingAndHoldMode() — the compute() call is still made but
// the integrator does not accumulate.
// ────────────────────────────────────────────────────────────────
void runPIDs() {
  rollInput = roll;
  pitchInput = pitch;
  yawRateInput = yawRate;

  // Altitude deadband — suppress correction for small barometer noise
  float altError = altSetpoint - height;
  altInput = (fabsf(altError) < ALT_DEADBAND_M)
             ? altSetpoint // error appears zero to PID
             : height;

  float dt = global_dt;

  if (dt < 0.0001f) dt = 0.005f;
  pidRoll.compute(dt);
  pidPitch.compute(dt);
  pidYawRate.compute(dt);
  pidAlt.compute(dt);
}

// ────────────────────────────────────────────────────────────────
// iBUS IMPLEMENTATION
// ────────────────────────────────────────────────────────────────

// ────────────────────────────────────────────────────────────────
// parseIBusPacket()
// Parses one complete iBUS frame from the provided HardwareSerial.
// Searches for header bytes 0x20 0x40, reads 32-byte packet, validates
// checksum (0xFFFF minus sum of bytes 0-29), extracts 14 channels as
// little-endian uint16 and clamps each to [1000, 2000]. Returns true
// if a valid packet was found and channels[] was populated.
// Discards bytes one at a time until a valid header is found.
// Called by iBUSTask on Core 0 at maximum parse rate.
// Packet format:
// Byte 0: 0x20 (length)
// Byte 1: 0x40 (protocol ID)
// Bytes 2-29: 14 channels as uint16 little-endian (1000-2000 range)
// Bytes 30-31: checksum = 0xFFFF - sum(bytes 0-29)
// ────────────────────────────────────────────────────────────────
static bool parseIBusPacket(HardwareSerial &serial, uint16_t channels[14]) {
  if (serial.available() < IBUS_PACKET_LEN) return false;

  uint8_t buf[IBUS_PACKET_LEN];

  // Search for header bytes 0x20 0x40
  // Discard bytes until header found or buffer exhausted
  while (serial.available() >= IBUS_PACKET_LEN) {
    if (serial.peek() == IBUS_HEADER1) {
      serial.readBytes(buf, IBUS_PACKET_LEN);

      if (buf[0] != IBUS_HEADER1 || buf[1] != IBUS_HEADER2) continue;

      // Validate checksum
      uint16_t checksum = 0xFFFF;
      for (int i = 0; i < 30; i++) checksum -= buf[i];

      uint16_t rxChecksum = buf[30] | (buf[31] << 8);
      if (checksum != rxChecksum) continue;

      // Extract 14 channels (bytes 2-29, 2 bytes each, little-endian)
      for (int i = 0; i < IBUS_NUM_CHANNELS; i++) {
        channels[i] = buf[2 + i*2] | (buf[3 + i*2] << 8);
        // Clamp to valid RC range
        channels[i] = constrain(channels[i], 1000, 2000);
      }
      return true;
    } else {
      serial.read(); // discard byte and keep searching
    }
  }
  return false;
}

// ────────────────────────────────────────────────────────────────
// DSHOT IMPLEMENTATION
// ────────────────────────────────────────────────────────────────

// ────────────────────────────────────────────────────────────────
// buildDshotPacket()
// Encodes a DSHOT value into 17 RMT items in the provided buffer.
// Packet structure: 11-bit throttle value shifted left by 1, telemetry
// bit inserted at bit 0, 4-bit CRC appended (XOR of three nibbles).
// Each bit encoded as high/low pulse pair using T1H/T1L or T0H/T0L
// tick counts for DSHOT300 at 80MHz RMT clock. Item 17 is the
// inter-frame pause (DSHOT_PAUSE_TICKS, ~33µs). Buffer must be
// at least 17 rmt_item32_t elements.
// ────────────────────────────────────────────────────────────────
static void buildDshotPacket(rmt_item32_t *items, uint16_t value, bool requestTelemetry) {
  uint16_t packet = (value << 1) | (requestTelemetry ? 1 : 0);
  uint8_t crc = dshotCRC(packet);
  packet = (packet << 4) | crc;

  for (int i = 0; i < 16; i++) {
    bool bit = (packet >> (15 - i)) & 0x01;
    if (bit) {
      items[i].duration0 = DSHOT_T1H_TICKS;
      items[i].level0 = 1;
      items[i].duration1 = DSHOT_T1L_TICKS;
      items[i].level1 = 0;
    } else {
      items[i].duration0 = DSHOT_T0H_TICKS;
      items[i].level0 = 1;
      items[i].duration1 = DSHOT_T0L_TICKS;
      items[i].level1 = 0;
    }
  }
  // Pause item — line stays low for inter-frame gap
  items[16].duration0 = DSHOT_PAUSE_TICKS;
  items[16].level0 = 0;
  items[16].duration1 = 0;
  items[16].level1 = 0;
}

// ────────────────────────────────────────────────────────────────
// dshotCRC()
// Computes the 4-bit DSHOT CRC by XORing the three 4-bit nibbles
// of the 12-bit packet value (bits 15-4, which already includes the
// telemetry bit at position 0 shifted in by the caller). Returns
// the lower 4 bits of the XOR result.
// ────────────────────────────────────────────────────────────────
uint8_t dshotCRC(uint16_t value) {
  // CRC is XOR of three 4-bit nibbles of the 12-bit value (bits 15-4 of packet)
  // Here value already includes the telemetry bit shifted in
  return ((value ^ (value >> 4) ^ (value >> 8)) & 0x0F);
}

// ────────────────────────────────────────────────────────────────
// initDshotTxChannel()
// Configures one RMT peripheral channel for DSHOT TX output on the
// specified GPIO pin. Uses RMT_DEFAULT_CONFIG_TX with clock divider
// DSHOT_RMT_CLK_DIV=1 (80MHz, 12.5ns per tick). Installs RMT driver
// with no RX buffer. Must be called once per motor channel before
// any dshotSendValue() calls on that channel.
// ────────────────────────────────────────────────────────────────
static void initDshotTxChannel(rmt_channel_t ch, int pin) {
  rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX((gpio_num_t)pin, ch);
  cfg.clk_div = DSHOT_RMT_CLK_DIV;
  rmt_config(&cfg);
  rmt_driver_install(ch, 0, 0);
}

// ────────────────────────────────────────────────────────────────
// dshotSendValue()
// Sends one DSHOT packet on the specified RMT channel non-blocking.
// Clamps value to [DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX] unless
// value is 0 (stop command). Builds packet into the per-channel
// dshotItems buffer via buildDshotPacket() then calls rmt_write_items()
// with wait_tx_done=false so the function returns immediately while
// the RMT hardware transmits the frame autonomously.
// ────────────────────────────────────────────────────────────────
void dshotSendValue(rmt_channel_t ch, uint16_t value, bool requestTelemetry) {
  // Clamp to valid range
  if (value != 0 && value < DSHOT_THROTTLE_MIN) value = DSHOT_THROTTLE_MIN;
  if (value > DSHOT_THROTTLE_MAX) value = DSHOT_THROTTLE_MAX;

  int chIdx = (int)ch;
  buildDshotPacket(dshotItems[chIdx], value, requestTelemetry);
  rmt_write_items(ch, dshotItems[chIdx], 17, false); // non-blocking
}

// ────────────────────────────────────────────────────────────────
// dshotWriteAll()
// Sends DSHOT values to all four motors simultaneously by calling
// dshotSendValue() on all four RMT channels back-to-back, then
// waits for all four transmissions to complete via rmt_wait_tx_done()
// with a 2ms timeout per channel. All four motors receive their
// new throttle values within one RMT frame period of each other.
// ────────────────────────────────────────────────────────────────
void dshotWriteAll(uint16_t m1v, uint16_t m2v, uint16_t m3v, uint16_t m4v) {
  dshotSendValue(RMT_CH_M1, m1v, false);
  dshotSendValue(RMT_CH_M2, m2v, false);
  dshotSendValue(RMT_CH_M3, m3v, false);
  dshotSendValue(RMT_CH_M4, m4v, false);

  rmt_wait_tx_done(RMT_CH_M1, pdMS_TO_TICKS(2));
  rmt_wait_tx_done(RMT_CH_M2, pdMS_TO_TICKS(2));
  rmt_wait_tx_done(RMT_CH_M3, pdMS_TO_TICKS(2));
  rmt_wait_tx_done(RMT_CH_M4, pdMS_TO_TICKS(2));
}

// ────────────────────────────────────────────────────────────────
// stopMotors()
// Sends DSHOT_CMD_STOP (value 0) to all four motors simultaneously.
// Value 0 is the DSHOT disarm command — ESCs respond by stopping
// the motor immediately. Called on disarm, failsafe, battery cutoff,
// and any other condition requiring immediate motor shutdown.
// ────────────────────────────────────────────────────────────────
void stopMotors() {
  dshotWriteAll(DSHOT_CMD_STOP, DSHOT_CMD_STOP,
                DSHOT_CMD_STOP, DSHOT_CMD_STOP);
}

// ────────────────────────────────────────────────────────────────
// MOTOR MIXING — DSHOT units (48-2047)
// ────────────────────────────────────────────────────────────────

// ────────────────────────────────────────────────────────────────
// mixAndWriteMotors()
// Computes per-motor DSHOT values from throttle and PID outputs
// using an X-configuration mixer:
//   M1 FR CCW: throttle - roll + pitch - yaw
//   M2 RR CW:  throttle - roll - pitch + yaw
//   M3 FL CW:  throttle + roll + pitch + yaw
//   M4 RL CCW: throttle + roll - pitch - yaw
//
// Throttle calculation has three modes:
//   Manual: rcThrottle maps to 80% of DSHOT range directly.
//   Hold/Ramp: blends between manual throttle and a fixed 50% hold
//     base using altHoldThrottleRamp (0=manual, 1=hold). PID
//     contribution is also ramped in over the same window. This
//     prevents sudden throttle jumps on altitude hold engage and
//     disengage. altHoldDisengaging uses the same blended path so
//     the pilot regains manual throttle gradually.
//   The fixed 50% hold base gives the altitude PID equal authority
//     to increase or decrease thrust regardless of stick position.
//
// All outputs clamped to [DSHOT_MIN, DSHOT_THROTTLE_MAX].
// ────────────────────────────────────────────────────────────────
void mixAndWriteMotors() {
  // Map rcThrottle (0-1000) to DSHOT range (DSHOT_MIN to DSHOT_MAX)
  float throttle;
  if (altitudeHoldEnabled || altHoldDisengaging) {
    // altOutput is in the same scale as before (offset around 1500)
    // Remap: 1000-2000 PWM → DSHOT_MIN to DSHOT_MAX
    float manualThrottle = DSHOT_MIN + (rcThrottle / 1000.0f) * (float)(DSHOT_MAX - DSHOT_MIN) * 0.8f;
    float holdThrottle = DSHOT_MIN + (float)(DSHOT_MAX - DSHOT_MIN) * 0.5f;
    float blended = manualThrottle + altHoldThrottleRamp * (holdThrottle - manualThrottle);
    float pidContribution = altHoldThrottleRamp * altOutput;
    throttle = constrain(blended + pidContribution, (float)DSHOT_MIN, (float)DSHOT_THROTTLE_MAX);
  } else {
    // rcThrottle 0-1000 → DSHOT_MIN to DSHOT_MAX*0.8 (leave headroom)
    throttle = DSHOT_MIN + (rcThrottle / 1000.0f) * (float)(DSHOT_MAX - DSHOT_MIN) * 0.8f;
  }

  float rOut = rollOutput;
  float pOut = pitchOutput;
  float yOut = yawRateOutput;

  // X configuration mixer
  float m1_raw = throttle - rOut + pOut - yOut; // FR CCW
  float m2_raw = throttle - rOut - pOut + yOut; // RR CW
  float m3_raw = throttle + rOut + pOut + yOut; // FL CW
  float m4_raw = throttle + rOut - pOut - yOut; // RL CCW

  // Clamp to valid DSHOT range — all motors share the same min
  m1 = (uint16_t)constrain((int)m1_raw, DSHOT_MIN, DSHOT_THROTTLE_MAX);
  m2 = (uint16_t)constrain((int)m2_raw, DSHOT_MIN, DSHOT_THROTTLE_MAX);
  m3 = (uint16_t)constrain((int)m3_raw, DSHOT_MIN, DSHOT_THROTTLE_MAX);
  m4 = (uint16_t)constrain((int)m4_raw, DSHOT_MIN, DSHOT_THROTTLE_MAX);

  dshotWriteAll(m1, m2, m3, m4);
}

// ────────────────────────────────────────────────────────────────
// UTILITY / OUTPUT FUNCTIONS
// ────────────────────────────────────────────────────────────────

// ────────────────────────────────────────────────────────────────
// buzzerTone()
// Produces a blocking buzzer tone at the specified frequency and
// duration using LEDC PWM at 50% duty cycle. Detaches the LEDC
// channel and drives the pin LOW after the tone to prevent residual
// DC from damaging a passive buzzer. Blocking — do not call from
// flight-critical code paths during flight. Safe to call during
// initialisation and calibration sequences.
// ────────────────────────────────────────────────────────────────
void buzzerTone(int freq, int duration_ms) {
  ledcSetup(BUZZER_LEDC_CHANNEL, freq, BUZZER_LEDC_RES);
  ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
  ledcWrite(BUZZER_LEDC_CHANNEL, 512);
  delay(duration_ms);
  ledcWrite(BUZZER_LEDC_CHANNEL, 0);
  ledcDetachPin(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
}

// ────────────────────────────────────────────────────────────────
// buzzerAlert()
// Queues a finite sequence of beeps into the BuzzerState machine.
// Non-blocking — returns immediately, updateBuzzer() drives timing.
// Priority system: lower priority alerts are ignored if a higher
// priority alert is already active. Immediately silences any current
// tone before queuing the new sequence. Parameters:
//   count       — number of beeps
//   duration_ms — on-time per beep
//   freq        — tone frequency in Hz
//   pause_ms    — off-time between beeps
//   priority    — 0=low, 1=medium, 2=high (failsafe/battery)
// ────────────────────────────────────────────────────────────────
void buzzerAlert(int count, int duration_ms, int freq, int pause_ms, uint8_t priority) {
  if (buzzer.active && priority < buzzer.priority) return;

  ledcWrite(BUZZER_LEDC_CHANNEL, 0);
  ledcDetachPin(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
  buzzer.active = true;
  buzzer.beepsRemaining = count;
  buzzer.frequency = freq;
  buzzer.duration_ms = duration_ms;
  buzzer.pause_ms = pause_ms;
  buzzer.continuous = false;
  buzzer.priority = priority;
  buzzer.isTone = false;
  buzzer.lastToggle = millis();
}

// ────────────────────────────────────────────────────────────────
// buzzerAlertContinuous()
// Queues a repeating beep pattern into the BuzzerState machine that
// continues indefinitely until buzzerStop() is called. Used for
// persistent alerts: low battery cutoff (800Hz), battery cutoff
// override (600Hz). Non-blocking. Higher priority than buzzerAlert()
// calls with lower priority values. Parameters match buzzerAlert()
// but beepsRemaining is unused — continuous flag drives repetition.
// ────────────────────────────────────────────────────────────────
void buzzerAlertContinuous(int freq, int duration_ms, int pause_ms, uint8_t priority) {
  if (buzzer.active && priority < buzzer.priority) return;

  ledcWrite(BUZZER_LEDC_CHANNEL, 0);
  ledcDetachPin(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
  buzzer.active = true;
  buzzer.beepsRemaining = 0;
  buzzer.frequency = freq;
  buzzer.duration_ms = duration_ms;
  buzzer.pause_ms = pause_ms;
  buzzer.continuous = true;
  buzzer.priority = priority;
  buzzer.isTone = false;
  buzzer.lastToggle = millis();
}

// ────────────────────────────────────────────────────────────────
// buzzerStop()
// Immediately silences the buzzer by writing 0 duty to the LEDC
// channel, detaching the pin, and driving it LOW. Resets all
// BuzzerState fields to their default inactive state including
// priority — allows any subsequent buzzerAlert() call to succeed
// regardless of the priority of the alert that was stopped.
// ────────────────────────────────────────────────────────────────
void buzzerStop() {
  ledcWrite(BUZZER_LEDC_CHANNEL, 0);
  ledcDetachPin(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
  buzzer.active = false;
  buzzer.isTone = false;
  buzzer.beepsRemaining = 0;
  buzzer.continuous = false;
  buzzer.priority = 0;
}

// ────────────────────────────────────────────────────────────────
// updateBuzzer()
// Non-blocking buzzer state machine driver, called every loop iteration.
// Manages the pause/tone cycle for both finite (buzzerAlert) and
// continuous (buzzerAlertContinuous) sequences. During pause phase:
// waits pause_ms then starts LEDC tone. During tone phase: waits
// duration_ms then stops tone. For finite sequences: decrements
// beepsRemaining and sets active=false when sequence completes.
// For continuous sequences: restarts the cycle without decrementing.
// ────────────────────────────────────────────────────────────────
void updateBuzzer() {
  if (!buzzer.active) return;

  unsigned long now = millis();
  unsigned long elapsed = now - buzzer.lastToggle;

  if (!buzzer.isTone) {
    // Waiting for next beep start (pause phase)
    if (elapsed >= (unsigned long)buzzer.pause_ms) {
      ledcSetup(BUZZER_LEDC_CHANNEL, buzzer.frequency, BUZZER_LEDC_RES);
      ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
      ledcWrite(BUZZER_LEDC_CHANNEL, 512);
      buzzer.isTone = true;
      buzzer.lastToggle = now;
    }
  } else {
    // Beep is playing
    if (elapsed >= (unsigned long)buzzer.duration_ms) {
      ledcWrite(BUZZER_LEDC_CHANNEL, 0);
      ledcDetachPin(BUZZER_PIN);
      digitalWrite(BUZZER_PIN, LOW);
      buzzer.isTone = false;
      buzzer.lastToggle = now;

      // If finite sequence, count down
      if (!buzzer.continuous && buzzer.beepsRemaining > 0) {
        buzzer.beepsRemaining--;
        if (buzzer.beepsRemaining <= 0) {
          buzzer.active = false;
        }
      }
    }
  }
}

// ────────────────────────────────────────────────────────────────
// updateLEDs()
// Updates all four status LEDs each loop iteration:
//   LED_ARMED    — solid on when armed, off when disarmed
//   LED_ALTHOLD  — solid on when altitude hold active
//   LED_FAILSAFE — fast blink (150ms) when receiver failsafe active
//   LED_LOWBAT   — solid on for cutoff, slow blink (200ms) for warning
// All blink states use static timestamps and state variables to avoid
// blocking. LED_LOWBAT solid-on during cutoff takes priority over blink.
// ────────────────────────────────────────────────────────────────
void updateLEDs() {
  unsigned long now = millis();

  digitalWrite(LED_ARMED, armed ? HIGH : LOW);
  digitalWrite(LED_ALTHOLD, altitudeHoldEnabled ? HIGH : LOW);

  static unsigned long lastFsBlink = 0;
  static bool fsBlinkState = false;
  if (receiverFailsafeActive) {
    if (now - lastFsBlink > 150) {
      fsBlinkState = !fsBlinkState;
      digitalWrite(LED_FAILSAFE, fsBlinkState);
      lastFsBlink = now;
    }
  } else {
    fsBlinkState = false;
    digitalWrite(LED_FAILSAFE, LOW);
  }

  static unsigned long lastLbBlink = 0;
  static bool lbBlinkState = false;
  if (lowBatCutoff) {
    digitalWrite(LED_LOWBAT, HIGH);
    lbBlinkState = true;
  } else if (lowBatWarning) {
    if (now - lastLbBlink > 200) {
      lbBlinkState = !lbBlinkState;
      digitalWrite(LED_LOWBAT, lbBlinkState);
      lastLbBlink = now;
    }
  } else {
    lbBlinkState = false;
    digitalWrite(LED_LOWBAT, LOW);
  }
}

// ────────────────────────────────────────────────────────────────
// debugOutput()
// Prints a compact one-line flight status string to Serial at 4Hz
// (250ms interval) when debugOutputEnabled is true. Disabled by
// default — enable via HTTP GET /debug/on to avoid Serial overhead
// affecting loop timing during flight. Output format:
//   Arm | AltHold | Failsafe | Roll | Pitch | Height | M1-M4 | VBat | dt | Hz
// ────────────────────────────────────────────────────────────────
void debugOutput() {
  if (!debugOutputEnabled) return;

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 250) return;
  lastPrint = millis();

  float loopHz = (global_dt > 0.0001f) ? (1.0f / global_dt) : 0.0f;

  Serial.printf(
    "Arm:%d Hold:%d FSafe:%d | R:%.1f P:%.1f H:%.2f | "
    "M1:%u M2:%u M3:%u M4:%u | "
    "VBat:%.2f | dt:%.1fms %.1fHz\n",
    armed, altitudeHoldEnabled, receiverFailsafeActive,
    roll, pitch, height,
    m1, m2, m3, m4,
    vbat, global_dt*1000.0f, loopHz);
}

// ────────────────────────────────────────────────────────────────
// HTTP TELEMETRY SERVER
// ────────────────────────────────────────────────────────────────

// ────────────────────────────────────────────────────────────────
// handleData()
// HTTP handler for GET /data — serialises current flight state to JSON.
// Takes rcMutex to snapshot RC values, dataMutex for flight data,
// and gpsMutex for GPS data, minimising time spent holding each lock.
// Builds JSON into a 1700-byte stack buffer via snprintf and warns
// to Serial if the buffer is insufficient. Served to the dashboard
// at ~10Hz via JavaScript setInterval. JSON fields include: attitude,
// altitude, velocity, motors, RC channels, battery (voltage, current,
// power, consumed, remaining, flight time, cells), GPS, magnetometer,
// loop timing, armed/hold/failsafe/override state flags.
// ────────────────────────────────────────────────────────────────
void handleData() {
  // ── RC snapshot — read directly from shared vars under rcMutex ──
  // Avoids race with Core 1 readReceiver() which writes pwm* outside dataMutex
  uint16_t l_pwmRoll, l_pwmPitch, l_pwmThrottle;
  uint16_t l_pwmYaw, l_pwmArm, l_pwmAux;
  uint16_t l_pwmBatOverride, l_pwmDescent;

  xSemaphoreTake(rcMutex, portMAX_DELAY);
  l_pwmRoll = sharedPwmRoll;
  l_pwmPitch = sharedPwmPitch;
  l_pwmThrottle = sharedPwmThrottle;
  l_pwmYaw = sharedPwmYaw;
  l_pwmArm = sharedPwmArm;
  l_pwmAux = sharedPwmAux;
  l_pwmBatOverride = sharedPwmBatOverride;
  l_pwmDescent = sharedPwmDescent;
  xSemaphoreGive(rcMutex);

  // ── Flight data snapshot — under dataMutex ──
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  float l_roll = roll;
  float l_pitch = pitch;
  float l_yawRate = yawRate;
  float l_height = height;
  float l_velocity = velocity;
  float l_accelX = accelX;
  float l_accelY = accelY;
  float l_accelZ = accelZ;
  float l_gyroRoll = gyroRollRate;
  float l_gyroPitch = gyroPitchRate;
  float l_vbat = vbat;
  float l_currentAmps = currentAmps;
  float l_powerWatts = powerWatts;
  float l_consumedMah = consumedMah;
  float l_remainingMah = remainingMah;
  float l_flightTimeSec = flightTimeSec;
  float l_dt = global_dt;
  bool l_armed = armed;
  bool l_altHold = altitudeHoldEnabled;
  bool l_failsafe = receiverFailsafeActive;
  bool l_lowBatCutoff = lowBatCutoff;
  bool l_lowBatWarning = lowBatWarning;
  float l_temperature = temperature;
  float l_altSetpoint = altSetpoint;
  uint16_t l_m1 = m1;
  uint16_t l_m2 = m2;
  uint16_t l_m3 = m3;
  uint16_t l_m4 = m4;
  float l_magHeading = magHeading;
  bool l_magValid = magValid;
  float l_yaw = kalmanYaw.angle;
  bool l_batOverride = batCutoffOverrideActive;
  xSemaphoreGive(dataMutex);

  xSemaphoreTake(gpsMutex, portMAX_DELAY);
  bool l_gpsFixed = gpsFixed;
  float l_gpsLat = gpsLat;
  float l_gpsLng = gpsLng;
  float l_gpsAltitude = gpsAltitude;
  float l_gpsSpeed = gpsSpeed;
  float l_gpsCourse = gpsCourse;
  uint8_t l_gpsSatellites = gpsSatellites;
  unsigned long l_gpsAge = gpsAge;
  xSemaphoreGive(gpsMutex);

  char json[1800];
  int written = snprintf(json, sizeof(json),
    "{"
    "\"timestamp\":%lu,"
    "\"rcRoll\":%u,\"rcPitch\":%u,\"rcThrottle\":%u,"
    "\"rcYaw\":%u,\"rcArm\":%u,\"rcAux\":%u,"
    "\"rcBatOverride\":%u,\"rcDescent\":%u,"
    "\"roll\":%.2f,\"pitch\":%.2f,\"yawRate\":%.2f,"
    "\"height\":%.2f,\"velocity\":%.2f,"
    "\"accelX\":%.2f,\"accelY\":%.2f,\"accelZ\":%.2f,"
    "\"rotVelRoll\":%.2f,\"rotVelPitch\":%.2f,"
    "\"temperature\":%.2f,\"vbat\":%.2f,\"current\":%.1f,\"power\":%.1f,"
    "\"consumedMah\":%.0f,\"remainingMah\":%.0f,"
    "\"flightTimeSec\":%.0f,\"altSetpoint\":%.2f,"
    "\"m1\":%u,\"m2\":%u,\"m3\":%u,\"m4\":%u,"
    "\"armed\":%s,\"altitudeHoldEnabled\":%s,"
    "\"batOverride\":%s,"
    "\"failsafe\":%s,\"lowBatCutoff\":%s,\"lowBatWarning\":%s,"
    "\"magHeading\":%.1f,"
    "\"magValid\":%s,"
    "\"gpsFixed\":%s,"
    "\"gpsLat\":%.6f,"
    "\"gpsLng\":%.6f,"
    "\"gpsAlt\":%.1f,"
    "\"gpsSpeed\":%.1f,"
    "\"gpsCourse\":%.1f,"
    "\"gpsSats\":%u,"
    "\"gpsAge\":%lu,"
    "\"yaw\":%.1f,"
    "\"batteryCells\":%d,"
    "\"dt\":%.4f"
    "}",
    millis(),
    l_pwmRoll, l_pwmPitch, l_pwmThrottle, l_pwmYaw, l_pwmArm, l_pwmAux,
    l_pwmBatOverride, l_pwmDescent,
    l_roll, l_pitch, l_yawRate,
    l_height, l_velocity, l_accelX, l_accelY, l_accelZ,
    l_gyroRoll * GYRO_SCALE, l_gyroPitch * GYRO_SCALE,
    l_temperature, l_vbat, l_currentAmps, l_powerWatts,
    l_consumedMah, l_remainingMah, l_flightTimeSec, l_altSetpoint,
    l_m1, l_m2, l_m3, l_m4,
    l_armed ? "true" : "false",
    l_altHold ? "true" : "false",
    l_batOverride ? "true" : "false",
    l_failsafe ? "true" : "false",
    l_lowBatCutoff ? "true" : "false",
    l_lowBatWarning ? "true" : "false",
    l_magHeading,
    l_magValid ? "true" : "false",
    l_gpsFixed ? "true" : "false",
    l_gpsLat, l_gpsLng,
    l_gpsAltitude,
    l_gpsSpeed,
    l_gpsCourse,
    l_gpsSatellites,
    l_gpsAge,
    l_yaw,
    batteryCells,
    l_dt
  );

  if (written >= (int)sizeof(json))
    Serial.println("WARNING: JSON buffer too small, output truncated!");

  server.send(200, "application/json", json);
}

// ────────────────────────────────────────────────────────────────
// handleRoot()
// HTTP handler for GET / — serves the complete single-page telemetry
// dashboard as inline HTML/CSS/JS. Dashboard features: artificial
// horizon (canvas), compass rose (canvas), attitude display, altitude
// bar, motor DSHOT bars, RC channel bars, battery bar with dynamic
// 3S/4S voltage labels and colour thresholds, GPS panel with Maps
// link, loop health panel. Polls /data at 100ms interval. Debug
// toggle available via HTTP. All rendering is client-side JavaScript
// — the ESP32 only serves the initial page and JSON data.
// ────────────────────────────────────────────────────────────────
void handleRoot() {
  server.send(200, "text/html", R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>QUAD FC</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap');

  :root {
    --bg:       #080c10;
    --bg2:      #0d1520;
    --bg3:      #111d2e;
    --border:   #1a3a5c;
    --accent:   #00d4ff;
    --green:    #00ff88;
    --yellow:   #ffcc00;
    --red:      #ff3344;
    --orange:   #ff6622;
    --dim:      #3a5a7a;
    --text:     #c8e0f0;
    --mono:     'Share Tech Mono', monospace;
    --sans:     'Rajdhani', sans-serif;
  }

  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--sans);
    font-size: 14px;
    min-height: 100vh;
    overflow-x: hidden;
  }

  /* Scanline overlay */
  body::before {
    content: '';
    position: fixed; inset: 0;
    background: repeating-linear-gradient(
      0deg,
      transparent,
      transparent 2px,
      rgba(0,0,0,0.08) 2px,
      rgba(0,0,0,0.08) 4px
    );
    pointer-events: none;
    z-index: 9999;
  }

  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 20px;
    background: var(--bg2);
    border-bottom: 1px solid var(--border);
  }

  .logo {
    font-family: var(--mono);
    font-size: 18px;
    color: var(--accent);
    letter-spacing: 4px;
  }

  .logo span { color: var(--dim); }

  #status-bar {
    display: flex;
    gap: 16px;
    align-items: center;
    font-family: var(--mono);
    font-size: 12px;
  }

  .pill {
    padding: 3px 10px;
    border-radius: 2px;
    border: 1px solid currentColor;
    letter-spacing: 1px;
    font-size: 11px;
    font-weight: 700;
    transition: all 0.2s;
  }

  .pill-armed    { color: var(--green);  border-color: var(--green);  background: rgba(0,255,136,0.08); }
  .pill-disarmed { color: var(--dim);    border-color: var(--dim);    background: transparent; }
  .pill-hold     { color: var(--yellow); border-color: var(--yellow); background: rgba(255,204,0,0.08); }
  .pill-failsafe { color: var(--red);    border-color: var(--red);    background: rgba(255,51,68,0.12); animation: blink 0.4s infinite; }
  .pill-lowbat   { color: var(--orange); border-color: var(--orange); background: rgba(255,102,34,0.1);  animation: blink 0.8s infinite; }
  .pill-gpsfix   { color: var(--green);  border-color: var(--green);  background: rgba(0,255,136,0.08); }
  .pill-nosig    { color: var(--dim);    border-color: var(--dim);    background: transparent; }
  .pill-batoverride { color: var(--red); border-color: var(--red);    background: rgba(255,51,68,0.08); animation: blink 0.5s infinite; }

  @keyframes blink { 50% { opacity: 0.3; } }

  #hz-counter {
    font-family: var(--mono);
    font-size: 11px;
    color: var(--dim);
  }

  /* ── Main grid ─────────────────────────────────────────────── */
  .grid {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    grid-template-rows: auto auto auto;
    gap: 10px;
    padding: 12px;
  }

  @media (max-width: 900px) {
    .grid { grid-template-columns: 1fr 1fr; }
  }
  @media (max-width: 580px) {
    .grid { grid-template-columns: 1fr; }
  }

  /* ── Panel ─────────────────────────────────────────────────── */
  .panel {
    background: var(--bg2);
    border: 1px solid var(--border);
    border-radius: 3px;
    padding: 14px;
    position: relative;
    overflow: hidden;
  }

  .panel::before {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 2px;
    background: linear-gradient(90deg, var(--accent), transparent);
  }

  .panel-title {
    font-family: var(--mono);
    font-size: 10px;
    letter-spacing: 3px;
    color: var(--dim);
    text-transform: uppercase;
    margin-bottom: 12px;
  }

  /* ── Attitude display ──────────────────────────────────────── */
  .attitude-grid {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 8px;
  }

  .att-item { text-align: center; }

  .att-label {
    font-family: var(--mono);
    font-size: 9px;
    color: var(--dim);
    letter-spacing: 2px;
    margin-bottom: 4px;
  }

  .att-value {
    font-family: var(--mono);
    font-size: 26px;
    font-weight: 700;
    line-height: 1;
    color: var(--accent);
    transition: color 0.3s;
  }

  .att-unit {
    font-size: 11px;
    color: var(--dim);
    margin-left: 2px;
  }

  /* ── Horizon ───────────────────────────────────────────────── */
  #horizon-wrap {
    display: flex;
    justify-content: center;
    margin-top: 12px;
  }

  #horizon-canvas {
    border: 1px solid var(--border);
    border-radius: 50%;
  }

  /* ── Compass ───────────────────────────────────────────────── */
  #compass-wrap {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 8px;
  }

  #compass-canvas { border-radius: 50%; }

  .compass-heading {
    font-family: var(--mono);
    font-size: 28px;
    color: var(--accent);
    text-align: center;
  }

  .compass-cardinal {
    font-family: var(--mono);
    font-size: 13px;
    color: var(--yellow);
    text-align: center;
    letter-spacing: 2px;
  }

  /* ── Motor bars ────────────────────────────────────────────── */
  .motor-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
  }

  .motor-item {}

  .motor-label {
    font-family: var(--mono);
    font-size: 9px;
    color: var(--dim);
    letter-spacing: 2px;
    display: flex;
    justify-content: space-between;
    margin-bottom: 4px;
  }

  .motor-val {
    color: var(--text);
    font-size: 11px;
  }

  .motor-bar-bg {
    background: var(--bg3);
    border: 1px solid var(--border);
    height: 8px;
    border-radius: 1px;
    overflow: hidden;
    margin-bottom: 2px;
  }

  .motor-bar-fill {
    height: 100%;
    background: linear-gradient(90deg, var(--accent), var(--green));
    transition: width 0.08s linear;
    border-radius: 1px;
  }

  .motor-rpm {
    font-family: var(--mono);
    font-size: 9px;
    color: var(--dim);
    text-align: right;
  }

  /* ── RC channels ───────────────────────────────────────────── */
  .rc-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }

  .rc-item {}

  .rc-label {
    font-family: var(--mono);
    font-size: 9px;
    color: var(--dim);
    letter-spacing: 2px;
    display: flex;
    justify-content: space-between;
    margin-bottom: 3px;
  }

  .rc-bar-bg {
    background: var(--bg3);
    border: 1px solid var(--border);
    height: 5px;
    border-radius: 1px;
    position: relative;
  }

  /* Centre marker */
  .rc-bar-bg::after {
    content: '';
    position: absolute;
    left: 50%; top: -1px; bottom: -1px;
    width: 1px;
    background: var(--border);
  }

  .rc-bar-fill {
    height: 100%;
    background: var(--yellow);
    transition: width 0.08s linear;
    border-radius: 1px;
  }

  /* ── GPS panel ─────────────────────────────────────────────── */
  .gps-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
  }

  .gps-item {}

  .gps-label {
    font-family: var(--mono);
    font-size: 9px;
    color: var(--dim);
    letter-spacing: 2px;
    margin-bottom: 3px;
  }

  .gps-value {
    font-family: var(--mono);
    font-size: 15px;
    color: var(--green);
    transition: color 0.3s;
  }

  .gps-value.stale { color: var(--dim); }

  .gps-map-link {
    display: block;
    margin-top: 10px;
    font-family: var(--mono);
    font-size: 10px;
    color: var(--accent);
    text-decoration: none;
    letter-spacing: 1px;
    border: 1px solid var(--border);
    padding: 5px 8px;
    text-align: center;
    transition: background 0.2s;
  }

  .gps-map-link:hover { background: rgba(0,212,255,0.08); }

  /* ── Sensor row ────────────────────────────────────────────── */
  .sensor-row {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    padding: 5px 0;
    border-bottom: 1px solid var(--bg3);
  }

  .sensor-row:last-child { border-bottom: none; }

  .sensor-key {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--dim);
    letter-spacing: 1px;
  }

  .sensor-val {
    font-family: var(--mono);
    font-size: 13px;
    color: var(--text);
  }

  /* ── Battery ───────────────────────────────────────────────── */
  #bat-bar-bg {
    background: var(--bg3);
    border: 1px solid var(--border);
    height: 14px;
    border-radius: 2px;
    overflow: hidden;
    margin-top: 8px;
  }

  #bat-bar-fill {
    height: 100%;
    transition: width 0.5s, background 0.5s;
    border-radius: 2px;
  }

  .bat-nums {
    display: flex;
    justify-content: space-between;
    margin-top: 5px;
    font-family: var(--mono);
    font-size: 11px;
    color: var(--dim);
  }

  #bat-voltage {
    font-family: var(--mono);
    font-size: 30px;
    color: var(--green);
    transition: color 0.3s;
  }

  /* ── Loop health ───────────────────────────────────────────── */
  .health-row {
    display: flex;
    justify-content: space-between;
    padding: 4px 0;
    border-bottom: 1px solid var(--bg3);
    font-family: var(--mono);
    font-size: 11px;
  }

  .health-key { color: var(--dim); letter-spacing: 1px; }
  .health-val { color: var(--text); }

  /* ── Altitude ──────────────────────────────────────────────── */
  #alt-value {
    font-family: var(--mono);
    font-size: 38px;
    color: var(--accent);
    text-align: center;
    line-height: 1;
  }

  #vel-value {
    font-family: var(--mono);
    font-size: 18px;
    color: var(--yellow);
    text-align: center;
    margin-top: 4px;
  }

  #alt-bar-bg {
    background: var(--bg3);
    border: 1px solid var(--border);
    height: 6px;
    border-radius: 1px;
    overflow: hidden;
    margin-top: 10px;
  }

  #alt-bar-fill {
    height: 100%;
    background: linear-gradient(90deg, var(--accent), var(--green));
    transition: width 0.1s linear;
    border-radius: 1px;
  }

  .pid-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 4px 0;
    border-bottom: 1px solid var(--bg3);
  }

  .pid-input {
    font-family: var(--mono);
    font-size: 13px;
    width: 72px;
    background: var(--bg3);
    border: 1px solid var(--border);
    color: var(--accent);
    padding: 3px 6px;
    text-align: right;
  }

  .pid-input:disabled {
    color: var(--dim);
    border-color: var(--bg3);
  }

</style>
</head>
<body>

<header>
  <div class="logo">QUAD<span>/</span>FC &nbsp; ESP32-S3</div>
  <div id="status-bar">
    <span id="pill-armed"    class="pill pill-disarmed">DISARMED</span>
    <span id="pill-hold"     class="pill pill-hold" style="display:none">ALT HOLD</span>
    <span id="pill-failsafe" class="pill pill-failsafe" style="display:none">FAILSAFE</span>
    <span id="pill-lowbat"   class="pill pill-lowbat"   style="display:none">LOW BAT</span>
    <span id="pill-gps"      class="pill pill-nosig">NO FIX</span>
    <span id="pill-batoverride" class="pill" style="display:none">BAT OVERRIDE</span>
    <span id="hz-counter">-- Hz</span>
  </div>
</header>

<div class="grid">

  <!-- ── ATTITUDE ─────────────────────────────────────────────── -->
  <div class="panel">
    <div class="panel-title">ATTITUDE</div>
    <div class="attitude-grid">
      <div class="att-item">
        <div class="att-label">ROLL</div>
        <div class="att-value" id="v-roll">0.0<span class="att-unit">°</span></div>
      </div>
      <div class="att-item">
        <div class="att-label">PITCH</div>
        <div class="att-value" id="v-pitch">0.0<span class="att-unit">°</span></div>
      </div>
      <div class="att-item">
        <div class="att-label">YAW</div>
        <div class="att-value" id="v-yaw">0.0<span class="att-unit">°</span></div>
      </div>
    </div>
    <div id="horizon-wrap">
      <canvas id="horizon-canvas" width="160" height="160"></canvas>
    </div>
  </div>

  <!-- ── COMPASS / HEADING ─────────────────────────────────────── -->
  <div class="panel">
    <div class="panel-title">HEADING</div>
    <div id="compass-wrap">
      <canvas id="compass-canvas" width="140" height="140"></canvas>
      <div class="compass-heading" id="v-heading">---°</div>
      <div class="compass-cardinal" id="v-cardinal">---</div>
    </div>
    <div style="margin-top:10px;">
      <div class="sensor-row">
        <span class="sensor-key">MAG X</span><span class="sensor-val" id="v-magx">---</span>
      </div>
      <div class="sensor-row">
        <span class="sensor-key">KAL YAW</span><span class="sensor-val" id="v-magy">---</span>
      </div>
      <div class="sensor-row">
        <span class="sensor-key">MAG VALID</span><span class="sensor-val" id="v-magvalid">---</span>
      </div>
    </div>
  </div>

  <!-- ── ALTITUDE ──────────────────────────────────────────────── -->
  <div class="panel">
    <div class="panel-title">ALTITUDE</div>
    <div id="alt-value">0.00<span style="font-size:16px;color:var(--dim)"> m</span></div>
    <div id="vel-value">0.00 m/s</div>
    <div id="alt-bar-bg"><div id="alt-bar-fill" style="width:0%"></div></div>
    <div style="margin-top:12px;">
      <div class="sensor-row">
        <span class="sensor-key">SETPOINT</span><span class="sensor-val" id="v-altsp">---</span>
      </div>
      <div class="sensor-row">
        <span class="sensor-key">TEMP</span><span class="sensor-val" id="v-temp">---</span>
      </div>
      <div class="sensor-row">
        <span class="sensor-key">GPS ALT</span><span class="sensor-val" id="v-gpsalt">---</span>
      </div>
    </div>
  </div>

  <!-- ── MOTORS ────────────────────────────────────────────────── -->
  <div class="panel">
    <div class="panel-title">MOTORS — DSHOT300</div>
    <div class="motor-grid">
      <div class="motor-item">
        <div class="motor-label"><span>M1 FR CCW</span><span class="motor-val" id="mv1">48</span></div>
        <div class="motor-bar-bg"><div class="motor-bar-fill" id="mb1" style="width:0%"></div></div>
        <div class="motor-rpm" id="mr1">-- RPM</div>
      </div>
      <div class="motor-item">
        <div class="motor-label"><span>M2 RR CW</span><span class="motor-val" id="mv2">48</span></div>
        <div class="motor-bar-bg"><div class="motor-bar-fill" id="mb2" style="width:0%"></div></div>
        <div class="motor-rpm" id="mr2">-- RPM</div>
      </div>
      <div class="motor-item">
        <div class="motor-label"><span>M3 FL CW</span><span class="motor-val" id="mv3">48</span></div>
        <div class="motor-bar-bg"><div class="motor-bar-fill" id="mb3" style="width:0%"></div></div>
        <div class="motor-rpm" id="mr3">-- RPM</div>
      </div>
      <div class="motor-item">
        <div class="motor-label"><span>M4 RL CCW</span><span class="motor-val" id="mv4">48</span></div>
        <div class="motor-bar-bg"><div class="motor-bar-fill" id="mb4" style="width:0%"></div></div>
        <div class="motor-rpm" id="mr4">-- RPM</div>
      </div>
    </div>
  </div>

  <!-- ── GPS ───────────────────────────────────────────────────── -->
  <div class="panel">
    <div class="panel-title">GPS — NEO-6M</div>
    <div class="gps-grid">
      <div class="gps-item">
        <div class="gps-label">LATITUDE</div>
        <div class="gps-value" id="v-lat">---.------</div>
      </div>
      <div class="gps-item">
        <div class="gps-label">LONGITUDE</div>
        <div class="gps-value" id="v-lng">---.------</div>
      </div>
      <div class="gps-item">
        <div class="gps-label">SPEED</div>
        <div class="gps-value" id="v-spd">-- km/h</div>
      </div>
      <div class="gps-item">
        <div class="gps-label">COURSE</div>
        <div class="gps-value" id="v-course">---°</div>
      </div>
      <div class="gps-item">
        <div class="gps-label">SATELLITES</div>
        <div class="gps-value" id="v-sats">--</div>
      </div>
      <div class="gps-item">
        <div class="gps-label">DATA AGE</div>
        <div class="gps-value" id="v-age">-- ms</div>
      </div>
    </div>
    <a class="gps-map-link" id="map-link" href="#" target="_blank">OPEN IN MAPS ↗</a>
  </div>

  <!-- ── RC CHANNELS ───────────────────────────────────────────── -->
  <div class="panel">
    <div class="panel-title">RC — iBUS</div>
    <div class="rc-grid">
      <div class="rc-item">
        <div class="rc-label"><span>ROLL CH1</span><span id="rc-roll">1500</span></div>
        <div class="rc-bar-bg"><div class="rc-bar-fill" id="rcb-roll" style="width:50%"></div></div>
      </div>
      <div class="rc-item">
        <div class="rc-label"><span>PITCH CH2</span><span id="rc-pitch">1500</span></div>
        <div class="rc-bar-bg"><div class="rc-bar-fill" id="rcb-pitch" style="width:50%"></div></div>
      </div>
      <div class="rc-item">
        <div class="rc-label"><span>THROTTLE CH3</span><span id="rc-thr">1000</span></div>
        <div class="rc-bar-bg"><div class="rc-bar-fill" id="rcb-thr" style="width:0%"></div></div>
      </div>
      <div class="rc-item">
        <div class="rc-label"><span>YAW CH4</span><span id="rc-yaw">1500</span></div>
        <div class="rc-bar-bg"><div class="rc-bar-fill" id="rcb-yaw" style="width:50%"></div></div>
      </div>
      <div class="rc-item">
        <div class="rc-label"><span>ARM CH5</span><span id="rc-arm">1000</span></div>
        <div class="rc-bar-bg"><div class="rc-bar-fill" id="rcb-arm" style="width:0%"></div></div>
      </div>
      <div class="rc-item">
        <div class="rc-label"><span>ALT HOLD CH6</span><span id="rc-aux">1000</span></div>
        <div class="rc-bar-bg"><div class="rc-bar-fill" id="rcb-aux" style="width:0%"></div></div>
      </div>
      <div class="rc-item">
        <div class="rc-label"><span>BAT OVRD CH7</span><span id="rc-batovr">1000</span></div>
        <div class="rc-bar-bg"><div class="rc-bar-fill" id="rcb-batovr" style="width:0%"></div></div>
      </div>
      <div class="rc-item">
        <div class="rc-label"><span>DESCENT CH8</span><span id="rc-descent">1000</span></div>
        <div class="rc-bar-bg"><div class="rc-bar-fill" id="rcb-descent" style="width:0%"></div></div>
      </div>
    </div>
  </div>

  <!-- ── BATTERY ───────────────────────────────────────────────── -->
  <div class="panel">
    <div class="panel-title">BATTERY</div>
    <div id="bat-voltage">--.-<span style="font-size:16px;color:var(--dim)"> V</span></div>
    <div id="bat-bar-bg"><div id="bat-bar-fill"></div></div>
    <div class="bat-nums" id="bat-nums-labels">
      <span>9.0V</span><span>11.1V</span><span>12.6V</span>
    </div>
    <div style="margin-top:8px;display:flex;gap:16px;align-items:baseline;">
      <div style="font-family:var(--mono);font-size:22px;" id="bat-current">--.- <span style="font-size:13px;color:var(--dim)">A</span></div>
      <div style="font-family:var(--mono);font-size:22px;" id="bat-power">--- <span style="font-size:13px;color:var(--dim)">W</span></div>
    </div>
    <div style="margin-top:8px;">
      <div class="sensor-row">
        <span class="sensor-key">USED</span>
        <span class="sensor-val" id="bat-consumed">--- mAh</span>
      </div>
      <div class="sensor-row">
        <span class="sensor-key">REMAINING</span>
        <span class="sensor-val" id="bat-remaining">--- mAh</span>
      </div>
      <div class="sensor-row">
        <span class="sensor-key">EST. TIME</span>
        <span class="sensor-val" id="bat-flighttime">---</span>
      </div>
      <div class="sensor-row">
        <span class="sensor-key">CELLS</span>
        <span class="sensor-val" id="v-cells">--</span>
      </div>
    </div>
    <div style="margin-top:10px">
      <div class="sensor-row">
        <span class="sensor-key">ACCEL X</span><span class="sensor-val" id="v-ax">---</span>
      </div>
      <div class="sensor-row">
        <span class="sensor-key">ACCEL Y</span><span class="sensor-val" id="v-ay">---</span>
      </div>
      <div class="sensor-row">
        <span class="sensor-key">ACCEL Z</span><span class="sensor-val" id="v-az">---</span>
      </div>
    </div>
  </div>

  <!-- ── LOOP HEALTH ───────────────────────────────────────────── -->
  <div class="panel">
    <div class="panel-title">LOOP HEALTH</div>
    <div class="health-row"><span class="health-key">LOOP Hz</span><span class="health-val" id="v-hz">--</span></div>
    <div class="health-row"><span class="health-key">LOOP dt</span><span class="health-val" id="v-dt">-- ms</span></div>
    <div class="health-row"><span class="health-key">UPTIME</span><span class="health-val" id="v-uptime">--</span></div>
    <div class="health-row"><span class="health-key">YAW RATE</span><span class="health-val" id="v-yr">-- °/s</span></div>
    <div class="health-row"><span class="health-key">GYRO R</span><span class="health-val" id="v-gr">-- °/s</span></div>
    <div class="health-row"><span class="health-key">GYRO P</span><span class="health-val" id="v-gp">-- °/s</span></div>
  </div>

  <div class="panel" id="pid-panel" style="grid-column: 1 / -1;">
    <div class="panel-title">PID TUNING — DISARMED ONLY</div>
    <div id="pid-armed-warning" style="display:none;font-family:var(--mono);font-size:11px;
        color:var(--red);margin-bottom:10px;letter-spacing:1px;">
      ⚠ DISARM BEFORE EDITING
    </div>
    <div style="display:grid;grid-template-columns:repeat(4,1fr);gap:16px;">
      <div>
        <div class="panel-title" style="margin-bottom:8px">ROLL</div>
        <div class="pid-row"><span class="sensor-key">Kp</span>
          <input class="pid-input" id="pid-roll-kp" type="number" step="0.1" min="0" max="10"></div>
        <div class="pid-row"><span class="sensor-key">Ki</span>
          <input class="pid-input" id="pid-roll-ki" type="number" step="0.05" min="0" max="10"></div>
        <div class="pid-row"><span class="sensor-key">Kd</span>
          <input class="pid-input" id="pid-roll-kd" type="number" step="0.1" min="0" max="10"></div>
        <div class="pid-row"><span class="sensor-key">D Hz</span>
          <input class="pid-input" id="pid-roll-dhz" type="number" step="1" min="1" max="100"></div>
      </div>
      <div>
        <div class="panel-title" style="margin-bottom:8px">PITCH</div>
        <div class="pid-row"><span class="sensor-key">Kp</span>
          <input class="pid-input" id="pid-pitch-kp" type="number" step="0.1" min="0" max="10"></div>
        <div class="pid-row"><span class="sensor-key">Ki</span>
          <input class="pid-input" id="pid-pitch-ki" type="number" step="0.05" min="0" max="10"></div>
        <div class="pid-row"><span class="sensor-key">Kd</span>
          <input class="pid-input" id="pid-pitch-kd" type="number" step="0.1" min="0" max="10"></div>
        <div class="pid-row"><span class="sensor-key">D Hz</span>
          <input class="pid-input" id="pid-pitch-dhz" type="number" step="1" min="1" max="100"></div>
      </div>
      <div>
        <div class="panel-title" style="margin-bottom:8px">YAW RATE</div>
        <div class="pid-row"><span class="sensor-key">Kp</span>
          <input class="pid-input" id="pid-yaw-kp" type="number" step="0.1" min="0" max="10"></div>
        <div class="pid-row"><span class="sensor-key">Ki</span>
          <input class="pid-input" id="pid-yaw-ki" type="number" step="0.05" min="0" max="10"></div>
        <div class="pid-row"><span class="sensor-key">Kd</span>
          <input class="pid-input" id="pid-yaw-kd" type="number" step="0.1" min="0" max="10"></div>
        <div class="pid-row"><span class="sensor-key">D Hz</span>
          <input class="pid-input" id="pid-yaw-dhz" type="number" step="1" min="1" max="100"></div>
      </div>
      <div>
        <div class="panel-title" style="margin-bottom:8px">ALTITUDE</div>
        <div class="pid-row"><span class="sensor-key">Kp</span>
          <input class="pid-input" id="pid-alt-kp" type="number" step="0.1" min="0" max="10"></div>
        <div class="pid-row"><span class="sensor-key">Ki</span>
          <input class="pid-input" id="pid-alt-ki" type="number" step="0.05" min="0" max="10"></div>
        <div class="pid-row"><span class="sensor-key">Kd</span>
          <input class="pid-input" id="pid-alt-kd" type="number" step="0.1" min="0" max="10"></div>
        <div class="pid-row"><span class="sensor-key">D Hz</span>
          <input class="pid-input" id="pid-alt-dhz" type="number" step="1" min="1" max="100"></div>
      </div>
    </div>
    <div style="margin-top:14px;display:flex;gap:12px;align-items:center;">
      <button id="pid-apply" onclick="applyPIDs()"
        style="font-family:var(--mono);font-size:11px;letter-spacing:2px;padding:6px 18px;
              background:transparent;border:1px solid var(--accent);color:var(--accent);
              cursor:pointer;">APPLY</button>
      <button onclick="loadPIDs()"
        style="font-family:var(--mono);font-size:11px;letter-spacing:2px;padding:6px 18px;
              background:transparent;border:1px solid var(--dim);color:var(--dim);
              cursor:pointer;">RELOAD</button>
      <span id="pid-status" style="font-family:var(--mono);font-size:11px;color:var(--dim)"></span>
    </div>
  </div>

</div><!-- /grid -->

<script>
// ── Fetch and update ──────────────────────────────────────────────
let lastTs = 0, frameCount = 0, hzDisplay = 0;
const DSHOT_MIN = 48, DSHOT_MAX = 1800;

function pct(val, lo, hi) {
  return Math.min(100, Math.max(0, (val - lo) / (hi - lo) * 100));
}

function rcPct(val) {
  return pct(val, 1000, 2000);
}

function dshotPct(val) {
  return pct(val, DSHOT_MIN, DSHOT_MAX);
}

function cardinal(deg) {
  const dirs = ['N','NNE','NE','ENE','E','ESE','SE','SSE',
                'S','SSW','SW','WSW','W','WNW','NW','NNW'];
  return dirs[Math.round(deg / 22.5) % 16];
}

function formatUptime(ms) {
  const s = Math.floor(ms / 1000);
  const m = Math.floor(s / 60);
  const h = Math.floor(m / 60);
  return `${String(h).padStart(2,'0')}:${String(m%60).padStart(2,'0')}:${String(s%60).padStart(2,'0')}`;
}

function batColor(v, cells) {
  const c = cells || 3;
  if (v < c * 3.17) return 'var(--red)';    // <3.17V/cell
  if (v < c * 3.50) return 'var(--orange)'; // <3.50V/cell
  if (v < c * 3.70) return 'var(--yellow)'; // <3.70V/cell
  return 'var(--green)';
}

async function fetchData() {
  try {
    const res = await fetch('/data');
    const d = await res.json();

    // ── Hz counter ──────────────────────────────────────────────
    frameCount++;
    if (Date.now() - lastTs >= 1000) {
      hzDisplay = frameCount;
      frameCount = 0;
      lastTs = Date.now();
    }
    document.getElementById('hz-counter').textContent = hzDisplay + ' Hz';

    // ── Status pills ────────────────────────────────────────────
    const pArmed = document.getElementById('pill-armed');
    pArmed.textContent = d.armed ? 'ARMED' : 'DISARMED';
    pArmed.className = 'pill ' + (d.armed ? 'pill-armed' : 'pill-disarmed');

    const pHold = document.getElementById('pill-hold');
    pHold.style.display = d.altitudeHoldEnabled ? '' : 'none';

    const pFs = document.getElementById('pill-failsafe');
    pFs.style.display = d.failsafe ? '' : 'none';

    const pLb = document.getElementById('pill-lowbat');
    pLb.style.display = (d.lowBatCutoff || d.lowBatWarning) ? '' : 'none';
    pLb.textContent = d.lowBatCutoff ? 'CUTOFF' : 'LOW BAT';

    const pGps = document.getElementById('pill-gps');
    pGps.textContent = d.gpsFixed ? `FIX ${d.gpsSats}` : 'NO FIX';
    pGps.className = 'pill ' + (d.gpsFixed ? 'pill-gpsfix' : 'pill-nosig');

    const pBatOvr = document.getElementById('pill-batoverride');
    pBatOvr.style.display = d.batOverride ? '' : 'none';
    pBatOvr.className = 'pill pill-batoverride';

    // ── Attitude ─────────────────────────────────────────────────
    document.getElementById('v-roll').innerHTML  = d.roll.toFixed(1)  + '<span class="att-unit">°</span>';
    document.getElementById('v-pitch').innerHTML = d.pitch.toFixed(1) + '<span class="att-unit">°</span>';
    document.getElementById('v-yaw').innerHTML   = d.yaw.toFixed(1)   + '<span class="att-unit">°</span>';
    drawHorizon(d.roll, d.pitch);

    // ── Compass ──────────────────────────────────────────────────
    const hdg = d.magValid ? d.magHeading : null;
    document.getElementById('v-heading').textContent  = hdg !== null ? hdg.toFixed(1) + '°' : '---°';
    document.getElementById('v-cardinal').textContent = hdg !== null ? cardinal(hdg) : '---';
    document.getElementById('v-magx').textContent = d.magValid ? d.magHeading.toFixed(1) + '°' : '---';
    document.getElementById('v-magy').textContent = d.magValid ? d.yaw.toFixed(1) + '°' : '---';
    document.getElementById('v-magvalid').textContent = d.magValid ? '✓ OK' : '✗ NO DATA';
    drawCompass(hdg !== null ? hdg : 0, d.magValid);

    // ── Altitude ─────────────────────────────────────────────────
    document.getElementById('alt-value').innerHTML =
      d.height.toFixed(2) + '<span style="font-size:16px;color:var(--dim)"> m</span>';
    document.getElementById('vel-value').textContent =
      (d.velocity >= 0 ? '+' : '') + d.velocity.toFixed(2) + ' m/s';
    document.getElementById('alt-bar-fill').style.width = pct(d.height, 0, 20) + '%';
    document.getElementById('v-altsp').textContent  = d.altSetpoint.toFixed(2) + ' m';
    document.getElementById('v-temp').textContent   = d.temperature.toFixed(1) + ' °C';
    document.getElementById('v-gpsalt').textContent = d.gpsFixed ? d.gpsAlt.toFixed(1) + ' m' : '--- m';

    // ── Motors ───────────────────────────────────────────────────
    [1,2,3,4].forEach(i => {
      const val = d['m'+i];
      document.getElementById('mv'+i).textContent = val;
      document.getElementById('mb'+i).style.width = dshotPct(val) + '%';
      document.getElementById('mr'+i).textContent = '-- RPM';
    });

    // ── GPS ──────────────────────────────────────────────────────
    const fixed = d.gpsFixed;
    document.getElementById('v-lat').textContent    = fixed ? d.gpsLat.toFixed(6) : '---.------';
    document.getElementById('v-lng').textContent    = fixed ? d.gpsLng.toFixed(6) : '---.------';
    document.getElementById('v-spd').textContent    = fixed ? d.gpsSpeed.toFixed(1) + ' km/h' : '-- km/h';
    document.getElementById('v-course').textContent = fixed ? d.gpsCourse.toFixed(1) + '°' : '---°';
    document.getElementById('v-sats').textContent   = d.gpsSats || '--';
    document.getElementById('v-age').textContent    = fixed ? d.gpsAge + ' ms' : '-- ms';
    [document.getElementById('v-lat'),
     document.getElementById('v-lng'),
     document.getElementById('v-spd'),
     document.getElementById('v-course')].forEach(el => {
      el.classList.toggle('stale', !fixed);
    });
    const mapLink = document.getElementById('map-link');
    if (fixed) {
      mapLink.href = `https://maps.google.com/?q=${d.gpsLat},${d.gpsLng}`;
      mapLink.style.color = 'var(--accent)';
    } else {
      mapLink.href = '#';
      mapLink.style.color = 'var(--dim)';
    }

    // ── RC channels ──────────────────────────────────────────────
    const rcMap = [
      ['roll',    d.rcRoll],
      ['pitch',   d.rcPitch],
      ['thr',     d.rcThrottle],
      ['yaw',     d.rcYaw],
      ['arm',     d.rcArm],
      ['aux',     d.rcAux],
      ['batovr',  d.rcBatOverride],
      ['descent', d.rcDescent],
    ];
    rcMap.forEach(([key, val]) => {
      document.getElementById('rc-' + key).textContent = val;
      document.getElementById('rcb-' + key).style.width = rcPct(val) + '%';
    });

    // ── Battery ──────────────────────────────────────────────────
    const vb = d.vbat;
    const cells = d.batteryCells || 3;
    const col= batColor(vb, cells);

    // Update bat-nums labels dynamically — add after col is set
    const vMin = cells * 3.0;
    const vNom = cells * 3.7;
    const vMax = cells * 4.2;

    document.getElementById('bat-nums-labels').innerHTML =
      `<span>${(cells*3.0).toFixed(1)}V</span>` +
      `<span>${(cells*3.7).toFixed(1)}V</span>` +
      `<span>${(cells*4.2).toFixed(1)}V</span>`;
    document.getElementById('bat-voltage').innerHTML =
      vb.toFixed(2) + '<span style="font-size:16px;color:var(--dim)"> V</span>';
    document.getElementById('bat-voltage').style.color = col;
    document.getElementById('bat-current').innerHTML =
      d.current.toFixed(1) + ' <span style="font-size:13px;color:var(--dim)">A</span>';
    document.getElementById('bat-power').innerHTML =
      d.power.toFixed(0) + ' <span style="font-size:13px;color:var(--dim)">W</span>';
    document.getElementById('bat-consumed').textContent =
      d.consumedMah.toFixed(0) + ' mAh';
    document.getElementById('bat-remaining').textContent =
      d.remainingMah.toFixed(0) + ' mAh';
    document.getElementById('v-cells').textContent =
      d.batteryCells + 'S';

    // Flight time display
    if (d.flightTimeSec < 0) {
      document.getElementById('bat-flighttime').textContent = '--- (low current)';
    } else {
      const mins = Math.floor(d.flightTimeSec / 60);
      const secs = Math.floor(d.flightTimeSec % 60);
      document.getElementById('bat-flighttime').textContent =
        `${mins}m ${String(secs).padStart(2,'0')}s`;
    }
    const batFill = document.getElementById('bat-bar-fill');
    batFill.style.width = pct(vb, vMin, vMax) + '%';
    batFill.style.background = col;

    document.getElementById('v-ax').textContent = d.accelX.toFixed(3) + ' m/s²';
    document.getElementById('v-ay').textContent = d.accelY.toFixed(3) + ' m/s²';
    document.getElementById('v-az').textContent = d.accelZ.toFixed(3) + ' m/s²';

    // ── Loop health ──────────────────────────────────────────────
    document.getElementById('v-hz').textContent = d.dt > 0 ? (1 / d.dt).toFixed(0) + ' Hz' : '-- Hz';
    document.getElementById('v-dt').textContent = (d.dt * 1000).toFixed(2) + ' ms';
    document.getElementById('v-uptime').textContent = formatUptime(d.timestamp);
    document.getElementById('v-yr').textContent = d.yawRate.toFixed(1) + ' °/s';
    document.getElementById('v-gr').textContent = d.rotVelRoll.toFixed(1)  + ' °/s';
    document.getElementById('v-gp').textContent = d.rotVelPitch.toFixed(1) + ' °/s';

    const isArmed = d.armed;
    document.querySelectorAll('.pid-input').forEach(el => el.disabled = isArmed);
    document.getElementById('pid-apply').disabled      = isArmed;
    document.getElementById('pid-apply').style.opacity = isArmed ? '0.3' : '1';
    document.getElementById('pid-armed-warning').style.display = isArmed ? '' : 'none';
  } catch(e) {}
}

// ── Artificial horizon ────────────────────────────────────────────
function drawHorizon(rollDeg, pitchDeg) {
  const canvas = document.getElementById('horizon-canvas');
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const cx = W/2, cy = H/2, r = W/2 - 2;

  ctx.clearRect(0, 0, W, H);
  ctx.save();
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI*2);
  ctx.clip();

  const rollRad = rollDeg  * Math.PI / 180;
  const pitchOff = pitchDeg / 45 * r * 0.8;

  ctx.save();
  ctx.translate(cx, cy + pitchOff);
  ctx.rotate(rollRad);

  // Sky
  ctx.fillStyle = '#0a2040';
  ctx.fillRect(-W, -H, W*2, H*2);
  // Ground
  ctx.fillStyle = '#1a0a00';
  ctx.fillRect(-W, 0, W*2, H*2);
  // Horizon line
  ctx.strokeStyle = '#f0a000';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-W, 0); ctx.lineTo(W, 0);
  ctx.stroke();
  ctx.restore();

  // Pitch ladder marks
  ctx.save();
  ctx.translate(cx, cy + pitchOff);
  ctx.rotate(rollRad);
  ctx.strokeStyle = 'rgba(255,255,255,0.4)';
  ctx.lineWidth = 1;
  for (let p = -30; p <= 30; p += 10) {
    if (p === 0) continue;
    const py = -p / 45 * r * 0.8;
    const lw = p % 20 === 0 ? 30 : 18;
    ctx.beginPath();
    ctx.moveTo(-lw, py); ctx.lineTo(lw, py);
    ctx.stroke();
  }
  ctx.restore();

  // Fixed reticle
  ctx.strokeStyle = '#00d4ff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(cx - 40, cy); ctx.lineTo(cx - 12, cy);
  ctx.moveTo(cx + 12, cy); ctx.lineTo(cx + 40, cy);
  ctx.moveTo(cx, cy - 6);  ctx.lineTo(cx, cy + 6);
  ctx.stroke();

  // Border
  ctx.strokeStyle = '#1a3a5c';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI*2);
  ctx.stroke();

  ctx.restore();
}

// ── Compass rose ──────────────────────────────────────────────────
function drawCompass(headingDeg, valid) {
  const canvas = document.getElementById('compass-canvas');
  const ctx    = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const cx = W/2, cy = H/2, r = W/2 - 6;

  ctx.clearRect(0, 0, W, H);

  // Background
  const grad = ctx.createRadialGradient(cx, cy, 0, cx, cy, r);
  grad.addColorStop(0, '#0d1520');
  grad.addColorStop(1, '#080c10');
  ctx.fillStyle = grad;
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI*2);
  ctx.fill();

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-headingDeg * Math.PI / 180);

  // Tick marks
  for (let i = 0; i < 360; i += 5) {
    const a   = i * Math.PI / 180;
    const len = i % 30 === 0 ? 10 : (i % 10 === 0 ? 6 : 3);
    const col = i % 90 === 0 ? '#00d4ff' : 'rgba(58,90,122,0.8)';
    ctx.strokeStyle = col;
    ctx.lineWidth   = i % 30 === 0 ? 1.5 : 0.8;
    ctx.beginPath();
    ctx.moveTo(Math.sin(a)*(r-len), -Math.cos(a)*(r-len));
    ctx.lineTo(Math.sin(a)*(r-2),   -Math.cos(a)*(r-2));
    ctx.stroke();
  }

  // Cardinal labels
  const cardinals = [['N',0,'#ff3344'],['E',90,'#00d4ff'],['S',180,'#00d4ff'],['W',270,'#00d4ff']];
  cardinals.forEach(([lbl, deg, col]) => {
    const a = deg * Math.PI / 180;
    ctx.fillStyle   = col;
    ctx.font        = 'bold 11px Share Tech Mono, monospace';
    ctx.textAlign   = 'center';
    ctx.textBaseline= 'middle';
    ctx.fillText(lbl, Math.sin(a)*(r-18), -Math.cos(a)*(r-18));
  });

  ctx.restore();

  // Fixed north needle (points up — heading rotates underneath)
  ctx.fillStyle   = valid ? '#ff3344' : '#3a5a7a';
  ctx.beginPath();
  ctx.moveTo(cx, cy - r + 22);
  ctx.lineTo(cx - 5, cy - r + 36);
  ctx.lineTo(cx + 5, cy - r + 36);
  ctx.closePath();
  ctx.fill();

  // Centre dot
  ctx.fillStyle = '#00d4ff';
  ctx.beginPath();
  ctx.arc(cx, cy, 3, 0, Math.PI*2);
  ctx.fill();

  // Border
  ctx.strokeStyle = '#1a3a5c';
  ctx.lineWidth   = 1.5;
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI*2);
  ctx.stroke();
}

// ── Poll at ~10Hz ─────────────────────────────────────────────────
let pollInterval = 100;

async function checkArmedState() {
  try {
    const res = await fetch('/status');
    const d = await res.json();
    pollInterval = d.armed ? 100 : 1000;  // 10Hz armed, 1Hz disarmed
  } catch(e) {}
}

async function scheduledFetch() {
  await fetchData();
  setTimeout(scheduledFetch, pollInterval);
}

async function loadPIDs() {
  try {
    const d = await (await fetch('/pid/get')).json();
    document.getElementById('pid-roll-kp').value   = d.rollKp.toFixed(2);
    document.getElementById('pid-roll-ki').value   = d.rollKi.toFixed(2);
    document.getElementById('pid-roll-kd').value   = d.rollKd.toFixed(2);
    document.getElementById('pid-roll-dhz').value  = d.rollDHz.toFixed(1);
    document.getElementById('pid-pitch-kp').value  = d.pitchKp.toFixed(2);
    document.getElementById('pid-pitch-ki').value  = d.pitchKi.toFixed(2);
    document.getElementById('pid-pitch-kd').value  = d.pitchKd.toFixed(2);
    document.getElementById('pid-pitch-dhz').value = d.pitchDHz.toFixed(1);
    document.getElementById('pid-yaw-kp').value    = d.yawKp.toFixed(2);
    document.getElementById('pid-yaw-ki').value    = d.yawKi.toFixed(2);
    document.getElementById('pid-yaw-kd').value    = d.yawKd.toFixed(2);
    document.getElementById('pid-yaw-dhz').value   = d.yawDHz.toFixed(1);
    document.getElementById('pid-alt-kp').value    = d.altKp.toFixed(2);
    document.getElementById('pid-alt-ki').value    = d.altKi.toFixed(2);
    document.getElementById('pid-alt-kd').value    = d.altKd.toFixed(2);
    document.getElementById('pid-alt-dhz').value   = d.altDHz.toFixed(1);
    document.getElementById('pid-status').textContent = 'loaded';
    document.getElementById('pid-status').style.color = 'var(--dim)';
  } catch(e) {
    document.getElementById('pid-status').textContent = 'load failed';
    document.getElementById('pid-status').style.color = 'var(--red)';
  }
}

async function applyPIDs() {
  const params = new URLSearchParams({
    rollKp:   document.getElementById('pid-roll-kp').value,
    rollKi:   document.getElementById('pid-roll-ki').value,
    rollKd:   document.getElementById('pid-roll-kd').value,
    rollDHz:  document.getElementById('pid-roll-dhz').value,
    pitchKp:  document.getElementById('pid-pitch-kp').value,
    pitchKi:  document.getElementById('pid-pitch-ki').value,
    pitchKd:  document.getElementById('pid-pitch-kd').value,
    pitchDHz: document.getElementById('pid-pitch-dhz').value,
    yawKp:    document.getElementById('pid-yaw-kp').value,
    yawKi:    document.getElementById('pid-yaw-ki').value,
    yawKd:    document.getElementById('pid-yaw-kd').value,
    yawDHz:   document.getElementById('pid-yaw-dhz').value,
    altKp:    document.getElementById('pid-alt-kp').value,
    altKi:    document.getElementById('pid-alt-ki').value,
    altKd:    document.getElementById('pid-alt-kd').value,
    altDHz:   document.getElementById('pid-alt-dhz').value,
  });
  try {
    const res = await fetch('/pid/set?' + params);
    const d   = await res.json();
    const status = document.getElementById('pid-status');
    if (d.ok) {
      status.textContent = 'applied ✓';
      status.style.color = 'var(--green)';
    } else {
      status.textContent = d.error || 'rejected';
      status.style.color = 'var(--red)';
    }
  } catch(e) {
    document.getElementById('pid-status').textContent = 'error';
    document.getElementById('pid-status').style.color = 'var(--red)';
  }
}

loadPIDs();
drawHorizon(0, 0);
drawCompass(0, false);
checkArmedState();
setInterval(checkArmedState, 2000);  // recheck armed state every 2s
scheduledFetch();
</script>
</body>
</html>
)rawhtml");
}
