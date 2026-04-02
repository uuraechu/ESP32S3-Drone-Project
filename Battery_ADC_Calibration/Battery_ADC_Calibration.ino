// ────────────────────────────────────────────────────────────────
// BATTERY ADC CALIBRATION SKETCH
// Collects ADC raw vs actual voltage pairs for quadratic curve fitting
//
// PROCEDURE:
// 1. Flash this sketch to your ESP32-S3
// 2. Connect your voltage divider circuit to BAT_ADC_PIN
// 3. Use a bench power supply or charged/discharging battery
// 4. At each stable voltage, note the ADC raw value from Serial Monitor
//    and measure the actual voltage with a multimeter simultaneously
// 5. Collect 6-8 pairs across the full range:
//      3S only:  9.0V → 12.6V
//      3S + 4S:  9.0V → 16.8V
// 6. Paste the pairs into https://arachnoid.com/polyfit
//    Set degree = 2, X = ADC raw, Y = actual voltage
//    Read off coefficients A (x²), B (x), C (constant)
// 7. Enter A, B, C into VDIV_A, VDIV_B, VDIV_C in your firmware
//
// SUGGESTED CALIBRATION VOLTAGES:
//   9.0V  (3S empty)
//   10.0V
//   11.0V
//   12.0V
//   12.6V (3S full)
//   13.5V (4S empty)   ← if calibrating for 4S
//   15.0V              ← if calibrating for 4S
//   16.8V (4S full)    ← if calibrating for 4S
// ────────────────────────────────────────────────────────────────

#define BAT_ADC_PIN 8 // IO8 — matches your flight firmware
#define NUM_SAMPLES 64 // samples averaged per reading
#define SAMPLE_DELAY_MS 5 // delay between samples
#define PRINT_INTERVAL_MS 2000 // print interval — time to read multimeter

void setup() {
  Serial.begin(115200);
  delay(2000);

  analogReadResolution(12); // 12-bit ADC = 0-4095
  analogSetAttenuation(ADC_11db); // 11dB attenuation = 0-3.3V range

  Serial.println();
  Serial.println("================================================");
  Serial.println("  BATTERY ADC CALIBRATION");
  Serial.println("================================================");
  Serial.println("  Pin:        IO8 (BAT_ADC_PIN)");
  Serial.println("  ADC res:    12-bit (0-4095)");
  Serial.println("  ADC atten:  11dB (0-3.3V input)");
  Serial.println();
  Serial.println("  For each reading:");
  Serial.println("  1. Set/wait for stable voltage");
  Serial.println("  2. Note ADC raw value printed below");
  Serial.println("  3. Measure actual voltage with multimeter");
  Serial.println("  4. Record both values as a pair");
  Serial.println();
  Serial.println("  Collect pairs across full voltage range then");
  Serial.println("  paste into arachnoid.com/polyfit (degree 2)");
  Serial.println("================================================");
  Serial.println();
  Serial.println("  Reading every 2 seconds...");
  Serial.println();
  Serial.println("  Sample# | ADC Raw | ADC Volts | Actual V (multimeter)");
  Serial.println("  --------|---------|-----------|----------------------");
}

void loop() {
  static int sampleNumber = 1;
  static unsigned long lastPrint = 0;

  unsigned long now = millis();
  if (now - lastPrint < PRINT_INTERVAL_MS) return;
  lastPrint = now;

  // Average NUM_SAMPLES reads
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(BAT_ADC_PIN);
    delay(SAMPLE_DELAY_MS);
  }
  float adcRaw = sum / (float)NUM_SAMPLES;
  float adcVolts = adcRaw * (3.3f / 4095.0f); // raw ADC input voltage

  // Print in table format
  Serial.printf("  %7d | %7.2f | %9.4f | <-- measure with multimeter\n",
                sampleNumber++, adcRaw, adcVolts);
}
