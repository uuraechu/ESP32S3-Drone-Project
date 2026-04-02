#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>

#define I2C_SCL 6
#define I2C_SDA 7

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void magCalibrationRoutine();

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!mag.begin()) {
    Serial.println("HMC5883L not found — check wiring!");
    while (1) delay(10);
  }

  Serial.println("HMC5883L ready.");
  Serial.println("Open Serial Monitor, then send any character to start.");

  // Wait for user to send a byte — gives time to open Serial Monitor
  while (!Serial.available()) delay(10);
  Serial.read(); // consume the trigger byte

  magCalibrationRoutine();

  Serial.println("Done. You can now upload your flight firmware.");
}

void loop() {
  // Nothing — calibration runs once in setup()
}

// Temporary calibration sketch — run once, read offsets from Serial
void magCalibrationRoutine() {
  float minX = 9999, maxX = -9999;
  float minY = 9999, maxY = -9999;
  float minZ = 9999, maxZ = -9999;

  Serial.println("Rotate drone through all orientations for 30 seconds...");
  unsigned long start = millis();

  while (millis() - start < 30000) {
    sensors_event_t event;
    mag.getEvent(&event);

    minX = min(minX, event.magnetic.x);
    maxX = max(maxX, event.magnetic.x);
    minY = min(minY, event.magnetic.y);
    maxY = max(maxY, event.magnetic.y);
    minZ = min(minZ, event.magnetic.z);
    maxZ = max(maxZ, event.magnetic.z);

    // Print live readings so you can see it's working
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      int remaining = (30000 - (int)(millis() - start)) / 1000;
      Serial.printf("X:%.2f Y:%.2f Z:%.2f  [%ds remaining]\n",
                    event.magnetic.x, event.magnetic.y, event.magnetic.z,
                    remaining);
    }

    delay(10);
  }

  // Hard iron offsets are the midpoint of min/max for each axis
  float offsetX = (maxX + minX) / 2.0f;
  float offsetY = (maxY + minY) / 2.0f;
  float offsetZ = (maxZ + minZ) / 2.0f;

  Serial.println("\n--- CALIBRATION COMPLETE ---");
  Serial.printf("Range X: [%.2f, %.2f]  span: %.2f\n", minX, maxX, maxX - minX);
  Serial.printf("Range Y: [%.2f, %.2f]  span: %.2f\n", minY, maxY, maxY - minY);
  Serial.printf("Range Z: [%.2f, %.2f]  span: %.2f\n", minZ, maxZ, maxZ - minZ);
  Serial.println();
  Serial.println("Copy these into your defines:");
  Serial.printf("#define MAG_OFFSET_X  %.4ff\n", offsetX);
  Serial.printf("#define MAG_OFFSET_Y  %.4ff\n", offsetY);
  Serial.printf("#define MAG_OFFSET_Z  %.4ff\n", offsetZ);
}
