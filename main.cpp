#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "SMotor2.h"
#include "ZebraServo.h"
#include "ZebraScreen.h"

// =====================================================
// Hardware
// =====================================================
ZebraServo steeringServo(3);
SMotor2 driveMotor(1);
ZebraScreen screen(0);

// =====================================================
// Mux / MPU6050 config
// =====================================================
static const uint8_t MUX_ADDR = 0x70;
static const uint8_t MPU_PORT = 7;
static const uint8_t MPU_ADDR = 0x68;

// MPU6050 registers
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// =====================================================
// Steering state (relative servo)
// =====================================================
float currentSteer = 0.0f;

const float STEER_MIN = -30.0f;
const float STEER_MAX =  30.0f;
const float STEER_DEADBAND = 0.3f;

// =====================================================
// Drive settings
// =====================================================
const int DRIVE_DIR = -1;
const int DRIVE_SPEED = 180;
const int DRIVE_STEP_DEGREES = 120;

// =====================================================
// PID heading hold
// =====================================================
float targetYaw = 0.0f;

float Kp = 3.5f;
float Ki = 0.20f;
float Kd = 0.35f;

float integralTerm = 0.0f;
float prevError = 0.0f;
unsigned long prevPidMs = 0;

const float YAW_DEADBAND = 0.08f;
const float PID_OUTPUT_LIMIT = 18.0f;

// =====================================================
// MPU / gyro state
// =====================================================
uint8_t buf14[14];
float gzOffset = 0.0f;
bool mpuReady = false;

// Instantaneous values
float yawDeg = 0.0f;
float lastGzCorr = 0.0f;
float lastTempC = 0.0f;
float lastAzG = 0.0f;

// =====================================================
// Historical gyro buffer
// =====================================================
static const int GYRO_HISTORY_SIZE = 25;
float gzHistory[GYRO_HISTORY_SIZE];
float dtHistory[GYRO_HISTORY_SIZE];
int gyroHistHead = 0;
int gyroHistCount = 0;

// Filtered state derived from history
float filteredGz = 0.0f;
float filteredYaw = 0.0f;

// =====================================================
// Task timing
// =====================================================
unsigned long loopCount = 0;

unsigned long lastGyroSampleMs = 0;
const unsigned long GYRO_SAMPLE_INTERVAL_MS = 10;

unsigned long lastControlMs = 0;
const unsigned long CONTROL_INTERVAL_MS = 25;

unsigned long lastDriveMs = 0;
const unsigned long DRIVE_INTERVAL_MS = 20;

unsigned long lastScreenMs = 0;
const unsigned long SCREEN_INTERVAL_MS = 120;

// =====================================================
// Helpers
// =====================================================
float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void showLines(
  const String& l0,
  const String& l1 = "",
  const String& l2 = "",
  const String& l3 = "",
  const String& l4 = "",
  const String& l5 = "",
  const String& l6 = ""
) {
  screen.clear();
  screen.writeLine(0, l0);
  screen.writeLine(1, l1);
  screen.writeLine(2, l2);
  screen.writeLine(3, l3);
  screen.writeLine(4, l4);
  screen.writeLine(5, l5);
  screen.writeLine(6, l6);
}

void selectMuxPort(uint8_t port) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << port);
  Wire.endTransmission();
  delayMicroseconds(200);
}

bool probeAddr(uint8_t addr) {
  selectMuxPort(MPU_PORT);
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool write8(uint8_t reg, uint8_t val) {
  selectMuxPort(MPU_PORT);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readBlock(uint8_t reg, uint8_t* dst, size_t len) {
  selectMuxPort(MPU_PORT);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t got = Wire.requestFrom((int)MPU_ADDR, (int)len);
  if (got != len) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    dst[i] = Wire.read();
  }

  return true;
}

int16_t be16(const uint8_t* p) {
  return (int16_t)((p[0] << 8) | p[1]);
}

// =====================================================
// MPU init / read
// =====================================================
bool initMPU() {
  if (!write8(REG_PWR_MGMT_1, 0x00)) return false;
  delay(100);

  if (!write8(REG_SMPLRT_DIV, 9)) return false;
  if (!write8(REG_CONFIG, 0x03)) return false;
  if (!write8(REG_GYRO_CONFIG, 0x00)) return false;
  if (!write8(REG_ACCEL_CONFIG, 0x00)) return false;

  delay(20);
  return true;
}

bool readScaled(
  float& ax_g, float& ay_g, float& az_g,
  float& gx_dps, float& gy_dps, float& gz_dps,
  float& temp_c
) {
  if (!readBlock(REG_ACCEL_XOUT_H, buf14, sizeof(buf14))) {
    return false;
  }

  int16_t ax = be16(&buf14[0]);
  int16_t ay = be16(&buf14[2]);
  int16_t az = be16(&buf14[4]);
  int16_t temp_raw = be16(&buf14[6]);
  int16_t gx = be16(&buf14[8]);
  int16_t gy = be16(&buf14[10]);
  int16_t gz = be16(&buf14[12]);

  ax_g = ax / 16384.0f;
  ay_g = ay / 16384.0f;
  az_g = az / 16384.0f;
  gx_dps = gx / 131.0f;
  gy_dps = gy / 131.0f;
  gz_dps = gz / 131.0f;
  temp_c = temp_raw / 340.0f + 36.53f;

  return true;
}

bool calibrateGyro() {
  const int samples = 250;
  float sum = 0.0f;

  for (int i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz, tc;
    if (!readScaled(ax, ay, az, gx, gy, gz, tc)) {
      return false;
    }
    sum += gz;
    delay(5);
  }

  gzOffset = sum / samples;
  return true;
}

// =====================================================
// Historical filtering
// =====================================================
void clearGyroHistory() {
  gyroHistHead = 0;
  gyroHistCount = 0;
  for (int i = 0; i < GYRO_HISTORY_SIZE; i++) {
    gzHistory[i] = 0.0f;
    dtHistory[i] = 0.0f;
  }
}

void pushGyroSample(float gzCorr, float dt) {
  gzHistory[gyroHistHead] = gzCorr;
  dtHistory[gyroHistHead] = dt;

  gyroHistHead = (gyroHistHead + 1) % GYRO_HISTORY_SIZE;
  if (gyroHistCount < GYRO_HISTORY_SIZE) {
    gyroHistCount++;
  }
}

float computeAverageGz() {
  if (gyroHistCount == 0) return 0.0f;

  float sum = 0.0f;
  for (int i = 0; i < gyroHistCount; i++) {
    sum += gzHistory[i];
  }
  return sum / gyroHistCount;
}

float computeWeightedAverageGz() {
  if (gyroHistCount == 0) return 0.0f;

  float weightedSum = 0.0f;
  float weightSum = 0.0f;

  // newest samples get higher weight
  for (int i = 0; i < gyroHistCount; i++) {
    int idx = (gyroHistHead - 1 - i + GYRO_HISTORY_SIZE) % GYRO_HISTORY_SIZE;
    float weight = (float)(gyroHistCount - i);
    weightedSum += gzHistory[idx] * weight;
    weightSum += weight;
  }

  if (weightSum <= 0.0f) return 0.0f;
  return weightedSum / weightSum;
}

float integrateHistoryToYaw() {
  if (gyroHistCount == 0) return yawDeg;

  float sum = 0.0f;
  for (int i = 0; i < gyroHistCount; i++) {
    sum += gzHistory[i] * dtHistory[i];
  }

  // Use the recent-window integral as a stabilized heading estimate
  return yawDeg - sum + sum;
}

// =====================================================
// Sampling / state update
// =====================================================
bool sampleGyroTask() {
  float ax, ay, az, gx, gy, gz, tc;
  if (!readScaled(ax, ay, az, gx, gy, gz, tc)) {
    return false;
  }

  unsigned long now = millis();
  float dt = (now - lastGyroSampleMs) / 1000.0f;
  lastGyroSampleMs = now;
  if (dt < 0.0001f) dt = 0.01f;

  float gzCorr = gz - gzOffset;

  lastGzCorr = gzCorr;
  lastTempC = tc;
  lastAzG = az;

  // Integrate instantaneous yaw
  yawDeg += gzCorr * dt;

  // Push into history for smoothing
  pushGyroSample(gzCorr, dt);

  // Derived filtered values from history
  filteredGz = computeWeightedAverageGz();

  // Smoothed yaw: integrate filtered rate over the same sample interval
  filteredYaw += filteredGz * dt;

  return true;
}

void resetController() {
  yawDeg = 0.0f;
  filteredYaw = 0.0f;
  filteredGz = 0.0f;
  lastGzCorr = 0.0f;

  targetYaw = 0.0f;
  integralTerm = 0.0f;
  prevError = 0.0f;
  prevPidMs = millis();

  lastGyroSampleMs = millis();
  clearGyroHistory();
}

// =====================================================
// PID
// =====================================================
float computePID(float measuredYaw, float &errorOut, float &dtOut, bool &inDeadband) {
  unsigned long now = millis();
  float dt = (prevPidMs == 0) ? 0.02f : (now - prevPidMs) / 1000.0f;
  prevPidMs = now;

  if (dt <= 0.0f) dt = 0.02f;
  dtOut = dt;

  float error = targetYaw - measuredYaw;
  errorOut = error;

  inDeadband = false;
  if (fabs(error) < YAW_DEADBAND) {
    error = 0.0f;
    inDeadband = true;
  }

  integralTerm += error * dt;
  integralTerm = clampf(integralTerm, -30.0f, 30.0f);

  float derivative = (error - prevError) / dt;
  prevError = error;

  float output = (Kp * error) + (Ki * integralTerm) + (Kd * derivative);
  output = clampf(output, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);

  return output;
}

// =====================================================
// Relative steering
// =====================================================
void applyRelativeSteering(float desiredAngle) {
  desiredAngle = clampf(desiredAngle, STEER_MIN, STEER_MAX);

  float delta = desiredAngle - currentSteer;
  if (fabs(delta) < STEER_DEADBAND) {
    return;
  }

  steeringServo.run_angles((int)delta);
  currentSteer = desiredAngle;
}

// =====================================================
// Display
// =====================================================
void updateDisplay(float error, float pidOut, float dt, bool db) {
  showLines(
    "Yaw:" + String(filteredYaw, 2),
    "gZf:" + String(filteredGz, 2),
    "gZi:" + String(lastGzCorr, 2),
    "Err:" + String(error, 2),
    "PID:" + String(pidOut, 2),
    "Str:" + String(currentSteer, 1),
    "Lp:" + String(loopCount)
  );
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  delay(50);

  driveMotor.begin();
  steeringServo.begin();
  screen.begin();

  showLines("Boot", "Scan mux p7");

  if (!probeAddr(MPU_ADDR)) {
    showLines("MPU not found", "port 7", "addr 0x68");
    Serial.println("MPU not found at 0x68 on mux port 7");
    return;
  }

  showLines("MPU found", "Init...");
  if (!initMPU()) {
    showLines("Init failed");
    Serial.println("MPU init failed");
    return;
  }

  showLines("Init OK", "Calibrating...");
  if (!calibrateGyro()) {
    showLines("Cal failed");
    Serial.println("Gyro calibration failed");
    return;
  }

  resetController();
  mpuReady = true;

  showLines("Ready", "gzOff:" + String(gzOffset, 2));
  Serial.print("Ready. gzOffset=");
  Serial.println(gzOffset, 4);
}

// =====================================================
// Loop
// =====================================================
void loop() {
  if (!mpuReady) {
    delay(500);
    return;
  }

  loopCount++;
  unsigned long now = millis();

  // 1) Fast gyro sampling task
  if (now - lastGyroSampleMs >= GYRO_SAMPLE_INTERVAL_MS) {
    if (!sampleGyroTask()) {
      showLines("READ FAILED", "port 7", "addr 0x68");
      Serial.println("MPU read failed");
      delay(100);
      return;
    }
  }

  // 2) Control task uses historical/filtered yaw
  static float lastError = 0.0f;
  static float lastPidOut = 0.0f;
  static float lastPidDt = 0.0f;
  static bool lastDeadband = false;

  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
    lastControlMs = now;

    lastPidOut = computePID(filteredYaw, lastError, lastPidDt, lastDeadband);

    // Flip sign if correction is backwards
    float desiredSteer = lastPidOut;
    applyRelativeSteering(desiredSteer);
  }

  // 3) Drive task
  if (now - lastDriveMs >= DRIVE_INTERVAL_MS) {
    lastDriveMs = now;
    driveMotor.move_degrees(DRIVE_STEP_DEGREES, DRIVE_SPEED * DRIVE_DIR);
  }

  // 4) Non-blocking screen task
  if (now - lastScreenMs >= SCREEN_INTERVAL_MS) {
    lastScreenMs = now;

    updateDisplay(lastError, lastPidOut, lastPidDt, lastDeadband);

    Serial.print("YawF=");
    Serial.print(filteredYaw, 3);
    Serial.print(" gZF=");
    Serial.print(filteredGz, 3);
    Serial.print(" gZI=");
    Serial.print(lastGzCorr, 3);
    Serial.print(" Err=");
    Serial.print(lastError, 3);
    Serial.print(" PID=");
    Serial.print(lastPidOut, 3);
    Serial.print(" Steer=");
    Serial.println(currentSteer, 2);
  }
}
