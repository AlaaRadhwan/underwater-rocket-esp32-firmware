#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"   // SparkFun ICM-20948 IMU
#include "Servo_Controller.h"
#include "Servo_Module.h"
#include "Motor_Module.h"
#include "ps5Controller.h"

// ===== OTA / Wi-Fi (AP-only) =====
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>    // for bandwidth/tx power tuning

// ---------- Logging control ----------
// 0 = no logs, 1 = important, 2 = verbose
#define LOG_LEVEL 1

#if LOG_LEVEL >= 1
  #define LOGI(...)        Serial.printf(__VA_ARGS__)
  #define LOGILN(lit_str)  Serial.println(F(lit_str))   // literal-only
#else
  #define LOGI(...)
  #define LOGILN(lit_str)
#endif

#if LOG_LEVEL >= 2
  #define LOGV(...)        Serial.printf(__VA_ARGS__)
  #define LOGVLN(lit_str)  Serial.println(F(lit_str))   // literal-only
#else
  #define LOGV(...)
  #define LOGVLN(lit_str)
#endif

// ---------- Wi-Fi / OTA creds ----------
#define AP_SSID      "Rocket-ESP32-OTA"
#include "secrets.h"      // UWR_AP_PASSWORD, UWR_PS5_MAC (not committed; see secrets.h.example)
#define AP_PASS      UWR_AP_PASSWORD   // SANITIZED: original hard-coded value removed
#define OTA_HOSTNAME "Rocket-ESP32"
#define OTA_PORT     3232              // match platformio.ini --port
// ====== Wiring / settings ======
#define SDA_PIN     21
#define SCL_PIN     22
#define I2C_HZ      100000      // keep conservative while PCA + IMU share bus
#define AD0_VAL     1           // 0 -> 0x68, 1 -> 0x69
#define SERIAL_BAUD 115200

// ====== Calibration settings ======
#define ACC_IN_MG       true
#define CALIB_SAMPLES   500
#define CALIB_DELAY_MS  5

// ====== PS5 MAC (pairing address) =====
static const char* PS5_MAC = UWR_PS5_MAC;   // SANITIZED: original hard-coded pairing address removed

// ====== Globals ======
ICM_20948_I2C imu;

// Biases (computed at startup)
float gyroBias[3]  = {0,0,0};  // dps
float accelBias[3] = {0,0,0};  // g

static inline float to_g(float mg_or_g) {
  return ACC_IN_MG ? (mg_or_g / 1000.0f) : mg_or_g;
}

// ==== Attitude filter globals ====
float q0=1, q1=0, q2=0, q3=0;      // quaternion
float twoKp = 2.0f * 0.5f;         // Proportional gain (Kp = 0.5)
float twoKi = 2.0f * 0.0f;         // Integral gain (Ki = 0)
float ix=0, iy=0, iz=0;            // Integral error
bool  yawZeroed=false;
float yaw0=0.0f;

// Simple 8-sample moving average on gz
const int GZ_WIN=8;
float gzBuf[GZ_WIN]={0};
int   gzIdx=0;
int   gzFilled=0;
float gzMA=0;

// === Output timing ===
bool headerPrinted = false;
unsigned long t0_ms = 0;
unsigned long lastMicros = 0;

// ====== ZARU (online bias adaptation) ======
float aMagLP = 1.0f, aDevLP = 0.0f;     // accel magnitude LP + deviation LP
const float GYRO_STILL_DPS = 0.2f;
const float AMAG_TOL_G     = 0.03f;
const float ADEV_TOL_G     = 0.005f;
const float BIAS_TAU_S     = 10.0f;

// ====== Control / UI ======
Servo_Controller servo(0, 2, 3, 1);
Motor_Module     motor(27);
Servo_Module     esc(4);

uint16_t manualThrottle = 1000;
bool killSwitchEngaged = false;
bool manualMode = false;  // false = AUTO, true = MANUAL
bool init_target_angles = true;

// ====== Heading lock on forward motion ======
const uint16_t THROTTLE_MOVE_US       = 1200;   // > idle => "moving"
const uint32_t HEADING_LOCK_DELAY_MS  = 300;    // must stay above threshold this long
bool     headingLocked = false;
uint32_t headingLockT0 = 0;
float    yawRefDeg     = 0.0f;

// Triangle-press heading lock request
bool triangleLockRequest = false;

// ====== Robust I2C helpers ======
const int SDA_PIN_REC = SDA_PIN;
const int SCL_PIN_REC = SCL_PIN;
uint32_t lastIMUokMs = 0;

bool i2cBusRecover() {
  pinMode(SDA_PIN_REC, INPUT_PULLUP);
  pinMode(SCL_PIN_REC, INPUT_PULLUP);
  delay(1);
  for (int i = 0; i < 9 && digitalRead(SDA_PIN_REC) == LOW; ++i) {
    pinMode(SCL_PIN_REC, OUTPUT);
    digitalWrite(SCL_PIN_REC, LOW);
    delayMicroseconds(5);
    pinMode(SCL_PIN_REC, INPUT_PULLUP);
    delayMicroseconds(5);
  }
  pinMode(SDA_PIN_REC, OUTPUT);
  digitalWrite(SDA_PIN_REC, LOW);
  delayMicroseconds(5);
  pinMode(SCL_PIN_REC, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(SDA_PIN_REC, INPUT_PULLUP);
  delayMicroseconds(5);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);
  Wire.setTimeOut(50);
  return (digitalRead(SDA_PIN_REC) == HIGH) && (digitalRead(SCL_PIN_REC) == HIGH);
}

bool readIMU_safe() {
  for (int t = 0; t < 3; ++t) {
    if (!imu.dataReady()) { delay(2); continue; }
    (void)imu.getAGMT();           // fills imu.agmt and updates imu.status
    if (imu.status == ICM_20948_Stat_Ok) {
      lastIMUokMs = millis();
      return true;
    }
    delay(5);
  }
  return false;
}

void recoverIMU() {
  i2cBusRecover();             // free bus if stuck
  imu.swReset();               // soft reset the IMU
  delay(10);
  imu.begin(Wire, AD0_VAL);    // re-begin
  delay(5);
}

void imuWatchdog(uint32_t staleMs = 500) {
  if (millis() - lastIMUokMs > staleMs) {
    recoverIMU();
  }
}

// Throttle PCA9685 write rate to avoid clumping 4 writes together
bool allowServoWrite() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= 3) { last = now; return true; } // 3–5 ms spacing helps
  return false;
}

// ==== IMU init & calibration ====
bool initIMU() {
  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);
  Wire.setTimeOut(50);                 // helps avoid 263-style timeouts
  ICM_20948_Status_e status = ICM_20948_Stat_Err;
  for (int i = 0; i < 10; i++) {
    status = imu.begin(Wire, AD0_VAL);
    if (status == ICM_20948_Stat_Ok) return true;
    delay(100);
  }
  return false;
}

void calibrateStill() {
  LOGILN("\n[Calib] Keep IMU still...");
  delay(500);

  double axSum=0, aySum=0, azSum=0;
  double gxSum=0, gySum=0, gzSum=0;
  uint16_t n = 0;

  while (n < CALIB_SAMPLES) {
    if (imu.dataReady()) {
      imu.getAGMT();

      float ax = to_g(imu.accX());
      float ay = to_g(imu.accY());
      float az = to_g(imu.accZ());

      float gx = imu.gyrX();
      float gy = imu.gyrY();
      float gz = imu.gyrZ();

      axSum += ax; aySum += ay; azSum += az;
      gxSum += gx; gySum += gy; gzSum += gz;
      n++;
    }
    delay(CALIB_DELAY_MS);
  }

  float axm = axSum / n, aym = aySum / n, azm = azSum / n;
  float gxm = gxSum / n, gym = gySum / n, gzm = gzSum / n;

  gyroBias[0] = gxm;
  gyroBias[1] = gym;
  gyroBias[2] = gzm;

  float normA = sqrtf(axm*axm + aym*aym + azm*azm);
  if (normA < 1e-6f) normA = 1.0f;

  float gxdir = axm / normA;
  float gydir = aym / normA;
  float gzdir = azm / normA;

  accelBias[0] = axm - gxdir;
  accelBias[1] = aym - gydir;
  accelBias[2] = azm - gzdir;

  LOGILN("[Calib] Complete.");
}

// === Helpers ===
static inline float clampf(float x, float a, float b){ return x < a ? a : (x > b ? b : x); }
float mapStick(int val, float scale = 30.0) {
  return constrain(val * scale / 128.0, -scale, scale);
}

// Convert quaternion to R/P/Y (deg)
static inline void quatToRPYdeg(float w,float x,float y,float z,
                                float &roll,float &pitch,float &yaw){
  float siny = 2.0f * (w*y - z*x);
  pitch = asinf(clampf(siny, -1.0f, 1.0f)) * 180.0f / PI;

  float sinr_cosp = 2.0f * (w*x + y*z);
  float cosr_cosp = 1.0f - 2.0f * (x*x + y*y);
  roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

  float siny_cosp = 2.0f * (w*z + x*y);
  float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
  yaw  = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
}

// Mahony 6-axis update (no mag). Units: accel in g, gyro in rad/s, dt in s
void mahony6Update(float ax,float ay,float az, float gx,float gy,float gz, float dt){
  float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (norm < 1e-6f) return;
  ax/=norm; ay/=norm; az/=norm;

  float vx = 2.0f*(q1*q3 - q0*q2);
  float vy = 2.0f*(q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  float ex = (ay*vz - az*vy);
  float ey = (az*vx - ax*vz);
  float ez = (ax*vy - ay*vx);

  if (twoKi > 0.0f){
    ix += twoKi * ex * dt;
    iy += twoKi * ey * dt;
    iz += twoKi * ez * dt;
  } else {
    ix = iy = iz = 0.0f;
  }

  gx += twoKp * ex + ix;
  gy += twoKp * ey + iy;
  gz += twoKp * ez + iz;

  float qw = q0, qx = q1, qy = q2, qz = q3;
  float halfDt = 0.5f * dt;
  q0 += (-qx*gx - qy*gy - qz*gz) * halfDt;
  q1 += ( qw*gx + qy*gz - qz*gy) * halfDt;
  q2 += ( qw*gy - qx*gz + qz*gx) * halfDt;
  q3 += ( qw*gz + qx*gy - qy*gx) * halfDt;

  float n = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (n > 0.0f){ q0/=n; q1/=n; q2/=n; q3/=n; }
}

// Online gyro-bias update (when device is clearly still)
void updateGyroBiasOnline(float ax_g, float ay_g, float az_g,
                          float gx_raw, float gy_raw, float gz_raw,
                          float dt)
{
  float amag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  float alpha = 1.0f - expf(-2.0f * PI * 2.0f * dt);  // ~2 Hz LP
  float err = fabsf(amag - aMagLP);
  aMagLP += alpha * (amag - aMagLP);
  aDevLP += alpha * (err   - aDevLP);

  bool gyroQuiet = (fabsf(gx_raw) < GYRO_STILL_DPS &&
                    fabsf(gy_raw) < GYRO_STILL_DPS &&
                    fabsf(gz_raw) < GYRO_STILL_DPS);
  bool accelNear1g = (fabsf(amag - 1.0f) < AMAG_TOL_G);
  bool accelQuiet  = (aDevLP < ADEV_TOL_G);

  bool still = gyroQuiet && accelNear1g && accelQuiet;
  if (!still) return;

  float a = dt / BIAS_TAU_S;
  gyroBias[0] = (1.0f - a) * gyroBias[0] + a * gx_raw;
  gyroBias[1] = (1.0f - a) * gyroBias[1] + a * gy_raw;
  gyroBias[2] = (1.0f - a) * gyroBias[2] + a * gz_raw;
}

// ===== OTA task (core 0), high priority, AP tuned =====
TaskHandle_t otaTaskHandle = nullptr;
volatile bool g_otaActive = false;

void otaTask(void*) {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  WiFi.softAPConfig(IPAddress(192,168,4,1),
                    IPAddress(192,168,4,1),
                    IPAddress(255,255,255,0));
  WiFi.softAP(AP_SSID, AP_PASS, 1 /*ch*/, 0 /*visible*/, 1 /*clients*/);

  // Use ESP-IDF APIs for compatibility with your Arduino core
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  esp_wifi_set_max_tx_power(78);  // 19.5 dBm

  IPAddress apIP = WiFi.softAPIP();
  LOGI("[OTA] AP up. SSID: %s  IP: %s\n", AP_SSID, apIP.toString().c_str());

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(AP_PASS);
  ArduinoOTA.setPort(OTA_PORT);

  ArduinoOTA.onStart([](){
    g_otaActive = true;
    motor.setThrottle(1000);
    esc.writeMicroseconds(1000);
    servo.resetServos(90);
    ps5.end();
    LOGILN("OTA start");
  });
  ArduinoOTA.onEnd([](){
    LOGILN("OTA end");
    delay(200);
    ps5.begin(PS5_MAC);
    g_otaActive = false;
  });
  ArduinoOTA.onError([](ota_error_t e){
    LOGI("OTA error: %d\n", e);
    delay(200);
    ps5.begin(PS5_MAC);
    g_otaActive = false;
  });

  ArduinoOTA.begin();

  for (;;) {
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ===== setup / loop =====
void setup() {
  Serial.begin(SERIAL_BAUD);

  // Start OTA FIRST (lifeline), higher priority on core 0
  xTaskCreatePinnedToCore(
    otaTask, "otaTask",
    4096,
    nullptr,
    4,           // higher prio favors OTA radio/CPU time
    &otaTaskHandle,
    0            // run on core 0
  );

  // Maintenance window: give yourself ~15 s where only OTA runs (optional but helpful)
  uint32_t tStart = millis();
  while (millis() - tStart < 5000) {
    if (g_otaActive) break;  // if OTA started, stay paused until it ends
    delay(50);
  }

  if (!g_otaActive) {
    if (!initIMU()) {
      LOGILN("IMU init failed. Check wiring/address.");
      // OTA still available, so we can halt safely
      while (true) delay(1000);
    }
    calibrateStill();

    t0_ms = millis();
    headerPrinted = false;
    yawZeroed = false;

    servo.begin();
    motor.begin();
    motor.armESC();

    // PCA I2C setup (same shared bus, slow it too)
    Servo_Module::configureBus(0x40, SDA_PIN, SCL_PIN, I2C_HZ, 50);

    ps5.begin(PS5_MAC);

    esc.writeMicroseconds(1000);
    servo.resetServos(90);
    servo.initServoAngles();
  }
}

void loop() {
  // If OTA is in progress, pause everything to keep radio happy
  if (g_otaActive) { delay(10); return; }

  // ===== PS5 connection handling =====
  static bool wasConnected = false;
  static bool triPrev = false;

  if (ps5.isConnected()) {
    if (!wasConnected) {
      LOGILN("PS5 controller connected");
      wasConnected = true;
    }

    // Emergency Kill
    if (ps5.Circle() && !killSwitchEngaged) {
      killSwitchEngaged = true;
      servo.resetServos(90);
      headingLocked = false;
      headingLockT0 = 0;
      LOGILN("Emergency Kill engaged");
    }
    if (killSwitchEngaged && ps5.Cross()) {
      killSwitchEngaged = false;
      LOGILN("Kill cleared. (Armed)");
    }

    // Mode toggles (quiet by default to save space)
    if (ps5.R1() && ps5.Cross()) {  // Manual
      manualMode = true;
    }
    if (ps5.L1() && ps5.Cross()) {  // Auto
      manualMode = false;
    }

    if (ps5.Square() && manualMode) {
      // triangleLockRequest = true;
      manualMode = false;
      delay(1000 * 5);
      triangleLockRequest = true;
      delay(1000);
      esc.writeMicroseconds(1500);
      delay(4000);
      esc.writeMicroseconds(1000);
    }

    // Heading lock on Triangle press (rising edge)
    bool triNow = ps5.Triangle();
    if (triNow && !triPrev && !killSwitchEngaged) {
      triangleLockRequest = true;  // act after IMU update for freshest yaw
    }
    triPrev = triNow;

    // Manual fins (blocked by kill)
    if (!killSwitchEngaged && manualMode) {
      if (allowServoWrite()) {
        float pitchDelta = mapStick(ps5.RStickY());
        float yawDelta   = mapStick(ps5.RStickX(), 20.0);
        servo.manualCombined(0, pitchDelta, yawDelta);
      }
    }

    // D-Pad Throttle step
    static bool upPrev = false, downPrev = false;
    bool upNow = ps5.Up();
    bool downNow = ps5.Down();

    if (upNow && !upPrev) {
      manualThrottle = min<uint16_t>(manualThrottle + 100, 2000);
      LOGV("Throttle increased to %d us\n", manualThrottle);
    }
    if (downNow && !downPrev) {
      manualThrottle = max<uint16_t>(manualThrottle - 100, 1000);
      LOGV("Throttle decreased to %d us\n", manualThrottle);
    }
    upPrev = upNow;
    downPrev = downNow;

    motor.setThrottle(manualThrottle);
    esc.writeMicroseconds(manualThrottle);

  } else {
    if (wasConnected) {
      LOGILN("PS5 disconnected. Restarting listener...");
      ps5.end();
      delay(200);
      ps5.begin(PS5_MAC);
      wasConnected = false;
    }
  }

  // ===== IMU processing =====
  if (!readIMU_safe()) {
    imuWatchdog(500);
    delay(1);
    return;
  }

  // RAW readings (before bias)
  float ax_g_raw = to_g(imu.accX());
  float ay_g_raw = to_g(imu.accY());
  float az_g_raw = to_g(imu.accZ());
  float gx_raw   = imu.gyrX();   // dps
  float gy_raw   = imu.gyrY();
  float gz_raw   = imu.gyrZ();

  // dt (s) with clamp
  unsigned long now = micros();
  float dt = (lastMicros==0) ? 0.005f : (now - lastMicros) * 1e-6f;
  lastMicros = now;
  dt = fminf(fmaxf(dt, 0.0005f), 0.02f);  // 0.5–20 ms

  // Online bias adaptation (only when clearly still)
  updateGyroBiasOnline(ax_g_raw, ay_g_raw, az_g_raw, gx_raw, gy_raw, gz_raw, dt);

  // Bias-corrected readings
  float ax = ax_g_raw - accelBias[0];
  float ay = ay_g_raw - accelBias[1];
  float az = az_g_raw - accelBias[2];
  float gx = gx_raw   - gyroBias[0];
  float gy = gy_raw   - gyroBias[1];
  float gz = gz_raw   - gyroBias[2];

  // Moving average on gz for smoother yaw integration
  gzMA -= gzBuf[gzIdx];
  gzBuf[gzIdx] = gz;
  gzMA += gz;
  gzIdx = (gzIdx + 1) % GZ_WIN;
  if (gzFilled < GZ_WIN) gzFilled++;
  float gzFiltered = gzMA / (float)gzFilled;

  // Mahony update
  const float D2R = PI/180.0f;
  mahony6Update(ax, ay, az, gx*D2R, gy*D2R, gzFiltered*D2R, dt);

  // Roll / Pitch / Yaw (yaw zeroed at start for logging only)
  float rollDeg, pitchDeg, yawDeg;
  quatToRPYdeg(q0, q1, q2, q3, rollDeg, pitchDeg, yawDeg);

  if (!yawZeroed) { yaw0 = yawDeg; yawZeroed = true; }
  float yawRel = yawDeg - yaw0;
  if (yawRel > 180.0f)  yawRel -= 360.0f;
  if (yawRel < -180.0f) yawRel += 360.0f;

  // Initialize target angles at start (placeholder until heading lock)
  if (init_target_angles) {
    servo.setTargetAngles(pitchDeg, yawDeg);
    init_target_angles = false;
  }

  // Triangle-triggered heading lock
  if (triangleLockRequest && !killSwitchEngaged) {
    yawRefDeg = yawDeg;
    servo.setTargetAngles(pitchDeg, yawRefDeg);
    headingLocked = true;
    headingLockT0 = millis();
    triangleLockRequest = false;
    LOGI("Heading LOCKED via Triangle at %.2f deg\n", yawRefDeg);
  }

  // Heading lock on forward motion
  if (!killSwitchEngaged) {
    if (manualThrottle >= THROTTLE_MOVE_US) {
      if (!headingLocked) {
        if (headingLockT0 == 0) headingLockT0 = millis();
        if (millis() - headingLockT0 >= HEADING_LOCK_DELAY_MS) {
          yawRefDeg = yawDeg;
          servo.setTargetAngles(pitchDeg, yawRefDeg);
          headingLocked = true;
          LOGI("Heading LOCKED at %.2f deg\n", yawRefDeg);
        }
      }
    } else {
      headingLockT0 = 0;
      if (headingLocked) { headingLocked = false; LOGILN("Heading UNLOCK (throttle low)"); }
    }
  }

  // AUTO mode fin control (blocked by kill)
  if (!manualMode && !killSwitchEngaged){
    if (allowServoWrite()) {
      servo.update(pitchDeg, yawDeg);
    }
  }

#if LOG_LEVEL >= 2
  // Optional CSV stream (verbose only)
  if (!headerPrinted) {
    Serial.println(F("time_s,roll_deg,pitch_deg,yaw_deg"));
    headerPrinted = true;
  }
  float t = (millis() - t0_ms) * 0.001f;
  Serial.print(t, 3);     Serial.print(',');
  Serial.print(rollDeg, 2);  Serial.print(',');
  Serial.print(pitchDeg, 2); Serial.print(',');
  Serial.println(yawRel, 2);
#endif

  // ~200 Hz
  delay(5);
}
