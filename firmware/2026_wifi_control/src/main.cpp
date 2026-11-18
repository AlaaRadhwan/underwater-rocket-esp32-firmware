#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"        // SparkFun ICM-20948 IMU
#include "Servo_Controller.h"
#include "Servo_Module.h"

// ===== OTA / Wi-Fi AP =====
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>

// ============================================================
// Logging
// ============================================================
// 0 = no logs, 1 = important logs, 2 = telemetry logs
#define LOG_LEVEL 1

#if LOG_LEVEL >= 1
  #define LOGI(...)        Serial.printf(__VA_ARGS__)
  #define LOGILN(lit_str)  Serial.println(F(lit_str))
#else
  #define LOGI(...)
  #define LOGILN(lit_str)
#endif

#if LOG_LEVEL >= 2
  #define LOGV(...)        Serial.printf(__VA_ARGS__)
  #define LOGVLN(lit_str)  Serial.println(F(lit_str))
#else
  #define LOGV(...)
  #define LOGVLN(lit_str)
#endif

// ============================================================
// Wi-Fi / OTA
// ============================================================
#define AP_SSID      "Rocket-Control"
#include "secrets.h"        // UWR_AP_PASSWORD (not committed)
#define AP_PASS      UWR_AP_PASSWORD
#define OTA_HOSTNAME "Rocket-ESP32"
#define OTA_PORT     3232

// ============================================================
// Wiring / settings
// ============================================================
#define SDA_PIN     21
#define SCL_PIN     22
#define I2C_HZ      100000      // conservative because PCA9685 + IMU share I2C
#define AD0_VAL     1           // 0 -> 0x68, 1 -> 0x69
#define SERIAL_BAUD 115200

#define PCA9685_ADDR 0x40
#define PWM_FREQ_HZ  50

// ============================================================
// Channel mapping
// ============================================================
// Corrected mapping: fins use 0,1,2,3 and ESC uses 4.
// Change these only if your wiring is different. 2 0 3 1
Servo_Controller servo(2, 0, 3, 1);
Servo_Module     esc(4);

// ============================================================
// Calibration settings
// ============================================================
#define ACC_IN_MG       true
#define CALIB_SAMPLES   500
#define CALIB_DELAY_MS  5

// ============================================================
// IMU globals
// ============================================================
ICM_20948_I2C imu;

float gyroBias[3]  = {0, 0, 0};  // dps
float accelBias[3] = {0, 0, 0};  // g

static inline float to_g(float mg_or_g) {
  return ACC_IN_MG ? (mg_or_g / 1000.0f) : mg_or_g;
}

// ============================================================
// Attitude filter globals
// ============================================================
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float twoKp = 2.0f * 0.5f;       // Kp = 0.5
float twoKi = 2.0f * 0.0f;       // Ki = 0
float ix = 0, iy = 0, iz = 0;

bool yawZeroed = false;
float yaw0 = 0.0f;

// Moving average on gz
const int GZ_WIN = 8;
float gzBuf[GZ_WIN] = {0};
int   gzIdx = 0;
int   gzFilled = 0;
float gzMA = 0;

// Timing
unsigned long t0_ms = 0;
unsigned long lastMicros = 0;

// ============================================================
// ZARU / online gyro bias adaptation
// ============================================================
float aMagLP = 1.0f, aDevLP = 0.0f;
const float GYRO_STILL_DPS = 0.2f;
const float AMAG_TOL_G     = 0.03f;
const float ADEV_TOL_G     = 0.005f;
const float BIAS_TAU_S     = 10.0f;

// ============================================================
// Stabilization and motor state
// ============================================================
static bool stabilizationEnabled = false;

// D-pad motor throttle command.
// Down  = 1000
// Left  = 1400
// Right = 1600
// Up    = 1900
static uint16_t motorThrottleUs = 1000;

// If true, motor output ramps toward target instead of jumping instantly.
static const bool USE_THROTTLE_SLEW = true;
static const uint16_t IDLE_THROTTLE_US = 1000;
static const uint16_t MIN_THROTTLE_US  = 1000;
static const uint16_t MAX_THROTTLE_US  = 2000;
static const float THROTTLE_SLEW_US_PER_S = 850.0f;

static float currentThrottleUs = IDLE_THROTTLE_US;
static uint32_t lastThrottleSlewMicros = 0;

// ============================================================
// OTA task state
// ============================================================
TaskHandle_t otaTaskHandle = nullptr;
volatile bool g_otaActive = false;

// ============================================================
// Wi-Fi control state
// ============================================================
WebServer webServer(80);

enum class RemoteCommand : uint8_t {
  NONE,
  STABILIZE_ON,
  STABILIZE_OFF,
  RESET_REFERENCE,
  NEUTRAL_FINS,
  SET_THROTTLE
};

portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
volatile RemoteCommand pendingCommand = RemoteCommand::NONE;
volatile uint16_t pendingThrottleUs = IDLE_THROTTLE_US;

volatile float sharedPitchDeg = 0.0f;
volatile float sharedYawDeg = 0.0f;
volatile uint32_t lastControlHeartbeatMs = 0;
volatile bool controlClientSeen = false;

static const uint32_t CONTROL_TIMEOUT_MS = 2500;

// ============================================================
// Helper functions
// ============================================================
static inline float clampf(float x, float a, float b) {
  return x < a ? a : (x > b ? b : x);
}

static inline float norm180(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

static uint16_t slewThrottleToward(uint16_t targetUs) {
  uint32_t now = micros();

  float dt = (lastThrottleSlewMicros == 0)
             ? 0.005f
             : (now - lastThrottleSlewMicros) * 1e-6f;

  lastThrottleSlewMicros = now;
  dt = fminf(fmaxf(dt, 0.0005f), 0.02f);

  targetUs = constrain(targetUs, MIN_THROTTLE_US, MAX_THROTTLE_US);

  float step = THROTTLE_SLEW_US_PER_S * dt;

  if (currentThrottleUs < targetUs) {
    currentThrottleUs = fminf(currentThrottleUs + step, (float)targetUs);
  } else {
    currentThrottleUs = fmaxf(currentThrottleUs - step, (float)targetUs);
  }

  currentThrottleUs = fminf(fmaxf(currentThrottleUs,
                                  (float)MIN_THROTTLE_US),
                                  (float)MAX_THROTTLE_US);

  return (uint16_t)currentThrottleUs;
}

static void writeMotorOutput() {
  uint16_t outUs;

  if (USE_THROTTLE_SLEW) {
    outUs = slewThrottleToward(motorThrottleUs);
  } else {
    outUs = motorThrottleUs;
    currentThrottleUs = motorThrottleUs;
  }

  esc.writeMicroseconds(outUs);
}

static void forceMotorIdle() {
  motorThrottleUs = IDLE_THROTTLE_US;
  currentThrottleUs = IDLE_THROTTLE_US;
  lastThrottleSlewMicros = 0;
  esc.writeMicroseconds(IDLE_THROTTLE_US);
}

static void safeOutputs() {
  stabilizationEnabled = false;
  forceMotorIdle();
  servo.resetServos(90);
}

// Convert quaternion to roll/pitch/yaw in degrees
static inline void quatToRPYdeg(float w, float x, float y, float z,
                                float &roll, float &pitch, float &yaw) {
  float sinp = 2.0f * (w * y - z * x);
  pitch = asinf(clampf(sinp, -1.0f, 1.0f)) * 180.0f / PI;

  float sinr_cosp = 2.0f * (w * x + y * z);
  float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
}

// Mahony 6-axis update.
// Accel in g, gyro in rad/s, dt in seconds.
void mahony6Update(float ax, float ay, float az,
                   float gx, float gy, float gz,
                   float dt) {
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm < 1e-6f) return;

  ax /= norm;
  ay /= norm;
  az /= norm;

  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  float ex = (ay * vz - az * vy);
  float ey = (az * vx - ax * vz);
  float ez = (ax * vy - ay * vx);

  if (twoKi > 0.0f) {
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

  q0 += (-qx * gx - qy * gy - qz * gz) * halfDt;
  q1 += ( qw * gx + qy * gz - qz * gy) * halfDt;
  q2 += ( qw * gy - qx * gz + qz * gx) * halfDt;
  q3 += ( qw * gz + qx * gy - qy * gx) * halfDt;

  float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (n > 0.0f) {
    q0 /= n;
    q1 /= n;
    q2 /= n;
    q3 /= n;
  }
}

void updateGyroBiasOnline(float ax_g, float ay_g, float az_g,
                          float gx_raw, float gy_raw, float gz_raw,
                          float dt) {
  float amag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  float alpha = 1.0f - expf(-2.0f * PI * 2.0f * dt);

  float err = fabsf(amag - aMagLP);

  aMagLP += alpha * (amag - aMagLP);
  aDevLP += alpha * (err - aDevLP);

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

// ============================================================
// I2C / IMU recovery
// ============================================================
uint32_t lastIMUokMs = 0;

bool i2cBusRecover() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(1);

  for (int i = 0; i < 9 && digitalRead(SDA_PIN) == LOW; ++i) {
    pinMode(SCL_PIN, OUTPUT);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
    pinMode(SCL_PIN, INPUT_PULLUP);
    delayMicroseconds(5);
  }

  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);

  pinMode(SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(5);

  pinMode(SDA_PIN, INPUT_PULLUP);
  delayMicroseconds(5);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);
  Wire.setTimeOut(50);

  return (digitalRead(SDA_PIN) == HIGH) && (digitalRead(SCL_PIN) == HIGH);
}

bool initIMU() {
  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);
  Wire.setTimeOut(50);

  ICM_20948_Status_e status = ICM_20948_Stat_Err;

  for (int i = 0; i < 10; i++) {
    status = imu.begin(Wire, AD0_VAL);
    if (status == ICM_20948_Stat_Ok) {
      lastIMUokMs = millis();
      return true;
    }
    delay(100);
  }

  return false;
}

bool readIMU_safe() {
  for (int t = 0; t < 3; ++t) {
    if (!imu.dataReady()) {
      delay(2);
      continue;
    }

    (void)imu.getAGMT();

    if (imu.status == ICM_20948_Stat_Ok) {
      lastIMUokMs = millis();
      return true;
    }

    delay(5);
  }

  return false;
}

void recoverIMU() {
  i2cBusRecover();
  imu.swReset();
  delay(10);
  imu.begin(Wire, AD0_VAL);
  delay(5);
}

void imuWatchdog(uint32_t staleMs = 500) {
  if (millis() - lastIMUokMs > staleMs) {
    recoverIMU();
  }
}

void calibrateStill() {
  LOGILN("\n[Calib] Keep IMU still...");
  delay(500);

  double axSum = 0, aySum = 0, azSum = 0;
  double gxSum = 0, gySum = 0, gzSum = 0;
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

      axSum += ax;
      aySum += ay;
      azSum += az;

      gxSum += gx;
      gySum += gy;
      gzSum += gz;

      n++;
    }

    delay(CALIB_DELAY_MS);
  }

  float axm = axSum / n;
  float aym = aySum / n;
  float azm = azSum / n;

  float gxm = gxSum / n;
  float gym = gySum / n;
  float gzm = gzSum / n;

  gyroBias[0] = gxm;
  gyroBias[1] = gym;
  gyroBias[2] = gzm;

  float normA = sqrtf(axm * axm + aym * aym + azm * azm);
  if (normA < 1e-6f) normA = 1.0f;

  float gxdir = axm / normA;
  float gydir = aym / normA;
  float gzdir = azm / normA;

  accelBias[0] = axm - gxdir;
  accelBias[1] = aym - gydir;
  accelBias[2] = azm - gzdir;

  LOGILN("[Calib] Complete.");
}

// ============================================================
// Wi-Fi browser control
// ============================================================
static const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Rocket Control</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: #0b1220;
      color: #e8eef8;
      font-family: Arial, sans-serif;
      text-align: center;
    }
    .wrap {
      width: min(620px, 94vw);
      margin: 22px auto;
    }
    .card {
      background: #151f31;
      border: 1px solid #2a3850;
      border-radius: 16px;
      padding: 18px;
      margin-bottom: 16px;
    }
    h1 { margin: 6px 0 18px; font-size: 1.7rem; }
    h2 { margin: 0 0 14px; font-size: 1.15rem; }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 12px;
    }
    button {
      min-height: 62px;
      border: 0;
      border-radius: 12px;
      font-size: 1rem;
      font-weight: 700;
      color: white;
      background: #34558b;
      touch-action: manipulation;
    }
    button:active { transform: scale(0.98); }
    .danger { background: #b52f3a; }
    .safe { background: #217a4a; }
    .motor { background: #6b4ca5; }
    .status {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 8px;
      text-align: left;
      font-size: 0.95rem;
    }
    .value {
      background: #0f1726;
      border-radius: 9px;
      padding: 10px;
    }
    #link {
      margin-top: 12px;
      font-weight: 700;
    }
    .ok { color: #69df9b; }
    .bad { color: #ff7b84; }
  </style>
</head>
<body>
<div class="wrap">
  <h1>Underwater Rocket Control</h1>

  <div class="card">
    <h2>Stabilization</h2>
    <div class="grid">
      <button class="safe" onclick="cmd('stab_on')">Enable & Lock</button>
      <button class="danger" onclick="cmd('stab_off')">Disable</button>
      <button onclick="cmd('reset_ref')">Reset Reference</button>
      <button onclick="cmd('neutral')">Neutral Fins</button>
    </div>
  </div>

  <div class="card">
    <h2>Motor Throttle</h2>
    <div class="grid">
      <button class="danger" onclick="throttle(1000)">IDLE — 1000</button>
      <button class="motor" onclick="throttle(1400)">LOW — 1400</button>
      <button class="motor" onclick="throttle(1600)">MEDIUM — 1600</button>
      <button class="motor" onclick="throttle(1900)">HIGH — 1900</button>
    </div>
  </div>

  <div class="card">
    <h2>Status</h2>
    <div class="status">
      <div class="value">Stabilization: <span id="stab">--</span></div>
      <div class="value">Motor target: <span id="target">--</span> us</div>
      <div class="value">Motor output: <span id="output">--</span> us</div>
      <div class="value">Pitch: <span id="pitch">--</span>°</div>
      <div class="value">Yaw: <span id="yaw">--</span>°</div>
      <div class="value">Clients: <span id="clients">--</span></div>
    </div>
    <div id="link" class="bad">Connecting…</div>
  </div>
</div>

<script>
let lastGood = 0;

async function request(url) {
  try {
    const r = await fetch(url, {cache: 'no-store'});
    if (!r.ok) throw new Error('HTTP ' + r.status);
    lastGood = Date.now();
    return r;
  } catch (e) {
    document.getElementById('link').textContent = 'Connection lost — motor will return to idle';
    document.getElementById('link').className = 'bad';
    throw e;
  }
}

async function cmd(action) {
  await request('/api/cmd?action=' + encodeURIComponent(action));
}

async function throttle(value) {
  await request('/api/cmd?action=throttle&value=' + value);
}

async function heartbeat() {
  try {
    await request('/api/heartbeat');
  } catch (_) {}
}

async function updateState() {
  try {
    const r = await request('/api/state');
    const s = await r.json();

    document.getElementById('stab').textContent = s.stabilization ? 'ON' : 'OFF';
    document.getElementById('target').textContent = s.motorTarget;
    document.getElementById('output').textContent = s.motorOutput;
    document.getElementById('pitch').textContent = s.pitch.toFixed(1);
    document.getElementById('yaw').textContent = s.yaw.toFixed(1);
    document.getElementById('clients').textContent = s.clients;
    document.getElementById('link').textContent = 'Wi-Fi control connected';
    document.getElementById('link').className = 'ok';
  } catch (_) {}
}

setInterval(heartbeat, 500);
setInterval(updateState, 400);
heartbeat();
updateState();
</script>
</body>
</html>
)HTML";

static void queueRemoteCommand(RemoteCommand command,
                               uint16_t throttleUs = IDLE_THROTTLE_US) {
  portENTER_CRITICAL(&commandMux);
  pendingThrottleUs = throttleUs;
  pendingCommand = command;
  portEXIT_CRITICAL(&commandMux);
}

static void markControlAlive() {
  lastControlHeartbeatMs = millis();
  controlClientSeen = true;
}

static void handleRoot() {
  markControlAlive();
  webServer.send_P(200, "text/html", CONTROL_PAGE);
}

static void handleHeartbeat() {
  markControlAlive();
  webServer.send(200, "text/plain", "OK");
}

static void handleState() {
  markControlAlive();

  char json[256];
  snprintf(json, sizeof(json),
           "{\"stabilization\":%s,"
           "\"motorTarget\":%u,"
           "\"motorOutput\":%u,"
           "\"pitch\":%.2f,"
           "\"yaw\":%.2f,"
           "\"clients\":%u}",
           stabilizationEnabled ? "true" : "false",
           motorThrottleUs,
           (uint16_t)currentThrottleUs,
           sharedPitchDeg,
           sharedYawDeg,
           WiFi.softAPgetStationNum());

  webServer.send(200, "application/json", json);
}

static void handleCommandRequest() {
  markControlAlive();

  if (!webServer.hasArg("action")) {
    webServer.send(400, "text/plain", "Missing action");
    return;
  }

  String action = webServer.arg("action");

  if (action == "stab_on") {
    queueRemoteCommand(RemoteCommand::STABILIZE_ON);
  } else if (action == "stab_off") {
    queueRemoteCommand(RemoteCommand::STABILIZE_OFF);
  } else if (action == "reset_ref") {
    queueRemoteCommand(RemoteCommand::RESET_REFERENCE);
  } else if (action == "neutral") {
    queueRemoteCommand(RemoteCommand::NEUTRAL_FINS);
  } else if (action == "throttle") {
    if (!webServer.hasArg("value")) {
      webServer.send(400, "text/plain", "Missing throttle value");
      return;
    }

    int value = webServer.arg("value").toInt();
    if (value != 1000 && value != 1400 &&
        value != 1600 && value != 1900) {
      webServer.send(400, "text/plain", "Invalid throttle value");
      return;
    }

    queueRemoteCommand(RemoteCommand::SET_THROTTLE, (uint16_t)value);
  } else {
    webServer.send(400, "text/plain", "Unknown action");
    return;
  }

  webServer.send(200, "text/plain", "OK");
}

static void processRemoteCommand(float pitchDeg, float yawDeg) {
  RemoteCommand command = RemoteCommand::NONE;
  uint16_t throttleUs = IDLE_THROTTLE_US;

  portENTER_CRITICAL(&commandMux);
  command = pendingCommand;
  throttleUs = pendingThrottleUs;
  pendingCommand = RemoteCommand::NONE;
  portEXIT_CRITICAL(&commandMux);

  switch (command) {
    case RemoteCommand::STABILIZE_ON:
      stabilizationEnabled = true;
      servo.setTargetAngles(pitchDeg, yawDeg);
      LOGILN("[WiFi] Stabilization ENABLED. Current pitch/yaw locked.");
      break;

    case RemoteCommand::STABILIZE_OFF:
      stabilizationEnabled = false;
      servo.resetServos(90);
      LOGILN("[WiFi] Stabilization DISABLED. Fins neutral.");
      break;

    case RemoteCommand::RESET_REFERENCE:
      servo.setTargetAngles(pitchDeg, yawDeg);
      LOGILN("[WiFi] Reference reset to current pitch/yaw.");
      break;

    case RemoteCommand::NEUTRAL_FINS:
      servo.resetServos(90);
      LOGILN("[WiFi] Fins neutralized once.");
      break;

    case RemoteCommand::SET_THROTTLE:
      motorThrottleUs = constrain(throttleUs,
                                  MIN_THROTTLE_US,
                                  MAX_THROTTLE_US);
      LOGI("[WiFi] Motor throttle set to %u us.\n", motorThrottleUs);
      break;

    case RemoteCommand::NONE:
    default:
      break;
  }
}

static void controlLinkFailsafe() {
  if (!controlClientSeen) return;

  if (millis() - lastControlHeartbeatMs > CONTROL_TIMEOUT_MS) {
    if (motorThrottleUs != IDLE_THROTTLE_US || stabilizationEnabled) {
      LOGILN("[FAILSAFE] Wi-Fi control heartbeat lost. Motor idle; stabilization disabled.");
    }

    stabilizationEnabled = false;
    motorThrottleUs = IDLE_THROTTLE_US;
    controlClientSeen = false;
  }
}

// ============================================================
// OTA / web-server task
// ============================================================
void otaTask(void*) {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  WiFi.softAPConfig(IPAddress(192, 168, 4, 1),
                    IPAddress(192, 168, 4, 1),
                    IPAddress(255, 255, 255, 0));

  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 1);

  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  esp_wifi_set_max_tx_power(78);  // 19.5 dBm

  IPAddress apIP = WiFi.softAPIP();
  LOGI("[OTA] AP up. SSID: %s  IP: %s\n",
       AP_SSID, apIP.toString().c_str());

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(AP_PASS);
  ArduinoOTA.setPort(OTA_PORT);

  ArduinoOTA.onStart([]() {
    g_otaActive = true;
    safeOutputs();
    LOGILN("[OTA] Start. Outputs safe.");
  });

  ArduinoOTA.onEnd([]() {
    LOGILN("[OTA] End.");
    delay(200);
    g_otaActive = false;
  });

  ArduinoOTA.onError([](ota_error_t e) {
    LOGI("[OTA] Error: %d\n", e);
    safeOutputs();
    delay(200);
    g_otaActive = false;
  });

  ArduinoOTA.begin();

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/api/heartbeat", HTTP_GET, handleHeartbeat);
  webServer.on("/api/state", HTTP_GET, handleState);
  webServer.on("/api/cmd", HTTP_GET, handleCommandRequest);
  webServer.onNotFound([]() {
    webServer.send(404, "text/plain", "Not found");
  });
  webServer.begin();

  LOGILN("[WiFi] Control page: http://192.168.4.1");

  for (;;) {
    ArduinoOTA.handle();
    webServer.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  LOGILN("\n[BOOT] Stabilize-only firmware with Wi-Fi browser control.");

  // Configure PCA9685 first so outputs can be made safe early.
  Servo_Module::configureBus(PCA9685_ADDR, SDA_PIN, SCL_PIN, I2C_HZ, PWM_FREQ_HZ);

  servo.begin();
  safeOutputs();
  servo.initServoAngles();

  // Start OTA as lifeline on core 0.
  xTaskCreatePinnedToCore(
    otaTask,
    "otaTask",
    4096,
    nullptr,
    4,
    &otaTaskHandle,
    0
  );

  // Give OTA a short early window.
  uint32_t tStart = millis();
  while (millis() - tStart < 3000) {
    if (g_otaActive) break;
    delay(50);
  }

  if (!initIMU()) {
    LOGILN("[IMU] Init failed. Check wiring/address. OTA remains available.");
    while (true) {
      safeOutputs();
      delay(1000);
    }
  }

  calibrateStill();

  t0_ms = millis();
  lastMicros = 0;
  yawZeroed = false;


  safeOutputs();

  LOGILN("[READY] Connect to Wi-Fi SSID: " AP_SSID);
  LOGILN("[READY] Open http://192.168.4.1 in a browser.");
  LOGILN("[READY] Wi-Fi loss timeout: 2.5 s -> motor idle and stabilization disabled.");
}

void loop() {
  if (g_otaActive) {
    safeOutputs();
    delay(10);
    return;
  }

  if (!readIMU_safe()) {
    // Safety behavior on IMU read failure.
    safeOutputs();
    imuWatchdog(500);
    delay(1);
    return;
  }

  // Raw IMU readings
  float ax_g_raw = to_g(imu.accX());
  float ay_g_raw = to_g(imu.accY());
  float az_g_raw = to_g(imu.accZ());

  float gx_raw = imu.gyrX();  // dps
  float gy_raw = imu.gyrY();
  float gz_raw = imu.gyrZ();

  // dt
  unsigned long now = micros();
  float dt = (lastMicros == 0) ? 0.005f : (now - lastMicros) * 1e-6f;
  lastMicros = now;
  dt = fminf(fmaxf(dt, 0.0005f), 0.02f);

  // Online bias update
  updateGyroBiasOnline(ax_g_raw, ay_g_raw, az_g_raw,
                       gx_raw, gy_raw, gz_raw, dt);

  // Bias-corrected values
  float ax = ax_g_raw - accelBias[0];
  float ay = ay_g_raw - accelBias[1];
  float az = az_g_raw - accelBias[2];

  float gx = gx_raw - gyroBias[0];
  float gy = gy_raw - gyroBias[1];
  float gz = gz_raw - gyroBias[2];

  // gz moving average
  gzMA -= gzBuf[gzIdx];
  gzBuf[gzIdx] = gz;
  gzMA += gz;

  gzIdx = (gzIdx + 1) % GZ_WIN;
  if (gzFilled < GZ_WIN) gzFilled++;

  float gzFiltered = gzMA / (float)gzFilled;

  // Attitude update
  const float D2R = PI / 180.0f;
  mahony6Update(ax, ay, az,
                gx * D2R,
                gy * D2R,
                gzFiltered * D2R,
                dt);

  float rollDeg, pitchDeg, yawDeg;
  quatToRPYdeg(q0, q1, q2, q3, rollDeg, pitchDeg, yawDeg);

  if (!yawZeroed) {
    yaw0 = yawDeg;
    yawZeroed = true;
  }

  float yawRel = norm180(yawDeg - yaw0);

  // Share current attitude with the web status endpoint.
  sharedPitchDeg = pitchDeg;
  sharedYawDeg = yawDeg;

  // Apply browser commands in the main control loop.
  processRemoteCommand(pitchDeg, yawDeg);
  controlLinkFailsafe();

  // Motor target is selected from the Wi-Fi control page.
  writeMotorOutput();

  // Stabilization
  if (stabilizationEnabled) {
    servo.update(pitchDeg, yawDeg);
  } else {
    servo.resetServos(90);
  }

#if LOG_LEVEL >= 2
  static uint32_t lastTelemetryMs = 0;
  if (millis() - lastTelemetryMs >= 100) {
    lastTelemetryMs = millis();

    float t = (millis() - t0_ms) * 0.001f;

    Serial.print(t, 3);              Serial.print(',');
    Serial.print(rollDeg, 2);        Serial.print(',');
    Serial.print(pitchDeg, 2);       Serial.print(',');
    Serial.print(yawDeg, 2);         Serial.print(',');
    Serial.print(yawRel, 2);         Serial.print(',');
    Serial.print(stabilizationEnabled ? 1 : 0); Serial.print(',');
    Serial.println((uint16_t)currentThrottleUs);
  }
#endif

#if LOG_LEVEL == 1
  static uint32_t lastStatusMs = 0;
  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();

    LOGI("[STAT] stab=%d motorTarget=%u motorOut=%u pitch=%.1f yaw=%.1f yawRel=%.1f\n",
         stabilizationEnabled ? 1 : 0,
         motorThrottleUs,
         (uint16_t)currentThrottleUs,
         pitchDeg,
         yawDeg,
         yawRel);
  }
#endif

  delay(5);
}
