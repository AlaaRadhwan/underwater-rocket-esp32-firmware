#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"
#include "secrets.h"   // UWR_AP_PASSWORD (not committed)
#include "Rocket_Controller.h"
#include "Motor_Module.h"

#include <WiFi.h>
#include <ArduinoOTA.h>

#include "WebServer_Module.h"
#include <ESPAsyncWebServer.h>

#define AD0_VAL 1
#define WIRE_PORT Wire

ICM_20948_I2C myICM;
Rocket_Controller rocket(13, 26, 25, 14);
Motor_Module motor(27);
WebServer_Module webServer("Rocket-OTA", UWR_AP_PASSWORD);

float roll = 0, pitch = 0, yaw = 0;
float offsetRoll = 0, offsetPitch = 0, offsetYaw = 0;
double P = 0, I = 0, D = 0; // legacy single-param hooks
bool imuReady = false;
unsigned long lastIMUTime = 0;

float filteredPitch = 0.0, filteredRoll = 0.0;

static void logMessage(const String &msg) {
  Serial.println(msg);
  Serial1.println(msg);
}

// static void sendAllPID() {
//   double krp, kri, krd, kpp, kpi, kpd, kyp, kyi, kyd;
//   rocket.getRollPID(krp, kri, krd);
//   rocket.getPitchPID(kpp, kpi, kpd);
//   rocket.getYawPID(kyp, kyi, kyd);

//   String json = String("{\"pid\":{") +
//                 "\"roll\":{\"kp\":" + String(krp, 3) + ",\"ki\":" + String(kri, 3) + ",\"kd\":" + String(krd, 3) + "}," +
//                 "\"pitch\":{\"kp\":" + String(kpp, 3) + ",\"ki\":" + String(kpi, 3) + ",\"kd\":" + String(kpd, 3) + "}," +
//                 "\"yaw\":{\"kp\":" + String(kyp, 3) + ",\"ki\":" + String(kyi, 3) + ",\"kd\":" + String(kyd, 3) + "}" +
//                 "}}";
//   webServer.sendToAll(json);
// }

// Send all current PID gains as JSON to the web UI
static void sendAllPID() {
  double rkp,rki,rkd, pkp,pki,pkd, ykp,yki,ykd;
  rocket.getRollPID(rkp,rki,rkd);
  rocket.getPitchPID(pkp,pki,pkd);
  rocket.getYawPID(ykp,yki,ykd);

  String json = String("{\"pid\":{") +
    "\"roll\":{\"kp\":"+String(rkp,3)+",\"ki\":"+String(rki,3)+",\"kd\":"+String(rkd,3)+"}," +
    "\"pitch\":{\"kp\":"+String(pkp,3)+",\"ki\":"+String(pki,3)+",\"kd\":"+String(pkd,3)+"}," +
    "\"yaw\":{\"kp\":"+String(ykp,3)+",\"ki\":"+String(yki,3)+",\"kd\":"+String(ykd,3)+"}" +
  "}}";
  webServer.sendToAll(json);
}


String handleCommand(const String &cmd)
{
  String c = cmd; c.trim();

  // Query: GET_PID=ALL | ROLL | PITCH | YAW
  if (c.startsWith("GET_PID=")) {
    String which = c.substring(8); which.trim();
    if (which == "ALL") { sendAllPID(); return "📤 PID ALL"; }

    double kp,ki,kd;
    if (which == "ROLL")  rocket.getRollPID(kp,ki,kd);
    else if (which == "PITCH") rocket.getPitchPID(kp,ki,kd);
    else if (which == "YAW")   rocket.getYawPID(kp,ki,kd);
    else return "❌ Unknown GET_PID target";

    String json = String("{\"pid\":{\"") + which + "\":{\"kp\":"+String(kp,3)+
                   ",\"ki\":"+String(ki,3)+",\"kd\":"+String(kd,3)+"}}}";
    webServer.sendToAll(json);
    return "📤 PID " + which;
  }

  // Set: KP_R=.. KI_R=.. KD_R=..  (also _P, _Y)
  auto setOne = [&](char axis, char term, double val){
    if (axis=='R') rocket.setRollPID(term=='P'?val:-1, term=='I'?val:-1, term=='D'?val:-1);
    else if (axis=='P') rocket.setPitchPID(term=='P'?val:-1, term=='I'?val:-1, term=='D'?val:-1);
    else if (axis=='Y') rocket.setYawPID(term=='P'?val:-1, term=='I'?val:-1, term=='D'?val:-1);
  };
  if (c.length()>=6 && c.charAt(0)=='K' && (c.charAt(1)=='P'||c.charAt(1)=='I'||c.charAt(1)=='D') &&
      c.charAt(2)=='_' && (c.charAt(3)=='R'||c.charAt(3)=='P'||c.charAt(3)=='Y') && c.charAt(4)=='=') {
    char term=c.charAt(1), axis=c.charAt(3);
    double v = c.substring(5).toFloat();
    setOne(axis, term, v);
    sendAllPID();                  // keep UI in sync
    return "✅ Set K";             // short ack
  }

  // Backward‑compat: P=/I=/D= (pitch only)
  if (c.startsWith("P=")) { rocket.setPitchPID(c.substring(2).toFloat(), -1, -1); sendAllPID(); return "✅ PITCH Kp"; }
  if (c.startsWith("I=")) { rocket.setPitchPID(-1, c.substring(2).toFloat(), -1); sendAllPID(); return "✅ PITCH Ki"; }
  if (c.startsWith("D=")) { rocket.setPitchPID(-1, -1, c.substring(2).toFloat()); sendAllPID(); return "✅ PITCH Kd"; }

  return "❓ Unknown command";
}

static void setupICM() {
  logMessage("🔧 Entering setupICM()...");
  int attempts = 0;
  while (myICM.begin(WIRE_PORT, AD0_VAL) != ICM_20948_Stat_Ok && attempts < 10) {
    logMessage("❌ ICM-20948 not found. Retrying...");
    attempts++;
    ArduinoOTA.handle();
    delay(500);
  }
  if (attempts >= 10) {
    logMessage("🆘 IMU failed. OTA-safe mode.");
    while (true) { ArduinoOTA.handle(); delay(20); }
  }
  logMessage("✅ ICM-20948 connected.");
  lastIMUTime = millis();
  imuReady = true;
}

static void readIMU() {
  if (!myICM.dataReady()) return;
  myICM.getAGMT();

  float ax = myICM.accX(), ay = myICM.accY(), az = myICM.accZ();
  float gx = myICM.gyrX(), gy = myICM.gyrY();
  float mx = myICM.magX(), my = myICM.magY();

  unsigned long now = millis();
  float dt = (now - lastIMUTime) / 1000.0f;
  lastIMUTime = now;

  float accPitch = atan2(ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
  float accRoll  = atan2(ay, sqrtf(ax*ax + az*az)) * 180.0f / PI;

  const float alpha = 0.98f;
  filteredPitch = alpha * (filteredPitch + gy * dt) + (1 - alpha) * accPitch;
  filteredRoll  = alpha * (filteredRoll  + gx * dt) + (1 - alpha) * accRoll;

  pitch = filteredPitch - offsetPitch;
  roll  = filteredRoll  - offsetRoll;

  yaw = atan2f(my, mx) * 180.0f / PI - offsetYaw;
  if (yaw < 0) yaw += 360;
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  WiFi.softAP("Rocket-OTA", UWR_AP_PASSWORD);
  IPAddress IP = WiFi.softAPIP();
  logMessage("📶 ESP32 AP started");
  logMessage("🌐 IP: " + IP.toString());

  ArduinoOTA.setHostname("esp32-rocket");
  ArduinoOTA.setPassword(UWR_AP_PASSWORD);
  ArduinoOTA.begin();
  logMessage("🟢 OTA Ready");

  // Give OTA a short window on boot
  for (int i = 0; i < 3000; i++) { ArduinoOTA.handle(); delay(1); }

  // Start WebSocket server and bind handler
  webServer.begin();
  webServer.onMessage([](const String &msg){
    String resp = handleCommand(msg);
    // send short acks too so you see logs on the page
    webServer.sendToAll(resp);
  });


  WIRE_PORT.begin();
  WIRE_PORT.setClock(100000);
  setupICM();

  // Zero orientation on boot
  logMessage("Started Zeroing...");
  for (int i = 0; i < 250; i++) {
    if (myICM.dataReady()) myICM.getAGMT();
    readIMU();
    delay(50);
  }
  filteredPitch = pitch; filteredRoll = roll;
  offsetRoll = filteredRoll; offsetPitch = filteredPitch;
  offsetYaw = atan2(myICM.magY(), myICM.magX()) * 180.0 / PI;
  if (offsetYaw < 0) offsetYaw += 360;

  logMessage("📍 Orientation zeroed at boot:");
  logMessage("Offset Roll: " + String(offsetRoll,1) + " | Pitch: " + String(offsetPitch,1) + " | Yaw: " + String(offsetYaw,1));

  rocket.begin();
  rocket.setTargetAngles(0,0,0);
  motor.begin();
  motor.armESC();

  rocket.update(0,0,0);
  delay(500);
  sendAllPID(); // push initial PID to UI
  logMessage("🚀 Rocket Ready.");
}

void loop() {
  ArduinoOTA.handle();

  if (imuReady) {
    readIMU();

    // nicer rounding for UI
    // pitch = round(pitch * 2.0) / 2.0;
    // roll  = round(roll  * 2.0) / 2.0;
    // yaw   = round(yaw   * 2.0) / 2.0;

    rocket.update(0, pitch, yaw);

    static unsigned long lastSend = 0;
    if (millis() - lastSend > 200) {
      lastSend = millis();
      String telemetry = "{\"roll\":" + String(roll,1) +
                         ",\"pitch\":" + String(pitch,1) +
                         ",\"yaw\":" + String(yaw,1) + "}";
      Serial.println(telemetry);
      webServer.sendToAll(telemetry);
    }
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    // String response = handlePIDCommand(cmd);
    // logMessage(response);
  }

  delay(20);
}
