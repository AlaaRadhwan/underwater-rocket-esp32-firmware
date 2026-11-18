#include "Servo_Controller.h"

Servo_Controller::Servo_Controller(uint8_t leftPin, uint8_t rightPin,
                                     uint8_t topPin, uint8_t bottomPin)
    : finLeft(leftPin, 1), finRight(rightPin, 2),
      finTop(topPin, 3), finBottom(bottomPin, 4),
      KpP(1.5), KiP(0.0), KdP(0),
      KpY(1.5), KiY(0.0), KdY(0.0),
      pitchPID(&inputPitch, &outputPitch, &targetPitch, KpP, KiP, KdP, DIRECT),
      yawPID(&inputYaw, &outputYaw, &targetYaw, KpY, KiY, KdY, DIRECT) {}

void Servo_Controller::begin()
{
    finLeft.begin();
    finRight.begin();
    finTop.begin();
    finBottom.begin();

    float angleLimit = 25.0;

    pitchPID.SetOutputLimits(-angleLimit, angleLimit);
    yawPID.SetOutputLimits(-angleLimit, angleLimit);

    pitchPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);

    lastUpdateUs = micros();

}

void Servo_Controller::resetServos(float angle)
{
    finLeft.writeAngle(angle);
    finRight.writeAngle(angle);
    finTop.writeAngle(angle);
    finBottom.writeAngle(angle);
}

static inline double applyDeadband(double v, double db_deg) {
  return (fabs(v) < db_deg) ? 0.0 : v;
}

static inline float limitRate(float prev, float target, float maxRateDegPerSec, float dt) {
  float maxStep = maxRateDegPerSec * dt;
  float delta = target - prev;
  if (delta >  maxStep) delta =  maxStep;
  if (delta < -maxStep) delta = -maxStep;
  return prev + delta;
}

static inline float median3(float a, float b, float c) {
  if (a > b) { float t=a; a=b; b=t; }
  if (b > c) { float t=b; b=c; c=t; }
  if (a > b) { float t=a; a=b; b=t; }
  return b; // median
}


void Servo_Controller::manualCombined(float roll, float pitch, float yaw)
{
    finLeft.writeAngle(90 + pitch);
    finRight.writeAngle(90 - pitch);
    finTop.writeAngle(90 - yaw);
    finBottom.writeAngle(90 + yaw);
}

void Servo_Controller::setServoAngle(String servo, float angle) {
    if (!(angle <= 50) && !(angle >= 140)){
        if (servo == "finLeft") finLeft.writeAngle(angle);
        if (servo == "finRight") finRight.writeAngle(angle);
        if (servo == "finTop") finTop.writeAngle(angle);   
        if (servo == "finBottom") finBottom.writeAngle(angle);
    }

}

void Servo_Controller::initServoAngles() {
    float angle = 90;
    int duration = 500;

    angle = 50;
    finLeft.writeAngle(angle);
    delay(duration);
    finTop.writeAngle(angle);
    delay(duration);
    finRight.writeAngle(angle);
    delay(duration);
    finBottom.writeAngle(angle);
    delay(duration);

    angle = 90;
    finLeft.writeAngle(angle);
    delay(duration);
    finTop.writeAngle(angle);
    delay(duration);
    finRight.writeAngle(angle);
    delay(duration);
    finBottom.writeAngle(angle);
    delay(duration);
}

void Servo_Controller::setTargetAngles(double pitch, double yaw) {
    targetPitch = pitch;
    targetYaw = yaw;
}
void Servo_Controller::update(double pitch, double yaw) {
    // --- Small helpers (local, no headers needed) ---
    auto applyDeadband = [](double v, double db_deg) -> double {
        return (fabs(v) < db_deg) ? 0.0 : v;
    };
    auto limitRate = [](float prev, float target, float maxRateDegPerSec, float dt) -> float {
        float maxStep = maxRateDegPerSec * dt;
        float delta = target - prev;
        if (delta >  maxStep) delta =  maxStep;
        if (delta < -maxStep) delta = -maxStep;
        return prev + delta;
    };
    auto median3 = [](float a, float b, float c) -> float {
        if (a > b) { float t=a; a=b; b=t; }
        if (b > c) { float t=b; b=c; c=t; }
        if (a > b) { float t=a; a=b; b=t; }
        return b;
    };

    // --- Persistent state (function-local) ---
    static float lastL = 90.f, lastR = 90.f, lastT = 90.f, lastB = 90.f;
    static unsigned long lastUs = 0;
    static float yawHist0 = 0.f, yawHist1 = 0.f;

    // --- Spike killer on yaw: median-of-3 of raw yaw ---
    float yaw_med = median3((float)yaw, yawHist0, yawHist1);
    yawHist1 = yawHist0;
    yawHist0 = (float)yaw;

    // --- Per-axis EMA (light on pitch, stronger on yaw) ---
    if (alphaPitch > 0.f && alphaPitch < 1.f)
        inputPitch = (1.f - alphaPitch) * inputPitch + alphaPitch * (float)pitch;
    else
        inputPitch = (float)pitch;

    if (alphaYaw > 0.f && alphaYaw < 1.f)
        inputYaw = (1.f - alphaYaw) * inputYaw + alphaYaw * yaw_med;
    else
        inputYaw = yaw_med;

    // --- PID compute ---
    pitchPID.Compute();

    // ===== YAW: circular filtering + circular error into PID_v1 =====
    auto norm180f = [](float a)->float {
        while (a >  180.f) a -= 360.f;
        while (a < -180.f) a += 360.f;
        return a;
    };
    auto angDiff180f = [&](float refDeg, float nowDeg)->float {
        return norm180f(refDeg - nowDeg);  // shortest signed error
    };

    // --- Circular low-pass on yaw using sin/cos (replaces median3 + linear EMA) ---
    static bool yawFiltInit = false;
    static float sYaw = 0.f, cYaw = 1.f;     // filtered sin/cos
    const float D2R = (float)M_PI / 180.f;

    float yawRaw = (float)yaw;               // raw yaw from IMU, already in [-180,180]
    float sy = sinf(yawRaw * D2R);
    float cy = cosf(yawRaw * D2R);

    if (!yawFiltInit) { sYaw = sy; cYaw = cy; yawFiltInit = true; }
    float a = (alphaYaw > 0.f && alphaYaw < 1.f) ? alphaYaw : 1.f;   // if alphaYaw==0 → no filtering
    sYaw = (1.f - a) * sYaw + a * sy;
    cYaw = (1.f - a) * cYaw + a * cy;
    float norm = sqrtf(sYaw*sYaw + cYaw*cYaw);
    if (norm > 1e-6f) { sYaw /= norm; cYaw /= norm; }

    inputYaw = atan2f(sYaw, cYaw) * (180.f / (float)M_PI);  // filtered yaw in [-180,180]

    // --- Compute wrapped error and feed PID_v1 with setpoint=0, input=-error ---
    float yawErr = angDiff180f((float)targetYaw, (float)inputYaw);
    Serial.println(yawErr);

    double savedTargetYaw = targetYaw;
    double savedInputYaw  = inputYaw;

    targetYaw = 0.0;                 // setpoint = 0
    inputYaw  = -(double)yawErr;     // so PID computes out ≈ Kp*yawErr (+Ki/Kd if you add later)

    yawPID.Compute();                // writes into outputYaw

    // restore members so the rest of your code still sees real target/measured yaw
    targetYaw = savedTargetYaw;
    inputYaw  = savedInputYaw;

    // --- Deadbands + sign convention (your existing logic) ---
    double op = applyDeadband(outputPitch, deadbandPitch);
    double oy = applyDeadband(outputYaw,   deadbandYaw);
    oy = oy * -1.0;   // keep your mechanical inversion

    // --- Map to fins, rate limit, maybeWrite(...) exactly as before ---
    float tgtL = 90.f + (float)op;
    float tgtR = 90.f - (float)op;
    float tgtT = 90.f - (float)oy;
    float tgtB = 90.f + (float)oy;

    unsigned long now = micros();
    float dt = (lastUs == 0) ? 0.01f : (now - lastUs) / 1e6f;
    lastUs = now;

    float cmdL = limitRate(lastL, tgtL, maxRatePitchDegPerSec, dt);
    float cmdR = limitRate(lastR, tgtR, maxRatePitchDegPerSec, dt);
    float cmdT = limitRate(lastT, tgtT, maxRateYawDegPerSec,   dt);
    float cmdB = limitRate(lastB, tgtB, maxRateYawDegPerSec,   dt);

    auto maybeWrite = [&](Servo_Module& s, float& last, float cmd, float minStep) {
        if (fabs(cmd - last) >= minStep) { s.writeAngle(cmd); last = cmd; }
    };
    maybeWrite(finLeft,   lastL, cmdL, minStepPitch);
    maybeWrite(finRight,  lastR, cmdR, minStepPitch);
    maybeWrite(finTop,    lastT, cmdT, minStepYaw);
    maybeWrite(finBottom, lastB, cmdB, minStepYaw);
}



void Servo_Controller::setPitchPID(double kp, double ki, double kd){
    if (kp >= 0){
        KpP = kp;
        pitchPID.SetTunings(KpP, KiP, KdP);
    }
    if (ki >= 0) {
        KiP = ki;
        pitchPID.SetTunings(KpP, KiP, KdP);
    }
    if (kd >= 0) {
        KdP = kd;
        pitchPID.SetTunings(KpP, KiP, KdP);
    }
}

void Servo_Controller::setYawPID(double kp, double ki, double kd) {
    if (kp >= 0){
        KpY = kp;
        yawPID.SetTunings(KpY, KiY, KdY);
    }
    if (ki >= 0) {
        KiY = ki;
        yawPID.SetTunings(KpY, KiY, KdY);
    }
    if (kd >= 0) {
        KdY = kd;
        yawPID.SetTunings(KpY, KiY, KdY);
    }
}

