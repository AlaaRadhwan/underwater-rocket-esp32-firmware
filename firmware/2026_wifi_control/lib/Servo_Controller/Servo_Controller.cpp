#include "Servo_Controller.h"

Servo_Controller::Servo_Controller(uint8_t leftChannel, uint8_t rightChannel,
                                   uint8_t topChannel, uint8_t bottomChannel)
    : finLeft(leftChannel), finRight(rightChannel), finTop(topChannel), finBottom(bottomChannel),
      pitchPID(&inputPitch, &outputPitch, &targetPitch, KpP, KiP, KdP, DIRECT),
      yawPID(&inputYaw, &outputYaw, &targetYaw, KpY, KiY, KdY, DIRECT) {}

float Servo_Controller::norm180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

float Servo_Controller::rateLimit(float prev, float target, float maxRateDegPerSec, float dt) {
    float maxStep = maxRateDegPerSec * dt;
    float delta = target - prev;
    if (delta > maxStep) delta = maxStep;
    if (delta < -maxStep) delta = -maxStep;
    return prev + delta;
}

double Servo_Controller::deadband(double v, double db) {
    return (fabs(v) < db) ? 0.0 : v;
}

void Servo_Controller::writeIfMoved(Servo_Module& s, float& last, float cmd, float minStep) {
    if (fabsf(cmd - last) >= minStep) {
        s.writeAngle(cmd);
        last = cmd;
    }
}

void Servo_Controller::begin() {
    finLeft.begin();
    finRight.begin();
    finTop.begin();
    finBottom.begin();

    const float angleLimit = 25.0f;
    pitchPID.SetOutputLimits(-angleLimit, angleLimit);
    yawPID.SetOutputLimits(-angleLimit, angleLimit);
    pitchPID.SetSampleTime(10);
    yawPID.SetSampleTime(10);
    pitchPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);

    resetServos(90.0f);
    lastUpdateUs = micros();
}

void Servo_Controller::resetServos(float angleDeg) {
    angleDeg = constrain(angleDeg, 0.0f, 180.0f);
    finLeft.writeAngle(angleDeg);
    finRight.writeAngle(angleDeg);
    finTop.writeAngle(angleDeg);
    finBottom.writeAngle(angleDeg);
    lastL = lastR = lastT = lastB = angleDeg;
}

void Servo_Controller::setServoAngle(const String& servo, float angleDeg) {
    angleDeg = constrain(angleDeg, 0.0f, 180.0f);
    if (servo == "finLeft") finLeft.writeAngle(angleDeg);
    else if (servo == "finRight") finRight.writeAngle(angleDeg);
    else if (servo == "finTop") finTop.writeAngle(angleDeg);
    else if (servo == "finBottom") finBottom.writeAngle(angleDeg);
}

void Servo_Controller::initServoAngles() {
    resetServos(90.0f);
}

void Servo_Controller::setTargetAngles(double pitchDeg, double yawDeg) {
    targetPitch = pitchDeg;
    targetYaw = norm180((float)yawDeg);
}

void Servo_Controller::update(double pitchDeg, double yawDeg) {
    uint32_t now = micros();
    float dt = (lastUpdateUs == 0) ? 0.01f : (now - lastUpdateUs) * 1e-6f;
    lastUpdateUs = now;
    dt = constrain(dt, 0.001f, 0.05f);

    if (alphaPitch > 0.0f && alphaPitch < 1.0f)
        inputPitch = (1.0f - alphaPitch) * inputPitch + alphaPitch * pitchDeg;
    else
        inputPitch = pitchDeg;

    const float D2R = PI / 180.0f;
    float yawRaw = norm180((float)yawDeg);
    float sy = sinf(yawRaw * D2R);
    float cy = cosf(yawRaw * D2R);

    if (!yawFilterInit) {
        sYaw = sy;
        cYaw = cy;
        yawFilterInit = true;
    }

    float a = (alphaYaw > 0.0f && alphaYaw < 1.0f) ? alphaYaw : 1.0f;
    sYaw = (1.0f - a) * sYaw + a * sy;
    cYaw = (1.0f - a) * cYaw + a * cy;
    float n = sqrtf(sYaw * sYaw + cYaw * cYaw);
    if (n > 1e-6f) { sYaw /= n; cYaw /= n; }

    float yawFiltered = atan2f(sYaw, cYaw) * 180.0f / PI;
    inputYaw = yawFiltered;

    pitchPID.Compute();

    // PID_v1 is linear, so use shortest signed yaw error as the controlled variable.
    float yawError = norm180((float)targetYaw - yawFiltered);
    double savedTargetYaw = targetYaw;
    double savedInputYaw = inputYaw;
    targetYaw = 0.0;
    inputYaw = (double)yawError;
    yawPID.Compute();
    targetYaw = savedTargetYaw;
    inputYaw = savedInputYaw;

    double op = deadband(outputPitch, deadbandPitch);
    double oy = -deadband(outputYaw, deadbandYaw);  // sign convention for current fin wiring

    float tgtL = constrain(90.0f + (float)op, 0.0f, 180.0f);
    float tgtR = constrain(90.0f - (float)op, 0.0f, 180.0f);
    float tgtT = constrain(90.0f - (float)oy, 0.0f, 180.0f);
    float tgtB = constrain(90.0f + (float)oy, 0.0f, 180.0f);

    float cmdL = rateLimit(lastL, tgtL, maxRatePitchDegPerSec, dt);
    float cmdR = rateLimit(lastR, tgtR, maxRatePitchDegPerSec, dt);
    float cmdT = rateLimit(lastT, tgtT, maxRateYawDegPerSec, dt);
    float cmdB = rateLimit(lastB, tgtB, maxRateYawDegPerSec, dt);

    writeIfMoved(finLeft, lastL, cmdL, minStepPitch);
    writeIfMoved(finRight, lastR, cmdR, minStepPitch);
    writeIfMoved(finTop, lastT, cmdT, minStepYaw);
    writeIfMoved(finBottom, lastB, cmdB, minStepYaw);
}

void Servo_Controller::setPitchPID(double kp, double ki, double kd) {
    if (kp >= 0) KpP = kp;
    if (ki >= 0) KiP = ki;
    if (kd >= 0) KdP = kd;
    pitchPID.SetTunings(KpP, KiP, KdP);
}

void Servo_Controller::setYawPID(double kp, double ki, double kd) {
    if (kp >= 0) KpY = kp;
    if (ki >= 0) KiY = ki;
    if (kd >= 0) KdY = kd;
    yawPID.SetTunings(KpY, KiY, KdY);
}
