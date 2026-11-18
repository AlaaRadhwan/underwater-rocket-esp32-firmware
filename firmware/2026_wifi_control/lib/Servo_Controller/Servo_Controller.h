#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>
#include "Servo_Module.h"

class Servo_Controller {
private:
    Servo_Module finLeft, finRight, finTop, finBottom;

    double targetPitch = 0.0, targetYaw = 0.0;
    double inputPitch = 0.0, inputYaw = 0.0;
    double outputPitch = 0.0, outputYaw = 0.0;

    double KpP = 1.5, KiP = 0.0, KdP = 0.0;
    double KpY = 1.5, KiY = 0.0, KdY = 0.0;

    PID pitchPID;
    PID yawPID;

    float lastL = 90.0f, lastR = 90.0f, lastT = 90.0f, lastB = 90.0f;
    float alphaPitch = 0.10f;
    float alphaYaw = 0.18f;
    float deadbandPitch = 2.0f;
    float deadbandYaw = 2.0f;
    float minStepPitch = 0.7f;
    float minStepYaw = 0.6f;
    float maxRatePitchDegPerSec = 180.0f;
    float maxRateYawDegPerSec = 120.0f;
    uint32_t lastUpdateUs = 0;

    bool yawFilterInit = false;
    float sYaw = 0.0f;
    float cYaw = 1.0f;

    static float norm180(float a);
    static float rateLimit(float prev, float target, float maxRateDegPerSec, float dt);
    static double deadband(double v, double db);
    void writeIfMoved(Servo_Module& s, float& last, float cmd, float minStep);

public:
    Servo_Controller(uint8_t leftChannel, uint8_t rightChannel,
                     uint8_t topChannel, uint8_t bottomChannel);

    void begin();
    void update(double pitchDeg, double yawDeg);
    void resetServos(float angleDeg = 90.0f);
    void setServoAngle(const String& servo, float angleDeg);
    void initServoAngles();
    void setTargetAngles(double pitchDeg, double yawDeg);
    void setPitchPID(double kp, double ki, double kd);
    void setYawPID(double kp, double ki, double kd);
};

#endif
