#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <Servo_Module.h>
#include <PID_v1.h>

class Servo_Controller
{
private:
    Servo_Module finLeft, finRight, finTop, finBottom;

    double targetPitch, targetYaw;
    double inputPitch, inputYaw;
    double outputPitch, outputYaw;

    double KpP, KiP, KdP;
    double KpY, KiY, KdY;

    PID pitchPID, yawPID;

    // smoothing state
    float lastL = 90, lastR = 90, lastT = 90, lastB = 90;

    // Existing smoothing knobs were shared; split them per axis:
    float alphaPitch = 0.10f, alphaYaw = 0.18f;

    float deadbandPitch = 2.0f, deadbandYaw = 6.0f;
    float minStepPitch  = 0.7f, minStepYaw  = 0.6f;

    float maxRatePitchDegPerSec = 180.0f;
    float maxRateYawDegPerSec   = 120.0f;

    // For median-of-3 on yaw
    float yawHist0 = 0.f, yawHist1 = 0.f; // last two raw yaw samples
       // 0..1, optional IMU LPF (0=off)
    unsigned long lastUpdateUs = 0;


public:
    Servo_Controller(uint8_t leftPin, uint8_t rightPin,
                     uint8_t topPin, uint8_t bottomPin);
    
    void begin();
    void update(double pitch, double yaw);
    void manualCombined(float roll, float pitch, float yaw);
    void resetServos(float angle);
    void setServoAngle(String servo, float angle);
    void initServoAngles();

    void setTargetAngles(double pitch, double yaw);
    void setPitchPID(double kp, double ki, double kd);
    void setYawPID(double kp, double ki, double kd);

};

#endif