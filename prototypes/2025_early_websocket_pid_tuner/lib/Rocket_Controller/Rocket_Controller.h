#ifndef ROCKET_CONTROLLER_H
#define ROCKET_CONTROLLER_H

#include <Arduino.h>
#include <Servo_Module.h>
#include <PID_v1.h>

class Rocket_Controller
{
private:
    // === Servos ===
    Servo_Module finLeft, finRight, finTop, finBottom;

    // === PID Variables ===
    double targetRoll, targetPitch, targetYaw;
    double inputRoll, inputPitch, inputYaw;
    double outputRoll, outputPitch, outputYaw;

    // === PID Gains ===
    double KpR, KiR, KdR;
    double KpP, KiP, KdP;
    double KpY, KiY, KdY;

    PID rollPID, pitchPID, yawPID;

public:
    Rocket_Controller(uint8_t leftPin, uint8_t rightPin,
                      uint8_t topPin, uint8_t bottomPin);

    void begin();
    void update(double roll, double pitch, double yaw);
    void setTargetAngles(double roll, double pitch, double yaw);
    void printStatus();

    void setRollPID(double kp, double ki, double kd);
    void setPitchPID(double kp, double ki, double kd);
    void setYawPID(double kp, double ki, double kd);

    void manualLeftRight(float delta);
    void manualTopBottom(float delta);
    void manualRoll(float angle);

    void getRollPID(double &kp, double &ki, double &kd);
    void getPitchPID(double &kp, double &ki, double &kd);
    void getYawPID(double &kp, double &ki, double &kd);
    
    void setServoAngle(String name, float angle);

    bool checkServoLimit(double servoInput);

    void manualCombined(float roll, float pitch, float yaw);
};

#endif
