#include "Rocket_Controller.h"

// === Constructor ===
Rocket_Controller::Rocket_Controller(uint8_t leftPin, uint8_t rightPin,
                                     uint8_t topPin, uint8_t bottomPin)
    : finLeft(leftPin, 1), finRight(rightPin, 2),
      finTop(topPin, 3), finBottom(bottomPin, 4),
      KpR(5.0), KiR(0.0), KdR(0.1),
      KpP(5.0), KiP(0.0), KdP(0.1),
      KpY(1.5), KiY(0.3), KdY(0.05),
      rollPID(&inputRoll, &outputRoll, &targetRoll, KpR, KiR, KdR, DIRECT),
      pitchPID(&inputPitch, &outputPitch, &targetPitch, KpP, KiP, KdP, DIRECT),
      yawPID(&inputYaw, &outputYaw, &targetYaw, KpY, KiY, KdY, DIRECT) {}

// === Setup ===
void Rocket_Controller::begin()
{
    finLeft.begin();
    finRight.begin();
    finTop.begin();
    finBottom.begin();

    rollPID.SetOutputLimits(-40, 40);
    pitchPID.SetOutputLimits(-40, 40);
    yawPID.SetOutputLimits(-40, 40);

    rollPID.SetMode(AUTOMATIC);
    pitchPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);

    Serial.println("✅ Rocket Controller Ready (PIDs & Servos)");
}

bool Rocket_Controller::checkServoLimit(double servoInput) {
    if (servoInput < 45 || servoInput > 135) return false;
    return true;
}

// === Update PID and Servos ===
void Rocket_Controller::update(double roll, double pitch, double yaw)
{
    bool safeAngle = true;

    inputRoll = roll;
    inputPitch = pitch;
    inputYaw = yaw;

    rollPID.Compute();
    pitchPID.Compute();
    yawPID.Compute();

    // Apply outputs to fins
    if (checkServoLimit(90 + outputYaw)) finLeft.writeAngle(90 + outputYaw);
    if (checkServoLimit(90 - outputYaw)) finRight.writeAngle(90 - outputYaw);
    if (checkServoLimit(90 + outputPitch)) finTop.writeAngle(90 + outputPitch);
    if (checkServoLimit(90 - outputPitch)) finBottom.writeAngle(90 - outputPitch);

    // Serial.print("Roll: ");
    // Serial.print(inputRoll, 1);
    // Serial.print(" Pitch: ");
    // Serial.print(inputPitch, 1);
    // Serial.print(" Yaw: ");
    // Serial.print(inputYaw, 1);
    // Serial.print(" | OutR: ");
    // Serial.print(outputRoll, 1);
    // Serial.print(" OutP: ");
    // Serial.print(outputPitch, 1);
    // Serial.print(" OutY: ");
    // Serial.println(outputYaw, 1);
}

// === Set New Targets ===
void Rocket_Controller::setTargetAngles(double roll, double pitch, double yaw)
{
    targetRoll = roll;
    targetPitch = pitch;
    targetYaw = yaw;
}

// === Print PID Gains and Targets ===
void Rocket_Controller::printStatus()
{
    Serial.println("=== PID Gains & Targets ===");
    Serial.printf("Roll  Kp: %.2f Ki: %.2f Kd: %.2f Target: %.2f°\n", KpR, KiR, KdR, targetRoll);
    Serial.printf("Pitch Kp: %.2f Ki: %.2f Kd: %.2f Target: %.2f°\n", KpP, KiP, KdP, targetPitch);
    Serial.printf("Yaw   Kp: %.2f Ki: %.2f Kd: %.2f Target: %.2f°\n", KpY, KiY, KdY, targetYaw);
}

void Rocket_Controller::setRollPID(double kp, double ki, double kd)
{
    if (kp >= 0)
    {
        KpR = kp;
        rollPID.SetTunings(KpR, KiR, KdR);
    }
    if (ki >= 0)
    {
        KiR = ki;
        rollPID.SetTunings(KpR, KiR, KdR);
    }
    if (kd >= 0)
    {
        KdR = kd;
        rollPID.SetTunings(KpR, KiR, KdR);
    }
}
void Rocket_Controller::setPitchPID(double kp, double ki, double kd)
{
    if (kp >= 0)
    {
        KpP = kp;
        pitchPID.SetTunings(KpP, KiP, KdP);
    }
    if (ki >= 0)
    {
        KiP = ki;
        pitchPID.SetTunings(KpP, KiP, KdP);
    }
    if (kd >= 0)
    {
        KdP = kd;
        pitchPID.SetTunings(KpP, KiP, KdP);
    }
}
void Rocket_Controller::setYawPID(double kp, double ki, double kd)
{
    if (kp >= 0)
    {
        KpY = kp;
        yawPID.SetTunings(KpY, KiY, KdY);
    }
    if (ki >= 0)
    {
        KiY = ki;
        yawPID.SetTunings(KpY, KiY, KdY);
    }
    if (kd >= 0)
    {
        KdY = kd;
        yawPID.SetTunings(KpY, KiY, KdY);
    }
}

void Rocket_Controller::manualLeftRight(float delta)
{
    finLeft.writeAngle(90 + delta);
    finRight.writeAngle(90 - delta);
    // Serial.println("Moving left and right");
}

void Rocket_Controller::manualTopBottom(float delta)
{
    finTop.writeAngle(90 + delta);
    finBottom.writeAngle(90 - delta);
    // Serial.println("Moving top and bottom");
}

void Rocket_Controller::manualRoll(float delta)
{
    finLeft.writeAngle(90 + delta);   // Up
    finRight.writeAngle(90 - delta);  // Down
    finTop.writeAngle(90 - delta);    // Down
    finBottom.writeAngle(90 + delta); // Up
}

void Rocket_Controller::setServoAngle(String name, float angle)
{
    angle = constrain(angle, 0, 180);
    if (name == "LEFT")
        finLeft.writeAngle(angle);
    else if (name == "RIGHT")
        finRight.writeAngle(angle);
    else if (name == "TOP")
        finTop.writeAngle(angle);
    else if (name == "BOTTOM")
        finBottom.writeAngle(angle);
}

void Rocket_Controller::manualCombined(float roll, float pitch, float yaw)
{
    finLeft.writeAngle(90 + roll + yaw);
    finRight.writeAngle(90 - roll + yaw);
    finTop.writeAngle(90 + pitch - yaw);
    finBottom.writeAngle(90 - pitch - yaw);
}


void Rocket_Controller::getRollPID(double &kp, double &ki, double &kd) {
    kp = KpR; ki = KiR; kd = KdR;
}
void Rocket_Controller::getPitchPID(double &kp, double &ki, double &kd) {
    kp = KpP; ki = KiP; kd = KdP;
}
void Rocket_Controller::getYawPID(double &kp, double &ki, double &kd) {
    kp = KpY; ki = KiY; kd = KdY;
}
