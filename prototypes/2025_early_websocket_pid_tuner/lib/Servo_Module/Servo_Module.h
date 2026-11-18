#ifndef SERVO_MODULE_H
#define SERVO_MODULE_H

#include <Arduino.h>

class Servo_Module
{
private:
    uint8_t pin;
    uint8_t channel;
    uint32_t freq;
    uint8_t resolution;
    uint32_t minPulseUs;
    uint32_t maxPulseUs;
    uint32_t maxDuty;

public:
    Servo_Module(uint8_t pin, uint8_t channel, uint32_t freq = 50, uint8_t resolution = 16,
                 uint32_t minPulseUs = 500, uint32_t maxPulseUs = 2500);

    void begin();
    void writeAngle(float angle); // 0–180 degrees
};

#endif
