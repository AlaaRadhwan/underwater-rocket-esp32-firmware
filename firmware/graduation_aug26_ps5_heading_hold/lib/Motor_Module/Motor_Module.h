#ifndef MOTOR_MODULE_H
#define MOTOR_MODULE_H

#include <Arduino.h>

class Motor_Module
{
private:
    uint8_t escPin;
    uint32_t pwmFreq;
    uint8_t pwmResolution;
    uint32_t maxDuty;

public:
    Motor_Module(uint8_t pin, uint32_t freq = 50, uint8_t resolution = 16);

    void begin();
    void armESC(uint16_t minPulseUs = 1000);
    void setThrottle(uint16_t microseconds);
};

#endif
