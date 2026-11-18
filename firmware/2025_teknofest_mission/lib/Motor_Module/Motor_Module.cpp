#include "Motor_Module.h"

Motor_Module::Motor_Module(uint8_t pin, uint32_t freq, uint8_t resolution)
{
    escPin = pin;
    pwmFreq = freq;
    pwmResolution = resolution;
    maxDuty = (1 << pwmResolution);
}

void Motor_Module::begin()
{
    ledcSetup(0, pwmFreq, pwmResolution); // Channel 0
    ledcAttachPin(escPin, 0);             // Attach pin to Channel 0
}

void Motor_Module::armESC(uint16_t minPulseUs)
{
    setThrottle(minPulseUs);
    delay(2000);
}

void Motor_Module::setThrottle(uint16_t microseconds)
{
    uint32_t duty = (microseconds * maxDuty) / 20000; // 20ms period
    ledcWrite(0, duty);                               // Channel 0
}
