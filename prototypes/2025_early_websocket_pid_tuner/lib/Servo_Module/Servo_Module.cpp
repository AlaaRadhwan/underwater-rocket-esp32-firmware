#include "Servo_Module.h"

Servo_Module::Servo_Module(uint8_t pin, uint8_t channel, uint32_t freq, uint8_t resolution,
                           uint32_t minPulseUs, uint32_t maxPulseUs)
{
    this->pin = pin;
    this->channel = channel;
    this->freq = freq;
    this->resolution = resolution;
    this->minPulseUs = minPulseUs;
    this->maxPulseUs = maxPulseUs;
    this->maxDuty = (1 << resolution);
}

void Servo_Module::begin()
{
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(pin, channel);
}

void Servo_Module::writeAngle(float angle)
{
    angle = constrain(angle, 0.0, 180.0);                             // Clamp angle to 0–180°
    uint32_t pulseWidth = map(angle, 0, 180, minPulseUs, maxPulseUs); // µs
    uint32_t duty = (pulseWidth * maxDuty) / 20000;                   // 20ms period for 50Hz
    ledcWrite(channel, duty);
}
