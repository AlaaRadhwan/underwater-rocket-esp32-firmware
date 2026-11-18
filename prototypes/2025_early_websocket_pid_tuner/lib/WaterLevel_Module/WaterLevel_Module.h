#ifndef WATERLEVEL_MODULE_H
#define WATERLEVEL_MODULE_H

#include <Arduino.h>

class WaterLevel_Module
{
private:
    uint8_t sensorPin;
    bool surfaced;

public:
    WaterLevel_Module(uint8_t pin);
    void begin();
    void update();
    bool isSurfaced(); // true = rocket at surface
};

#endif
