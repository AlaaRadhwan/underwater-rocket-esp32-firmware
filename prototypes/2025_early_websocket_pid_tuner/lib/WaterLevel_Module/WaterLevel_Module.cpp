#include "WaterLevel_Module.h"

WaterLevel_Module::WaterLevel_Module(uint8_t pin)
{
    sensorPin = pin;
    surfaced = false;
}

void WaterLevel_Module::begin()
{
    pinMode(sensorPin, INPUT);
}

void WaterLevel_Module::update()
{
    int state = digitalRead(sensorPin);
    surfaced = (state == HIGH); // Adjust if your sensor logic is inverted
}

bool WaterLevel_Module::isSurfaced()
{
    return surfaced;
}
