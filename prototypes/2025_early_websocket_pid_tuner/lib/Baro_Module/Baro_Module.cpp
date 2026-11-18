#include "Baro_Module.h"
#include <Wire.h>
#include <MS5611.h>

MS5611 ms5611;

void Baro_Module::begin()
{
    Wire.begin();
    if (!ms5611.begin())
    {
        Serial.println("❌ MS5611 not detected!");
        while (1)
            ; // Halt if sensor not found
    }
    Serial.println("✅ MS5611 initialized");
}

void Baro_Module::update()
{
    ms5611.read();
    pressure = ms5611.getPressure();       // mbar (hPa)
    temperature = ms5611.getTemperature(); // °C

    // Convert pressure to depth (meters)
    // depth = (P - P0) / (ρ * g) * 100
    float P_diff = pressure - surfacePressure; // Difference from surface pressure
    if (P_diff > 0)
    {
        depth = (P_diff * 100.0) / (rho * g); // Convert mbar to Pa (x100)
    }
    else
    {
        depth = 0.0; // Above surface
    }
}

float Baro_Module::getPressure()
{
    return pressure;
}

float Baro_Module::getTemperature()
{
    return temperature;
}

float Baro_Module::getDepth()
{
    return depth;
}
