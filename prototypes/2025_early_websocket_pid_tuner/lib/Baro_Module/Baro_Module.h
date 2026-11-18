#ifndef BARO_MODULE_H
#define BARO_MODULE_H

#include <Arduino.h>

class Baro_Module
{
private:
    float pressure;                        // in mbar
    float temperature;                     // in °C
    float depth;                           // in meters
    const float rho = 997.0;               // Water density (kg/m³) for freshwater
    const float g = 9.80665;               // Gravity (m/s²)
    const float surfacePressure = 1013.25; // Surface pressure in mbar (sea level)

public:
    void begin();
    void update();
    float getPressure();    // Returns pressure in mbar
    float getTemperature(); // Returns temperature in °C
    float getDepth();       // Returns depth in meters
};

#endif
