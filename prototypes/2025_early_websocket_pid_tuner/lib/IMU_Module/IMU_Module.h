#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <Arduino.h>
#include "ICM_20948.h"
#include <EEPROM.h>

#define WIRE_PORT Wire
#define AD0_VAL 1
#define EEPROM_SIZE 128

class IMU_Module
{
public:
    void begin();
    void update();
    float getRoll();
    float getPitch();
    float getYaw();

private:
    ICM_20948_I2C myICM;
    bool magEnabled = false;

    struct Quaternion
    {
        float w, x, y, z;
    };
    struct biasStore
    {
        int32_t header = 0x42;
        int32_t biasGyroX, biasGyroY, biasGyroZ;
        int32_t biasAccelX, biasAccelY, biasAccelZ;
        int32_t biasMagX, biasMagY, biasMagZ;
        int32_t sum = 0;
    };

    biasStore biases;
    Quaternion q;
    float roll = 0, pitch = 0, yaw = 0;
    bool dmpReady = false;
    unsigned long dmpStartTime = 0;

    void loadBiases();
    void saveBiases();
    void updateBiasChecksum(biasStore &store);
    bool isBiasValid(biasStore &store);
    void quaternionToEuler(const Quaternion &q, float &roll, float &pitch, float &yaw);
    void logGyroBiases(); // 👈 Add this here
};

#endif
