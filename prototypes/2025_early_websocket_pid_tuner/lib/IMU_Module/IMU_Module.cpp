#include "IMU_Module.h"

void IMU_Module::begin()
{
    Serial.begin(115200);
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
    EEPROM.begin(EEPROM_SIZE);

    while (myICM.begin(WIRE_PORT, AD0_VAL) != ICM_20948_Stat_Ok)
    {
        Serial.println("❌ ICM-20948 not found. Check wiring...");
        delay(500);
    }
    Serial.println("✅ ICM-20948 connected!");

    if (myICM.initializeDMP() != ICM_20948_Stat_Ok)
    {
        Serial.println("❌ Failed to initialize DMP!");
        while (1)
            ;
    }

    Serial.println("🕒 Warming up... draining FIFO");
    unsigned long warmupStart = millis();
    while (millis() - warmupStart < 5000)
    {
        icm_20948_DMP_data_t tmp;
        myICM.readDMPdataFromFIFO(&tmp);
        delay(10); // Small delay to avoid hammering I2C
    }
    Serial.println("✅ Warm-up complete!");

    if (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok)
    {
        magEnabled = true;
        Serial.println("✅ DMP 9-axis fusion enabled.");
    }
    else if (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok)
    {
        magEnabled = false;
        Serial.println("⚠️ Magnetometer unavailable. Using 6-axis fusion.");
    }
    else
    {
        Serial.println("❌ Failed to enable DMP sensors!");
        while (1)
            ;
    }

    myICM.enableFIFO();
    myICM.enableDMP();
    myICM.resetDMP();
    myICM.resetFIFO();
    Serial.println("✅ DMP and FIFO ready.");

    loadBiases();

    Serial.println("🕒 Hold the sensor still for 3 seconds...");
    delay(3000);

    dmpStartTime = millis(); // Start warm-up timer
}

void IMU_Module::update()
{
    if (!dmpReady && millis() - dmpStartTime < 5000)
    {
        icm_20948_DMP_data_t tmp;
        myICM.readDMPdataFromFIFO(&tmp);
        Serial.println("⏳ Warming up... DMP calibrating.");
        logGyroBiases();
        // KEEP processing FIFO to avoid overflow
        return;
    }
    else if (!dmpReady)
    {
        dmpReady = true;
        Serial.println("✅ DMP Warm-up complete!");
    }

    icm_20948_DMP_data_t data;
    myICM.readDMPdataFromFIFO(&data);

    if (myICM.status == ICM_20948_Stat_Ok)
    {
        if (magEnabled && (data.header & DMP_header_bitmap_Quat9))
        {
            q.x = ((float)data.Quat9.Data.Q1) / 1073741824.0f;
            q.y = ((float)data.Quat9.Data.Q2) / 1073741824.0f;
            q.z = ((float)data.Quat9.Data.Q3) / 1073741824.0f;
        }
        else if (!magEnabled && (data.header & DMP_header_bitmap_Quat6))
        {
            q.x = ((float)data.Quat6.Data.Q1) / 1073741824.0f;
            q.y = ((float)data.Quat6.Data.Q2) / 1073741824.0f;
            q.z = ((float)data.Quat6.Data.Q3) / 1073741824.0f;
        }
        else
        {
            return;
        }
        q.w = sqrt(1.0f - (q.x * q.x + q.y * q.y + q.z * q.z));
        quaternionToEuler(q, roll, pitch, yaw);
    }
}

void IMU_Module::loadBiases()
{
    EEPROM.get(0, biases);
    if (isBiasValid(biases))
    {
        myICM.setBiasGyroX(biases.biasGyroX);
        myICM.setBiasGyroY(biases.biasGyroY);
        myICM.setBiasGyroZ(biases.biasGyroZ);
        myICM.setBiasAccelX(biases.biasAccelX);
        myICM.setBiasAccelY(biases.biasAccelY);
        myICM.setBiasAccelZ(biases.biasAccelZ);
        myICM.setBiasCPassX(biases.biasMagX);
        myICM.setBiasCPassY(biases.biasMagY);
        myICM.setBiasCPassZ(biases.biasMagZ);
        Serial.println("✅ Biases restored from EEPROM.");
    }
    else
    {
        Serial.println("⚠️ No valid biases found in EEPROM.");
    }
}

void IMU_Module::saveBiases()
{
    myICM.getBiasGyroX(&biases.biasGyroX);
    myICM.getBiasGyroY(&biases.biasGyroY);
    myICM.getBiasGyroZ(&biases.biasGyroZ);
    myICM.getBiasAccelX(&biases.biasAccelX);
    myICM.getBiasAccelY(&biases.biasAccelY);
    myICM.getBiasAccelZ(&biases.biasAccelZ);
    myICM.getBiasCPassX(&biases.biasMagX);
    myICM.getBiasCPassY(&biases.biasMagY);
    myICM.getBiasCPassZ(&biases.biasMagZ);
    updateBiasChecksum(biases);
    EEPROM.put(0, biases);
    EEPROM.commit();
    Serial.println("💾 Biases saved to EEPROM.");
}

void IMU_Module::updateBiasChecksum(biasStore &store)
{
    store.sum = store.header + store.biasGyroX + store.biasGyroY + store.biasGyroZ +
                store.biasAccelX + store.biasAccelY + store.biasAccelZ +
                store.biasMagX + store.biasMagY + store.biasMagZ;
}

bool IMU_Module::isBiasValid(biasStore &store)
{
    int32_t calcSum = store.header + store.biasGyroX + store.biasGyroY + store.biasGyroZ +
                      store.biasAccelX + store.biasAccelY + store.biasAccelZ +
                      store.biasMagX + store.biasMagY + store.biasMagZ;
    return (store.header == 0x42 && store.sum == calcSum);
}

void IMU_Module::quaternionToEuler(const Quaternion &q, float &roll, float &pitch, float &yaw)
{
    float t0 = +2.0f * (q.w * q.x + q.y * q.z);
    float t1 = +1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    roll = atan2(t0, t1) * 180.0f / PI;

    float t2 = +2.0f * (q.w * q.y - q.z * q.x);
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    pitch = asin(t2) * 180.0f / PI;

    float t3 = +2.0f * (q.w * q.z + q.x * q.y);
    float t4 = +1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    yaw = atan2(t3, t4) * 180.0f / PI;
}

void IMU_Module::logGyroBiases()
{
    int32_t gxBias, gyBias, gzBias;
    myICM.getBiasGyroX(&gxBias);
    myICM.getBiasGyroY(&gyBias);
    myICM.getBiasGyroZ(&gzBias);
    Serial.printf("📡 Gyro Bias [dps]: X=%ld, Y=%ld, Z=%ld\n", gxBias, gyBias, gzBias);
}

float IMU_Module::getRoll() { return roll; }
float IMU_Module::getPitch() { return pitch; }
float IMU_Module::getYaw() { return yaw; }

// ... (keep loadBiases, saveBiases, updateBiasChecksum, isBiasValid, quaternionToEuler same as before)
