#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <MPU9250_WE.h>
#include "Config.h"

class MPU6050 {
public:
    MPU6050();
    void begin(int sda = IMU_SDA_PIN, int scl = IMU_SCL_PIN);
    void update();

    float getPitch() const;
    float getRoll() const;
    float getYaw() const;

    bool isConnected() const;

private:
    MPU9250_WE m_mpu;
    bool m_connected = false;
    bool m_initialized = false;
    float m_pitch = 0;
    float m_roll = 0;
    float m_yaw = 0;
    unsigned long m_lastUpdate = 0;

};

#endif
