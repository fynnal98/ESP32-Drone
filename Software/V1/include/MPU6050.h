// MPU6050.h
#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

#include "MPU9250_WE.h"     // Library-Typen (xyzFloat, MPU9250_WE, etc.)
#include "LowPassFilter.h"  // Vec3f + LowPassFilter3

class MPU6050
{
public:
    MPU6050();

    void begin(int sda, int scl);
    void update();

    float getPitch() const; // normiert (Grad/90)
    float getRoll() const;  // normiert (Grad/90)
    float getYaw() const;   // aktuell 0 (kein Mag)

    bool isConnected() const;

private:
    MPU9250_WE m_mpu;

    bool m_connected = false;
    bool m_initialized = false;

    unsigned long m_lastUpdate = 0;

    float m_roll  = 0.0f;   // in Grad
    float m_pitch = 0.0f;   // in Grad
    float m_yaw   = 0.0f;   // in Grad (nicht genutzt)

    LowPassFilter3 m_gyroLpf; // Gyro Lowpass (°/s)
};

#endif