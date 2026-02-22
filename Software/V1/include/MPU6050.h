// MPU6050.h
#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

#include "MPU9250_WE.h"     // xyzFloat, MPU9250_WE
#include "LowPassFilter.h"  // Vec3f, LowPassFilter3

class MPU6050
{
public:
  MPU6050();

  void begin(int sda, int scl);
  void update();

  // Gyro Bias Kalibrierung (Drone beim Aufruf still halten!)
  void calibrateGyro();
  bool isCalibrating() const;

  float getPitch() const; // normiert (Grad/90)
  float getRoll() const;  // normiert (Grad/90)
  float getYaw() const;   // aktuell 0 (kein Mag)

  bool isConnected() const;

private:
  MPU9250_WE m_mpu;

  bool m_connected = false;
  bool m_initialized = false;
  bool m_calibrating = false;

  unsigned long m_lastUpdate = 0;

  float m_roll  = 0.0f; // Grad
  float m_pitch = 0.0f; // Grad
  float m_yaw   = 0.0f;

  // Gyro Bias (°/s)
  float m_gyroBiasX = 0.0f;
  float m_gyroBiasY = 0.0f;
  float m_gyroBiasZ = 0.0f;

  // Gyro Lowpass (°/s)
  LowPassFilter3 m_gyroLpf;
};

#endif