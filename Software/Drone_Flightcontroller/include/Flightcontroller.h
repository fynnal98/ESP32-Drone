#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "MPU6050.h"
#include "SbusReceiver.h"
#include "PidController.h"

class FlightController {
public:
    FlightController(MPU6050* imu, SbusReceiver* sbus);
    void update(float dt);
    float getMotorRollOutput() const;
    float getMotorPitchOutput() const;

private:
    MPU6050* m_imu;
    SbusReceiver* m_sbus;

    PIDController m_pidRoll;
    PIDController m_pidPitch;

    float m_rollOut;
    float m_pitchOut;
};

#endif
