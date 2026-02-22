#pragma once

#include "Config.h"
#include "Pid.h"
#include "Mpu6050.h"
#include "SbusReceiver.h"

struct MotorOutputs
{
    float m1 = 0.0f;
    float m2 = 0.0f;
    float m3 = 0.0f;
    float m4 = 0.0f;
};

class FlightController
{
public:
    FlightController() = default;

    void setConfig(const FcConfig &cfg);
    void begin();
    void reset();

    // Level-Offsets: "was beim Leveln als 0° gilt"
    MotorOutputs update(SbusReceiver &rc,
                        const Mpu6050 &imu,
                        float dt,
                        bool armed,
                        float levelRollOffsetDeg,
                        float levelPitchOffsetDeg);

private:
    static float clampf(float v, float lo, float hi);
    float angleToRate(float angleSetDeg, float angleMeasDeg, Pid &anglePid, float dt);
    float rateControl(float rateSetDps, float rateMeasDps, Pid &ratePid, float dt);

private:
    FcConfig _cfg{};

    // Angle PIDs: input=deg, output=deg/s (Rate-Sollwert)
    Pid _pidAngleRoll;
    Pid _pidAnglePitch;

    // Rate PIDs: input=deg/s, output=normalized correction (Mixer-Units)
    Pid _pidRateRoll;
    Pid _pidRatePitch;
    Pid _pidRateYaw;
};