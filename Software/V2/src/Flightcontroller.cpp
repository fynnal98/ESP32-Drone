#include "FlightController.h"

float FlightController::clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void FlightController::setConfig(const FcConfig &cfg)
{
    _cfg = cfg;

    // Angle
    _pidAngleRoll.setGains(cfg.angleRoll.kp, cfg.angleRoll.ki, cfg.angleRoll.kd);
    _pidAngleRoll.setLimits(cfg.angleRoll.iLimit, cfg.angleRoll.outLimit);

    _pidAnglePitch.setGains(cfg.anglePitch.kp, cfg.anglePitch.ki, cfg.anglePitch.kd);
    _pidAnglePitch.setLimits(cfg.anglePitch.iLimit, cfg.anglePitch.outLimit);

    // Rate
    _pidRateRoll.setGains(cfg.rateRoll.kp, cfg.rateRoll.ki, cfg.rateRoll.kd);
    _pidRateRoll.setLimits(cfg.rateRoll.iLimit, cfg.rateRoll.outLimit);

    _pidRatePitch.setGains(cfg.ratePitch.kp, cfg.ratePitch.ki, cfg.ratePitch.kd);
    _pidRatePitch.setLimits(cfg.ratePitch.iLimit, cfg.ratePitch.outLimit);

    _pidRateYaw.setGains(cfg.rateYaw.kp, cfg.rateYaw.ki, cfg.rateYaw.kd);
    _pidRateYaw.setLimits(cfg.rateYaw.iLimit, cfg.rateYaw.outLimit);
}

void FlightController::begin()
{
    reset();
}

void FlightController::reset()
{
    _pidAngleRoll.reset();
    _pidAnglePitch.reset();

    _pidRateRoll.reset();
    _pidRatePitch.reset();
    _pidRateYaw.reset();
}

float FlightController::angleToRate(float angleSetDeg, float angleMeasDeg, Pid &anglePid, float dt)
{
    // output: deg/s
    return anglePid.update(angleSetDeg, angleMeasDeg, dt);
}

float FlightController::rateControl(float rateSetDps, float rateMeasDps, Pid &ratePid, float dt)
{
    // output: normalized correction (Mixer-Units)
    return ratePid.update(rateSetDps, rateMeasDps, dt);
}

MotorOutputs FlightController::update(SbusReceiver &rc,
                                      const Mpu6050 &imu,
                                      float dt,
                                      bool armed,
                                      float levelRollOffsetDeg,
                                      float levelPitchOffsetDeg)
{
    MotorOutputs out{};

    if (!armed)
    {
        return out;
    }

    // Throttle 0..1
    float thr = clampf(rc.throttle(), 0.0f, 1.0f);

    // Unter Cutoff: Motoren AUS + keine Regelung
    if (thr < THR_CUTOFF)
    {
        return out;
    }

    // Throttle auf Idle..THROTTLE_MAX skalieren (Headroom fürs Mixing)
    const float thrScaled = clampf(
        THROTTLE_IDLE + thr * (THROTTLE_MAX - THROTTLE_IDLE),
        0.0f, 1.0f);

    // Korrekturen bei niedrigem Throttle sanft hochfahren
    float rampDen = _cfg.corrRampFullAtThr;
    if (rampDen < 0.05f) rampDen = 0.05f; // Schutz
    float corrScale = (thr - THR_CUTOFF) / rampDen;
    corrScale = clampf(corrScale, 0.0f, 1.0f);

    // Stick -> Angle Sollwert (Angle Mode)
    const float rollSetDeg  = rc.roll()  * MAX_ANGLE_DEG;
    const float pitchSetDeg = rc.pitch() * MAX_ANGLE_DEG;

    // Gemessene Winkel (mit Level-Offset)
    const float rollMeasDeg  = imu.rollDeg()  - levelRollOffsetDeg;
    const float pitchMeasDeg = imu.pitchDeg() - levelPitchOffsetDeg;

    // Angle -> Rate Sollwerte (deg/s), begrenzen
    const float rollRateSet = clampf(
        angleToRate(rollSetDeg, rollMeasDeg, _pidAngleRoll, dt),
        -MAX_RATE_ROLL_DPS, MAX_RATE_ROLL_DPS);

    const float pitchRateSet = clampf(
        angleToRate(pitchSetDeg, pitchMeasDeg, _pidAnglePitch, dt),
        -MAX_RATE_PITCH_DPS, MAX_RATE_PITCH_DPS);

    // Yaw: direkt Rate Sollwert aus Stick
    const float yawRateSet = rc.yaw() * MAX_RATE_YAW_DPS;

    // Rate PIDs (Gyro rates)
    float rollCorr  = rateControl(rollRateSet,  imu.gyroX_dps(), _pidRateRoll,  dt);
    float pitchCorr = rateControl(pitchRateSet, imu.gyroY_dps(), _pidRatePitch, dt);
    float yawCorr   = rateControl(yawRateSet,   imu.gyroZ_dps(), _pidRateYaw,   dt);

    // Low-throttle scaling
    rollCorr  *= corrScale;
    pitchCorr *= corrScale;
    yawCorr   *= corrScale;

    // QUADX Mixer:
    // (m1..m4 sind "normalized motor commands" 0..1 nach clamp)
    float m1 = thrScaled + pitchCorr + rollCorr - yawCorr;
    float m2 = thrScaled + pitchCorr - rollCorr + yawCorr;
    float m3 = thrScaled - pitchCorr - rollCorr - yawCorr;
    float m4 = thrScaled - pitchCorr + rollCorr + yawCorr;

    // Optional: Front/Back swap (wenn Layout/Belegung vertauscht ist)
    if (MIX_SWAP_FRONT_BACK)
    {
        const float t1 = m1, t2 = m2;
        m1 = m4; m2 = m3; m3 = t2; m4 = t1;
    }

    out.m1 = clampf(m1, 0.0f, 1.0f);
    out.m2 = clampf(m2, 0.0f, 1.0f);
    out.m3 = clampf(m3, 0.0f, 1.0f);
    out.m4 = clampf(m4, 0.0f, 1.0f);

    return out;
}