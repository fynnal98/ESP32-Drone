#include "Flightcontroller.h"
#include "Config.h"

FlightController::FlightController(MPU6050* imu, SbusReceiver* sbus)
    : m_imu(imu), m_sbus(sbus),
      m_pidRoll(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD),
      m_pidPitch(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD),
      m_rollOut(0), m_pitchOut(0) {}

void FlightController::update(float dt) {
    float targetRoll  = m_sbus->getRoll();   // Sollwert
    float targetPitch = m_sbus->getPitch();  // Sollwert

    float actualRoll  = m_imu->getRoll();    // Istwert
    float actualPitch = m_imu->getPitch();   // Istwert

    m_rollOut  = m_pidRoll.update(targetRoll,  actualRoll,  dt);
    m_pitchOut = m_pidPitch.update(targetPitch, actualPitch, dt);
}

float FlightController::getMotorRollOutput() const { return m_rollOut; }
float FlightController::getMotorPitchOutput() const { return m_pitchOut; }
