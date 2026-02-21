#include "PidController.h"
#include <cmath>


PIDController::PIDController(float kp, float ki, float kd)
    : m_kp(kp), m_ki(ki), m_kd(kd) {}

float PIDController::update(float target, float actual, float dt) {
    float error = target - actual;
    if (fabs(error) < 0.02f) error = 0;
    m_integral += error * dt;
    float derivative = (error - m_prevError) / dt;
    m_prevError = error;
    return m_kp * error + m_ki * m_integral + m_kd * derivative;
}

void PIDController::reset() {
    m_prevError = 0;
    m_integral = 0;
}
