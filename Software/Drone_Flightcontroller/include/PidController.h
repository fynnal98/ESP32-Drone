#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    float update(float target, float actual, float dt);
    void reset();

private:
    float m_kp, m_ki, m_kd;
    float m_prevError = 0;
    float m_integral = 0;
};

#endif
