#pragma once
#include <stdint.h>

class Pid {
public:
  Pid() = default;
  Pid(float kp, float ki, float kd, float iLimit, float outLimit);

  void setGains(float kp, float ki, float kd);
  void setLimits(float iLimit, float outLimit);

  void reset();
  float update(float setpoint, float measurement, float dt);

private:
  float _kp{0}, _ki{0}, _kd{0};
  float _iLimit{0}, _outLimit{0};
  float _i{0};
  float _prevErr{0};
  bool _hasPrev{false};
};