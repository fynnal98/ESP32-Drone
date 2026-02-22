#include "Pid.h"
#include <math.h>

Pid::Pid(float kp, float ki, float kd, float iLimit, float outLimit)
: _kp(kp), _ki(ki), _kd(kd), _iLimit(iLimit), _outLimit(outLimit) {}

void Pid::setGains(float kp, float ki, float kd) { _kp = kp; _ki = ki; _kd = kd; }
void Pid::setLimits(float iLimit, float outLimit) { _iLimit = iLimit; _outLimit = outLimit; }

void Pid::reset() {
  _i = 0.0f;
  _prevErr = 0.0f;
  _hasPrev = false;
}

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float Pid::update(float setpoint, float measurement, float dt) {
  const float err = setpoint - measurement;

  // I
  _i += err * dt * _ki;
  _i = clampf(_i, -_iLimit, _iLimit);

  // D on error (simple, works ok for rates)
  float d = 0.0f;
  if (_hasPrev && dt > 1e-6f) {
    d = (err - _prevErr) / dt;
  }
  _prevErr = err;
  _hasPrev = true;

  float out = (_kp * err) + _i + (_kd * d);
  out = clampf(out, -_outLimit, _outLimit);
  return out;
}