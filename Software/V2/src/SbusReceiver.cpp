#include "SbusReceiver.h"
#include <math.h>

void SbusReceiver::begin() {
  Serial1.begin(100000, SERIAL_8E2, SBUS_RX_PIN, -1, SBUS_INVERTED);
  delay(50);
  while (Serial1.available()) (void)Serial1.read();
  _sbus.Begin();
  _lastOkMs = millis();
}

void SbusReceiver::update() {
  if (_sbus.Read()) {
    _data = _sbus.data();
    if (!_data.failsafe) {
      _lastOkMs = millis();
    }
  }
}

bool SbusReceiver::signalLost() const {
  if (_data.failsafe) return true;
  return (millis() - _lastOkMs) > SBUS_SIGNAL_TIMEOUT_MS;
}

float SbusReceiver::applyDeadband(float x) const {
  return (fabsf(x) < RC_DEADBAND) ? 0.0f : x;
}

float SbusReceiver::mapSym(int raw) const {
  const float n = ((float)(raw - SBUS_INPUT_MIN) / (float)(SBUS_INPUT_MAX - SBUS_INPUT_MIN));
  float v = n * 2.0f - 1.0f;
  if (v < -1.0f) v = -1.0f;
  if (v >  1.0f) v =  1.0f;
  return v;
}

float SbusReceiver::mapThr(int raw) const {
  const float n = ((float)(raw - SBUS_INPUT_MIN) / (float)(SBUS_INPUT_MAX - SBUS_INPUT_MIN));
  float v = n;
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  return v;
}

float SbusReceiver::roll() {
  float raw = applyDeadband(mapSym(_data.ch[CH_ROLL]));
  _r = RC_SMOOTH_ALPHA * raw + (1.0f - RC_SMOOTH_ALPHA) * _r;
  return _r;
}

float SbusReceiver::pitch() {
  float raw = applyDeadband(mapSym(_data.ch[CH_PITCH]));
  _p = RC_SMOOTH_ALPHA * raw + (1.0f - RC_SMOOTH_ALPHA) * _p;
  return _p;
}

float SbusReceiver::yaw() {
  float raw = applyDeadband(mapSym(_data.ch[CH_YAW]));
  _y = RC_SMOOTH_ALPHA * raw + (1.0f - RC_SMOOTH_ALPHA) * _y;
  return _y;
}

float SbusReceiver::throttle() {
  float raw = mapThr(_data.ch[CH_THROTTLE]);
  _t = RC_SMOOTH_ALPHA * raw + (1.0f - RC_SMOOTH_ALPHA) * _t;
  return _t;
}

bool SbusReceiver::armed() const {
  return _data.ch[CH_ARM] > 1000; // wie bei dir
}