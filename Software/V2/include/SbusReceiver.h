#pragma once
#include <Arduino.h>
#include "sbus.h"
#include "Config.h"

class SbusReceiver {
public:
  void begin();
  void update();

  bool signalLost() const;

  // normalized [-1..+1] except throttle [0..1]
  float roll();
  float pitch();
  float yaw();
  float throttle();

  bool armed() const;

private:
  float mapSym(int raw) const;
  float mapThr(int raw) const;
  float applyDeadband(float x) const;

private:
  bfs::SbusRx _sbus = bfs::SbusRx(&Serial1, SBUS_RX_PIN, -1, SBUS_INVERTED);
  bfs::SbusData _data{};

  uint32_t _lastOkMs{0};

  // smoothed
  float _r{0}, _p{0}, _y{0}, _t{0};
};