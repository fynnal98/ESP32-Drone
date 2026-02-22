#pragma once
#include <Arduino.h>

class Mpu6050 {
public:
  bool begin(int sda, int scl);
  bool isConnected() const { return _connected; }

  void calibrate(uint16_t samples = 500);
  void update(float dt);

  // Angles (deg)
  float rollDeg()  const { return _rollDeg; }
  float pitchDeg() const { return _pitchDeg; }

  // Rates (deg/s)
  float gyroX_dps() const { return _gx; }
  float gyroY_dps() const { return _gy; }
  float gyroZ_dps() const { return _gz; }

private:
  bool readRaw(int16_t &ax, int16_t &ay, int16_t &az,
               int16_t &gx, int16_t &gy, int16_t &gz);

private:
  bool _connected{false};

  // offsets (raw)
  float _gxo{0}, _gyo{0}, _gzo{0};

  // scaled
  float _gx{0}, _gy{0}, _gz{0};

  // angles
  float _rollDeg{0}, _pitchDeg{0};
};