#include "Mpu6050.h"
#include "Config.h"
#include <Wire.h>
#include <math.h>

static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
static constexpr uint8_t REG_GYRO_CFG   = 0x1B;
static constexpr uint8_t REG_ACCEL_CFG  = 0x1C;
static constexpr uint8_t REG_DATA       = 0x3B;

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

bool Mpu6050::begin(int sda, int scl) {
  Wire.begin(sda, scl);
  Wire.setClock(400000);

  // Wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    _connected = false;
    return false;
  }

  // Gyro: ±500 dps (0x08)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_CFG);
  Wire.write(0x08);
  Wire.endTransmission();

  // Accel: ±4g (0x08)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_CFG);
  Wire.write(0x08);
  Wire.endTransmission();

  _connected = true;
  return true;
}

bool Mpu6050::readRaw(int16_t &ax, int16_t &ay, int16_t &az,
                      int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_DATA);
  if (Wire.endTransmission(false) != 0) return false;

  const uint8_t n = Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (n != 14) return false;

  auto rd16 = []() -> int16_t {
    const int16_t hi = (int16_t)Wire.read();
    const int16_t lo = (int16_t)Wire.read();
    return (int16_t)((hi << 8) | (lo & 0xFF));
  };

  ax = rd16();
  ay = rd16();
  az = rd16();
  (void)rd16(); // temp
  gx = rd16();
  gy = rd16();
  gz = rd16();
  return true;
}

void Mpu6050::calibrate(uint16_t samples) {
  if (!_connected) return;

  int64_t sx=0, sy=0, sz=0;
  int16_t ax, ay, az, gx, gy, gz;

  for (uint16_t i = 0; i < samples; i++) {
    if (readRaw(ax, ay, az, gx, gy, gz)) {
      sx += gx;
      sy += gy;
      sz += gz;
    }
    delay(2);
  }

  _gxo = (float)sx / (float)samples;
  _gyo = (float)sy / (float)samples;
  _gzo = (float)sz / (float)samples;
}

void Mpu6050::update(float dt) {
  if (!_connected) return;

  int16_t axr, ayr, azr, gxr, gyr, gzr;
  if (!readRaw(axr, ayr, azr, gxr, gyr, gzr)) return;

  // Scales for selected ranges:
  // Gyro ±500 dps => 65.5 LSB/(deg/s)
  // Accel ±4g     => 8192 LSB/g
  const float gx = ((float)gxr - _gxo) / 65.5f;
  const float gy = ((float)gyr - _gyo) / 65.5f;
  const float gz = ((float)gzr - _gzo) / 65.5f;

  _gx = gx; _gy = gy; _gz = gz;

  const float ax = (float)axr / 8192.0f;
  const float ay = (float)ayr / 8192.0f;
  const float az = (float)azr / 8192.0f;

  // accel angles (deg)
  const float rollAcc  = atan2f(ay, az) * 180.0f / (float)M_PI;
  const float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;

  // integrate gyro for roll/pitch
  const float rollGyro  = _rollDeg  + gx * dt;
  const float pitchGyro = _pitchDeg + gy * dt;

  // complementary filter
  _rollDeg  = COMP_ALPHA * rollGyro  + (1.0f - COMP_ALPHA) * rollAcc;
  _pitchDeg = COMP_ALPHA * pitchGyro + (1.0f - COMP_ALPHA) * pitchAcc;

  // clamp angles a bit (guards)
  _rollDeg  = clampf(_rollDeg,  -90.0f, 90.0f);
  _pitchDeg = clampf(_pitchDeg, -90.0f, 90.0f);
}