// MPU6050.cpp
#include "MPU6050.h"
#include "Config.h"
#include <math.h>

// Hilfsfunktion für Kreuzprodukt
static inline xyzFloat cross(const xyzFloat &a, const xyzFloat &b)
{
  return { a.y*b.z - a.z*b.y,
           a.z*b.x - a.x*b.z,
           a.x*b.y - a.y*b.x };
}

MPU6050::MPU6050()
: m_mpu(MPU9250_WE(0x68))
{
}

void MPU6050::begin(int sda, int scl)
{
  Wire.begin(sda, scl);
  delay(500);

  int r = m_mpu.init();
  Serial.print("mpu.init() returned = ");
  Serial.println(r);

  // Bei deiner Library: 0 = OK
  if (r != 0)
  {
    Serial.println("IMU NICHT gefunden!");
    m_connected = false;
    return;
  }

  // === Acc & Gyro konfigurieren ===
  m_mpu.setAccRange(MPU9250_ACC_RANGE_4G);
  m_mpu.setGyrRange(MPU9250_GYRO_RANGE_500);     // ±500 dps → 65.5 LSB/°/s
  m_mpu.enableGyrAxes(MPU9250_ENABLE_XYZ);

  // interner DLPF (optional, hilft gegen Rauschen)
  m_mpu.enableGyrDLPF();
  m_mpu.setGyrDLPF(MPU9250_DLPF_3);              // 41 Hz
  m_mpu.setSampleRateDivider(5);                 // ~200 Hz

  // externer Lowpass (zusätzlich)
  m_gyroLpf.setAlpha(GYRO_LPF_ALPHA);
  m_gyroLpf.reset();

  m_initialized = false;
  m_lastUpdate = 0;

  m_connected = true;
  Serial.println("IMU verbunden.");
}

void MPU6050::calibrateGyro()
{
  if (!m_connected)
    return;

  m_calibrating = true;

  double sx = 0.0, sy = 0.0, sz = 0.0;

  for (int i = 0; i < IMU_GYRO_CALIB_SAMPLES; i++)
  {
    xyzFloat g = m_mpu.getGyrRawValues();

    float gx = g.x / 65.5f; // °/s
    float gy = g.y / 65.5f;
    float gz = g.z / 65.5f;

    sx += gx;
    sy += gy;
    sz += gz;

    delay(IMU_GYRO_CALIB_DELAY_MS);
  }

  m_gyroBiasX = (float)(sx / IMU_GYRO_CALIB_SAMPLES);
  m_gyroBiasY = (float)(sy / IMU_GYRO_CALIB_SAMPLES);
  m_gyroBiasZ = (float)(sz / IMU_GYRO_CALIB_SAMPLES);

  // Filter neu starten (damit Bias-Änderung keinen Sprung erzeugt)
  m_gyroLpf.reset();

  m_calibrating = false;
}

bool MPU6050::isCalibrating() const
{
  return m_calibrating;
}

void MPU6050::update()
{
  if (!m_connected)
    return;

  unsigned long now = millis();
  float dt = (m_lastUpdate > 0) ? (now - m_lastUpdate) / 1000.0f : 0.01f;
  m_lastUpdate = now;

  if (dt <= 0.0f || dt > 0.2f)
    dt = 0.01f;

  // Sensorwerte lesen
  xyzFloat accRaw = m_mpu.getAccRawValues();
  xyzFloat gyrRaw = m_mpu.getGyrRawValues();

  // Gyro: raw -> °/s
  float gyroX = gyrRaw.x / 65.5f;
  float gyroY = gyrRaw.y / 65.5f;
  float gyroZ = gyrRaw.z / 65.5f;

  // === Gyro Bias abziehen (Kalibrierung) ===
  gyroX -= m_gyroBiasX;
  gyroY -= m_gyroBiasY;
  gyroZ -= m_gyroBiasZ;

  // === Gyro Lowpass anwenden ===
  {
    Vec3f gOut = m_gyroLpf.update(Vec3f{ gyroX, gyroY, gyroZ });
    gyroX = gOut.x;
    gyroY = gOut.y;
    gyroZ = gOut.z;
  }

  // Offset-Kompensation (Sensorversatz in m, aus Config.h)
  const xyzFloat r = { IMU_OFFSET_X, IMU_OFFSET_Y, IMU_OFFSET_Z };

  // omega in rad/s
  constexpr float DEG2RAD = PI / 180.0f;
  xyzFloat omega = { gyroX * DEG2RAD, gyroY * DEG2RAD, gyroZ * DEG2RAD };

  // alpha ≈ d(omega)/dt
  static bool havePrev = false;
  static xyzFloat omegaPrev = {0,0,0};
  xyzFloat alpha = {0,0,0};

  if (havePrev)
  {
    alpha.x = (omega.x - omegaPrev.x) / dt;
    alpha.y = (omega.y - omegaPrev.y) / dt;
    alpha.z = (omega.z - omegaPrev.z) / dt;
  }
  else
  {
    havePrev = true;
  }
  omegaPrev = omega;

  // a_rot = alpha x r + omega x (omega x r)
  xyzFloat term1 = cross(alpha, r);
  xyzFloat wxr   = cross(omega, r);
  xyzFloat term2 = cross(omega, wxr);
  xyzFloat a_rot = { term1.x + term2.x, term1.y + term2.y, term1.z + term2.z };

  // Acc: raw -> g (±4g → 8192 LSB/g), dann a_rot abziehen
  constexpr float ACC_SENS = 8192.0f;
  constexpr float G = 9.80665f;

  xyzFloat acc_g    = { accRaw.x / ACC_SENS, accRaw.y / ACC_SENS, accRaw.z / ACC_SENS };
  xyzFloat acc_mps2 = { acc_g.x * G, acc_g.y * G, acc_g.z * G };

  acc_mps2.x -= a_rot.x;
  acc_mps2.y -= a_rot.y;
  acc_mps2.z -= a_rot.z;

  xyzFloat acc_corr = { acc_mps2.x / G, acc_mps2.y / G, acc_mps2.z / G }; // wieder in g

  // Acc-Winkel (Grad)
  float rollAcc  = atan2(acc_corr.y, acc_corr.z) * 180.0f / PI;
  float pitchAcc = atan2(-acc_corr.x, sqrt(acc_corr.y * acc_corr.y + acc_corr.z * acc_corr.z)) * 180.0f / PI;

  // Initialisierung
  if (!m_initialized)
  {
    m_roll = rollAcc;
    m_pitch = pitchAcc;
    m_initialized = true;

    // Filterzustand initialisieren (damit er direkt sauber startet)
    m_gyroLpf.reset(Vec3f{ gyroX, gyroY, gyroZ });
  }

  // Komplementärfilter
  const float alphaF = MPU_COMPLEMENTARY_ALPHA;
  m_roll  = alphaF * (m_roll  + gyroX * dt) + (1.0f - alphaF) * rollAcc;
  m_pitch = alphaF * (m_pitch + gyroY * dt) + (1.0f - alphaF) * pitchAcc;

  m_yaw = 0.0f;
}

float MPU6050::getPitch() const { return m_pitch / 90.0f; }
float MPU6050::getRoll()  const { return m_roll  / 90.0f; }
float MPU6050::getYaw()   const { return m_yaw; }

bool MPU6050::isConnected() const { return m_connected; }