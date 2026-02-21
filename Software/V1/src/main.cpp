// main.cpp
#include <Arduino.h>
#include "Config.h"
#include "SbusReceiver.h"
#include "MPU6050.h"
#include "FlightController.h"

// === Globale Objekte ===
SbusReceiver sbus;
MPU6050 imu;
FlightController controller(&imu, &sbus);

static unsigned long lastUpdate = 0;

static inline void StopAllMotors()
{
  ledcWrite(PWM_CH_MOTOR1, 0);
  ledcWrite(PWM_CH_MOTOR2, 0);
  ledcWrite(PWM_CH_MOTOR3, 0);
  ledcWrite(PWM_CH_MOTOR4, 0);
}

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("BOOT");

  // === SBUS init ===
  sbus.begin();

  // === IMU init ===
  imu.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  if (imu.isConnected())
    Serial.println("IMU verbunden");
  else
    Serial.println("IMU NICHT gefunden, fliege ohne Stabilisierung");

  // === PWM init ===
  ledcSetup(PWM_CH_MOTOR1, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CH_MOTOR2, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CH_MOTOR3, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CH_MOTOR4, PWM_FREQ, PWM_RES_BITS);

  ledcAttachPin(PIN_MOTOR1, PWM_CH_MOTOR1);
  ledcAttachPin(PIN_MOTOR2, PWM_CH_MOTOR2);
  ledcAttachPin(PIN_MOTOR3, PWM_CH_MOTOR3);
  ledcAttachPin(PIN_MOTOR4, PWM_CH_MOTOR4);

  StopAllMotors();

  lastUpdate = millis();
}

void loop()
{
  const unsigned long now = millis();

  // === Timing ===
  float dt = (now - lastUpdate) / 1000.0f;
  lastUpdate = now;
  if (dt <= 0.0f || dt > 0.5f)
    dt = 0.01f;

  // === SBUS update ===
  sbus.update();
  if (sbus.isSignalLost())
  {
    StopAllMotors();

    static unsigned long lastMsg = 0;
    if (now - lastMsg > 300)
    {
      lastMsg = now;
      Serial.println("SBUS FAILSAFE aktiv");
    }

    delay(10);
    return;
  }

  // === ARM Status ===
  const bool armed = sbus.isArmedSwitchOn();

  // === Steuerwerte vom Sender ===
  float throttle = sbus.getThrottle(); // 0..1
  float pitchRaw = sbus.getPitch();    // -1..1
  float rollRaw  = sbus.getRoll();     // -1..1
  float yawRaw   = sbus.getYaw();      // -1..1

  // Optional: Idle nur wenn armed und throttle > 0
#if defined(IDLE_THROTTLE)
  if (armed && throttle > 0.05f && throttle < IDLE_THROTTLE)
    throttle = IDLE_THROTTLE;
#endif

  float pitchCorr = 0.0f;
  float rollCorr  = 0.0f;

  // === IMU-Korrekturen ===
  if (imu.isConnected())
  {
    imu.update();
    controller.update(dt);

    pitchCorr = controller.getMotorPitchOutput();
    rollCorr  = controller.getMotorRollOutput();

    if (isnan(pitchCorr) || isinf(pitchCorr)) pitchCorr = 0.0f;
    if (isnan(rollCorr)  || isinf(rollCorr))  rollCorr  = 0.0f;

#if defined(PID_CORRECTION_LIMIT)
    pitchCorr = constrain(pitchCorr, -PID_CORRECTION_LIMIT, PID_CORRECTION_LIMIT);
    rollCorr  = constrain(rollCorr,  -PID_CORRECTION_LIMIT, PID_CORRECTION_LIMIT);
#endif
  }

  // === Mixer (Stick + Stabilisierung) ===
  const float pitchOut = pitchRaw - (CORRECTION_BLEND_FACTOR * pitchCorr);
  const float rollOut  = rollRaw  - (CORRECTION_BLEND_FACTOR * rollCorr);
  const float yawOut   = yawRaw;

  float m1 = throttle + pitchOut + rollOut - yawOut;
  float m2 = throttle + pitchOut - rollOut + yawOut;
  float m3 = throttle - pitchOut - rollOut - yawOut;
  float m4 = throttle - pitchOut + rollOut + yawOut;

  // Motorwerte 0..1 begrenzen (wichtig vor PWM)
  m1 = constrain(m1, 0.0f, 1.0f);
  m2 = constrain(m2, 0.0f, 1.0f);
  m3 = constrain(m3, 0.0f, 1.0f);
  m4 = constrain(m4, 0.0f, 1.0f);

  const int pwm1 = (int)(m1 * PWM_MAX_VALUE);
  const int pwm2 = (int)(m2 * PWM_MAX_VALUE);
  const int pwm3 = (int)(m3 * PWM_MAX_VALUE);
  const int pwm4 = (int)(m4 * PWM_MAX_VALUE);

  // === Motoren nur wenn ARM ===
  if (armed)
  {
    ledcWrite(PWM_CH_MOTOR1, pwm1);
    ledcWrite(PWM_CH_MOTOR2, pwm2);
    ledcWrite(PWM_CH_MOTOR3, pwm3);
    ledcWrite(PWM_CH_MOTOR4, pwm4);
  }
  else
  {
    StopAllMotors();
  }

  // optional: Debug selten
  /*
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 200) {
    lastPrint = now;
    Serial.print("armed="); Serial.print((int)armed);
    Serial.print(" thr=");  Serial.print(throttle, 2);
    Serial.print(" roll="); Serial.print(imu.getRoll(), 3);
    Serial.print(" pitch=");Serial.print(imu.getPitch(), 3);
    Serial.print(" pc=");   Serial.print(pitchCorr, 3);
    Serial.print(" rc=");   Serial.println(rollCorr, 3);
  }
  */

  delay(10);
}