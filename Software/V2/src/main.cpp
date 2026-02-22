#include <Arduino.h>
#include "Config.h"
#include "SbusReceiver.h"
#include "Mpu6050.h"
#include "FlightController.h"

static SbusReceiver rc;
static Mpu6050 imu;
static FlightController fc;

static uint32_t lastMicros = 0;

// Level offsets (werden nach Boot-Delay gemittelt)
static float levelRollOffDeg = 0.0f;
static float levelPitchOffDeg = 0.0f;

static void motorsInit() {
  ledcSetup(PWM_CH_MOTOR1, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CH_MOTOR2, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CH_MOTOR3, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CH_MOTOR4, PWM_FREQ, PWM_RES_BITS);

  ledcAttachPin(PIN_MOTOR1, PWM_CH_MOTOR1);
  ledcAttachPin(PIN_MOTOR2, PWM_CH_MOTOR2);
  ledcAttachPin(PIN_MOTOR3, PWM_CH_MOTOR3);
  ledcAttachPin(PIN_MOTOR4, PWM_CH_MOTOR4);

  ledcWrite(PWM_CH_MOTOR1, 0);
  ledcWrite(PWM_CH_MOTOR2, 0);
  ledcWrite(PWM_CH_MOTOR3, 0);
  ledcWrite(PWM_CH_MOTOR4, 0);
}

static void motorsOff() {
  ledcWrite(PWM_CH_MOTOR1, 0);
  ledcWrite(PWM_CH_MOTOR2, 0);
  ledcWrite(PWM_CH_MOTOR3, 0);
  ledcWrite(PWM_CH_MOTOR4, 0);
}

static void motorsWrite(const MotorOutputs& m) {
  const uint32_t d1 = (uint32_t)(m.m1 * (float)PWM_MAX);
  const uint32_t d2 = (uint32_t)(m.m2 * (float)PWM_MAX);
  const uint32_t d3 = (uint32_t)(m.m3 * (float)PWM_MAX);
  const uint32_t d4 = (uint32_t)(m.m4 * (float)PWM_MAX);

  ledcWrite(PWM_CH_MOTOR1, d1);
  ledcWrite(PWM_CH_MOTOR2, d2);
  ledcWrite(PWM_CH_MOTOR3, d3);
  ledcWrite(PWM_CH_MOTOR4, d4);
}

static void bootDelayAndCalibrate() {
  // 1) Delay damit du die Drohne sauber hinlegen kannst
  const uint32_t waitMs = BOOT_WAIT_MS;
  Serial.printf("BOOT: Warte %lums zum Hinlegen...\n", (unsigned long)waitMs);
  uint32_t t0 = millis();
  while (millis() - t0 < waitMs) {
    motorsOff();
    delay(50);
  }

  // 2) Gyro kalibrieren (Drohne muss ruhig liegen)
  if (imu.isConnected()) {
    Serial.println("BOOT: Gyro Kalibrierung (ruhig liegen lassen)...");
    imu.calibrate(600);
    Serial.println("BOOT: Gyro Kalibrierung fertig");
  }

  // 3) Level-Offset mitteln (Roll/Pitch Null setzen)
  //    (nur sinnvoll, wenn rollDeg/pitchDeg aus Acc+Gyro kommen)
  if (imu.isConnected()) {
    Serial.println("BOOT: Level-Offset messen (still & level)...");
    const uint32_t levelMs = LEVEL_CAL_MS;
    uint32_t start = millis();
    uint32_t n = 0;
    float sumR = 0.0f, sumP = 0.0f;

    // kleine dt Updates, damit roll/pitch frisch sind
    uint32_t last = micros();
    while (millis() - start < levelMs) {
      uint32_t now = micros();
      float dt = (float)(now - last) / 1000000.0f;
      last = now;
      if (dt < DT_MIN) dt = DT_MIN;
      if (dt > DT_MAX) dt = DT_MAX;

      imu.update(dt);
      sumR += imu.rollDeg();
      sumP += imu.pitchDeg();
      n++;
      motorsOff();
      delay(5);
    }

    if (n > 0) {
      levelRollOffDeg = sumR / (float)n;
      levelPitchOffDeg = sumP / (float)n;
    }
    Serial.printf("BOOT: Level-Offset gesetzt rollOff=%.2f pitchOff=%.2f\n",
                  levelRollOffDeg, levelPitchOffDeg);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  motorsInit();
  rc.begin();

  const bool imuOk = imu.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  if (!imuOk) {
    Serial.println("MPU6050 NICHT gefunden");
  } else {
    Serial.println("MPU6050 OK");
  }

  // PID config (falls du das nutzt)
  fc.setConfig(FC_CFG);
  fc.begin();

  bootDelayAndCalibrate();

  lastMicros = micros();
}

void loop() {
  const uint32_t nowUs = micros();
  const uint32_t periodUs = 1000000UL / LOOP_HZ;

  if ((uint32_t)(nowUs - lastMicros) < periodUs) {
    delayMicroseconds(30);
    return;
  }

  float dt = (float)(nowUs - lastMicros) / 1000000.0f;
  lastMicros = nowUs;
  if (dt < DT_MIN) dt = DT_MIN;
  if (dt > DT_MAX) dt = 1.0f / (float)LOOP_HZ;

  rc.update();

  if (rc.signalLost()) {
    motorsOff();
    fc.reset();
    return;
  }

  if (imu.isConnected()) {
    imu.update(dt);
  }

  static bool armed = false;
  const bool armSwitch = rc.armed();
  const float thr = rc.throttle();

  if (!armSwitch) {
    armed = false;
    motorsOff();
    fc.reset();
    return;
  }

  if (!armed) {
    motorsOff();
    fc.reset();
    if (thr < THROTTLE_MIN_ARM) {
      armed = true;
      fc.reset();
    }
    return;
  }

  if (thr < THR_CUTOFF) {
    motorsOff();
    fc.reset();
    return;
  }

  // WICHTIG: Offsets übergeben
const MotorOutputs m = fc.update(rc, imu, dt, armed, levelRollOffDeg, levelPitchOffDeg);  motorsWrite(m);

Serial.printf("Rlvl=%.2f Plvl=%.2f\n",
              imu.rollDeg() - levelRollOffDeg,
              imu.pitchDeg() - levelPitchOffDeg);
}