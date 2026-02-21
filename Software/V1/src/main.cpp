#include <Arduino.h>
#include "Config.h"
#include "SbusReceiver.h"
#include "MPU6050.h"
#include "FlightController.h"

// === Globale Objekte ===
SbusReceiver sbus;
MPU6050 imu;
FlightController controller(&imu, &sbus);

unsigned long lowBattSince = 0;
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(115200);

  // === SBUS init ===
  sbus.begin();

  // === IMU init ===
  imu.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  if (imu.isConnected()) {
    Serial.println("IMU verbunden");
  } else {
    Serial.println("IMU NICHT gefunden, fliege ohne Stabilisierung");
  }

  // === PWM init ===
  ledcSetup(PWM_CH_MOTOR1, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CH_MOTOR2, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CH_MOTOR3, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CH_MOTOR4, PWM_FREQ, PWM_RES_BITS);

  ledcAttachPin(PIN_MOTOR1, PWM_CH_MOTOR1);
  ledcAttachPin(PIN_MOTOR2, PWM_CH_MOTOR2);
  ledcAttachPin(PIN_MOTOR3, PWM_CH_MOTOR3);
  ledcAttachPin(PIN_MOTOR4, PWM_CH_MOTOR4);

  // === LED init ===
  pinMode(PIN_LED_WARNING, OUTPUT);
  digitalWrite(PIN_LED_WARNING, LOW);

  lastUpdate = millis();
}

void loop() {
  unsigned long now = millis();

  // === Akku-Messung ===
  int mv = analogReadMilliVolts(PIN_VBAT);     // Millivolt am ADC-Pin
  float v_adc = mv / 1000.0f;                  // in Volt
  float v_batt = v_adc * ((VBAT_R1 + VBAT_R2) / VBAT_R2);


  if (v_batt < VBAT_WARNING_THRESHOLD) {
    if (lowBattSince == 0){
      lowBattSince = now;
    }
    if (now - lowBattSince > VBatt_WARNING_Timeout) {
      digitalWrite(PIN_LED_WARNING, HIGH);
    }
  } else {
    lowBattSince = 0;
    digitalWrite(PIN_LED_WARNING, LOW);
  }

  // === Timing ===
  float dt = (now - lastUpdate) / 1000.0f;
  lastUpdate = now;
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.01f; // Fallback 10ms
  }

  // === SBUS update ===
  sbus.update();
  if (sbus.isSignalLost()) {
    ledcWrite(PWM_CH_MOTOR1, 0);
    ledcWrite(PWM_CH_MOTOR2, 0);
    ledcWrite(PWM_CH_MOTOR3, 0);
    ledcWrite(PWM_CH_MOTOR4, 0);
    Serial.println("SBUS FAILSAFE aktiv");
      // LED blinkt alle 300 ms (ca. 2 Hz)
    if ((millis() / 300) % 2 == 0) {
      digitalWrite(PIN_LED_WARNING, HIGH);
    } else {
      digitalWrite(PIN_LED_WARNING, LOW);
    }
    delay(10);
    return;
  } else {
    digitalWrite(PIN_LED_WARNING, LOW);
  }

  // === Steuerwerte vom Sender ===
  float baseThrottle = sbus.getThrottle();
  float pitchRaw     = sbus.getPitch();
  float rollRaw      = sbus.getRoll();
  float yawOut       = sbus.getYaw();

  float pitchCorr = 0;
  float rollCorr  = 0;

  // === IMU-Korrekturen ===
  if (imu.isConnected()) {
    imu.update();
    controller.update(dt);

    pitchCorr = controller.getMotorPitchOutput();
    rollCorr  = controller.getMotorRollOutput();

    // Schutz vor NaN/Inf
    if (isnan(pitchCorr) || isinf(pitchCorr)) pitchCorr = 0;
    if (isnan(rollCorr)  || isinf(rollCorr))  rollCorr  = 0;

    // Serial.printf("IMU: Roll=%.2f Pitch=%.2f\n", imu.getRoll(), imu.getPitch());
  }

  // === Mixer ===
  float pitchOut = pitchRaw - CORRECTION_BLEND_FACTOR * pitchCorr;
  float rollOut  = rollRaw  - CORRECTION_BLEND_FACTOR * rollCorr;


  float m1 = baseThrottle + pitchOut + rollOut - yawOut;
  float m2 = baseThrottle + pitchOut - rollOut + yawOut;
  float m3 = baseThrottle - pitchOut - rollOut - yawOut;
  float m4 = baseThrottle - pitchOut + rollOut + yawOut;

  int pwm1 = constrain((int)(m1 * PWM_MAX_VALUE), 0, PWM_MAX_VALUE);
  int pwm2 = constrain((int)(m2 * PWM_MAX_VALUE), 0, PWM_MAX_VALUE);
  int pwm3 = constrain((int)(m3 * PWM_MAX_VALUE), 0, PWM_MAX_VALUE);
  int pwm4 = constrain((int)(m4 * PWM_MAX_VALUE), 0, PWM_MAX_VALUE);

  // === Motoren nur wenn ARM ===
  bool armed = sbus.isArmedSwitchOn();
  if (armed) {
    ledcWrite(PWM_CH_MOTOR1, pwm1);
    ledcWrite(PWM_CH_MOTOR2, pwm2);
    ledcWrite(PWM_CH_MOTOR3, pwm3);
    ledcWrite(PWM_CH_MOTOR4, pwm4);
  } else {
    ledcWrite(PWM_CH_MOTOR1, 0);
    ledcWrite(PWM_CH_MOTOR2, 0);
    ledcWrite(PWM_CH_MOTOR3, 0);
    ledcWrite(PWM_CH_MOTOR4, 0);
  }

  // === Statusausgabe ===
  // Serial.printf("BAT: %.2f V | ARMED: %s | PWM: [%4d %4d %4d %4d]  IMU: %s\n",
  //               v_batt,
  //               armed ? "JA" : "NEIN",
  //               pwm1, pwm2, pwm3, pwm4,
  //               imu.isConnected() ? "OK" : "FEHLT");

  // Serial.printf("Roll=%.2f Pitch=%.2f Yaw=%.2f\n",
  //             sbus.getRoll(), sbus.getPitch(), sbus.getYaw());


  Serial.printf("RAW: CH1=%d CH2=%d CH3=%d CH4=%d\n",
                (int)sbus.getRawChannel(0),
                (int)sbus.getRawChannel(1),
                (int)sbus.getRawChannel(2),
                (int)sbus.getRawChannel(3));

  delay(10); 
}
