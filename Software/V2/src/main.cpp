#include <Arduino.h>

static const int PWM_PIN = 5;      // hier deinen Motor-PWM-Pin eintragen
static const int PWM_CH  = 0;
static const int PWM_RES = 8;       // 0..255
static const int PWM_FREQ = 20000;  // 20 kHz (leiser bei DC-Motoren)

static void setDuty(int duty)
{
  duty = constrain(duty, 0, (1 << PWM_RES) - 1);
  ledcWrite(PWM_CH, duty);
}

void setup()
{
  Serial.begin(115200);
  delay(300);

  Serial.println("MOSFET Driver Test startet");
  Serial.println("Wichtig: Motor hat eigene Versorgung, GND gemeinsam, Diode korrekt.");

  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CH);

  setDuty(0);
}

void loop()
{
  Serial.println("\nFULL ON (2s) -> FULL OFF (1s)");
  setDuty(255);
  delay(2000);
  setDuty(0);
  delay(1000);

  Serial.println("Ramp UP");
  for (int duty = 0; duty <= 255; duty += 5)
  {
    setDuty(duty);
    Serial.print("Duty="); Serial.print(duty);
    Serial.print("  ("); Serial.print((duty * 100) / 255); Serial.println("%)");
    delay(80);
  }

  Serial.println("Hold 50% (2s)");
  setDuty(128);
  delay(2000);

  Serial.println("Ramp DOWN");
  for (int duty = 255; duty >= 0; duty -= 5)
  {
    setDuty(duty);
    Serial.print("Duty="); Serial.print(duty);
    Serial.print("  ("); Serial.print((duty * 100) / 255); Serial.println("%)");
    delay(80);
  }

  Serial.println("Pause (2s)");
  delay(2000);
}