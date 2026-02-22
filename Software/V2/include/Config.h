#pragma once
#include <stdint.h>

// === Boot / Calibration Timing ===
static constexpr uint32_t BOOT_WAIT_MS = 6000;        // Zeit zum Hinlegen nach Akku
static constexpr uint32_t LEVEL_CAL_MS = 1500;        // Dauer Level-Mittelung

/* =========================
 * SBUS
 * ========================= */
static constexpr int SBUS_RX_PIN     = 20;
static constexpr bool SBUS_INVERTED  = true;
static constexpr uint32_t SBUS_SIGNAL_TIMEOUT_MS = 300;

static constexpr int SBUS_INPUT_MIN = 307;
static constexpr int SBUS_INPUT_MAX = 1693;

/* Channel mapping (Radiolink typisch) */
static constexpr int CH_ROLL     = 0; // CH1
static constexpr int CH_PITCH    = 1; // CH2
static constexpr int CH_THROTTLE = 2; // CH3
static constexpr int CH_YAW      = 3; // CH4
static constexpr int CH_ARM      = 9; // AUX

/* RC shaping */
static constexpr float RC_DEADBAND     = 0.03f;
static constexpr float RC_SMOOTH_ALPHA = 0.85f;

/* =========================
 * IMU (MPU6050)
 * ========================= */
static constexpr int IMU_SDA_PIN = 6;
static constexpr int IMU_SCL_PIN = 7;

static constexpr float COMP_ALPHA = 0.98f;

/* =========================
 * Motors / Pins
 * ========================= */
static constexpr int PIN_MOTOR1 = 5;
static constexpr int PIN_MOTOR2 = 2;
static constexpr int PIN_MOTOR3 = 3;
static constexpr int PIN_MOTOR4 = 4;

/* =========================
 * PWM (LEDC) — dein funktionierendes Setup
 * ========================= */
static constexpr uint32_t PWM_FREQ     = 12000;
static constexpr uint8_t  PWM_RES_BITS = 10;
static constexpr uint32_t PWM_MAX      = (1u << PWM_RES_BITS) - 1u;

static constexpr int PWM_CH_MOTOR1 = 0;
static constexpr int PWM_CH_MOTOR2 = 1;
static constexpr int PWM_CH_MOTOR3 = 2;
static constexpr int PWM_CH_MOTOR4 = 3;

/* =========================
 * Control Loop
 * ========================= */
static constexpr uint32_t LOOP_HZ = 500;
static constexpr float DT_MIN = 1.0f / 2000.0f;
static constexpr float DT_MAX = 1.0f / 50.0f;

/* Stabilize */
static constexpr float MAX_ANGLE_DEG = 35.0f;

/* Rate limits */
static constexpr float MAX_RATE_ROLL_DPS  = 350.0f;
static constexpr float MAX_RATE_PITCH_DPS = 350.0f;
static constexpr float MAX_RATE_YAW_DPS   = 220.0f;

/* Safety */
static constexpr float THROTTLE_MIN_ARM = 0.02f; // arm nur bei low throttle
static constexpr float THR_CUTOFF       = 0.02f; // darunter: motor off + pid reset
static constexpr float THROTTLE_IDLE    = 0.10f; // erst aktiv wenn thr > cutoff
static constexpr float THROTTLE_MAX     = 0.95f; // headroom fürs Mixing

/* Mixer option */
static constexpr bool MIX_SWAP_FRONT_BACK = false;

// =========================
// PID Config (tunable)
// =========================
struct PidGains {
  float kp;
  float ki;
  float kd;
  float iLimit;
  float outLimit;
};

struct FcConfig {
  // Angle (outer loop) -> outputs desired rate in deg/s
  PidGains angleRoll;
  PidGains anglePitch;

  // Rate (inner loop) -> outputs normalized correction
  PidGains rateRoll;
  PidGains ratePitch;
  PidGains rateYaw;

  // Optional: scaling for corrections at low throttle
  float corrRampFullAtThr; // e.g. 0.35f means full corrections at ~35% throttle
};

static constexpr FcConfig FC_CFG = {
  /* angleRoll  */ { 4.5f, 0.0f, 0.06f, 40.0f, 220.0f },
  /* anglePitch */ { 4.5f, 0.0f, 0.06f, 40.0f, 220.0f },

                 /* { kp   , ki   , kd     , iLimit, outLimit } */
  /* rateRoll   */ { 0.3f, 0.10f, 0.0012f, 0.90f, 1.60f },
  /* ratePitch  */ { 0.3f, 0.10f, 0.0012f, 0.90f, 1.60f },
  /* rateYaw    */ { 0.08f, 0.05f, 0.0000f, 0.40f, 0.80f },

  /* corrRampFullAtThr */ 0.06f
};