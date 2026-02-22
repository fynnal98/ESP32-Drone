#ifndef CONFIG_H
#define CONFIG_H



// === SBUS ===
#define SBUS_RX_PIN      20
#define SBUS_INVERTED    true
#define SBUS_SIGNAL_TIMEOUT_MS 250
#define SBUS_SMOOTHING_ALPHA  0.8f  
#define SBUS_INPUT_MIN 307
#define SBUS_INPUT_MAX 1693
#define SBUS_DEADBAND 0.03f
#define ROLL_SCALE   0.2f
#define PITCH_SCALE  0.2f
#define YAW_SCALE    0.2f
#define THROTTLE_SCALE  1.00f
#define IDLE_THROTTLE 0.08f

// === PWM ===
#define PWM_FREQ         12000
#define PWM_RES_BITS     10
#define PWM_MAX_VALUE    ((1 << PWM_RES_BITS) - 1)

// PWM-Kanäle
#define PWM_CH_MOTOR1    0
#define PWM_CH_MOTOR2    1
#define PWM_CH_MOTOR3    2
#define PWM_CH_MOTOR4    3

// Pins für Motor-PWM
#define PIN_MOTOR1       3
#define PIN_MOTOR2       4
#define PIN_MOTOR3       5
#define PIN_MOTOR4       2

// === IMU (MPU6050) ===
#define IMU_OFFSET_X   0.000f 
#define IMU_OFFSET_Y   0.000f
#define IMU_OFFSET_Z   0.000f
#define IMU_SDA_PIN      6
#define IMU_SCL_PIN      7
#define CORRECTION_BLEND_FACTOR  0.2f
#define MPU_COMPLEMENTARY_ALPHA 0.98f
#define GYRO_LPF_ALPHA 0.7f
#define IMU_GYRO_CALIB_SAMPLES  300
#define IMU_GYRO_CALIB_DELAY_MS 2


// === PID Parameter ===

#define PID_CORRECTION_LIMIT 0.5f
 
#define PID_ROLL_KP   0.8f
#define PID_ROLL_KI   0.03f
#define PID_ROLL_KD   0.02f

#define PID_PITCH_KP  0.8f
#define PID_PITCH_KI  0.03f
#define PID_PITCH_KD  0.02f



#endif
