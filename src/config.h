#ifndef CONFIG_H
#define CONFIG_H

// ==============================================================================
// Mars Rover — Configuration File
// ==============================================================================
// Single source of truth for all pin numbers and tunable constants.
// Update pins here after physical bring-up confirms actual wiring.
//
// Note: GPIO 45/46 are strapping pins on ESP32-S3. If Rear-Right motor behavior
// is unstable during boot, reassign those pins to non-strapping alternatives.
// ==============================================================================

// ------------------------------------------------------------------------------
// Motor Pins — 3x L298N (2 motors each, 6 motors total)
// ------------------------------------------------------------------------------
// Each motor: 2 direction pins (IN1/IN2) + 1 PWM enable pin (EN)

// L298N #1: Front-Left & Front-Right
#define MOTOR_FL_IN1    1
#define MOTOR_FL_IN2    2
#define MOTOR_FL_EN     3

#define MOTOR_FR_IN1    6
#define MOTOR_FR_IN2    7
#define MOTOR_FR_EN     8

// L298N #2: Mid-Left & Mid-Right
#define MOTOR_ML_IN1    11
#define MOTOR_ML_IN2    12
#define MOTOR_ML_EN     13

#define MOTOR_MR_IN1    16
#define MOTOR_MR_IN2    17
#define MOTOR_MR_EN     18

// L298N #3: Rear-Left & Rear-Right
#define MOTOR_RL_IN1    21
#define MOTOR_RL_IN2    38
#define MOTOR_RL_EN     39

#define MOTOR_RR_IN1    42
#define MOTOR_RR_IN2    45  // Strapping pin — reassign if boot issues occur
#define MOTOR_RR_EN     46  // Strapping pin — reassign if boot issues occur

// ------------------------------------------------------------------------------
// Encoder Pins — 6 motors, 2 channels each (A/B for quadrature)
// ------------------------------------------------------------------------------
#define ENCODER_FL_A    4
#define ENCODER_FL_B    5

#define ENCODER_FR_A    9
#define ENCODER_FR_B    10

#define ENCODER_ML_A    14
#define ENCODER_ML_B    15

#define ENCODER_MR_A    19
#define ENCODER_MR_B    20

#define ENCODER_RL_A    40
#define ENCODER_RL_B    41

#define ENCODER_RR_A    47
#define ENCODER_RR_B    48

// ------------------------------------------------------------------------------
// Ultrasonic Sensor Pins — 3x HC-SR04
// ------------------------------------------------------------------------------
#define SENSOR_LEFT_TRIG    22
#define SENSOR_LEFT_ECHO    23

#define SENSOR_CENTER_TRIG  24
#define SENSOR_CENTER_ECHO  25

#define SENSOR_RIGHT_TRIG   26
#define SENSOR_RIGHT_ECHO   27

// ------------------------------------------------------------------------------
// IMU Pins — MPU6050 (I2C)
// ------------------------------------------------------------------------------
#define IMU_SDA         33
#define IMU_SCL         34
// MPU6050 VCC connects to 3.3V (not 5V — confirm on breakout board)

// ------------------------------------------------------------------------------
// PWM Configuration
// ------------------------------------------------------------------------------
#define PWM_FREQ        1000   // 1 kHz PWM frequency for motor control
#define PWM_RESOLUTION  8      // 8-bit resolution (0-255 duty cycle range)

// ------------------------------------------------------------------------------
// Control Loop Constants
// ------------------------------------------------------------------------------

// Speed mapping — distance (cm) to duty cycle (0-255)
#define SPEED_MIN_DISTANCE      15.0f   // Below this = emergency stop
#define SPEED_MAX_DISTANCE      100.0f  // Above this = max cruising speed
#define SPEED_MIN_DUTY          40      // Minimum PWM to overcome motor stiction
#define SPEED_MAX_DUTY          230     // Max cruise (< 255 to leave steering headroom)

// Steering
#define STEERING_GAIN           0.5f    // How aggressively distance delta affects steering
#define STEERING_MAX_BIAS       50      // Maximum PWM bias added/subtracted per side

// Smoothing
#define SPEED_SMOOTHING_FACTOR  0.2f    // 0.0 = no smoothing, 1.0 = instant (recommend 0.1-0.3)

// Control loop timing
#define CONTROL_LOOP_MS         75      // Target loop period in milliseconds (~13 Hz)

// Sensor timeout
#define SENSOR_TIMEOUT_US       30000   // 30ms timeout for HC-SR04 echo (max ~5m range)

// Serial
#define SERIAL_BAUD             115200  // Serial monitor baud rate

#endif // CONFIG_H
