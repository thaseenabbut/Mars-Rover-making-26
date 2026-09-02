#include "Motors.h"
#include "config.h"

Motors::Motors() {
    // Constructor
}

void Motors::begin() {
    // 1. Configure direction pins (IN1/IN2) as OUTPUT
    pinMode(MOTOR_FL_IN1, OUTPUT);
    pinMode(MOTOR_FL_IN2, OUTPUT);
    pinMode(MOTOR_FR_IN1, OUTPUT);
    pinMode(MOTOR_FR_IN2, OUTPUT);

    pinMode(MOTOR_ML_IN1, OUTPUT);
    pinMode(MOTOR_ML_IN2, OUTPUT);
    pinMode(MOTOR_MR_IN1, OUTPUT);
    pinMode(MOTOR_MR_IN2, OUTPUT);

    pinMode(MOTOR_RL_IN1, OUTPUT);
    pinMode(MOTOR_RL_IN2, OUTPUT);
    pinMode(MOTOR_RR_IN1, OUTPUT);
    pinMode(MOTOR_RR_IN2, OUTPUT);

    // Initial state: stop (both IN pins low)
    stop();

    // 2. Configure PWM channels (ledc setup for ESP32)
    ledcSetup(PWM_CHANNEL_FL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_FR, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_ML, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_MR, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_RL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_RR, PWM_FREQ, PWM_RESOLUTION);

    // 3. Attach PWM channels to enable pins (EN)
    ledcAttachPin(MOTOR_FL_EN, PWM_CHANNEL_FL);
    ledcAttachPin(MOTOR_FR_EN, PWM_CHANNEL_FR);
    ledcAttachPin(MOTOR_ML_EN, PWM_CHANNEL_ML);
    ledcAttachPin(MOTOR_MR_EN, PWM_CHANNEL_MR);
    ledcAttachPin(MOTOR_RL_EN, PWM_CHANNEL_RL);
    ledcAttachPin(MOTOR_RR_EN, PWM_CHANNEL_RR);

    // Make sure we start at 0 duty
    ledcWrite(PWM_CHANNEL_FL, 0);
    ledcWrite(PWM_CHANNEL_FR, 0);
    ledcWrite(PWM_CHANNEL_ML, 0);
    ledcWrite(PWM_CHANNEL_MR, 0);
    ledcWrite(PWM_CHANNEL_RL, 0);
    ledcWrite(PWM_CHANNEL_RR, 0);
}

void Motors::setSpeed(int leftSpeed, int rightSpeed) {
    // Apply speed to all 3 left motors simultaneously
    setMotor(MOTOR_FL_IN1, MOTOR_FL_IN2, PWM_CHANNEL_FL, leftSpeed);
    setMotor(MOTOR_ML_IN1, MOTOR_ML_IN2, PWM_CHANNEL_ML, leftSpeed);
    setMotor(MOTOR_RL_IN1, MOTOR_RL_IN2, PWM_CHANNEL_RL, leftSpeed);

    // Apply speed to all 3 right motors simultaneously
    setMotor(MOTOR_FR_IN1, MOTOR_FR_IN2, PWM_CHANNEL_FR, rightSpeed);
    setMotor(MOTOR_MR_IN1, MOTOR_MR_IN2, PWM_CHANNEL_MR, rightSpeed);
    setMotor(MOTOR_RR_IN1, MOTOR_RR_IN2, PWM_CHANNEL_RR, rightSpeed);
}

void Motors::setMotor(uint8_t in1, uint8_t in2, uint8_t pwmChannel, int speed) {
    // Clamp speed into range -255 to +255
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        // Forward: IN1 High, IN2 Low
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        ledcWrite(pwmChannel, speed);
    } else if (speed < 0) {
        // Reverse: IN1 Low, IN2 High
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        ledcWrite(pwmChannel, -speed); // PWM takes positive duty cycle
    } else {
        // Stop: Coast/Brake (both low)
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        ledcWrite(pwmChannel, 0);
    }
}

void Motors::stop() {
    // Left side off
    digitalWrite(MOTOR_FL_IN1, LOW);
    digitalWrite(MOTOR_FL_IN2, LOW);
    digitalWrite(MOTOR_ML_IN1, LOW);
    digitalWrite(MOTOR_ML_IN2, LOW);
    digitalWrite(MOTOR_RL_IN1, LOW);
    digitalWrite(MOTOR_RL_IN2, LOW);

    // Right side off
    digitalWrite(MOTOR_FR_IN1, LOW);
    digitalWrite(MOTOR_FR_IN2, LOW);
    digitalWrite(MOTOR_MR_IN1, LOW);
    digitalWrite(MOTOR_MR_IN2, LOW);
    digitalWrite(MOTOR_RR_IN1, LOW);
    digitalWrite(MOTOR_RR_IN2, LOW);

    // PWM to 0
    ledcWrite(PWM_CHANNEL_FL, 0);
    ledcWrite(PWM_CHANNEL_FR, 0);
    ledcWrite(PWM_CHANNEL_ML, 0);
    ledcWrite(PWM_CHANNEL_MR, 0);
    ledcWrite(PWM_CHANNEL_RL, 0);
    ledcWrite(PWM_CHANNEL_RR, 0);
}
