#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// ==============================================================================
// Motors — L298N Motor Driver Control
// ==============================================================================
// Controls 6 DC motors through 3 L298N dual H-bridge drivers.
// All 3 left-side motors move together, all 3 right-side motors move together.
// Direction is set via IN1/IN2 pins, speed via PWM on EN pins.
// ==============================================================================

class Motors {
public:
    Motors();

    // Initialize all motor pins and PWM channels
    void begin();

    // Set speed for left and right motor groups
    // Speed range: -255 to +255
    //   Positive = forward, negative = reverse, 0 = stop
    void setSpeed(int leftSpeed, int rightSpeed);

    // Emergency stop — cuts power to all motors immediately
    void stop();

private:
    // PWM channel assignments (ESP32 ledc channels 0-5)
    static const uint8_t PWM_CHANNEL_FL = 0;
    static const uint8_t PWM_CHANNEL_FR = 1;
    static const uint8_t PWM_CHANNEL_ML = 2;
    static const uint8_t PWM_CHANNEL_MR = 3;
    static const uint8_t PWM_CHANNEL_RL = 4;
    static const uint8_t PWM_CHANNEL_RR = 5;

    // Helper: set one motor's direction and PWM duty cycle
    void setMotor(uint8_t in1, uint8_t in2, uint8_t pwmChannel, int speed);
};

#endif // MOTORS_H
