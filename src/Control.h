#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "Motors.h"
#include "Sensors.h"
#include "Encoders.h"
#include "IMU.h"

// ==============================================================================
// Control — Obstacle Avoidance & Steering Controller
// ==============================================================================
// Implements continuous, smooth obstacle avoidance:
// 1. Reads 3 distance sensors
// 2. Maps center distance to target speed (linear ramp with min/max cutoffs)
// 3. Calculates steering bias from left-right difference
// 4. Smooths current speed toward target speed via exponential filter
// 5. Outputs differential drive PWM to left and right motor banks
// ==============================================================================

struct ControlState {
    float currentSpeed;
    float targetSpeed;
    float steeringBias;
    int leftPWM;
    int rightPWM;
    SensorDistances distances;
};

class Control {
public:
    Control(Motors& motors, Sensors& sensors, Encoders& encoders, IMU& imu);

    // Call once during setup()
    void begin();

    // Call every loop cycle — automatically respects CONTROL_LOOP_MS
    void update();

    // Read internal state for debugging/telemetry
    ControlState getState() const;

private:
    Motors& motors;
    Sensors& sensors;
    Encoders& encoders;
    IMU& imu;

    float currentSpeed;
    unsigned long lastUpdateTime;
    ControlState lastState;

    // Helper: compute target speed from center distance
    float computeTargetSpeed(float centerDistance);

    // Helper: compute steering bias from left and right distances
    float computeSteeringBias(float leftDistance, float rightDistance);
};

#endif // CONTROL_H
