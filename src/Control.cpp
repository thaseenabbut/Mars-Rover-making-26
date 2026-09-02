#include "Control.h"
#include "config.h"

Control::Control(Motors& m, Sensors& s, Encoders& e, IMU& i)
    : motors(m), sensors(s), encoders(e), imu(i), currentSpeed(0.0f), lastUpdateTime(0) {
    memset(&lastState, 0, sizeof(lastState));
}

void Control::begin() {
    currentSpeed = 0.0f;
    lastUpdateTime = millis();
}

void Control::update() {
    unsigned long now = millis();
    // Non-blocking rate limiter: only run control logic every CONTROL_LOOP_MS
    if (now - lastUpdateTime < CONTROL_LOOP_MS) {
        return;
    }
    lastUpdateTime = now;

    // --------------------------------------------------------------------------
    // 1. Read Sensors
    // --------------------------------------------------------------------------
    SensorDistances dists = sensors.readAll();

    // --------------------------------------------------------------------------
    // 2. Compute Target Speed from Center Sensor
    // --------------------------------------------------------------------------
    float targetSpeed = computeTargetSpeed(dists.center);

    // --------------------------------------------------------------------------
    // 3. Compute Steering Bias from Left vs Right
    // --------------------------------------------------------------------------
    // Positive bias = steer right (more left speed, less right speed)
    // Negative bias = steer left (less left speed, more right speed)
    float steeringBias = computeSteeringBias(dists.left, dists.right);

    // --------------------------------------------------------------------------
    // 4. Smooth Speed Changes (Exponential Filter)
    // --------------------------------------------------------------------------
    // If target speed is 0 (hard stop / obstacle close), stop immediately for safety!
    if (targetSpeed <= 0.0f) {
        currentSpeed = 0.0f;
    } else {
        currentSpeed += (targetSpeed - currentSpeed) * SPEED_SMOOTHING_FACTOR;
    }

    // --------------------------------------------------------------------------
    // 5. Compute Differential Wheel Speeds
    // --------------------------------------------------------------------------
    int leftPWM = 0;
    int rightPWM = 0;

    if (currentSpeed > 0.0f) {
        leftPWM  = (int)(currentSpeed + steeringBias);
        rightPWM = (int)(currentSpeed - steeringBias);

        // Clamp to valid 8-bit PWM bounds [0, 255]
        if (leftPWM > 255) leftPWM = 255;
        if (leftPWM < 0)   leftPWM = 0;
        if (rightPWM > 255) rightPWM = 255;
        if (rightPWM < 0)   rightPWM = 0;
    }

    // --------------------------------------------------------------------------
    // 6. FUTURE EXTENSION HOOK: Closed-Loop PID Speed Control
    // --------------------------------------------------------------------------
    // In future iterations:
    // long encoderCounts[6];
    // encoders.getCounts(encoderCounts);
    // Calculate current left/right RPM and feed into a PID controller to adjust
    // leftPWM and rightPWM to match desired target RPM precisely on rough terrain.
    // --------------------------------------------------------------------------

    // --------------------------------------------------------------------------
    // 7. FUTURE EXTENSION HOOK: IMU Tilt / Stability Guard
    // --------------------------------------------------------------------------
    // In future iterations:
    // IMUData imuData = imu.read();
    // If pitch or roll exceeds safety angle (e.g. rover is tipping over on a rock),
    // override motors to emergency stop.
    // --------------------------------------------------------------------------

    // --------------------------------------------------------------------------
    // 8. Apply Motor Output
    // --------------------------------------------------------------------------
    motors.setSpeed(leftPWM, rightPWM);

    // Save state for debug logs
    lastState.currentSpeed = currentSpeed;
    lastState.targetSpeed  = targetSpeed;
    lastState.steeringBias = steeringBias;
    lastState.leftPWM      = leftPWM;
    lastState.rightPWM     = rightPWM;
    lastState.distances    = dists;
}

float Control::computeTargetSpeed(float centerDistance) {
    // Hard safety stop
    if (centerDistance <= SPEED_MIN_DISTANCE) {
        return 0.0f;
    }

    // Full speed ahead
    if (centerDistance >= SPEED_MAX_DISTANCE) {
        return (float)SPEED_MAX_DUTY;
    }

    // Linear ramp between min and max distance
    float ratio = (centerDistance - SPEED_MIN_DISTANCE) / (SPEED_MAX_DISTANCE - SPEED_MIN_DISTANCE);
    return SPEED_MIN_DUTY + ratio * (SPEED_MAX_DUTY - SPEED_MIN_DUTY);
}

float Control::computeSteeringBias(float leftDistance, float rightDistance) {
    // Delta = Right - Left
    // If obstacle is on the left -> rightDistance is larger -> positive bias -> steer right
    // If obstacle is on the right -> leftDistance is larger -> negative bias -> steer left
    float delta = rightDistance - leftDistance;
    float bias = delta * STEERING_GAIN;

    // Clamp bias to maximum allowed offset
    if (bias > STEERING_MAX_BIAS)  bias = STEERING_MAX_BIAS;
    if (bias < -STEERING_MAX_BIAS) bias = -STEERING_MAX_BIAS;

    return bias;
}

ControlState Control::getState() const {
    return lastState;
}
