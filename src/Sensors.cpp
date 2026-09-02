#include "Sensors.h"
#include "config.h"

Sensors::Sensors() {}

void Sensors::begin() {
    // Left
    pinMode(SENSOR_LEFT_TRIG, OUTPUT);
    pinMode(SENSOR_LEFT_ECHO, INPUT);
    // Center
    pinMode(SENSOR_CENTER_TRIG, OUTPUT);
    pinMode(SENSOR_CENTER_ECHO, INPUT);
    // Right
    pinMode(SENSOR_RIGHT_TRIG, OUTPUT);
    pinMode(SENSOR_RIGHT_ECHO, INPUT);
}

SensorDistances Sensors::readAll() {
    SensorDistances dists;
    dists.left   = readSensor(SENSOR_LEFT_TRIG, SENSOR_LEFT_ECHO);
    dists.center = readSensor(SENSOR_CENTER_TRIG, SENSOR_CENTER_ECHO);
    dists.right  = readSensor(SENSOR_RIGHT_TRIG, SENSOR_RIGHT_ECHO);
    return dists;
}

float Sensors::readSensor(uint8_t trigPin, uint8_t echoPin) {
    // 1. Trigger Pulse: Minimum 10us high
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 2. Measure Echo: Duration of pulse high
    // Use timeout to prevent hanging the control loop
    long durationUs = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT_US);

    // 3. Convert to cm: (durationUs / 2) / 29.1
    if (durationUs == 0) {
        // Timeout occurred, assume out of range (far)
        return SPEED_MAX_DISTANCE + 50.0f;
    }

    float distanceCm = (float)durationUs * 0.01716f; // speed of sound ~343m/s = 0.0343cm/us
    return distanceCm;
}
