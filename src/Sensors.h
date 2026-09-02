#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// ==============================================================================
// Sensors — HC-SR04 Ultrasonic Distance Sensors
// ==============================================================================
// Reads the 3 front-facing distance sensors (Left, Center, Right).
// Uses `pulseIn` for timing the echo. If an object is out of range,
// returns a highly positive max value safely so the rover keeps moving.
// ==============================================================================

struct SensorDistances {
    float left;    // Distance in cm
    float center;  // Distance in cm
    float right;   // Distance in cm
};

class Sensors {
public:
    Sensors();

    // Initialize pin modes for all 3 sensors
    void begin();

    // Read all three sensors and return distance array/struct
    // Returns Max distance if the read times out or fails
    SensorDistances readAll();

private:
    // Helper: read a single sensor and return distance in cm
    float readSensor(uint8_t trigPin, uint8_t echoPin);
};

#endif // SENSORS_H
