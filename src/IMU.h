#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ==============================================================================
// IMU — MPU6050 6-DOF Gyroscope & Accelerometer
// ==============================================================================
// Reads the accelerometer and gyroscope via I2C.
// Standalone module for now — data is read and exposed, but not yet fed back
// into the control loop per spec.
// ==============================================================================

struct IMUData {
    // Accelerometer (m/s^2)
    float accelX;
    float accelY;
    float accelZ;
    // Gyroscope (rad/s)
    float gyroX;
    float gyroY;
    float gyroZ;
    // Temperature (deg C)
    float temp;
};

class IMU {
public:
    IMU();

    // Initialize I2C bus and the MPU6050 sensor
    bool begin();

    // Read current sensor data
    IMUData read();

    // Check if the IMU is currently initialized/connected
    bool isConnected();

private:
    Adafruit_MPU6050 mpu;
    bool connected;
};

#endif // IMU_H
