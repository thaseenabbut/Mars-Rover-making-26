#include "IMU.h"
#include "config.h"

IMU::IMU() : connected(false) {}

bool IMU::begin() {
    // 1. Initialize custom I2C pins for the ESP32-S3
    Wire.begin(IMU_SDA, IMU_SCL);

    // 2. Try to initialize the MPU6050
    if (!mpu.begin(0x68, &Wire)) {
        // Some breakout boards use 0x69 if AD0 is pulled high
        if (!mpu.begin(0x69, &Wire)) {
            connected = false;
            return false;
        }
    }

    connected = true;

    // 3. Configure default ranges
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    return true;
}

IMUData IMU::read() {
    IMUData data = {0};

    if (!connected) {
        return data;
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    data.accelX = a.acceleration.x;
    data.accelY = a.acceleration.y;
    data.accelZ = a.acceleration.z;

    data.gyroX = g.gyro.x;
    data.gyroY = g.gyro.y;
    data.gyroZ = g.gyro.z;

    data.temp = temp.temperature;

    return data;
}

bool IMU::isConnected() {
    return connected;
}
