#include <Arduino.h>
#include "config.h"
#include "Motors.h"
#include "Sensors.h"
#include "Encoders.h"
#include "IMU.h"
#include "Control.h"

// ==============================================================================
// Global Module Instances
// ==============================================================================
Motors   motors;
Sensors  sensors;
Encoders encoders;
IMU      imu;
Control  control(motors, sensors, encoders, imu);

// Telemetry timer (print every 500ms so Serial output stays readable)
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 500;

void setup() {
    // 1. Initialize Serial for diagnostics
    Serial.begin(SERIAL_BAUD);
    delay(1000); // Allow USB-Serial to settle
    Serial.println("\n=================================");
    Serial.println("  MARS ROVER FIRMWARE BOOTING... ");
    Serial.println("=================================");

    // 2. Initialize Motors
    Serial.print("[BOOT] Initializing Motors... ");
    motors.begin();
    Serial.println("OK");

    // 3. Initialize Distance Sensors
    Serial.print("[BOOT] Initializing Ultrasonic Sensors... ");
    sensors.begin();
    Serial.println("OK");

    // 4. Initialize Encoders (standalone ISRs)
    Serial.print("[BOOT] Initializing Encoders... ");
    encoders.begin();
    Serial.println("OK");

    // 5. Initialize IMU (I2C MPU6050)
    Serial.print("[BOOT] Initializing MPU6050 IMU... ");
    if (imu.begin()) {
        Serial.println("OK");
    } else {
        Serial.println("FAILED (check 3.3V/SDA/SCL wiring or I2C address)");
    }

    // 6. Initialize Main Control Loop
    Serial.print("[BOOT] Initializing Controller... ");
    control.begin();
    Serial.println("OK");

    Serial.println("[BOOT] Rover Ready! Starting Control Loop...\n");
}

void loop() {
    // 1. Run main control loop (handles rate limiting internally)
    control.update();

    // 2. Periodic Telemetry over Serial
    unsigned long now = millis();
    if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryTime = now;

        ControlState state = control.getState();

        // Print Sensor Distances
        Serial.printf("[DIST] L: %5.1f cm | C: %5.1f cm | R: %5.1f cm\n",
                      state.distances.left, state.distances.center, state.distances.right);

        // Print Control Status
        Serial.printf("[CTRL] TargetSpd: %5.1f | CurrSpd: %5.1f | Bias: %5.1f\n",
                      state.targetSpeed, state.currentSpeed, state.steeringBias);

        // Print Motor Outputs
        Serial.printf("[MTRS] Left PWM: %3d | Right PWM: %3d\n",
                      state.leftPWM, state.rightPWM);

        // Read standalone IMU data
        if (imu.isConnected()) {
            IMUData imuData = imu.read();
            Serial.printf("[IMU ] Accel: (%4.1f, %4.1f, %4.1f) m/s^2 | Gyro: (%4.2f, %4.2f, %4.2f) rad/s\n",
                          imuData.accelX, imuData.accelY, imuData.accelZ,
                          imuData.gyroX, imuData.gyroY, imuData.gyroZ);
        }

        // Read standalone Encoder counts
        long encCounts[6];
        encoders.getCounts(encCounts);
        Serial.printf("[ENCS] FL:%ld FR:%ld ML:%ld MR:%ld RL:%ld RR:%ld\n",
                      encCounts[0], encCounts[1], encCounts[2],
                      encCounts[3], encCounts[4], encCounts[5]);

        Serial.println("-------------------------------------------------------------------");
    }
}
