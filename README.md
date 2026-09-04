# Mars Rover — Autonomous Obstacle Avoidance Firmware

Firmware for a 6-wheeled autonomous rover, built for a college Mars rover competition.
The rover uses ultrasonic distance sensing to continuously steer and adjust speed while
navigating — no stop-and-go, no discrete "detect object, turn left/right" behavior.

## Overview

- **Platform:** ESP32-S3
- **Language:** C++ (Arduino framework, via PlatformIO)
- **Drive:** 6x DC gearmotors (12V, 300RPM) across 3x L298N dual H-bridge drivers
- **Sensing:** 3x HC-SR04 ultrasonic distance sensors (left, center, right)
- **IMU:** MPU6050 (accelerometer + gyroscope, I2C) — wired and readable, not yet
  integrated into the control loop
- **Power:** 11.1V 3S LiPo (4000mAh) → LM2596 buck converter → 5V regulated rail

## Control approach

Rather than a fixed "detect obstacle → stop → turn" pattern, the rover:

1. Continuously reads all 3 ultrasonic sensors
2. Maps target speed to the center sensor's distance (closer = slower, clamped between a
   min/max duty cycle, hard stop below a safety threshold)
3. Computes a steering bias from the left/right sensor difference, applied as a
   differential adjustment between the two motor groups (not a hard turn)
4. Smooths speed changes every control loop cycle (exponential smoothing) so motor output
   ramps gradually instead of jumping

Encoders are wired on all 6 motors from the start, but closed-loop PID speed correction is
intentionally deferred until basic open-loop driving is validated.

## Project structure

```
src/
└── main.cpp       # All firmware logic — motors, sensors, encoders, IMU, control loop, boot
```

*Note: all logic is kept in a single `main.cpp` file per course requirements, rather than
split across separate modules.*

## Building

Built with [PlatformIO](https://platformio.org/). Open the project folder in an editor with
the PlatformIO extension (e.g. VS Code) and build/upload as normal, or via CLI:

```
pio run
pio run --target upload
```

## Status

Firmware is written and compiles cleanly. Hardware bring-up (motor drivers, sensors, IMU)
is in progress — see the project's testing checklist for the bring-up order.

## Team

Built by a team of 6 for a college Mars rover competition.
