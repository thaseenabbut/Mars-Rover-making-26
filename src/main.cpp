#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ==============================================================================
// Mars Rover — Single-File Firmware
// ==============================================================================
// All modules (config, Motors, Sensors, Encoders, IMU, Control) merged into one
// file. Logic, pin definitions, comments, and constants are identical to the
// multi-file version — only the file boundaries are removed.
// ==============================================================================


// ==============================================================================
// SECTION: Configuration
// ==============================================================================
// Single source of truth for all pin numbers and tunable constants.
// Update pins here after physical bring-up confirms actual wiring.
//
// Note: GPIO 45/46 are strapping pins on ESP32-S3. If Rear-Right motor behavior
// is unstable during boot, reassign those pins to non-strapping alternatives.

// ------------------------------------------------------------------------------
// Motor Pins — 3x L298N (2 motors each, 6 motors total)
// ------------------------------------------------------------------------------
// Each motor: 2 direction pins (IN1/IN2) + 1 PWM enable pin (EN)

// L298N #1: Front-Left & Front-Right
#define MOTOR_FL_IN1    1
#define MOTOR_FL_IN2    2
#define MOTOR_FL_EN     3

#define MOTOR_FR_IN1    6
#define MOTOR_FR_IN2    7
#define MOTOR_FR_EN     8

// L298N #2: Mid-Left & Mid-Right
#define MOTOR_ML_IN1    11
#define MOTOR_ML_IN2    12
#define MOTOR_ML_EN     13

#define MOTOR_MR_IN1    16
#define MOTOR_MR_IN2    17
#define MOTOR_MR_EN     18

// L298N #3: Rear-Left & Rear-Right
#define MOTOR_RL_IN1    21
#define MOTOR_RL_IN2    38
#define MOTOR_RL_EN     39

#define MOTOR_RR_IN1    42
#define MOTOR_RR_IN2    45  // Strapping pin — reassign if boot issues occur
#define MOTOR_RR_EN     46  // Strapping pin — reassign if boot issues occur

// ------------------------------------------------------------------------------
// Encoder Pins — 6 motors, 2 channels each (A/B for quadrature)
// ------------------------------------------------------------------------------
#define ENCODER_FL_A    4
#define ENCODER_FL_B    5

#define ENCODER_FR_A    9
#define ENCODER_FR_B    10

#define ENCODER_ML_A    14
#define ENCODER_ML_B    15

#define ENCODER_MR_A    19
#define ENCODER_MR_B    20

#define ENCODER_RL_A    40
#define ENCODER_RL_B    41

#define ENCODER_RR_A    47
#define ENCODER_RR_B    48

// ------------------------------------------------------------------------------
// Ultrasonic Sensor Pins — 3x HC-SR04
// ------------------------------------------------------------------------------
#define SENSOR_LEFT_TRIG    22
#define SENSOR_LEFT_ECHO    23

#define SENSOR_CENTER_TRIG  24
#define SENSOR_CENTER_ECHO  25

#define SENSOR_RIGHT_TRIG   26
#define SENSOR_RIGHT_ECHO   27

// ------------------------------------------------------------------------------
// IMU Pins — MPU6050 (I2C)
// ------------------------------------------------------------------------------
#define IMU_SDA         33
#define IMU_SCL         34
// MPU6050 VCC connects to 3.3V (not 5V — confirm on breakout board)

// ------------------------------------------------------------------------------
// PWM Configuration
// ------------------------------------------------------------------------------
#define PWM_FREQ        1000   // 1 kHz PWM frequency for motor control
#define PWM_RESOLUTION  8      // 8-bit resolution (0-255 duty cycle range)

// ------------------------------------------------------------------------------
// Control Loop Constants
// ------------------------------------------------------------------------------

// Speed mapping — distance (cm) to duty cycle (0-255)
#define SPEED_MIN_DISTANCE      15.0f   // Below this = emergency stop
#define SPEED_MAX_DISTANCE      100.0f  // Above this = max cruising speed
#define SPEED_MIN_DUTY          40      // Minimum PWM to overcome motor stiction
#define SPEED_MAX_DUTY          230     // Max cruise (< 255 to leave steering headroom)

// Steering
#define STEERING_GAIN           0.5f    // How aggressively distance delta affects steering
#define STEERING_MAX_BIAS       50      // Maximum PWM bias added/subtracted per side

// Smoothing
#define SPEED_SMOOTHING_FACTOR  0.2f    // 0.0 = no smoothing, 1.0 = instant (recommend 0.1-0.3)

// Control loop timing
#define CONTROL_LOOP_MS         75      // Target loop period in milliseconds (~13 Hz)

// Sensor timeout
#define SENSOR_TIMEOUT_US       30000   // 30ms timeout for HC-SR04 echo (max ~5m range)

// Serial
#define SERIAL_BAUD             115200  // Serial monitor baud rate


// ==============================================================================
// SECTION: Structs (shared between modules)
// ==============================================================================

struct SensorDistances {
    float left;    // Distance in cm
    float center;  // Distance in cm
    float right;   // Distance in cm
};

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

struct ControlState {
    float currentSpeed;
    float targetSpeed;
    float steeringBias;
    int leftPWM;
    int rightPWM;
    SensorDistances distances;
};


// ==============================================================================
// SECTION: Motors — L298N Motor Driver Control
// ==============================================================================
// Controls 6 DC motors through 3 L298N dual H-bridge drivers.
// All 3 left-side motors move together, all 3 right-side motors move together.
// Direction is set via IN1/IN2 pins, speed via PWM on EN pins.

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

Motors::Motors() {}

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


// ==============================================================================
// SECTION: Sensors — HC-SR04 Ultrasonic Distance Sensors
// ==============================================================================
// Reads the 3 front-facing distance sensors (Left, Center, Right).
// Uses `pulseIn` for timing the echo. If an object is out of range,
// returns a highly positive max value safely so the rover keeps moving.

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


// ==============================================================================
// SECTION: Encoders — Motor Wheel Encoders (Quadrature)
// ==============================================================================
// Set up ISRs to count pulses per motor.
// Closed-loop PID control is deferred per spec; these are just for logging/
// calibration/potential future use.

class Encoders {
public:
    Encoders();

    // Initialize all 6 encoder channels (12 pins total)
    void begin();

    // Get current pulse counts for all 6 motors
    void getCounts(long* counts);

    // Reset all pulse counts to 0
    void reset();

    // ISR functions must be static and global-ish; these handle the count updates
    static void handleFL();
    static void handleFR();
    static void handleML();
    static void handleMR();
    static void handleRL();
    static void handleRR();

private:
    // Atomic or volatile counters needed for ISR updates
    static volatile long countFL, countFR, countML, countMR, countRL, countRR;
};

// Define static counters
volatile long Encoders::countFL = 0;
volatile long Encoders::countFR = 0;
volatile long Encoders::countML = 0;
volatile long Encoders::countMR = 0;
volatile long Encoders::countRL = 0;
volatile long Encoders::countRR = 0;

Encoders::Encoders() {}

void Encoders::begin() {
    // Configure all Encoder A and B pins as inputs with pullups
    // A channel triggers the interrupt, B channel is read to determine direction

    // Front Left
    pinMode(ENCODER_FL_A, INPUT_PULLUP);
    pinMode(ENCODER_FL_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_FL_A), handleFL, RISING);

    // Front Right
    pinMode(ENCODER_FR_A, INPUT_PULLUP);
    pinMode(ENCODER_FR_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_FR_A), handleFR, RISING);

    // Mid Left
    pinMode(ENCODER_ML_A, INPUT_PULLUP);
    pinMode(ENCODER_ML_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_ML_A), handleML, RISING);

    // Mid Right
    pinMode(ENCODER_MR_A, INPUT_PULLUP);
    pinMode(ENCODER_MR_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_MR_A), handleMR, RISING);

    // Rear Left
    pinMode(ENCODER_RL_A, INPUT_PULLUP);
    pinMode(ENCODER_RL_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RL_A), handleRL, RISING);

    // Rear Right
    pinMode(ENCODER_RR_A, INPUT_PULLUP);
    pinMode(ENCODER_RR_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RR_A), handleRR, RISING);
}

void Encoders::getCounts(long* counts) {
    // Briefly disable interrupts to cleanly copy volatile variables
    noInterrupts();
    counts[0] = countFL;
    counts[1] = countFR;
    counts[2] = countML;
    counts[3] = countMR;
    counts[4] = countRL;
    counts[5] = countRR;
    interrupts();
}

void Encoders::reset() {
    noInterrupts();
    countFL = 0;
    countFR = 0;
    countML = 0;
    countMR = 0;
    countRL = 0;
    countRR = 0;
    interrupts();
}

// ------------------------------------------------------------------------------
// Interrupt Service Routines (ISRs)
// Keep these extremely light! No serial prints, minimal logic.
// Read channel B on the A rising edge: if B is HIGH, rotating one way, if LOW, the other.
// ------------------------------------------------------------------------------

void IRAM_ATTR Encoders::handleFL() {
    (digitalRead(ENCODER_FL_B) == HIGH) ? countFL++ : countFL--;
}

void IRAM_ATTR Encoders::handleFR() {
    // Right side may need inverted direction depending on wiring
    (digitalRead(ENCODER_FR_B) == HIGH) ? countFR++ : countFR--;
}

void IRAM_ATTR Encoders::handleML() {
    (digitalRead(ENCODER_ML_B) == HIGH) ? countML++ : countML--;
}

void IRAM_ATTR Encoders::handleMR() {
    (digitalRead(ENCODER_MR_B) == HIGH) ? countMR++ : countMR--;
}

void IRAM_ATTR Encoders::handleRL() {
    (digitalRead(ENCODER_RL_B) == HIGH) ? countRL++ : countRL--;
}

void IRAM_ATTR Encoders::handleRR() {
    (digitalRead(ENCODER_RR_B) == HIGH) ? countRR++ : countRR--;
}


// ==============================================================================
// SECTION: IMU — MPU6050 6-DOF Gyroscope & Accelerometer
// ==============================================================================
// Reads the accelerometer and gyroscope via I2C.
// Standalone module for now — data is read and exposed, but not yet fed back
// into the control loop per spec.

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


// ==============================================================================
// SECTION: Control — Obstacle Avoidance & Steering Controller
// ==============================================================================
// Implements continuous, smooth obstacle avoidance:
// 1. Reads 3 distance sensors
// 2. Maps center distance to target speed (linear ramp with min/max cutoffs)
// 3. Calculates steering bias from left-right difference
// 4. Smooths current speed toward target speed via exponential filter
// 5. Outputs differential drive PWM to left and right motor banks

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


// ==============================================================================
// SECTION: Global Module Instances
// ==============================================================================
Motors   motors;
Sensors  sensors;
Encoders encoders;
IMU      imu;
Control  control(motors, sensors, encoders, imu);

// Telemetry timer (print every 500ms so Serial output stays readable)
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 500;


// ==============================================================================
// SECTION: Entry Point
// ==============================================================================

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
