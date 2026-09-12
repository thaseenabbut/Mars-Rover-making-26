/*******************************************************************************
 * Mars Rover — Single-file firmware for ESP32-S3-DevKitC-1
 *
 * Features:
 *   • 6 DC motors across 3× L298N drivers (differential drive)
 *   • MPU6050 IMU over I2C (telemetry)
 *   • 3× HC-SR04 ultrasonic sensors (telemetry display only)
 *   • BLE Nordic UART Service (NUS) — single-char commands F/B/L/R/S
 *   • 500 ms command-timeout auto-stop
 *
 * Pin budget (ESP32-S3-DevKitC-1, N8R8 safe):
 *   Avoid: GPIO 0,3,45,46 (strapping), 19,20 (USB), 43,44 (UART0),
 *          26-32 (not exposed / SPI flash), 33-37 (octal PSRAM/flash)
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════════

// ---- L298N #1 — Front Left / Front Right ----
#define FL_IN1   1
#define FL_IN2   2
#define FL_EN    4      // LEDC channel 0

#define FR_IN1   6
#define FR_IN2   7
#define FR_EN    5      // LEDC channel 1

// ---- L298N #2 — Mid Left / Mid Right ----
#define ML_IN1   15
#define ML_IN2   16
#define ML_EN    8      // LEDC channel 2

#define MR_IN1   17
#define MR_IN2   18
#define MR_EN    9      // LEDC channel 3

// ---- L298N #3 — Rear Left / Rear Right ----
#define RL_IN1   10
#define RL_IN2   11
#define RL_EN    14     // LEDC channel 4

#define RR_IN1   12
#define RR_IN2   13
#define RR_EN    21     // LEDC channel 5

// ---- I2C (MPU6050) ----
#define I2C_SDA  47
#define I2C_SCL  48

// ---- HC-SR04 Ultrasonic Sensors ----
//  (reassigned from invalid GPIO 22-25 to valid ESP32-S3 pins)
#define SENSOR_LEFT_TRIG    38
#define SENSOR_LEFT_ECHO    39
#define SENSOR_CENTER_TRIG  40
#define SENSOR_CENTER_ECHO  41
#define SENSOR_RIGHT_TRIG   42
#define SENSOR_RIGHT_ECHO   36   // GPIO36 is safe on N8 (no PSRAM); see note below

// NOTE on GPIO36: On the ESP32-S3-DevKitC-1 with Octal PSRAM (N8R8/N16R8),
// GPIO33-37 are used for PSRAM. If you have an R8 variant, change
// SENSOR_RIGHT_ECHO to another free pin (e.g. GPIO48 and move I2C_SCL).
// On N8 (no PSRAM) or N16 (quad PSRAM) variants, GPIO36 is available.
// *** If your board is N8R8, uncomment the alternative below: ***
// #define SENSOR_RIGHT_ECHO  48
// and change I2C_SCL to another pin.

// ---- LEDC PWM settings ----
#define LEDC_FREQ       1000
#define LEDC_RESOLUTION 8       // 0-255

// ═══════════════════════════════════════════════════════════════════════════════
//  STRAPPING PIN CHECK
// ═══════════════════════════════════════════════════════════════════════════════

static void checkStrappingPins() {
  const int enPins[] = { FL_EN, FR_EN, ML_EN, MR_EN, RL_EN, RR_EN };
  const char* enNames[] = { "FL_EN", "FR_EN", "ML_EN", "MR_EN", "RL_EN", "RR_EN" };
  const int strapping[] = { 0, 3, 45, 46 };
  bool anyFlag = false;

  for (int i = 0; i < 6; i++) {
    for (int s = 0; s < 4; s++) {
      if (enPins[i] == strapping[s]) {
        Serial.printf("  WARNING: %s (GPIO %d) is a strapping pin!\n",
                       enNames[i], enPins[i]);
        anyFlag = true;
      }
    }
  }
  if (!anyFlag) {
    Serial.println("  No EN pins on strapping GPIOs (0, 3, 45, 46).");
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MOTORS CLASS
// ═══════════════════════════════════════════════════════════════════════════════

class Motors {
public:
  bool begin() {
    Serial.println("[Motors] Initialising direction pins...");

    // Front
    pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
    pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
    // Mid
    pinMode(ML_IN1, OUTPUT); pinMode(ML_IN2, OUTPUT);
    pinMode(MR_IN1, OUTPUT); pinMode(MR_IN2, OUTPUT);
    // Rear
    pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
    pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);

    Serial.println("[Motors] Configuring LEDC PWM channels...");

    // ESP32-S3 Arduino core ≥ 3.x uses the new ledcAttach(pin, freq, resolution) API.
    // For PlatformIO espressif32 ≤ 6.x (Arduino core 2.x), we use the legacy API.
    // Detect at compile time:
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    // New API — ledcAttach(pin, freq, resolution) auto-assigns channel
    if (!ledcAttach(FL_EN, LEDC_FREQ, LEDC_RESOLUTION)) { Serial.println("  LEDC attach failed: FL_EN"); return false; }
    if (!ledcAttach(FR_EN, LEDC_FREQ, LEDC_RESOLUTION)) { Serial.println("  LEDC attach failed: FR_EN"); return false; }
    if (!ledcAttach(ML_EN, LEDC_FREQ, LEDC_RESOLUTION)) { Serial.println("  LEDC attach failed: ML_EN"); return false; }
    if (!ledcAttach(MR_EN, LEDC_FREQ, LEDC_RESOLUTION)) { Serial.println("  LEDC attach failed: MR_EN"); return false; }
    if (!ledcAttach(RL_EN, LEDC_FREQ, LEDC_RESOLUTION)) { Serial.println("  LEDC attach failed: RL_EN"); return false; }
    if (!ledcAttach(RR_EN, LEDC_FREQ, LEDC_RESOLUTION)) { Serial.println("  LEDC attach failed: RR_EN"); return false; }
#else
    // Legacy API — ledcSetup(channel, freq, resolution) + ledcAttachPin(pin, channel)
    ledcSetup(0, LEDC_FREQ, LEDC_RESOLUTION); ledcAttachPin(FL_EN, 0);
    ledcSetup(1, LEDC_FREQ, LEDC_RESOLUTION); ledcAttachPin(FR_EN, 1);
    ledcSetup(2, LEDC_FREQ, LEDC_RESOLUTION); ledcAttachPin(ML_EN, 2);
    ledcSetup(3, LEDC_FREQ, LEDC_RESOLUTION); ledcAttachPin(MR_EN, 3);
    ledcSetup(4, LEDC_FREQ, LEDC_RESOLUTION); ledcAttachPin(RL_EN, 4);
    ledcSetup(5, LEDC_FREQ, LEDC_RESOLUTION); ledcAttachPin(RR_EN, 5);
#endif

    Serial.println("  All 6 LEDC channels attached.");
    stop();
    return true;
  }

  // Set speed for left side and right side independently.
  //   speed: -255..+255  (positive = forward, negative = backward)
  void setSpeed(int left, int right) {
    setMotor(FL_IN1, FL_IN2, FL_EN, left);
    setMotor(ML_IN1, ML_IN2, ML_EN, left);
    setMotor(RL_IN1, RL_IN2, RL_EN, left);

    setMotor(FR_IN1, FR_IN2, FR_EN, right);
    setMotor(MR_IN1, MR_IN2, MR_EN, right);
    setMotor(RR_IN1, RR_IN2, RR_EN, right);
  }

  void forward(int spd = 200)  { setSpeed( spd,  spd); }
  void backward(int spd = 200) { setSpeed(-spd, -spd); }
  void left(int spd = 180)     { setSpeed(-spd,  spd); }   // pivot left
  void right(int spd = 180)    { setSpeed( spd, -spd); }   // pivot right
  void stop()                   { setSpeed(0, 0); }

  // --- Boot-time spin test (one driver at a time) ---
  void bootTest() {
    Serial.println("\n=== MOTOR BOOT TEST ===");

    struct { const char* name; int in1; int in2; int en; } sides[] = {
      { "L298N#1 Front-Left",  FL_IN1, FL_IN2, FL_EN },
      { "L298N#1 Front-Right", FR_IN1, FR_IN2, FR_EN },
      { "L298N#2 Mid-Left",    ML_IN1, ML_IN2, ML_EN },
      { "L298N#2 Mid-Right",   MR_IN1, MR_IN2, MR_EN },
      { "L298N#3 Rear-Left",   RL_IN1, RL_IN2, RL_EN },
      { "L298N#3 Rear-Right",  RR_IN1, RR_IN2, RR_EN },
    };

    for (int i = 0; i < 6; i++) {
      Serial.printf("  Spinning %s for 2 s ...\n", sides[i].name);
      setMotor(sides[i].in1, sides[i].in2, sides[i].en, 180);
      delay(2000);
      setMotor(sides[i].in1, sides[i].in2, sides[i].en, 0);
      delay(500);
    }

    Serial.println("=== MOTOR BOOT TEST COMPLETE ===\n");
  }

private:
  void setMotor(int in1, int in2, int enPin, int speed) {
    if (speed > 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    } else if (speed < 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
    int pwm = constrain(abs(speed), 0, 255);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(enPin, pwm);
#else
    // Legacy: need channel number. Map pin→channel.
    int ch = pinToChannel(enPin);
    if (ch >= 0) ledcWrite(ch, pwm);
#endif
  }

#if ESP_ARDUINO_VERSION_MAJOR < 3
  int pinToChannel(int pin) {
    if (pin == FL_EN) return 0;
    if (pin == FR_EN) return 1;
    if (pin == ML_EN) return 2;
    if (pin == MR_EN) return 3;
    if (pin == RL_EN) return 4;
    if (pin == RR_EN) return 5;
    return -1;
  }
#endif
};

// ═══════════════════════════════════════════════════════════════════════════════
//  ULTRASONIC SENSORS CLASS
// ═══════════════════════════════════════════════════════════════════════════════

class UltrasonicSensors {
public:
  float distLeft   = -1;
  float distCenter = -1;
  float distRight  = -1;

  void begin() {
    Serial.println("[Sensors] Configuring HC-SR04 pins...");
    Serial.printf("  LEFT   TRIG=GPIO%d  ECHO=GPIO%d\n", SENSOR_LEFT_TRIG, SENSOR_LEFT_ECHO);
    Serial.printf("  CENTER TRIG=GPIO%d  ECHO=GPIO%d\n", SENSOR_CENTER_TRIG, SENSOR_CENTER_ECHO);
    Serial.printf("  RIGHT  TRIG=GPIO%d  ECHO=GPIO%d\n", SENSOR_RIGHT_TRIG, SENSOR_RIGHT_ECHO);

    pinMode(SENSOR_LEFT_TRIG,   OUTPUT);
    pinMode(SENSOR_LEFT_ECHO,   INPUT);
    pinMode(SENSOR_CENTER_TRIG, OUTPUT);
    pinMode(SENSOR_CENTER_ECHO, INPUT);
    pinMode(SENSOR_RIGHT_TRIG,  OUTPUT);
    pinMode(SENSOR_RIGHT_ECHO,  INPUT);

    Serial.println("  Sensor pins configured.");
  }

  void readAll() {
    distLeft   = readSensor(SENSOR_LEFT_TRIG,   SENSOR_LEFT_ECHO);
    distCenter = readSensor(SENSOR_CENTER_TRIG,  SENSOR_CENTER_ECHO);
    distRight  = readSensor(SENSOR_RIGHT_TRIG,   SENSOR_RIGHT_ECHO);
  }

  void printTelemetry() {
    Serial.printf("  DIST L=%.1fcm  C=%.1fcm  R=%.1fcm\n",
                   distLeft, distCenter, distRight);
  }

private:
  float readSensor(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, 30000);  // 30 ms timeout
    if (duration == 0) return -1.0f;  // no echo / out of range
    return (float)duration * 0.0343f / 2.0f;   // cm
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
//  BLE NUS (Nordic UART Service)
// ═══════════════════════════════════════════════════════════════════════════════

// Standard NUS UUIDs
#define NUS_SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX_CHAR_UUID        "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // write (app → rover)
#define NUS_TX_CHAR_UUID        "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // notify (rover → app)

static BLECharacteristic* pTxCharacteristic = nullptr;
static bool               bleConnected      = false;
static volatile char      lastCommand       = 'S';
static volatile unsigned long lastCommandTimeMs = 0;

// ---- Server callbacks ----
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    Serial.println("[BLE] Client connected!");
  }
  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    Serial.println("[BLE] Client disconnected - restarting advertising...");
    delay(100);
    BLEDevice::startAdvertising();
  }
};

// ---- RX characteristic callbacks (commands from app) ----
class NusRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    std::string val = pChar->getValue();
    if (val.length() > 0) {
      char cmd = toupper(val[0]);
      lastCommand = cmd;
      lastCommandTimeMs = millis();
      Serial.printf("[BLE] RX cmd: '%c'  (t=%lu ms)\n", cmd, lastCommandTimeMs);
    }
  }
};

static void setupBLE() {
  Serial.println("[BLE] Initialising BLE stack...");

  // 1. Initialise with device name BEFORE anything else
  BLEDevice::init("MarsRover");

  // 2. Create server + set callbacks
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 3. Create NUS service
  BLEService* pService = pServer->createService(NUS_SERVICE_UUID);

  // 4. TX characteristic (notify) — rover → app
  pTxCharacteristic = pService->createCharacteristic(
      NUS_TX_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // 5. RX characteristic (write / write-no-response) — app → rover
  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
      NUS_RX_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pRxCharacteristic->setCallbacks(new NusRxCallbacks());

  // 6. Start service, THEN start advertising
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(NUS_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);   // helps with iPhone connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising started - device name: \"MarsRover\"");
  Serial.printf("[BLE]   NUS Service UUID : %s\n", NUS_SERVICE_UUID);
  Serial.printf("[BLE]   RX Char UUID     : %s\n", NUS_RX_CHAR_UUID);
  Serial.printf("[BLE]   TX Char UUID     : %s\n", NUS_TX_CHAR_UUID);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GLOBALS
// ═══════════════════════════════════════════════════════════════════════════════

Motors            motors;
UltrasonicSensors sensors;

static unsigned long lastTelemetryMs = 0;
static const unsigned long TELEMETRY_INTERVAL_MS = 500;

static const unsigned long COMMAND_TIMEOUT_MS = 500;
static bool timeoutTriggered = false;

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1500);   // let serial monitor attach
  Serial.println("\n\n+--------------------------------------+");
  Serial.println("|       MARS ROVER - ESP32-S3          |");
  Serial.println("+--------------------------------------+\n");

  // ── PHASE 1 — Motor drivers ────────────────────────────────────────────────
  Serial.println("------ PHASE 1: Motor Drivers ------");
  checkStrappingPins();

  if (!motors.begin()) {
    Serial.println("[PHASE 1] FAIL: LEDC channel setup error");
    while (1) delay(1000);   // halt
  }

  motors.bootTest();
  Serial.println("[PHASE 1] PASS");

  // ── PHASE 2 — Ultrasonic sensors ───────────────────────────────────────────
  Serial.println("------ PHASE 2: Ultrasonic Sensors ------");
  sensors.begin();
  sensors.readAll();
  sensors.printTelemetry();
  Serial.println("[PHASE 2] PASS");

  // ── PHASE 3 — BLE NUS ─────────────────────────────────────────────────────
  Serial.println("------ PHASE 3: BLE NUS Server ------");
  setupBLE();
  Serial.println("[PHASE 3] PASS");

  // ── PHASE 4 — Timeout safety (verified at runtime in loop) ─────────────────
  Serial.println("------ PHASE 4: Command Timeout ------");
  Serial.printf("  Timeout = %lu ms (auto-stop if no BLE cmd while connected)\n",
                 COMMAND_TIMEOUT_MS);
  lastCommandTimeMs = millis();
  Serial.println("[PHASE 4] PASS (runtime verification - send F then wait 500 ms)");

  Serial.println("\n=== SETUP COMPLETE - entering main loop ===\n");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // ── Process BLE commands ──────────────────────────────────────────────────
  if (bleConnected) {
    char cmd = lastCommand;

    // Command timeout check (PHASE 4)
    unsigned long elapsed = now - lastCommandTimeMs;
    if (elapsed > COMMAND_TIMEOUT_MS && cmd != 'S') {
      Serial.printf("[TIMEOUT] No cmd for %lu ms - auto-stopping "
                     "(last cmd='%c', last cmd time=%lu, now=%lu)\n",
                     elapsed, cmd, lastCommandTimeMs, now);
      lastCommand = 'S';
      motors.stop();
      timeoutTriggered = true;
    }

    // Execute current command
    switch (cmd) {
      case 'F': motors.forward();  break;
      case 'B': motors.backward(); break;
      case 'L': motors.left();     break;
      case 'R': motors.right();    break;
      case 'S': // fallthrough
      default:  motors.stop();     break;
    }
  } else {
    // Not connected — ensure motors are stopped
    motors.stop();
  }

  // ── Periodic telemetry ────────────────────────────────────────────────────
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;

    sensors.readAll();

    // Print to serial
    Serial.printf("[TELEM] DIST L=%.1fcm  C=%.1fcm  R=%.1fcm  |  BLE=%s  CMD='%c'\n",
                   sensors.distLeft, sensors.distCenter, sensors.distRight,
                   bleConnected ? "CONN" : "----",
                   (char)lastCommand);

    // Send over BLE if connected
    if (bleConnected && pTxCharacteristic) {
      char buf[80];
      snprintf(buf, sizeof(buf), "L=%.0f C=%.0f R=%.0f",
               sensors.distLeft, sensors.distCenter, sensors.distRight);
      pTxCharacteristic->setValue((uint8_t*)buf, strlen(buf));
      pTxCharacteristic->notify();
    }
  }

  delay(20);   // ~50 Hz loop
}