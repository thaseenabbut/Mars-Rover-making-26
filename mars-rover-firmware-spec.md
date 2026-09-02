# Mars Rover — Firmware Spec (for Claude Code)

## Context
College Mars rover competition. Team of 6. Build phase starts Monday, ~15 days total.
Goal for this doc: implement the ESP32-S3 firmware in **C++ (Arduino framework)** for
continuous obstacle avoidance driving — not a stop/detect/turn rover, but smooth,
continuously-adjusted steering and speed.

Editor: Arduino IDE (or PlatformIO). Flashing: Arduino IDE's built-in upload (esptool runs
under the hood). Circuit planned in CirkitDesigner (app.cirkitdesigner.com).

Note: originally planned in MicroPython for faster iteration while learning; switched to
C++/Arduino on the team's decision.

---

## Hardware

- **ESP32-S3** — main controller, MicroPython
- **6x Encoder Metal Gearmotor**, 12V DC, 300RPM, regular wheels (not mecanum)
- **3x L298N** motor drivers (2 motors each)
- **3x HC-SR04** ultrasonic distance sensors (front-left, front-center, front-right)
- **MPU6050** IMU (accelerometer + gyroscope), I2C — not yet wired in the current diagram
- **11.1V 3S LiPo** (4000mAh) → **LM2596 buck converter** → 5V for ESP32-S3/electronics
- No camera on this build

Common ground rail ties: LiPo, all 3 L298Ns, LM2596, ESP32-S3, sensors.

**Wiring status as of now:** power rail fully planned (LiPo → 3x L298N + LM2596 in parallel,
ESP32-S3 powered off LM2596 5V output). L298N motor-side (OUT terminals) connected. L298N
**signal pins to ESP32-S3 are the current gap** — being wired next, using the pin map below.
Encoders and MPU6050 not yet wired in the diagram. All current pin assignments (including
sensor pins) are placeholders for a deadline submission and will be re-verified once physical
parts arrive Monday — treat the pin map below as the reference to code against, but expect it
to get finalized/corrected during hands-on bring-up.

---

## Pin Map (ESP32-S3)

### Motors — 3x L298N (2 motors each)

| L298N | Motor | IN (Dir 1) | IN (Dir 2) | EN (PWM) | Encoder A | Encoder B |
|---|---|---|---|---|---|---|
| #1 | Front-Left  | 1  | 2  | 3  | 4  | 5  |
| #1 | Front-Right | 6  | 7  | 8  | 9  | 10 |
| #2 | Mid-Left    | 11 | 12 | 13 | 14 | 15 |
| #2 | Mid-Right   | 16 | 17 | 18 | 19 | 20 |
| #3 | Rear-Left   | 21 | 38 | 39 | 40 | 41 |
| #3 | Rear-Right  | 42 | 45*| 46*| 47 | 48 |

\* GPIO 45/46 are strapping pins on ESP32-S3 — avoid for active signals if possible; confirm
against the exact board's datasheet once parts arrive, reassign Rear-Right's dir/PWM pins if needed.

### Ultrasonic Sensors — 3x HC-SR04 (reserved pins — not the placeholder ones in the current diagram)

| Sensor | Trig | Echo |
|---|---|---|
| Front-Left   | 22 | 23 |
| Front-Center | 24 | 25 |
| Front-Right  | 26 | 27 |

### MPU6050 (IMU) — I2C

| Signal | Pin |
|---|---|
| SDA | 33 |
| SCL | 34 |
| VCC | 3.3V (confirm on breakout — not 5V) |

---

## Control System Design

**Goal:** continuous obstacle detection + avoidance while moving (no stop-and-go), smooth
dynamic speed control. This is the differentiator vs. the typical "detect object, hard turn
left/right" competition rover.

### Steering
Differential steering based on sensor delta:
```
steering_bias = distance_right - distance_left
```
Positive → steer right, negative → steer left. Applied as a differential adjustment to
left/right wheel speeds, not a hard/discrete turn.

### Speed mapping
Target speed scales **continuously** with the center sensor's distance reading (not fixed
speed modes/steps), clamped between a min and max duty cycle. Example curve:
- 100cm+ → near max speed
- 25cm → very slow
- below ~15cm safety threshold → hard stop

### Smoothing
Exponential smoothing/ramping applied every control loop cycle so PWM changes gradually
instead of jumping:
```
current_speed += (target_speed - current_speed) * smoothing_factor
```

### Main loop (~50–100ms per cycle)
1. Read all 3 ultrasonic sensors
2. Compute `target_speed` from center distance
3. Compute `steering_bias` from left/right difference
4. Smooth `current_speed` toward `target_speed`
5. Apply `current_speed ± steering_bias` to left/right motor groups via PWM (all 3 motors per
   side move together as one group — front/mid/rear-left share a speed, front/mid/rear-right
   share a speed)
6. Repeat

Base cruising speed intentionally capped below max duty (e.g. ~90%) to leave headroom for
steering corrections in either direction.

### Encoders (deferred)
Each motor's encoder is wired from day one, but **closed-loop speed correction is deferred**
until basic open-loop driving works — i.e. no PID yet. For now, encoder pins should be
initialized/readable (via interrupt/ISR or polling) but not fed back into the control loop.
PID (comparing measured RPM to target RPM and adjusting PWM) comes later, as its own phase —
flag clearly in code comments where that hook will go, since this concept needs to be
re-explained before implementing it.

### MPU6050
Not yet integrated into the control loop design — wire up I2C reads (accel/gyro) as a
standalone module for now; how it factors into steering/stability is still open.

---

## Suggested firmware structure

Rough module split for Claude Code to scaffold in C++ (Arduino framework, .ino + .h/.cpp pairs):

```
rover/
├── rover.ino           # entry point — setup() + loop(), ties everything together
├── config.h            # ALL pin numbers + tunable constants (single source of truth)
├── Motors.h/.cpp        # L298N driver class: setDirection(), setSpeed(), per motor group
├── Sensors.h/.cpp       # HC-SR04 read functions, distance calc
├── Encoders.h/.cpp      # encoder pin setup + pulse counting via ISR (not yet used in control loop)
├── IMU.h/.cpp           # MPU6050 I2C init + read (standalone, not yet integrated)
└── Control.h/.cpp       # the actual control loop: steeringBias, speed mapping, smoothing
```

Keep pin numbers and tunable constants (smoothing factor, speed curve breakpoints, safety
stop distance, PWM frequency) centralized in `config.h` so they're easy to re-tune during
Days 10–12 (testing & tuning phase) without hunting through logic code.

**Why this split:**
- `config.h` centralizing pins/constants means Days 10–12 tuning is a one-file edit, not a hunt through logic code.
- `Control.h/.cpp` separate from `rover.ino` keeps `loop()` dead simple (`control.update();`), while the actual steering/speed math lives somewhere testable on its own.
- Each hardware class (Motors/Sensors/Encoders/IMU) only knows its own hardware — `Motors` doesn't know what a "steering bias" is, it just exposes `setSpeed(left, right)`. That's what makes it easy to bring up one piece at a time (Days 1–4) before wiring them together.
- Arduino IDE auto-compiles all `.h`/`.cpp` files in the sketch folder alongside the `.ino`, so no manual build config is needed for this structure.

Note: encoder ISRs (interrupt service routines) need to be lightweight in C++ — avoid doing math or serial prints directly inside the interrupt handler; just increment a counter and let `Control.cpp` read it on the next loop pass.

---

## Build/coding timeline (for reference)

- **Days 1–2:** bring-up in pairs — ESP32 + single motor; HC-SR04 individually; MPU6050 + power system
- **Days 3–4:** scale up — all 6 motors via 3 L298Ns; all 3 ultrasonics simultaneously
- **Days 5–7:** integration — full control loop (sensors → speed/steering calc → motor output)
- **Days 8–9:** full physical assembly, wiring finalized, power confirmed under full load
- **Days 10–12:** real-world testing & tuning (thresholds, smoothing factor, steering sensitivity)
- **Days 13–14:** buffer + polish, edge case testing
- **Day 15:** final run-through, no major changes

## Open questions
- Chassis — confirm whether provided by organizers or team-built
- Rear-Right L298N dir/PWM pins (45/46) — reassign if they conflict with strapping pin behavior on the actual board
- How MPU6050 data will factor into the control loop (not yet designed)
