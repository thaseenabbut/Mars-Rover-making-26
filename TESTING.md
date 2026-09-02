# Mars Rover — Hardware Bring-Up Checklist

Firmware is written and compiles clean. This is the test order for when parts arrive on
campus — go in this sequence, don't skip ahead, since later steps assume earlier ones work.
Matches the Days 1–2 pairing plan: Pair A (motors), Pair B (sensors), Pair C (IMU + power).

---

## 0. Power rail (do this first, before anything else is plugged in)

- [ ] LiPo → XT60 → confirm +/- polarity with multimeter (red = +, black = -)
- [ ] LM2596 buck converter wired to LiPo rail (IN+/IN-)
- [ ] **Before connecting ESP32-S3:** measure LM2596 OUT+/OUT- with multimeter, trim pot until
      it reads **5.0–5.1V**. Do not skip this — overvoltage can kill the ESP32-S3.
- [ ] Connect ESP32-S3 5V/GND to LM2596 output
- [ ] Flash firmware, open Serial Monitor (115200 baud) — confirm the board boots and prints
      the setup() diagnostics without crashing/rebooting in a loop (a boot loop usually means
      a wiring or strapping-pin issue — see note at the bottom)

**Checkpoint:** ESP32-S3 boots clean, serial output visible, no resets. Don't move on until
this is solid — every other subsystem depends on stable power.

---

## Pair A — Motors (one L298N, one motor first)

- [ ] Wire **one** L298N to the power rail (12V, GND) — confirm with multimeter it reads
      battery voltage across its output terminals
- [ ] Wire that L298N's IN1/IN2/EN to the ESP32-S3 per `config.h`
- [ ] Connect just **one** motor to OUT1/OUT2
- [ ] Send a simple test command (or use a basic serial command if firmware supports one) to
      spin that motor forward, then reverse
- [ ] If it spins the wrong direction: don't rewire anything, just swap the two motor leads on
      OUT1/OUT2 — direction is arbitrary from the driver's perspective
- [ ] Repeat for the second motor on the same L298N
- [ ] Once both motors on L298N #1 work, wire up L298N #2 and #3 the same way
- [ ] Test all 6 motors individually before testing them together

**Checkpoint:** all 6 motors spin correctly in both directions when driven individually.

---

## Pair B — Ultrasonic sensors (one at a time)

- [ ] Wire one HC-SR04's Trig/Echo per `config.h`, VCC to 5V rail, GND to common ground
- [ ] Watch serial output — confirm it reports a distance reading, and that the number
      changes sensibly when you move your hand toward/away from the sensor
- [ ] Test at very close range (~5cm) and far range (~2m+) — confirm no garbage/negative
      values, and that far/open readings hit the timeout fallback cleanly instead of erroring
- [ ] Repeat for the second and third sensor
- [ ] With all 3 wired, confirm all 3 report independently and don't interfere with each other
      (this can happen with bad grounding — if readings glitch when multiple are active,
      double check common ground)

**Checkpoint:** all 3 sensors report stable, sensible distances independently and together.

---

## Pair C — IMU + power system (can run in parallel with A/B once power checkpoint is done)

- [ ] Wire MPU6050 SDA/SCL/VCC(3.3V!)/GND per `config.h`
- [ ] Confirm serial output shows IMU init as OK, not FAILED (check I2C address fallback
      logic — 0x68 vs 0x69 — if it fails, verify wiring before assuming address issue)
- [ ] Confirm accel/gyro values look sane at rest (roughly 0 on gyro, ~9.8 m/s² on one accel
      axis depending on orientation) and change when you tilt/move the board
- [ ] Re-confirm power system stays stable under motor load once motors are also running
      (voltage sag under load is a common failure point — recheck LM2596 output with a
      multimeter while motors are spinning)

**Checkpoint:** IMU reads sensibly, power stays stable with motors + sensors + IMU all active.

---

## Days 3–4 — Integration (only after all of the above pass individually)

- [ ] All 6 motors + all 3 sensors + IMU running simultaneously, no crashes, no brownouts
- [ ] Confirm encoder tick counts increase when motors spin (even though not used in control
      loop yet) — sanity check that wiring is correct for when PID gets added later
- [ ] Only once everything above is stable: let `control.update()` run for real and observe
      the rover actually steer/speed-adjust in response to obstacles

---

## If the ESP32-S3 won't boot cleanly / resets in a loop

Check `config.h` for **GPIO 45 and 46** first — these are strapping pins on the ESP32-S3 and
if something is pulling them the wrong way at boot, the board can fail to start correctly.
Reassign Rear-Right's IN2/EN pins to different free GPIOs (see "spare pins" note in the
firmware spec) if this happens.
