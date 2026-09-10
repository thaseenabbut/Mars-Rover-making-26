#include <Arduino.h>

// L298N #1
// Front Left motor
#define FL_IN1 41
#define FL_IN2 42

// Front Right motor
#define FR_IN1 6
#define FR_IN2 7

void setup() {
  // Direction pins
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);

  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);

  // Move both motors forward
  digitalWrite(FL_IN1, HIGH);
  digitalWrite(FL_IN2, LOW);

  digitalWrite(FR_IN1, HIGH);
  digitalWrite(FR_IN2, LOW);
}

void loop() {
  // Motors keep rotating
}