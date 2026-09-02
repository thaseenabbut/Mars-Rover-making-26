#include "Encoders.h"
#include "config.h"

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
