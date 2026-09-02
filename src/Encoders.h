#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

// ==============================================================================
// Encoders — Motor Wheel Encoders (Quadrature)
// ==============================================================================
// Set up ISRs to count pulses per motor.
// Closed-loop PID control is deferred per spec; these are just for logging/
// calibration/potential future use.
// ==============================================================================

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

#endif // ENCODERS_H
