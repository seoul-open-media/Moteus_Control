#ifndef SOLENOID_H
#define SOLENOID_H

#include <Arduino.h>

// Initialize electromagnets and solenoid
void initSolenoid();

// Update function - call from main loop to handle timing
void updateSolenoid();

// Trigger functions for digit4 control
void triggerElectromagnets();  // All 3 EMs on for 50ms
void triggerSolenoid();         // Solenoid on for 50ms

// Type A control functions
void triggerTypeA_1();  // digit4=1: EM1, EM2, EM3 on
void triggerTypeA_2();  // digit4=2: Solenoid on
void triggerTypeA_3();  // digit4=3: All on

// Motor brake function - hold electromagnet until motor stops moving
void triggerMotorBrake();

// Update encoder positions for brake monitoring
void updateBrakeEncoders(double ext1, double ext2);

// Stop motor brake manually
void stopMotorBrake();

#endif
