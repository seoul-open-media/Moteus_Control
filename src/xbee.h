#ifndef XBEE_H
#define XBEE_H

#include <Arduino.h>

// XBee configuration
#define XBEE_BAUD 115200

// XBee state for continuous control
extern bool xbeeControlActive;
extern float xbeeTargetM1;
extern float xbeeTargetM2;

// XBee communication functions (Serial4 disabled — see updateWiredSerial)
void initXBee();
// void updateXBee();        // [DISABLED] XBee wireless via Serial4
void updateXBeeControl();

// Wired serial binary protocol (replaces XBee)
// Receives same [0xFF][0xFF][R1_MSB][R1_LSB]...[R13_MSB][R13_LSB] over Serial (USB, 115200)
void updateWiredSerial();

#endif
