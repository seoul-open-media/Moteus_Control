#ifndef XBEE_H
#define XBEE_H

#include <Arduino.h>

// XBee configuration
#define XBEE_BAUD 115200

// XBee state for continuous control
extern bool xbeeControlActive;
extern float xbeeTargetM1;
extern float xbeeTargetM2;

// XBee communication functions
void initXBee();
void updateXBee();
void updateXBeeControl();  // Call from main loop to send continuous commands
bool parseXBeeCommand(String command);

#endif
