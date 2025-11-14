#ifndef XBEE_H
#define XBEE_H

#include <Arduino.h>

// XBee configuration
#define XBEE_BAUD 115200

// XBee communication functions
void initXBee();
void updateXBee();
bool parseXBeeCommand(String command);

#endif
