#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// External reference to display object
extern Adafruit_SSD1306 display;

// Function declarations
void initDisplay();
void updateDisplay();  // Will query motors directly
void displayStatus(const char* message);
void displayError(const char* error);
void displayDebug(const char* message);  // Show debug message on OLED
void scanI2C();  // I2C scanner for debugging

#endif
