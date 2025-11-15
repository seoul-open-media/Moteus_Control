#ifndef NEOKEY_H
#define NEOKEY_H

#include <Adafruit_NeoKey_1x4.h>
#include <seesaw_neopixel.h>

// External reference to NeoKey object
extern Adafruit_NeoKey_1x4 neokey;

// Function declarations
void initNeoKey();
void updateNeoKey();
void setKeyColor(uint8_t key, uint32_t color);

// Key assignments
#define KEY_POS_0      0
#define KEY_POS_025    1
#define KEY_POS_075    2
#define KEY_STOP       3

// Color definitions
#define COLOR_OFF       0x000000
#define COLOR_RED       0xFF0000
#define COLOR_GREEN     0x00FF00
#define COLOR_BLUE      0x0000FF
#define COLOR_YELLOW    0xFFFF00
#define COLOR_CYAN      0x00FFFF
#define COLOR_MAGENTA   0xFF00FF
#define COLOR_WHITE     0xFFFFFF
#define COLOR_DIM_RED   0x100000
#define COLOR_DIM_GREEN 0x001000
#define COLOR_DIM_BLUE  0x000010
#define COLOR_DIM_CYAN  0x001010

#endif
