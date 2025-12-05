#ifndef CONFIG_H
#define CONFIG_H

//——————————————————————————————————————————————————————————————————————————————
//  Hardware Pin Configuration
//——————————————————————————————————————————————————————————————————————————————
static const byte MCP2517_SCK = 13;  // SCK input of MCP2517
static const byte MCP2517_SDI = 11;  // SDI input of MCP2517
static const byte MCP2517_SDO = 12;  // SDO output of MCP2517
static const byte MCP2517_CS  = 10;  // CS input of MCP2517

//——————————————————————————————————————————————————————————————————————————————
//  OLED Display Configuration - Using Wire1 I2C bus
//——————————————————————————————————————————————————————————————————————————————
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // I2C address (0x3C or 0x3D)

//——————————————————————————————————————————————————————————————————————————————
//  Robot & Motor Configuration
//——————————————————————————————————————————————————————————————————————————————
// IMPORTANT: Set unique ROBOT_ID for each unit (1-100)
static const uint8_t ROBOT_ID = 4;  // Change this for each robot!

// Robot Type: 'A' or 'B' (determines electromagnet/solenoid behavior)
// Type A: digit4=1→EM1+EM2+EM3, digit4=2→SOL, digit4=3→ALL
// Type B: digit4=1→EM1+EM2, digit4=2→SOL+EM3, digit4=3→ALL
static const char ROBOT_TYPE = 'A';  // 'A' or 'B'

static const double MOTOR1_DIRECTION = 1.0;   // 1.0 = normal, -1.0 = reversed
static const double MOTOR2_DIRECTION = 1.0;   // 1.0 = normal, -1.0 = reversed

static const int MOTOR1_ID = 1;
static const int MOTOR2_ID = 2;

//——————————————————————————————————————————————————————————————————————————————
//  Control Parameters
//——————————————————————————————————————————————————————————————————————————————
static const double TOLERANCE = 0.05;       // Position tolerance (external revolutions)
static const double MAX_VELOCITY = 15.0;    // Maximum velocity (rev/s)
static const double SLOW_ZONE = 0.10;       // Start slowing down within this distance
static const double BRAKE_ZONE = 0.06;      // Active braking zone
static const unsigned long LOOP_PERIOD_MS = 10;  // 100Hz control loop (10ms)
static const unsigned long TIMEOUT_MS = 15000;   // 15 second timeout

//——————————————————————————————————————————————————————————————————————————————
//  Electromagnet & Solenoid Pin Configuration
//——————————————————————————————————————————————————————————————————————————————
static const uint8_t EM1_PIN = 3;        // Electromagnet 1 (PWM capable)
static const uint8_t EM2_PIN = 4;        // Electromagnet 2 (PWM capable)
static const uint8_t SOLENOID_PIN = 5;   // Solenoid (PWM capable)
static const uint8_t EM3_PIN = 6;        // Electromagnet 3 (PWM capable)

#endif
