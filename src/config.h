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

// Motor Calibration Offset (Units: Revolutions)
// Adjust this value to align the zero point (0 degree) for each robot
// Robot 1: 0.0
// Robot 2,3,4: Adjust if angle is incorrect (e.g., 0.5 for 180 degree shift)
static const double MOTOR_OFFSET = 0.0; 

// Robot Type Configuration
// All robots are Type A:
// digit4=1 -> EM1+EM2+EM3
// digit4=2 -> SOL
// digit4=3 -> ALL
// REMOVED: ROBOT_TYPE variable (Hardcoded to Type A behavior)

static const double MOTOR1_DIRECTION = 1.0;   // 1.0 = normal, -1.0 = reversed
static const double MOTOR2_DIRECTION = 1.0;   // 1.0 = normal, -1.0 = reversed

static const int MOTOR1_ID = 1;
static const int MOTOR2_ID = 2;

//——————————————————————————————————————————————————————————————————————————————
//  Control Parameters
//——————————————————————————————————————————————————————————————————————————————
static const double POSITION_TOLERANCE = 0.01;   // Position tolerance (3.6 degrees) - disengage when reached
static const double MAX_VELOCITY = 5;          // Maximum velocity (rev/s)
static const double SLOW_ZONE = 0.03;            // Start slowing down within this distance (10.8 degrees)
static const double BRAKE_ZONE = 0.06;           // Active braking zone (18 degrees)
static const double OVERSHOOT_ZONE = 0.02;       // Overshoot detection zone (7.2 degrees) - stop if target crossed
static const unsigned long LOOP_PERIOD_MS = 10;  // 100Hz control loop (10ms)
static const unsigned long TIMEOUT_MS = 15000;   // 15 second timeout

// Electromagnetic brake parameters
static const double ENCODER_CHANGE_THRESHOLD = 0.0005; // Encoder change threshold to release brake (0.18 degrees)
static const unsigned long MIN_BRAKE_DURATION_MS = 50; // Minimum brake duration before checking encoder (ms)
static const unsigned long MAX_BRAKE_DURATION_MS = 2000; // Maximum brake duration for safety (ms)
static const unsigned long BRAKE_CHECK_INTERVAL_MS = 20; // How often to check encoder changes (ms)
static const int BRAKE_STABLE_CHECKS = 3; // Number of consecutive stable checks before releasing brake

//——————————————————————————————————————————————————————————————————————————————
//  Electromagnet & Solenoid Pin Configuration
//——————————————————————————————————————————————————————————————————————————————
static const uint8_t EM1_PIN = 3;        // Electromagnet 1 (PWM capable)
static const uint8_t EM2_PIN = 4;        // Electromagnet 2 (PWM capable)
static const uint8_t SOLENOID_PIN = 5;   // Solenoid (PWM capable)
static const uint8_t EM3_PIN = 6;        // Electromagnet 3 (PWM capable)

#endif
