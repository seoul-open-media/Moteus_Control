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
//  Motor Configuration
//——————————————————————————————————————————————————————————————————————————————
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

#endif
