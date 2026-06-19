#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ACAN2517FD.h>
#include <Moteus.h>
using Moteus = MoteusController<ACAN2517FD>;

// External references to motor objects (defined in main.cpp)
extern Moteus moteus1;
extern Moteus moteus2;
extern Moteus::PositionMode::Command position_cmd1;
extern Moteus::PositionMode::Command position_cmd2;
extern Moteus::PositionMode::Format position_fmt;

// External reference to solenoid functions
void triggerMotorBrake();

// Global flag to interrupt current motor command
extern volatile bool interruptCommand;
extern volatile bool commandRunning;
extern String pendingCommand;  // Store command that triggered interrupt
extern unsigned long lastXBeeCommand;  // Timestamp of last XBee command

// Function declarations
void moveToEncoderPosition(double target_ext1, double target_ext2);
void moveToEncoderPosition(double target_ext1, double target_ext2, float max_speed_limit);
void oscillateMotors();
void printDiagnostics();
void printDiagnosticsAll();
void checkMotorTemperature();

#endif
