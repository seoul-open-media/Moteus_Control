#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ACAN2517FD.h>
#include <Moteus.h>

// External references to motor objects (defined in main.cpp)
extern Moteus moteus1;
extern Moteus moteus2;
extern Moteus::PositionMode::Command position_cmd1;
extern Moteus::PositionMode::Command position_cmd2;
extern Moteus::PositionMode::Format position_fmt;

// Function declarations
void moveToEncoderPosition(double target_ext1, double target_ext2);
void oscillateMotors();
void printDiagnostics();
void printDiagnosticsAll();
void checkMotorTemperature();

#endif
