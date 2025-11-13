//——————————————————————————————————————————————————————————————————————————————
// Demonstration of control and monitoring of 2 moteus controllers
// running on a CANBed FD from longan labs.
//  * https://mjbots.com/products/moteus-r4-11
//  * https://www.longan-labs.cc/1030009.html
// ——————————————————————————————————————————————————————————————————————————————

#include <ACAN2517FD.h>
#include <Moteus.h>
#include "config.h"
#include "motor_control.h"

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517FD can (MCP2517_CS, SPI, 255) ; // Last argument is 255 -> no interrupt pin

Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 1; // Moteus ID (by Tview)
  return options;
}());

Moteus moteus2(can, []() {
  Moteus::Options options;
  options.id = 2; // Moteus ID (by Tview)
  return options;
}());

Moteus::PositionMode::Command position_cmd1;
Moteus::PositionMode::Command position_cmd2;
Moteus::PositionMode::Format position_fmt;

// Forward declarations - removed, now in motor_control.h

void setup() {
  pinMode (LED_BUILTIN, OUTPUT);

  // Let the world know we have begun!
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("started"));

  SPI.begin();

  // This operates the CAN-FD bus at 1Mbit for both the arbitration
  // and data rate.  Most arduino shields cannot operate at 5Mbps
  // correctly, so the moteus Arduino library permanently disables
  // BRS.
  ACAN2517FDSettings settings(
      ACAN2517FDSettings::OSC_40MHz, 1000ll * 1000ll, DataBitRateFactor::x1);

  // The atmega32u4 on the CANbed has only a tiny amount of memory.
  // The ACAN2517FD driver needs custom settings so as to not exhaust
  // all of SRAM just with its buffers.
  settings.mArbitrationSJW = 2;
  settings.mDriverTransmitFIFOSize = 1;
  settings.mDriverReceiveFIFOSize = 2;


  const uint32_t errorCode = can.begin(settings, NULL);

  while (errorCode != 0) {
    Serial.print(F("CAN error 0x"));
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  position_fmt.position = Moteus::kInt16;
  position_fmt.velocity = Moteus::kInt16;
  position_fmt.maximum_torque = Moteus::kInt16;
  position_fmt.feedforward_torque = Moteus::kInt16;
  position_fmt.kp_scale = Moteus::kInt16;
  position_fmt.kd_scale = Moteus::kInt16;
  position_fmt.stop_position = Moteus::kInt16;
  position_fmt.velocity_limit = Moteus::kInt16;
  position_fmt.accel_limit = Moteus::kInt16;

  position_cmd1.velocity_limit = 0.5;
  position_cmd1.accel_limit = 0.1;

  position_cmd2.velocity_limit = 0.5;
  position_cmd2.accel_limit = 0.1;

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus1.SetStop();
  moteus2.SetStop();
  Serial.println(F("all stopped"));
  delay(100);
}

// Signed power function (like basilisk code) to handle negative values
double signedpow(double base, double exponent) {
  if (base >= 0) {
    return pow(base, exponent);
  } else {
    return -pow(-base, exponent);
  }
}

void loop() {
  // Call periodic diagnostics (runs every 100ms automatically)
  // Uncomment the line below to enable continuous diagnostics output
  // printDiagnosticsAll();
  
  // Check motor temperature every loop iteration
  checkMotorTemperature();

  if (Serial.available()) {
    String val = Serial.readStringUntil('\n');
    val.trim();  // Remove whitespace
    
    // Simple single-digit commands for preset positions
    if (val == "1") {
      Serial.println(F("Command 1: Moving Motor 1 to encoder position -0.25"));
      // Query motors first to get current positions
      Moteus::Query::Format query_fmt;
      query_fmt.abs_position = Moteus::kFloat;
      moteus1.SetQuery(&query_fmt);
      moteus2.SetQuery(&query_fmt);
      delay(50);
      double current_ext2 = moteus2.last_result().values.abs_position;
      moveToEncoderPosition(-0.25, current_ext2);
      return;
    }
    
    if (val == "2") {
      Serial.println(F("Command 2: Moving Motor 1 to encoder position 0.25"));
      // Query motors first to get current positions
      Moteus::Query::Format query_fmt;
      query_fmt.abs_position = Moteus::kFloat;
      moteus1.SetQuery(&query_fmt);
      moteus2.SetQuery(&query_fmt);
      delay(50);
      double current_ext2 = moteus2.last_result().values.abs_position;
      moveToEncoderPosition(0.25, current_ext2);
      return;
    }
    
    if (val == "3") {
      Serial.println(F("Command 3: Moving Motor 2 to encoder position -0.25"));
      // Query motors first to get current positions
      Moteus::Query::Format query_fmt;
      query_fmt.abs_position = Moteus::kFloat;
      moteus1.SetQuery(&query_fmt);
      moteus2.SetQuery(&query_fmt);
      delay(50);
      double current_ext1 = moteus1.last_result().values.abs_position;
      moveToEncoderPosition(current_ext1, -0.25);
      return;
    }
    
    if (val == "4") {
      Serial.println(F("Command 4: Moving Motor 2 to encoder position 0.25"));
      // Query motors first to get current positions
      Moteus::Query::Format query_fmt;
      query_fmt.abs_position = Moteus::kFloat;
      moteus1.SetQuery(&query_fmt);
      moteus2.SetQuery(&query_fmt);
      delay(50);
      double current_ext1 = moteus1.last_result().values.abs_position;
      moveToEncoderPosition(current_ext1, 0.25);
      return;
    }
    if (val.indexOf("osc") != -1) {
      // Oscillate both motors
      Serial.println(F("Starting oscillation (limited range)..."));
      oscillateMotors();
      return;
    }
  }
}
