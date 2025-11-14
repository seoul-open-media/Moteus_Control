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
#include "display.h"
#include "neokey.h"
#include "xbee.h"

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
 // while (!Serial) {}
 delay(1000);
  Serial.println(F("started"));

  // Initialize OLED display on Wire1
  initDisplay();
  displayStatus("Initializing...");
  
  // Scan I2C bus to verify display is connected
 // scanI2C();
  
  // Initialize NeoKey on Wire (I2C bus 0)
  initNeoKey();
  
  // Initialize XBee on Serial1
  initXBee();

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
  
  displayStatus("Motors ready");
  delay(1000);
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
  // Query motors for current status (needed for display update)
  static unsigned long lastQueryTime = 0;
  unsigned long currentTime = millis();
  
  // Query motors at 20Hz (every 50ms) to keep display updated
  if (currentTime - lastQueryTime >= 50) {
    Moteus::Query::Format query_fmt;
    query_fmt.mode = Moteus::kInt8;
    query_fmt.position = Moteus::kFloat;
    query_fmt.velocity = Moteus::kFloat;
    query_fmt.abs_position = Moteus::kFloat;
    query_fmt.motor_temperature = Moteus::kFloat;
    
    moteus1.SetQuery(&query_fmt);
    moteus2.SetQuery(&query_fmt);
    
    lastQueryTime = currentTime;
  }
  
  // Call periodic diagnostics (runs every 100ms automatically)
  // Uncomment the line below to enable continuous diagnostics output
  // printDiagnosticsAll();
  
  // Check motor temperature every loop iteration
  checkMotorTemperature();
  
  // Update OLED display with current motor status
  updateDisplay();
  
  // Check NeoKey for button presses
  updateNeoKey();
  
  // Check XBee for wireless commands
  updateXBee();

  // Check if there's a pending command from interrupt
  String val = "";
  if (pendingCommand.length() > 0) {
    val = pendingCommand;
    pendingCommand = "";  // Clear pending command
    Serial.print(F("Processing pending: "));
    Serial.println(val);
  }
  else if (Serial.available()) {
    val = Serial.readStringUntil('\n');
    val.trim();  // Remove whitespace
  }
  
  if (val.length() > 0) {
    // Simple single-digit commands for preset positions
    if (val == "1") {
      Serial.println(F("Command 1: Moving Motor 1 to encoder position -0.25"));
      displayDebug("M1 -> -0.25");
      // Query motors first to get current positions
      Moteus::Query::Format query_fmt;
      query_fmt.abs_position = Moteus::kFloat;
      moteus1.SetQuery(&query_fmt);
      moteus2.SetQuery(&query_fmt);
      delay(50);
      double current_ext2 = moteus2.last_result().values.abs_position;
      moveToEncoderPosition(-0.25, current_ext2);
    }
    
    else if (val == "2") {
      Serial.println(F("Command 2: Moving Motor 1 to encoder position 0.25"));
      displayDebug("M1 -> 0.25");
      // Query motors first to get current positions
      Moteus::Query::Format query_fmt;
      query_fmt.abs_position = Moteus::kFloat;
      moteus1.SetQuery(&query_fmt);
      moteus2.SetQuery(&query_fmt);
      delay(50);
      double current_ext2 = moteus2.last_result().values.abs_position;
      moveToEncoderPosition(0.25, current_ext2);
    }
    
    else if (val == "3") {
      Serial.println(F("Command 3: Moving Motor 2 to encoder position -0.25"));
      displayDebug("M2 -> -0.25");
      // Query motors first to get current positions
      Moteus::Query::Format query_fmt;
      query_fmt.abs_position = Moteus::kFloat;
      moteus1.SetQuery(&query_fmt);
      moteus2.SetQuery(&query_fmt);
      delay(50);
      double current_ext1 = moteus1.last_result().values.abs_position;
      moveToEncoderPosition(current_ext1, -0.25);
    }
    
    else if (val == "4") {
      Serial.println(F("Command 4: Moving Motor 2 to encoder position 0.25"));
      displayDebug("M2 -> 0.25");
      // Query motors first to get current positions
      Moteus::Query::Format query_fmt;
      query_fmt.abs_position = Moteus::kFloat;
      moteus1.SetQuery(&query_fmt);
      moteus2.SetQuery(&query_fmt);
      delay(50);
      double current_ext1 = moteus1.last_result().values.abs_position;
      moveToEncoderPosition(current_ext1, 0.25);
    }
    else if (val.indexOf("osc") != -1) {
      // Oscillate both motors
      Serial.println(F("Starting oscillation (limited range)..."));
      displayDebug("Oscillating...");
      oscillateMotors();
    }
    
    else if (val == "scan" || val == "i2c") {
      // Scan I2C bus for debugging
      displayDebug("Scanning I2C...");
      scanI2C();
    }
  }
}
