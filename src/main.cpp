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
#include "xbee.h"
#include "solenoid.h"

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
  Serial.setTimeout(20);  // Prevent 1s blocking in readStringUntil() during PD operation
 // while (!Serial) {}
 delay(1000);
  Serial.println(F("started"));

  // Initialize OLED display on Wire1
  initDisplay();
  displayStatus("Initializing...");
  
  // Scan I2C bus to verify display is connected
 // scanI2C();
  
  // Initialize XBee on Serial4
  // initXBee();  // [XBee DISABLED] — XBee on Serial4 commented out
  Serial.println(F("Wired serial control active (USB Serial, 115200)"));
  Serial.print(F("Robot ID: "));
  Serial.println(ROBOT_ID);
  
  // Initialize electromagnets and solenoid
  initSolenoid();

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
  // Note: Motor queries are handled by updateXBeeControl() when engaged (100Hz)
  // When disengaged, we need to query here for display updates
  static unsigned long lastQueryTime = 0;
  unsigned long currentTime = millis();
  
  // Query motors at 20Hz (every 50ms) — always, regardless of XBee state
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
  
  // Check wired serial for binary packets from PD (same protocol as XBee)
  updateWiredSerial();
  
  // [XBee DISABLED] XBee wireless communication commented out
  // updateXBee();          // Was: XBee via Serial4
  // updateXBeeControl();   // No-op anyway
  
  // Update electromagnet/solenoid timing
  updateSolenoid();

  // Check if there's a pending command from interrupt
  String val = "";
  if (pendingCommand.length() > 0) {
    val = pendingCommand;
    pendingCommand = "";  // Clear pending command
    Serial.print(F("Processing pending: "));
    Serial.println(val);
  }
  else if (Serial.available() && Serial.peek() != 0xFF) {
    // Only read text commands when incoming byte is not a binary packet header
    val = Serial.readStringUntil('\n');
    val.trim();  // Remove whitespace
  }
  
  if (val.length() > 0) {
    // Simple single-digit commands for preset positions
    if (val == "0") {
      Serial.println(F("Command 0: Stop all motors"));
      displayDebug("STOP");
      interruptCommand = true;  // Interrupt any running movement
      moteus1.SetStop();
      moteus2.SetStop();
      commandRunning = false;
      xbeeControlActive = false;
      Serial.println(F("Motors stopped and disengaged"));
    }
    
    else if (val == "1") {
      Serial.println(F("Command 1: Moving Motor 2 to 0 degrees [FAST]"));
      displayDebug("M2 -> 0deg");
      Moteus::Query::Format query_fmt;
      query_fmt.abs_position = Moteus::kFloat;
      moteus1.SetQuery(&query_fmt);
      moteus2.SetQuery(&query_fmt);
      delay(50);
      double current_ext1 = moteus1.last_result().values.abs_position;
      moveToEncoderPosition(current_ext1, ENCODER_CENTER + 0.0,   MAX_VELOCITY_FAST);
    }
    
    else if (val == "2") {
      Serial.println(F("Command 2: Moving Motor 2 to -45 degrees [FAST]"));
      displayDebug("M2 -> -45deg");
      Moteus::Query::Format query_fmt;
      query_fmt.abs_position = Moteus::kFloat;
      moteus1.SetQuery(&query_fmt);
      moteus2.SetQuery(&query_fmt);
      delay(50);
      double current_ext1 = moteus1.last_result().values.abs_position;
      moveToEncoderPosition(current_ext1, ENCODER_CENTER - 0.125, MAX_VELOCITY_FAST);
    }
    
    else if (val == "3") {
      Serial.println(F("Command 3: Moving Motor 2 to 45 degrees [FAST]"));
      displayDebug("M2 -> 45deg");
      Moteus::Query::Format query_fmt;
      query_fmt.abs_position = Moteus::kFloat;
      moteus1.SetQuery(&query_fmt);
      moteus2.SetQuery(&query_fmt);
      delay(50);
      double current_ext1 = moteus1.last_result().values.abs_position;
      moveToEncoderPosition(current_ext1, ENCODER_CENTER + 0.125, MAX_VELOCITY_FAST);
    }
    
    else if (val == "4") {
      Serial.println(F("Command 4: Moving Motor 2 to 90 degrees [FAST]"));
      displayDebug("M2 -> 90deg");
      Moteus::Query::Format query_fmt;
      query_fmt.abs_position = Moteus::kFloat;
      moteus1.SetQuery(&query_fmt);
      moteus2.SetQuery(&query_fmt);
      delay(50);
      double current_ext1 = moteus1.last_result().values.abs_position;
      moveToEncoderPosition(current_ext1, ENCODER_CENTER + 0.25,  MAX_VELOCITY_FAST);
    }
    else if (val.indexOf("osc") != -1) {
      // Oscillate both motors
      Serial.println(F("Starting oscillation (limited range)..."));
      displayDebug("Oscillating...");
      oscillateMotors();
    }
    
    else if (val == "5") {
      Serial.println(F("Command 5: EM1+EM2+EM3 ON (50ms)"));
      displayDebug("EM x3 ON");
      triggerElectromagnets();
    }

    else if (val == "6") {
      Serial.println(F("Command 6: Solenoid ON (50ms)"));
      displayDebug("SOL ON");
      triggerSolenoid();
    }

    else if (val == "scan" || val == "i2c") {
      // Scan I2C bus for debugging
      displayDebug("Scanning I2C...");
      scanI2C();
    }
  }
}
