#include "xbee.h"
#include "motor_control.h"
#include "display.h"
#include "config.h"
#include "solenoid.h"
#include <Moteus.h>

// External declarations from main.cpp
extern Moteus moteus1;
extern Moteus moteus2;
extern volatile bool interruptCommand;
extern volatile bool commandRunning;
extern String pendingCommand;

// REMOVED: Unused XBee control variables (xbeeControlActive, xbeeTargetM1/M2, m1/m2_settled)
// REMOVED: Unused cached motor values (cachedM1Pos, cachedM2Pos, cachedM1Temp, cachedM2Temp)
// REMOVED: Unused fault recovery variables (m2_in_fault, m2_fault_detected_time, m2_recovery_attempts)

// Position constraints: -90 to +90 degrees = -0.25 to +0.25 revolutions
const float MIN_POSITION = -0.25;
const float MAX_POSITION = 0.25;

// REMOVED: Unused motor velocity limits (xbeeMaxVelM1, xbeeMaxVelM2)

void initXBee() {
  // Initialize Serial4 for XBee communication
  Serial4.begin(XBEE_BAUD);
  
  Serial.println(F("XBee initialized on Serial4"));
  Serial.print(F("Robot ID: "));
  Serial.println(ROBOT_ID);
  Serial.print(F("Baud rate: "));
  Serial.println(XBEE_BAUD);
  
  char buf[32];
  snprintf(buf, sizeof(buf), "Robot #%d", ROBOT_ID);
  displayDebug(buf);
}

void updateXBee() {
  // New protocol: [0xFF] [0xFF] [R1_MSB] [R1_LSB] [R2_MSB] [R2_LSB] ... [R13_MSB] [R13_LSB]
  // Total: 2 header + 26 data bytes = 28 bytes
  
  static unsigned long lastDebugTime = 0;
  if (Serial4.available() > 0 && (millis() - lastDebugTime > 500)) {
    Serial.print(F("[XBee] Available bytes: "));
    Serial.println(Serial4.available());
    lastDebugTime = millis();
  }
  
  // Check if we have enough bytes for one packet (2 header + 26 data)
  if (Serial4.available() < 28) {
    return;  // Not enough data yet
  }
  
  // Look for first header byte (0xFF)
  uint8_t byte1 = Serial4.read();
  if (byte1 != 0xFF) {
    return;  // Not a valid header
  }
  
  // Check second header byte (0xFF)
  uint8_t byte2 = Serial4.read();
  if (byte2 != 0xFF) {
    return;  // Not a valid header
  }
    
  Serial.println(F("[XBee] Header 0xFF 0xFF found!"));
  
  // Read 26 bytes for 13 robots (2 bytes each)
  uint8_t robot_data[26];
  for (int i = 0; i < 26; i++) {
    robot_data[i] = Serial4.read();
  }
  
  // Get data for this robot (ROBOT_ID 1-13 → index 0-12)
  if (ROBOT_ID < 1 || ROBOT_ID > 13) {
    Serial.print(F("[XBee] Invalid ROBOT_ID: "));
    Serial.println(ROBOT_ID);
    return;
  }
  
  int idx = (ROBOT_ID - 1) * 2;  // Each robot has 2 bytes
  uint8_t msb = robot_data[idx];
  uint8_t lsb = robot_data[idx + 1];
  
  Serial.print(F("[XBee] Robot "));
  Serial.print(ROBOT_ID);
  Serial.print(F(" MSB="));
  Serial.print(msb);
  Serial.print(F(" LSB="));
  Serial.println(lsb);
  
  // Combine MSB and LSB to get decimal value (0-65535)
  uint16_t value = ((uint16_t)msb << 8) | lsb;

  if (value == 0) {
    return; // Ignore if value is 0
  }
  
  // Extract 4 digits
  uint8_t digit1 = (value / 1000) % 10;  // Max speed (0-9)
  uint8_t digit2 = (value / 100) % 10;   // Ignored
  uint8_t digit3 = (value / 10) % 10;    // Angle control (1-9)
  uint8_t digit4 = value % 10;           // Solenoid control
  
  // Display 4-digit number on screen and serial
  char displayBuf[32];
  snprintf(displayBuf, sizeof(displayBuf), "XB:%04d (%d%d%d%d)", value, digit1, digit2, digit3, digit4);
  displayDebug(displayBuf);
  
  Serial.println(F("========================================"));
  Serial.print(F("[XBee] 4-DIGIT VALUE: "));
  Serial.print(value);
  Serial.print(F(" = "));
  Serial.print(digit1);
  Serial.print(digit2);
  Serial.print(digit3);
  Serial.println(digit4);
  Serial.print(F("[XBee] Digit1(speed)="));
  Serial.print(digit1);
  Serial.print(F(" Digit2(ignore)="));
  Serial.print(digit2);
  Serial.print(F(" Digit3(angle)="));
  Serial.print(digit3);
  Serial.print(F(" Digit4(solenoid)="));
  Serial.println(digit4);
  Serial.println(F("========================================"));
  
  // Process digit3: Angle control (0-9 -> -90 to +90 degrees)
  // 0 -> -90, 9 -> +90
  
  // Process digit1: Max speed (0-9 -> 0 to MAX_VELOCITY)
  float max_speed = ((float)digit1 / 9.0) * MAX_VELOCITY;
  if (max_speed < 0.5) max_speed = 0.5;  // Minimum speed to prevent disengage
  
  Serial.print(F("[XBee] Max speed: "));
  Serial.print(max_speed, 2);
  Serial.println(F(" rev/s"));
  
  // Map digit3 (0-9) to angle (-90 to +90 degrees)
  // 0 -> -90deg, 9 -> +90deg
  float angle_deg = ((float)digit3 / 9.0) * 180.0 - 90.0;
  float pos2_logical = angle_deg / 360.0;  // Convert to revolutions (Relative to zero)
  
  // Clamp logical position to limits (-0.25 to +0.25)
  if (pos2_logical < MIN_POSITION) pos2_logical = MIN_POSITION;
  if (pos2_logical > MAX_POSITION) pos2_logical = MAX_POSITION;
  
  // Apply calibration offset
  float pos2 = pos2_logical + MOTOR_OFFSET;
  
  Serial.print(F("[XBee] Motor 2: "));
  Serial.print(angle_deg, 1);
  Serial.print(F(" deg (logical) => "));
  Serial.print(pos2, 4);
  Serial.println(F(" rev (physical)"));
  
  // Process digit4: Solenoid control
  if (digit4 == 2) {
    triggerTypeA_2();  // Solenoid only (All robots are Type A now)
    Serial.println(F("[XBee] Solenoid activated"));
  }
  
  // Query current position of M1
  Moteus::Query::Format query_fmt;
  query_fmt.abs_position = Moteus::kFloat;
  moteus1.SetQuery(&query_fmt);
  delay(5);  // Reduced delay
  double current_ext1 = moteus1.last_result().values.abs_position;
  
  // Move M2 to target position (same as serial command logic)
  Serial.print(F("[XBee] Moving M2 to "));
  Serial.print(pos2, 4);
  Serial.print(F(" rev ("));
  Serial.print(angle_deg, 1);
  Serial.println(F(" deg)"));
  
  moveToEncoderPosition(current_ext1, pos2, max_speed);  // Pass velocity limit
}

// REMOVED: parseXBeeCommand() - not used (binary protocol only)
// REMOVED: updateXBeeControl() - continuous control disabled
