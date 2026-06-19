#include "xbee.h"
#include "motor_control.h"
#include "display.h"
#include "config.h"
#include "solenoid.h"
#include <Moteus.h>
using Moteus = MoteusController<ACAN2517FD>;

// External declarations from main.cpp
extern Moteus moteus1;
extern Moteus moteus2;
extern volatile bool interruptCommand;
extern volatile bool commandRunning;
extern String pendingCommand;

// XBee control state
bool  xbeeControlActive = false;
float xbeeTargetM1 = 0.0f;
float xbeeTargetM2 = 0.5f;  // ENCODER_CENTER = 0 deg
float xbeeMaxVelM1 = 10.0f;
float xbeeMaxVelM2 = 10.0f;
bool  m1_settled = false;
bool  m2_settled = false;
// Cached display values
float cachedM1Pos = 0.0f;
float cachedM2Pos = 0.0f;
float cachedM1Temp = 0.0f;
float cachedM2Temp = 0.0f;

// Position constraints: -90 to +90 degrees = -0.25 to +0.25 revolutions
const float MIN_POSITION = -0.25;
const float MAX_POSITION = 0.25;

// REMOVED: Unused motor velocity limits (xbeeMaxVelM1, xbeeMaxVelM2)

void initXBee() {
  // [XBee DISABLED] Serial4 XBee communication commented out
  // Serial4.begin(XBEE_BAUD);  // XBee on Serial4
  // Serial.println(F("XBee initialized on Serial4"));
  
  // Wired serial uses Serial (USB, 115200) already initialized in main.cpp
  Serial.println(F("Wired serial control active (Serial USB, 115200)"));
  Serial.print(F("Robot ID: "));
  Serial.println(ROBOT_ID);
  
  char buf[32];
  snprintf(buf, sizeof(buf), "Robot #%d", ROBOT_ID);
  displayDebug(buf);
}

// [XBee DISABLED] updateXBee() commented out — XBee wireless communication disabled
// void updateXBee() { ... }  // Used Serial4 at 115200 for XBee radio packets

// ——————————————————————————————————————————————————————————————————————————————
// Wired Serial Binary Protocol (replaces XBee)
// Protocol: [0xFF] [0xFF] [R1_MSB] [R1_LSB] ... [R13_MSB] [R13_LSB]
// Total: 2 header + 26 data bytes = 28 bytes
// Source: Pure Data (PD) over USB Serial at 115200 baud
// ——————————————————————————————————————————————————————————————————————————————
void updateWiredSerial() {
  // Only process binary packets — peek to avoid consuming text commands
  if (Serial.available() < 28 || Serial.peek() != 0xFF) {
    return;  // Not enough data or not a binary packet header
  }

  // Consume first header byte (0xFF already confirmed by peek)
  Serial.read();  // discard 0xFF byte1

  // Check second header byte (0xFF)
  uint8_t byte2 = Serial.read();
  if (byte2 != 0xFF) {
    return;  // Not a valid header
  }

  Serial.println(F("[SER] Header 0xFF 0xFF found!"));
  xbeeControlActive = true;  // Mark as active (reuses existing flag)

  // Read 26 bytes for 13 robots (2 bytes each)
  uint8_t robot_data[26];
  for (int i = 0; i < 26; i++) {
    robot_data[i] = Serial.read();
  }

  // Get data for this robot (ROBOT_ID 1-13 → index 0-12)
  if (ROBOT_ID < 1 || ROBOT_ID > 13) {
    Serial.print(F("[SER] Invalid ROBOT_ID: "));
    Serial.println(ROBOT_ID);
    return;
  }

  int idx = (ROBOT_ID - 1) * 2;  // Each robot has 2 bytes
  uint8_t msb = robot_data[idx];
  uint8_t lsb = robot_data[idx + 1];

  Serial.print(F("[SER] Robot "));
  Serial.print(ROBOT_ID);
  Serial.print(F(" MSB="));
  Serial.print(msb);
  Serial.print(F(" LSB="));
  Serial.println(lsb);

  // Combine MSB and LSB to get decimal value (0-65535)
  uint16_t value = ((uint16_t)msb << 8) | lsb;

  if (value == 0) {
    return;  // Ignore if value is 0
  }

  // Extract 4 digits
  uint8_t digit1 = (value / 1000) % 10;  // Max speed (0-9)
  uint8_t digit2 = (value / 100) % 10;   // Ignored
  uint8_t digit3 = (value / 10) % 10;    // Angle control (0-9)
  uint8_t digit4 = value % 10;           // Solenoid control

  // Display 4-digit number on screen and serial
  char displayBuf[32];
  snprintf(displayBuf, sizeof(displayBuf), "SER:%04d (%d%d%d%d)", value, digit1, digit2, digit3, digit4);
  displayDebug(displayBuf);

  Serial.println(F("========================================"));
  Serial.print(F("[SER] 4-DIGIT VALUE: "));
  Serial.print(value);
  Serial.print(F(" = "));
  Serial.print(digit1);
  Serial.print(digit2);
  Serial.print(digit3);
  Serial.println(digit4);
  Serial.print(F("[SER] Digit1(speed)="));
  Serial.print(digit1);
  Serial.print(F(" Digit2(ignore)="));
  Serial.print(digit2);
  Serial.print(F(" Digit3(angle)="));
  Serial.print(digit3);
  Serial.print(F(" Digit4(solenoid)="));
  Serial.println(digit4);
  Serial.println(F("========================================"));

  // Process digit1: Max speed (D1=0 -> 0.3 rev/s, D1=9 -> MAX_VELOCITY_FAST)
  const float MIN_VELOCITY = 0.3f;
  float max_speed = MIN_VELOCITY + ((float)digit1 / 9.0f) * (MAX_VELOCITY_FAST - MIN_VELOCITY);
  if (max_speed < MIN_VELOCITY) max_speed = MIN_VELOCITY;

  Serial.print(F("[SER] Max speed: "));
  Serial.print(max_speed, 2);
  Serial.println(F(" rev/s"));

  // Map digit3 (0-9) to angle (-90 to +90 degrees)
  float angle_deg = ((float)digit3 / 9.0) * 180.0 - 90.0;
  float pos2_logical = angle_deg / 360.0;  // Convert to revolutions

  // Clamp to limits (-0.25 to +0.25 rev)
  if (pos2_logical < MIN_POSITION) pos2_logical = MIN_POSITION;
  if (pos2_logical > MAX_POSITION) pos2_logical = MAX_POSITION;

  // Apply center offset + calibration offset (physical target)
  float pos2 = ENCODER_CENTER + pos2_logical + MOTOR_OFFSET;

  Serial.print(F("[SER] Motor 2: "));
  Serial.print(angle_deg, 1);
  Serial.print(F(" deg => "));
  Serial.print(pos2, 4);
  Serial.println(F(" rev (physical)"));

  // Process digit4: Solenoid/Electromagnet control
  // 1 = EM1+EM2+EM3, 2 = Solenoid
  if (digit4 == 1) {
    triggerElectromagnets();
    Serial.println(F("[SER] Electromagnets triggered"));
  } else if (digit4 == 2) {
    triggerTypeA_2();  // Solenoid only
    Serial.println(F("[SER] Solenoid triggered"));
  }

  // Query current position of M1
  Moteus::Query::Format query_fmt;
  query_fmt.abs_position = Moteus::kFloat;
  moteus1.SetQuery(&query_fmt);
  delay(5);
  double current_ext1 = moteus1.last_result().values.abs_position;

  // Move M2 to target position
  moveToEncoderPosition(current_ext1, pos2, max_speed);
}

// REMOVED: parseXBeeCommand() - not used (binary protocol only)
void updateXBeeControl() {}
