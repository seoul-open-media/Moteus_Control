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

// XBee control state
bool xbeeControlActive = false;
float xbeeTargetM1 = 0.0;
float xbeeTargetM2 = 0.0;
bool m1_settled = false;
bool m2_settled = false;

// Cache motor values for display (updated each control loop)
float cachedM1Pos = 0.0;
float cachedM2Pos = 0.0;
float cachedM1Temp = 0.0;
float cachedM2Temp = 0.0;

// Position constraints: -90 to +90 degrees = -0.25 to +0.25 revolutions
const float MIN_POSITION = -0.25;
const float MAX_POSITION = 0.25;

// Motor velocity limits (adjustable via XBee)
float xbeeMaxVelM1 = 400.0;  // Default max velocity for M1
float xbeeMaxVelM2 = 400.0;  // Default max velocity for M2

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
  // Check if data is available from XBee
  // Search for header bytes (0xFF 0xFF) in the stream
  static unsigned long lastDebugTime = 0;
  if (Serial4.available() > 0 && (millis() - lastDebugTime > 500)) {
    Serial.print(F("[XBee] Available bytes: "));
    Serial.println(Serial4.available());
    lastDebugTime = millis();
  }
  
  // Wait for at least 2 bytes to check header
  while (Serial4.available() >= 2) {
      // Look for first header byte
      uint8_t byte1 = Serial4.read();
      if (byte1 != 0xFF) {
        continue;  // Keep searching
      }
      
      // Second byte is global control byte
      uint8_t controlByte = Serial4.read();
      
      Serial.print(F("[XBee] Header found! Control byte: "));
      Serial.println(controlByte);
      
      // Handle global control commands
      if (controlByte == 1) {
        // Stop
        Serial.println(F("[XBee] STOP!"));
        moteus1.SetStop();
        moteus2.SetStop();
        xbeeControlActive = false;
        m1_settled = false;
        m2_settled = false;
        // Don't show debug message - status is shown in main display as DISENGAGED
        continue;  // Don't process robot data
      }
      
      // Any other control byte (0, 0xFF, etc.): Normal operation, continue processing
      Serial.println(F("[XBee] Normal operation mode"));
      
      // Now wait for the remaining 26 bytes
      unsigned long startWait = millis();
      while (Serial4.available() < 26 && (millis() - startWait < 100)) {
        delay(1);  // Wait a bit for data
      }
      
      if (Serial4.available() < 26) {
        Serial.println(F("[XBee] Timeout waiting for data"));
        return;
      }

      // Read 13 robots' MSB/LSB
      uint8_t robot_data[26];
      for (int i = 0; i < 26; ++i) {
        robot_data[i] = Serial4.read();
      }

      // Broadcast: if all bytes are 0xFF, all robots should act
      bool broadcast = true;
      for (int i = 0; i < 26; ++i) {
        if (robot_data[i] != 0xFF) {
          broadcast = false;
          break;
        }
      }

      // Robot IDs are 1-13, index 0-12
      int idx = ROBOT_ID - 1;
      if ((ROBOT_ID >= 1 && ROBOT_ID <= 13) && (!broadcast)) {
        uint8_t msb = robot_data[idx * 2];
        uint8_t lsb = robot_data[idx * 2 + 1];
        Serial.print(F("Robot "));
        Serial.print(ROBOT_ID);
        Serial.print(F(" MSB: "));
        Serial.print(msb);
        Serial.print(F(" LSB: "));
        Serial.println(lsb);

        uint16_t value = ((uint16_t)msb << 8) | lsb;
        uint8_t digit1 = (value / 1000) % 10; // velocity
        uint8_t digit2 = (value / 100) % 10;  // motor 1 position
        uint8_t digit3 = (value / 10) % 10;   // motor 2 position
        uint8_t digit4 = value % 10;          // electromagnet/solenoid control

        Serial.print(F("Parsed for Robot "));
        Serial.print(ROBOT_ID);
        Serial.print(F(" value: "));
        Serial.print(value);
        Serial.print(F(" digits: v="));
        Serial.print(digit1);
        Serial.print(F(" m1="));
        Serial.print(digit2);
        Serial.print(F(" m2="));
        Serial.print(digit3);
        Serial.print(F(" d4="));
        Serial.print(digit4);
        Serial.println();

        // Map velocity: digit1 0-9 with custom mapping (higher values to prevent stalling)
        // 0→0, 1→20, 2→30, 3→40, 4→50, 5→60, 6→70, 7→80, 8→90, 9→100
        const float vel_map[10] = {0, 20, 30, 40, 50, 60, 70, 80, 90, 100};
        float velocity = (digit1 <= 9) ? vel_map[digit1] : 100.0;
        
        // Map position: 0=stay, 1-4=negative range, 5=center, 6-9=positive range
        float pos1, pos2;
        if (digit2 == 0) {
          pos1 = xbeeTargetM1;  // Keep current target
        } else if (digit2 >= 1 && digit2 <= 4) {
          pos1 = -0.25 + (digit2 - 1) * (0.25 / 3.0);  // Map 1-4 to -0.25 to 0
        } else if (digit2 == 5) {
          pos1 = 0.0;  // Center
        } else {  // 6-9
          pos1 = (digit2 - 6) * (0.25 / 3.0);  // Map 6-9 to 0 to 0.25
        }
        
        if (digit3 == 0) {
          pos2 = xbeeTargetM2;  // Keep current target
        } else if (digit3 >= 1 && digit3 <= 4) {
          pos2 = -0.25 + (digit3 - 1) * (0.25 / 3.0);  // Map 1-4 to -0.25 to 0
        } else if (digit3 == 5) {
          pos2 = 0.0;  // Center
        } else {  // 6-9
          pos2 = (digit3 - 6) * (0.25 / 3.0);  // Map 6-9 to 0 to 0.25
        }
        
        // Clamp positions to safe range
        pos1 = constrain(pos1, -0.25, 0.25);
        pos2 = constrain(pos2, -0.25, 0.25);
        
        Serial.print(F("[Mapping] vel="));
        Serial.print(velocity, 1);
        Serial.print(F(" d2="));
        Serial.print(digit2);
        Serial.print(F(" -> pos1="));
        Serial.print(pos1, 4);
        Serial.print(F(" d3="));
        Serial.print(digit3);
        Serial.print(F(" -> pos2="));
        Serial.println(pos2, 4);

        lastXBeeCommand = millis();
        xbeeTargetM1 = pos1;
        xbeeTargetM2 = pos2;
        xbeeMaxVelM1 = velocity;
        xbeeMaxVelM2 = velocity;
        m1_settled = false;
        m2_settled = false;
        xbeeControlActive = true;
        
        Serial.print(F("[XBee] Motor targets: M1="));
        Serial.print(pos1, 3);
        Serial.print(F(" M2="));
        Serial.print(pos2, 3);
        Serial.print(F(" vel="));
        Serial.println(velocity, 1);
        
        // Handle digit4: electromagnet/solenoid control based on ROBOT_TYPE
        if (ROBOT_TYPE == 'A') {
          // Type A behavior
          if (digit4 == 1) {
            triggerTypeA_1();  // EM1, EM2, EM3
          } else if (digit4 == 2) {
            triggerTypeA_2();  // Solenoid
          } else if (digit4 == 3) {
            triggerTypeA_3();  // All
          }
        } else if (ROBOT_TYPE == 'B') {
          // Type B behavior
          if (digit4 == 1) {
            triggerTypeB_1();  // EM1, EM2
          } else if (digit4 == 2) {
            triggerTypeB_2();  // Solenoid, EM3
          } else if (digit4 == 3) {
            triggerTypeB_3();  // All
          }
        }
      } else if (broadcast) {
        // Example: broadcast command, all robots use the same value
        uint8_t msb = robot_data[0];
        uint8_t lsb = robot_data[1];
        uint16_t value = ((uint16_t)msb << 8) | lsb;
        uint8_t digit1 = (value / 1000) % 10; // velocity
        uint8_t digit2 = (value / 100) % 10;  // motor 1 position
        uint8_t digit3 = (value / 10) % 10;   // motor 2 position
        // uint8_t digit4 = value % 10;          // reserved

        // Map velocity: digit1 0-9 with custom mapping (higher values to prevent stalling)
        // 0→0, 1→20, 2→30, 3→40, 4→50, 5→60, 6→70, 7→80, 8→90, 9→100
        const float vel_map[10] = {0, 20, 30, 40, 50, 60, 70, 80, 90, 100};
        float velocity = (digit1 <= 9) ? vel_map[digit1] : 100.0;
        
        // Map position: 0=stay, 1-4=negative range, 5=center, 6-9=positive range
        float pos1, pos2;
        if (digit2 == 0) {
          pos1 = xbeeTargetM1;  // Keep current target
        } else if (digit2 >= 1 && digit2 <= 4) {
          pos1 = -0.25 + (digit2 - 1) * (0.25 / 3.0);  // Map 1-4 to -0.25 to 0
        } else if (digit2 == 5) {
          pos1 = 0.0;  // Center
        } else {  // 6-9
          pos1 = (digit2 - 6) * (0.25 / 3.0);  // Map 6-9 to 0 to 0.25
        }
        
        if (digit3 == 0) {
          pos2 = xbeeTargetM2;  // Keep current target
        } else if (digit3 >= 1 && digit3 <= 4) {
          pos2 = -0.25 + (digit3 - 1) * (0.25 / 3.0);  // Map 1-4 to -0.25 to 0
        } else if (digit3 == 5) {
          pos2 = 0.0;  // Center
        } else {  // 6-9
          pos2 = (digit3 - 6) * (0.25 / 3.0);  // Map 6-9 to 0 to 0.25
        }
        
        // Clamp positions to safe range
        pos1 = constrain(pos1, -0.25, 0.25);
        pos2 = constrain(pos2, -0.25, 0.25);
        
        Serial.print(F("[Broadcast Mapping] vel="));
        Serial.print(velocity, 1);
        Serial.print(F(" d2="));
        Serial.print(digit2);
        Serial.print(F(" -> pos1="));
        Serial.print(pos1, 4);
        Serial.print(F(" d3="));
        Serial.print(digit3);
        Serial.print(F(" -> pos2="));
        Serial.println(pos2, 4);

        lastXBeeCommand = millis();
        xbeeTargetM1 = pos1;
        xbeeTargetM2 = pos2;
        xbeeMaxVelM1 = velocity;
        xbeeMaxVelM2 = velocity;
        m1_settled = false;
        m2_settled = false;
        xbeeControlActive = true;
      }
      
      // Successfully processed a complete message
      break;
  }
}

bool parseXBeeCommand(String command) {
  command.trim();
  
  // Echo command back to XBee (for Pure Data confirmation)
  Serial4.print(F("ACK: "));
  Serial4.println(command);
  
  // Parse different command formats from Pure Data
  
  // Format 1: Simple position commands "M1 0.5" or "M2 -0.25"
  if (command.startsWith("M1 ")) {
    float target = command.substring(3).toFloat();
    Serial.print(F("XBee: M1 -> "));
    Serial.println(target, 4);
    
    char buf[32];
    snprintf(buf, sizeof(buf), "XB:M1->%.2f", target);
    displayDebug(buf);
    
    // Query current position of M2
    Moteus::Query::Format query_fmt;
    query_fmt.abs_position = Moteus::kFloat;
    moteus2.SetQuery(&query_fmt);
    delay(10);
    double current_ext2 = moteus2.last_result().values.abs_position;
    
    moveToEncoderPosition(target, current_ext2);
    return true;
  }
  
  if (command.startsWith("M2 ")) {
    float target = command.substring(3).toFloat();
    Serial.print(F("XBee: M2 -> "));
    Serial.println(target, 4);
    
    char buf[32];
    snprintf(buf, sizeof(buf), "XB:M2->%.2f", target);
    displayDebug(buf);
    
    // Query current position of M1
    Moteus::Query::Format query_fmt;
    query_fmt.abs_position = Moteus::kFloat;
    moteus1.SetQuery(&query_fmt);
    delay(10);
    double current_ext1 = moteus1.last_result().values.abs_position;
    
    moveToEncoderPosition(current_ext1, target);
    return true;
  }
  
  // Format 2: Both motors "BOTH 0.5 0.75" or "B 0.0 0.25"
  if (command.startsWith("BOTH ") || command.startsWith("B ")) {
    int spaceIndex = command.indexOf(' ');
    String rest = command.substring(spaceIndex + 1);
    
    int secondSpace = rest.indexOf(' ');
    if (secondSpace > 0) {
      float target1 = rest.substring(0, secondSpace).toFloat();
      float target2 = rest.substring(secondSpace + 1).toFloat();
      
      Serial.print(F("XBee: Both M1->"));
      Serial.print(target1, 4);
      Serial.print(F(" M2->"));
      Serial.println(target2, 4);
      
      char buf[32];
      snprintf(buf, sizeof(buf), "XB:%.2f,%.2f", target1, target2);
      displayDebug(buf);
      
      moveToEncoderPosition(target1, target2);
      return true;
    }
  }
  
  // Format 3: Stop command
  if (command == "STOP" || command == "S") {
    Serial.println(F("XBee: Emergency STOP"));
    displayDebug("XBee STOP!");
    
    moteus1.SetStop();
    moteus2.SetStop();
    
    interruptCommand = true;
    commandRunning = false;
    
    return true;
  }
  
  // Format 4: Query status
  if (command == "STATUS" || command == "?") {
    Moteus::Query::Format query_fmt;
    query_fmt.abs_position = Moteus::kFloat;
    query_fmt.motor_temperature = Moteus::kFloat;
    
    moteus1.SetQuery(&query_fmt);
    moteus2.SetQuery(&query_fmt);
    delay(10);
    
    const auto& v1 = moteus1.last_result().values;
    const auto& v2 = moteus2.last_result().values;
    
    // Send status back to Pure Data
    Serial4.print(F("STATUS M1:"));
    Serial4.print(v1.abs_position, 4);
    Serial4.print(F(" T:"));
    Serial4.print(v1.motor_temperature, 1);
    Serial4.print(F(" M2:"));
    Serial4.print(v2.abs_position, 4);
    Serial4.print(F(" T:"));
    Serial4.println(v2.motor_temperature, 1);
    
    return true;
  }
  
  // Format 5: Oscillate command
  if (command == "OSC" || command == "OSCILLATE") {
    Serial.println(F("XBee: Starting oscillation"));
    displayDebug("XBee OSC");
    
    oscillateMotors();
    return true;
  }
  
  // Unknown command
  Serial.print(F("XBee: Unknown command: "));
  Serial.println(command);
  
  Serial4.print(F("ERROR: Unknown command: "));
  Serial4.println(command);
  
  return false;
}

// Continuous XBee control - call from main loop
void updateXBeeControl() {
  if (!xbeeControlActive) return;
  
  static unsigned long lastUpdate = 0;
  static unsigned long lastDebug = 0;
  if (millis() - lastUpdate < 10) return;  // 100Hz update rate
  lastUpdate = millis();
  
  bool shouldDebug = (millis() - lastDebug > 1000);
  
  // Query current positions and velocities
  Moteus::Query::Format query_fmt;
  query_fmt.mode = Moteus::kInt8;
  query_fmt.position = Moteus::kFloat;
  query_fmt.abs_position = Moteus::kFloat;
  query_fmt.velocity = Moteus::kFloat;
  query_fmt.motor_temperature = Moteus::kFloat;
  moteus1.SetQuery(&query_fmt);
  moteus2.SetQuery(&query_fmt);
  
  // Poll CAN bus for responses (more efficient than delay - allows other processing)
  unsigned long pollStart = micros();
  while (micros() - pollStart < 5000) {  // Max 5ms timeout
    // Process any pending CAN messages
    if (!isnan(moteus1.last_result().values.abs_position) && 
        !isnan(moteus2.last_result().values.abs_position)) {
      break;  // Got both responses
    }
  }
  
  float currentM1 = moteus1.last_result().values.abs_position;
  float currentM2 = moteus2.last_result().values.abs_position;
  float velM1 = moteus1.last_result().values.velocity;
  float velM2 = moteus2.last_result().values.velocity;
  
  // Cache values for display
  cachedM1Pos = currentM1;
  cachedM2Pos = currentM2;
  cachedM1Temp = moteus1.last_result().values.motor_temperature;
  cachedM2Temp = moteus2.last_result().values.motor_temperature;
  
  if (shouldDebug) {
    Serial.print(F("[Query] M1 pos="));
    Serial.print(currentM1, 3);
    Serial.print(F(" temp="));
    Serial.print(moteus1.last_result().values.motor_temperature, 1);
    Serial.print(F(" M2 pos="));
    Serial.print(currentM2, 3);
    Serial.print(F(" temp="));
    Serial.println(moteus2.last_result().values.motor_temperature, 1);
  }
  
  // Calculate errors
  float error1 = xbeeTargetM1 - currentM1;
  float error2 = xbeeTargetM2 - currentM2;
  
  // Handle wrap-around
  if (error1 > 0.5) error1 -= 1.0;
  else if (error1 < -0.5) error1 += 1.0;
  if (error2 > 0.5) error2 -= 1.0;
  else if (error2 < -0.5) error2 += 1.0;
  
  if (shouldDebug) {
    Serial.print(F("[Err] M1: err="));
    Serial.print(error1, 3);
    Serial.print(F(" M2: err="));
    Serial.println(error2, 3);
  }
  
  const float deadband = 0.01;  // Small deadband for responsive tracking (~3.6 degrees)
  const float brake_zone = 0.03;  // Small brake zone - switch to gentle control sooner
  
  // Use per-motor velocity limits
  float max_vel_m1 = xbeeMaxVelM1;
  float max_vel_m2 = xbeeMaxVelM2;
  
  if (shouldDebug) {
    Serial.print(F("[Vel] max_vel_m1="));
    Serial.print(max_vel_m1, 1);
    Serial.print(F(" max_vel_m2="));
    Serial.println(max_vel_m2, 1);
  }
  
  // Scale PD gains with velocity - use LINEAR scaling so velocity limit matters
  // Much lower P gains so velocity naturally scales with max_vel setting
  float vel_scale_m1 = max_vel_m1 / 400.0;  // Linear scaling
  float vel_scale_m2 = max_vel_m2 / 400.0;
  
  // Very low P gains: error of 0.25 rev (90°) should give ~0.5 × max_vel
  // This ensures velocity scales with velocity digit, not always saturated
  float Kp_far_m1 = max_vel_m1 * 2.0;   // Far: error of 0.5 rev (180°) -> max_vel
  float Kp_near_m1 = max_vel_m1 * 1.0;  // Near: error of 1.0 rev (360°) -> max_vel
  float Kd_m1 = max(2.0, 1.5 * vel_scale_m1);
  
  float Kp_far_m2 = max_vel_m2 * 2.0;
  float Kp_near_m2 = max_vel_m2 * 1.0;
  float Kd_m2 = max(2.0, 1.5 * vel_scale_m2);
  
  // Calculate velocities with PD control
  float vel1 = 0, vel2 = 0;
  
  // Motor 1 control
  bool m1_active = abs(error1) > deadband;
  bool m2_active = abs(error2) > deadband;
  
  // Check if motors have settled
  if (!m1_active && !m1_settled) {
    m1_settled = true;
    Serial.println(F("M1 settled"));
  } else if (m1_active && m1_settled) {
    m1_settled = false;  // Left deadband, no longer settled
  }
  
  if (!m2_active && !m2_settled) {
    m2_settled = true;
    Serial.println(F("M2 settled"));
  } else if (m2_active && m2_settled) {
    m2_settled = false;
  }
  
  // Calculate velocities for active motors
  if (m1_active) {
    if (abs(error1) > brake_zone) {
      // Far from target - fast approach
      vel1 = error1 * Kp_far_m1 - velM1 * Kd_m1;
    } else {
      // Close to target - gentle approach
      vel1 = error1 * Kp_near_m1 - velM1 * (Kd_m1 * 1.5);
    }
    vel1 = constrain(vel1, -max_vel_m1, max_vel_m1);
  }
  
  // Motor 2 control
  if (m2_active) {
    if (abs(error2) > brake_zone) {
      vel2 = error2 * Kp_far_m2 - velM2 * Kd_m2;
    } else {
      vel2 = error2 * Kp_near_m2 - velM2 * (Kd_m2 * 1.5);
    }
    vel2 = constrain(vel2, -max_vel_m2, max_vel_m2);
  }
  
  if (shouldDebug) {
    Serial.print(F("[Calc] vel1="));
    Serial.print(vel1, 2);
    Serial.print(F(" (lim="));
    Serial.print(max_vel_m1, 0);
    Serial.print(F(") vel2="));
    Serial.print(vel2, 2);
    Serial.print(F(" (lim="));
    Serial.print(max_vel_m2, 0);
    Serial.println(F(")"));
    lastDebug = millis();
  }
  
  // Always send commands to keep motors engaged
  // When settled, just send velocity=0 to hold in place
  // Use HIGH CONSTANT acceleration so motors reach target velocity quickly
  Moteus::PositionMode::Command cmd1;
  cmd1.position = NaN;
  cmd1.velocity = m1_settled ? 0.0 : vel1;
  cmd1.velocity_limit = max_vel_m1;
  cmd1.accel_limit = 800.0;  // High constant acceleration - same for all velocities
  
  Moteus::PositionMode::Command cmd2;
  cmd2.position = NaN;
  cmd2.velocity = m2_settled ? 0.0 : vel2;
  cmd2.velocity_limit = max_vel_m2;
  cmd2.accel_limit = 800.0;  // High constant acceleration - same for all velocities
  
  moteus1.SetPosition(cmd1, &position_fmt);
  moteus2.SetPosition(cmd2, &position_fmt);
}
