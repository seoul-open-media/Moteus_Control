#include "xbee.h"
#include "motor_control.h"
#include "display.h"
#include "config.h"
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
  if (Serial4.available()) {
    // Look for packet header (0xFF = 255)
    uint8_t header = Serial4.read();
    
    if (header == 0xFF) {
      // Wait for robot_id byte
      unsigned long startTime = millis();
      while (!Serial4.available() && (millis() - startTime < 100)) {}
      
      if (Serial4.available()) {
        uint8_t robot_id = Serial4.read();
        
        // Check if command is for this robot (or broadcast to all robots with ID 0)
        if (robot_id != ROBOT_ID && robot_id != 0) {
          // Command not for this robot, ignore it
          return;
        }
        
        // Wait for command byte
        while (!Serial4.available() && (millis() - startTime < 100)) {}
        if (!Serial4.available()) return;
        
        uint8_t cmd = Serial4.read();
        
        // Parse binary commands
        if (cmd == 0x01 || cmd == 0x02) { // Move Motor (1 or 2)
          // Read 3 bytes: position_MSB, position_LSB, velocity
          while (Serial4.available() < 3 && (millis() - startTime < 100)) {}
          if (Serial4.available() >= 3) {
            int16_t rawPos = (Serial4.read() << 8) | Serial4.read();
            uint8_t rawVel = Serial4.read();
            
            // Convert position
            float target = rawPos / 65535.0;
            target += 0.5;  // Add 180 degree offset
            if (target > 0.5) target -= 1.0;  // Wrap around
            target = constrain(target, MIN_POSITION, MAX_POSITION);
            
            // Convert velocity (0-255 maps to 1-500 rev/s)
            float velocity = 1.0 + (rawVel / 255.0) * 499.0;
            
            // Mark timestamp for stall detection
            lastXBeeCommand = millis();
            
            // Update motor target and velocity
            if (cmd == 0x01) {
              xbeeTargetM1 = target;
              xbeeMaxVelM1 = velocity;
              m1_settled = false;
              Serial.print(F("Robot"));
              Serial.print(ROBOT_ID);
              Serial.print(F(" M1: pos="));
              Serial.print(target, 3);
              Serial.print(F(" vel="));
              Serial.println(velocity, 1);
            } else {
              xbeeTargetM2 = target;
              xbeeMaxVelM2 = velocity;
              m2_settled = false;
              Serial.print(F("Robot"));
              Serial.print(ROBOT_ID);
              Serial.print(F(" M2: pos="));
              Serial.print(target, 3);
              Serial.print(F(" vel="));
              Serial.println(velocity, 1);
            }
            
            xbeeControlActive = true;
          }
        }
        else if (cmd == 0x04) { // STOP/KILL
          Serial.println(F("XBee: Emergency STOP"));
          displayDebug("XBee STOP!");
          
          moteus1.SetStop();
          moteus2.SetStop();
          
          // Disable XBee control loop
          xbeeControlActive = false;
          m1_settled = false;
          m2_settled = false;
          
          interruptCommand = true;
          commandRunning = false;
        }
        else if (cmd == 0x05) { // STATUS
          Moteus::Query::Format query_fmt;
          query_fmt.abs_position = Moteus::kFloat;
          query_fmt.motor_temperature = Moteus::kFloat;
          
          moteus1.SetQuery(&query_fmt);
          moteus2.SetQuery(&query_fmt);
          delay(10);
          
          const auto& v1 = moteus1.last_result().values;
          const auto& v2 = moteus2.last_result().values;
          
          // Send status back as binary: FF 05 [M1 pos 4 bytes] [M1 temp 2 bytes] [M2 pos 4 bytes] [M2 temp 2 bytes]
          Serial4.write(0xFF);
          Serial4.write(0x05);
          Serial4.write((uint8_t*)&v1.abs_position, 4);
          int16_t temp1 = (int16_t)(v1.motor_temperature * 10);
          Serial4.write((uint8_t*)&temp1, 2);
          Serial4.write((uint8_t*)&v2.abs_position, 4);
          int16_t temp2 = (int16_t)(v2.motor_temperature * 10);
          Serial4.write((uint8_t*)&temp2, 2);
          
          Serial.print(F("XBee: STATUS sent M1:"));
          Serial.print(v1.abs_position, 4);
          Serial.print(F(" M2:"));
          Serial.println(v2.abs_position, 4);
        }
        else {
          Serial.print(F("XBee: Unknown command: 0x"));
          Serial.println(cmd, HEX);
        }
      }
    }
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
  if (millis() - lastUpdate < 10) return;  // 100Hz update rate
  lastUpdate = millis();
  
  // Query current positions and velocities
  Moteus::Query::Format query_fmt;
  query_fmt.abs_position = Moteus::kFloat;
  query_fmt.velocity = Moteus::kFloat;
  moteus1.SetQuery(&query_fmt);
  moteus2.SetQuery(&query_fmt);
  delay(5);
  
  float currentM1 = moteus1.last_result().values.abs_position;
  float currentM2 = moteus2.last_result().values.abs_position;
  float velM1 = moteus1.last_result().values.velocity;
  float velM2 = moteus2.last_result().values.velocity;
  
  // Calculate errors
  float error1 = xbeeTargetM1 - currentM1;
  float error2 = xbeeTargetM2 - currentM2;
  
  
  // Handle wrap-around
  if (error1 > 0.5) error1 -= 1.0;
  else if (error1 < -0.5) error1 += 1.0;
  if (error2 > 0.5) error2 -= 1.0;
  else if (error2 < -0.5) error2 += 1.0;
  
  const float deadband = 0.01;  // Small deadband for responsive tracking (~3.6 degrees)
  const float brake_zone = 0.03;  // Small brake zone - switch to gentle control sooner
  
  // Use per-motor velocity limits
  float max_vel_m1 = xbeeMaxVelM1;
  float max_vel_m2 = xbeeMaxVelM2;
  
  // Scale PD gains with velocity (base gains for 400 rev/s)
  // Use square root scaling for P gain, but keep D gain higher at low speeds
  float vel_scale_m1 = sqrt(max_vel_m1 / 400.0);
  float vel_scale_m2 = sqrt(max_vel_m2 / 400.0);
  
  // At slow speeds, use higher damping to prevent overshoot
  // Minimum Kd = 15 even at slowest speeds
  float Kp_far_m1 = 2400.0 * vel_scale_m1;
  float Kp_near_m1 = 800.0 * vel_scale_m1;
  float Kd_m1 = max(15.0, 10.0 * vel_scale_m1);
  
  float Kp_far_m2 = 2400.0 * vel_scale_m2;
  float Kp_near_m2 = 800.0 * vel_scale_m2;
  float Kd_m2 = max(15.0, 10.0 * vel_scale_m2);
  
  // Calculate velocities with PD control
  float vel1 = 0, vel2 = 0;
  
  // Motor 1 control
  bool m1_active = abs(error1) > deadband;
  bool m2_active = abs(error2) > deadband;
  
  // Check if motors have settled
  if (!m1_active && !m1_settled) {
    m1_settled = true;
  } else if (m1_active && m1_settled) {
    m1_settled = false;  // Left deadband, no longer settled
  }
  
  if (!m2_active && !m2_settled) {
    m2_settled = true;
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
  
  // Always send commands to keep motors engaged
  // When settled, just send velocity=0 to hold in place
  // Scale acceleration with velocity: accel = velocity * 0.3
  Moteus::PositionMode::Command cmd1;
  cmd1.position = NaN;
  cmd1.velocity = m1_settled ? 0.0 : vel1;
  cmd1.velocity_limit = max_vel_m1;
  cmd1.accel_limit = max_vel_m1 * 0.3;  // Acceleration scales with velocity
  
  Moteus::PositionMode::Command cmd2;
  cmd2.position = NaN;
  cmd2.velocity = m2_settled ? 0.0 : vel2;
  cmd2.velocity_limit = max_vel_m2;
  cmd2.accel_limit = max_vel_m2 * 0.3;  // Acceleration scales with velocity
  
  moteus1.SetPosition(cmd1, &position_fmt);
  moteus2.SetPosition(cmd2, &position_fmt);
}
