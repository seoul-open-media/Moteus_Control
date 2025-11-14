#include "xbee.h"
#include "motor_control.h"
#include "display.h"
#include <Moteus.h>

// External declarations from main.cpp
extern Moteus moteus1;
extern Moteus moteus2;
extern volatile bool interruptCommand;
extern volatile bool commandRunning;
extern String pendingCommand;

void initXBee() {
  // Initialize Serial1 for XBee communication
  Serial1.begin(XBEE_BAUD);
  
  Serial.println(F("XBee initialized on Serial1"));
  Serial.print(F("Baud rate: "));
  Serial.println(XBEE_BAUD);
  
  displayDebug("XBee ready");
}

void updateXBee() {
  // Check if data is available from XBee
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      Serial.print(F("XBee received: "));
      Serial.println(command);
      
      // If a command is currently running, set interrupt flag
      if (commandRunning) {
        pendingCommand = command;
        interruptCommand = true;
        Serial.println(F("Interrupting current command via XBee"));
        displayDebug("XBee interrupt");
      } else {
        // Parse and execute command immediately
        parseXBeeCommand(command);
      }
    }
  }
}

bool parseXBeeCommand(String command) {
  command.trim();
  
  // Echo command back to XBee (for Pure Data confirmation)
  Serial1.print(F("ACK: "));
  Serial1.println(command);
  
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
    Serial1.print(F("STATUS M1:"));
    Serial1.print(v1.abs_position, 4);
    Serial1.print(F(" T:"));
    Serial1.print(v1.motor_temperature, 1);
    Serial1.print(F(" M2:"));
    Serial1.print(v2.abs_position, 4);
    Serial1.print(F(" T:"));
    Serial1.println(v2.motor_temperature, 1);
    
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
  
  Serial1.print(F("ERROR: Unknown command: "));
  Serial1.println(command);
  
  return false;
}
