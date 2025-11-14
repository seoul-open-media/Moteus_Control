#include "motor_control.h"
#include "config.h"
#include "display.h"
#include "neokey.h"

// Global flag to interrupt current motor command
volatile bool interruptCommand = false;
volatile bool commandRunning = false;
String pendingCommand = "";

void printDiagnostics() {
  const auto& v1 = moteus1.last_result().values;
  const auto& v2 = moteus2.last_result().values;
  
  Serial.print(F("M1_abs="));
  Serial.print(v1.abs_position, 6);
  Serial.print(F(" temp="));
  Serial.print(v1.motor_temperature, 1);
  Serial.print(F("°C | M2_abs="));
  Serial.print(v2.abs_position, 6);
  Serial.print(F(" temp="));
  Serial.print(v2.motor_temperature, 1);
  Serial.println(F("°C"));
}

void checkMotorTemperature() {
  static unsigned long lastTempCheckTime = 0;
  unsigned long currentTime = millis();
  
  // Check temperature every 250ms
  if (currentTime - lastTempCheckTime < 250) {
    return;  // Not time to check yet
  }
  lastTempCheckTime = currentTime;
  
  const auto& v1 = moteus1.last_result().values;
  const auto& v2 = moteus2.last_result().values;
  
  const float TEMP_LIMIT = 60.0;  // Temperature limit in Celsius
  
  if (v1.motor_temperature > TEMP_LIMIT) {
    Serial.println(F(""));
    Serial.println(F("!!! CRITICAL: MOTOR 1 OVERHEATING !!!"));
    Serial.print(F("Temperature: "));
    Serial.print(v1.motor_temperature, 1);
    Serial.println(F("°C (Limit: 60°C)"));
    Serial.println(F("STOPPING ALL MOTORS AND HALTING..."));
    
    displayError("MOTOR 1 OVERHEAT!");
    
    moteus1.SetStop();
    moteus2.SetStop();
    
    while(true) {
      // Halt - infinite loop
      delay(1000);
      Serial.println(F("HALTED: Motor 1 overheated. Reset required."));
    }
  }
  
  if (v2.motor_temperature > TEMP_LIMIT) {
    Serial.println(F(""));
    Serial.println(F("!!! CRITICAL: MOTOR 2 OVERHEATING !!!"));
    Serial.print(F("Temperature: "));
    Serial.print(v2.motor_temperature, 1);
    Serial.println(F("°C (Limit: 60°C)"));
    Serial.println(F("STOPPING ALL MOTORS AND HALTING..."));
    
    displayError("MOTOR 2 OVERHEAT!");
    
    moteus1.SetStop();
    moteus2.SetStop();
    
    while(true) {
      // Halt - infinite loop
      delay(1000);
      Serial.println(F("HALTED: Motor 2 overheated. Reset required."));
    }
  }
}

void printDiagnosticsAll() {
  static unsigned long lastDiagnosticTime = 0;
  unsigned long currentTime = millis();
  
  // Run at 100ms intervals (10Hz)
  if (currentTime - lastDiagnosticTime >= 100) {
    lastDiagnosticTime = currentTime;
    
    // Query format for all available data
    Moteus::Query::Format query_fmt;
    query_fmt.mode = Moteus::kInt8;
    query_fmt.position = Moteus::kFloat;
    query_fmt.velocity = Moteus::kFloat;
    query_fmt.torque = Moteus::kFloat;
    query_fmt.q_current = Moteus::kFloat;
    query_fmt.d_current = Moteus::kFloat;
    query_fmt.abs_position = Moteus::kFloat;
    query_fmt.motor_temperature = Moteus::kFloat;
    query_fmt.voltage = Moteus::kFloat;
    query_fmt.temperature = Moteus::kFloat;
    query_fmt.fault = Moteus::kInt8;
    
    // Query both motors
    moteus1.SetQuery(&query_fmt);
    moteus2.SetQuery(&query_fmt);
    
    const auto& v1 = moteus1.last_result().values;
    const auto& v2 = moteus2.last_result().values;
    
    // Print Motor 1 data
    Serial.println(F("=== MOTOR 1 ==="));
    Serial.print(F("  Mode: "));
    Serial.println(static_cast<int>(v1.mode));
    Serial.print(F("  Position: "));
    Serial.print(v1.position, 4);
    Serial.println(F(" rev"));
    Serial.print(F("  Velocity: "));
    Serial.print(v1.velocity, 4);
    Serial.println(F(" rev/s"));
    Serial.print(F("  Torque: "));
    Serial.print(v1.torque, 4);
    Serial.println(F(" Nm"));
    Serial.print(F("  Q Current: "));
    Serial.print(v1.q_current, 3);
    Serial.println(F(" A"));
    Serial.print(F("  D Current: "));
    Serial.print(v1.d_current, 3);
    Serial.println(F(" A"));
    Serial.print(F("  Abs Position (Encoder): "));
    Serial.print(v1.abs_position, 6);
    Serial.println(F(" rev"));
    Serial.print(F("  Motor Temp: "));
    Serial.print(v1.motor_temperature, 2);
    Serial.println(F(" °C"));
    Serial.print(F("  Voltage: "));
    Serial.print(v1.voltage, 2);
    Serial.println(F(" V"));
    Serial.print(F("  Fault: "));
    Serial.println(static_cast<int>(v1.fault));
    
    // Print Motor 2 data
    Serial.println(F("=== MOTOR 2 ==="));
    Serial.print(F("  Mode: "));
    Serial.println(static_cast<int>(v2.mode));
    Serial.print(F("  Position: "));
    Serial.print(v2.position, 4);
    Serial.println(F(" rev"));
    Serial.print(F("  Velocity: "));
    Serial.print(v2.velocity, 4);
    Serial.println(F(" rev/s"));
    Serial.print(F("  Torque: "));
    Serial.print(v2.torque, 4);
    Serial.println(F(" Nm"));
    Serial.print(F("  Q Current: "));
    Serial.print(v2.q_current, 3);
    Serial.println(F(" A"));
    Serial.print(F("  D Current: "));
    Serial.print(v2.d_current, 3);
    Serial.println(F(" A"));
    Serial.print(F("  Abs Position (Encoder): "));
    Serial.print(v2.abs_position, 6);
    Serial.println(F(" rev"));
    Serial.print(F("  Motor Temp: "));
    Serial.print(v2.motor_temperature, 2);
    Serial.println(F(" °C"));
    Serial.print(F("  Voltage: "));
    Serial.print(v2.voltage, 2);
    Serial.println(F(" V"));
    Serial.print(F("  Fault: "));
    Serial.println(static_cast<int>(v2.fault));
    Serial.println();
  }
}

void oscillateMotors() {
  commandRunning = true;
  interruptCommand = false;
  
  Serial.println(F("=== Oscillation Mode (Velocity Control) ==="));
  Serial.println(F("Using external encoder feedback - Type 'stop' to exit"));
  displayDebug("Oscillate start");
  
  // Query format to get abs_position (external encoder)
  Moteus::Query::Format query_fmt;
  query_fmt.mode = Moteus::kInt8;
  query_fmt.position = Moteus::kFloat;
  query_fmt.velocity = Moteus::kFloat;
  query_fmt.abs_position = Moteus::kFloat;
  query_fmt.fault = Moteus::kInt8;
  
  // Set query format for both motors
  moteus1.SetQuery(&query_fmt);
  moteus2.SetQuery(&query_fmt);
  delay(50);
  
  // Get starting positions
  double start_ext1 = moteus1.last_result().values.abs_position;
  double start_ext2 = moteus2.last_result().values.abs_position;
  double start_mpos1 = moteus1.last_result().values.position;
  double start_mpos2 = moteus2.last_result().values.position;
  
  Serial.print(F("Start M1: mpos="));
  Serial.print(start_mpos1, 3);
  Serial.print(F(" ext="));
  Serial.print(start_ext1, 4);
  Serial.print(F(" | M2: mpos="));
  Serial.print(start_mpos2, 3);
  Serial.print(F(" ext="));
  Serial.println(start_ext2, 4);
  
  // Check if motors are in mode 11 (stuck at limits)
  if ((int)moteus1.last_result().values.mode == 11 || 
      (int)moteus2.last_result().values.mode == 11) {
    Serial.println(F("*** ERROR: Motors in Mode 11 (stuck at limits) ***"));
    Serial.println(F("Manually reposition motors away from limits first"));
    commandRunning = false;
    return;
  }
  
  int loop_count = 0;
  unsigned long start_time = millis();
  
  Serial.println(F("Using continuous sine wave velocity"));
  
  while (true) {
    // Check NeoKey for button presses (especially STOP button)
    static unsigned long lastNeoKeyCheck = 0;
    unsigned long current_time = millis();
    if (current_time - lastNeoKeyCheck >= 50) {  // Check every 50ms
      uint8_t buttons = neokey.read();
      // Check if STOP button (key 3) is pressed
      if (buttons & (1 << KEY_STOP)) {
        Serial.println(F("STOP button pressed during oscillation!"));
        displayDebug("K: STOP!");
        setKeyColor(KEY_STOP, COLOR_RED);
        interruptCommand = true;
        lastNeoKeyCheck = current_time;
      }
      lastNeoKeyCheck = current_time;
    }
    
    // Check for interrupt command from serial input
    if (Serial.available()) {
      pendingCommand = Serial.readStringUntil('\n');
      pendingCommand.trim();
      interruptCommand = true;
    }
    
    // Check for interrupt command
    if (interruptCommand) {
      Serial.println(F("Oscillation interrupted!"));
      displayDebug("Interrupted!");
      commandRunning = false;
      interruptCommand = false;
      moteus1.SetStop();
      moteus2.SetStop();
      
      // Reset STOP key LED if it was the stop button
      setKeyColor(KEY_STOP, COLOR_DIM_RED);
      return;
    }
    
    // Use sine wave velocity - motors oscillate continuously
    const auto time = millis() - start_time;
    const double period_ms = 4000.0;  // 4 second period
    const double max_velocity = 0.5;  // Maximum velocity in rev/s
    
    // Calculate velocity using sine wave (opposite phases for M1 and M2)
    double velocity1 = max_velocity * sin(2.0 * M_PI * time / period_ms);
    double velocity2 = max_velocity * sin(2.0 * M_PI * time / period_ms + M_PI);
    
    // Send velocity commands
    Moteus::PositionMode::Command pos_cmd1;
    pos_cmd1.position = NaN;
    pos_cmd1.velocity = velocity1 * MOTOR1_DIRECTION;
    
    Moteus::PositionMode::Command pos_cmd2;
    pos_cmd2.position = NaN;
    pos_cmd2.velocity = velocity2 * MOTOR2_DIRECTION;
    
    moteus1.SetPosition(pos_cmd1, &position_fmt);
    moteus2.SetPosition(pos_cmd2, &position_fmt);
    
    loop_count++;
    if (loop_count % 50 == 0) {
      const auto& r1 = moteus1.last_result().values;
      const auto& r2 = moteus2.last_result().values;
      
      Serial.print(F("time="));
      Serial.print(time);
      Serial.print(F(" | M1: mpos="));
      Serial.print(r1.position, 2);
      Serial.print(F(" vel="));
      Serial.print(velocity1, 2);
      Serial.print(F(" | M2: mpos="));
      Serial.print(r2.position, 2);
      Serial.print(F(" vel="));
      Serial.println(velocity2, 2);
    }
    
    delay(10);  // 100Hz control loop
  }
}

void moveToEncoderPosition(double target_ext1, double target_ext2) {
  commandRunning = true;
  interruptCommand = false;
  
  Serial.println(F("=== Move to Encoder Position ==="));
  
  char buf[32];
  snprintf(buf, sizeof(buf), "Tgt: %.2f,%.2f", target_ext1, target_ext2);
  displayDebug(buf);
  
  // Encoder stall detection - track last 4 readings
  double last_ext1[4] = {-999, -999, -999, -999};
  double last_ext2[4] = {-999, -999, -999, -999};
  int reading_index = 0;
  
  // Query format to get abs_position (external encoder)
  Moteus::Query::Format query_fmt;
  query_fmt.mode = Moteus::kInt8;
  query_fmt.position = Moteus::kFloat;
  query_fmt.velocity = Moteus::kFloat;
  query_fmt.abs_position = Moteus::kFloat;
  query_fmt.trajectory_complete = Moteus::kInt8;
  
  // Get current positions
  moteus1.SetQuery(&query_fmt);
  moteus2.SetQuery(&query_fmt);
  
  unsigned long query_start = millis();
  while (millis() - query_start < 10) {}
  
  double current_ext1 = moteus1.last_result().values.abs_position;
  double current_ext2 = moteus2.last_result().values.abs_position;
  
  Serial.print(F("M1: current_ext="));
  Serial.print(current_ext1, 4);
  Serial.print(F(" target_ext="));
  Serial.print(target_ext1, 4);
  Serial.print(F(" | M2: current_ext="));
  Serial.print(current_ext2, 4);
  Serial.print(F(" target_ext="));
  Serial.println(target_ext2, 4);
  
  // Calculate initial errors
  double ext_error1 = target_ext1 - current_ext1;
  double ext_error2 = target_ext2 - current_ext2;
  
  Serial.print(F("M1: ext_error="));
  Serial.print(ext_error1, 4);
  Serial.print(F(" M2: ext_error="));
  Serial.println(ext_error2, 4);
  
  int loop_count = 0;
  unsigned long start_time = millis();
  unsigned long last_loop_time = millis();
  
  bool motor1_done = (abs(ext_error1) < TOLERANCE);
  bool motor2_done = (abs(ext_error2) < TOLERANCE);
  
  // State tracking for stopping
  int motor1_stop_count = 0;
  int motor2_stop_count = 0;
  double prev_error1 = ext_error1;
  double prev_error2 = ext_error2;
  
  // Set high velocity and acceleration limits
  position_cmd1.velocity_limit = 20.0;
  position_cmd1.accel_limit = 10.0;
  position_cmd1.maximum_torque = 6.0;
  
  position_cmd2.velocity_limit = 20.0;
  position_cmd2.accel_limit = 10.0;
  position_cmd2.maximum_torque = 6.0;
  
  while (true) {
    unsigned long current_time = millis();
    
    // Check NeoKey for button presses (especially STOP button)
    static unsigned long lastNeoKeyCheck = 0;
    if (current_time - lastNeoKeyCheck >= 50) {  // Check every 50ms
      uint8_t buttons = neokey.read();
      // Check if STOP button (key 3) is pressed
      if (buttons & (1 << KEY_STOP)) {
        Serial.println(F("STOP button pressed during movement!"));
        displayDebug("K: STOP!");
        setKeyColor(KEY_STOP, COLOR_RED);
        interruptCommand = true;
        lastNeoKeyCheck = current_time;
      }
      lastNeoKeyCheck = current_time;
    }
    
    // Check for interrupt command from serial input
    if (Serial.available()) {
      pendingCommand = Serial.readStringUntil('\n');
      pendingCommand.trim();
      interruptCommand = true;
    }
    
    // Check for interrupt command
    if (interruptCommand) {
      Serial.println(F("Movement interrupted!"));
      displayDebug("Interrupted!");
      commandRunning = false;
      interruptCommand = false;
      moteus1.SetStop();
      moteus2.SetStop();
      
      // Reset STOP key LED if it was the stop button
      setKeyColor(KEY_STOP, COLOR_DIM_RED);
      return;
    }
    
    // Only execute control loop at specified rate
    if (current_time - last_loop_time >= LOOP_PERIOD_MS) {
      last_loop_time = current_time;
      
      // Get current encoder positions
      const bool got1 = moteus1.SetQuery(&query_fmt);
      const bool got2 = moteus2.SetQuery(&query_fmt);
      
      // Motor 1 control logic
      if (got1) {
        double current_ext1 = moteus1.last_result().values.abs_position;
        
        // Store reading for stall detection
        last_ext1[reading_index] = current_ext1;
        
        // Check if encoder is stuck (4 consecutive identical readings within epsilon)
        if (reading_index >= 3 && !motor1_done) {
          const double EPSILON = 1e-6;
          bool stuck1 = (abs(last_ext1[0] - last_ext1[1]) < EPSILON && 
                        abs(last_ext1[1] - last_ext1[2]) < EPSILON && 
                        abs(last_ext1[2] - last_ext1[3]) < EPSILON);
          if (stuck1 && abs(target_ext1 - current_ext1) > TOLERANCE) {
            Serial.println(F(""));
            Serial.println(F("!!! ENCODER STALL DETECTED - MOTOR 1 !!!"));
            Serial.print(F("Encoder stuck at: "));
            Serial.print(current_ext1, 6);
            Serial.println(F(" (4 consecutive identical readings)"));
            Serial.println(F("STOPPING ALL MOTORS FOR SAFETY..."));
            
            displayError("M1 ENCODER STUCK!");
            
            moteus1.SetStop();
            moteus2.SetStop();
            commandRunning = false;
            interruptCommand = false;
            
            // Reset STOP key LED
            setKeyColor(KEY_STOP, COLOR_DIM_RED);
            return;
          }
        }
        
        double error1 = target_ext1 - current_ext1;
        
        // Handle wrap-around
        if (error1 > 0.5) {
          error1 = error1 - 1.0;
        } else if (error1 < -0.5) {
          error1 = error1 + 1.0;
        }
        
        double abs_error1 = abs(error1);
        
        // Check if we overshot
        bool overshot = (prev_error1 * error1 < 0) && (abs_error1 > 0.4);
        prev_error1 = error1;
        
        if (abs_error1 < TOLERANCE) {
          motor1_stop_count++;
          if (motor1_stop_count > 2) {
            motor1_done = true;
          }
          position_cmd1.position = NaN;
          position_cmd1.velocity = 0.0;
        } else if (overshot || abs_error1 < BRAKE_ZONE) {
          motor1_stop_count = 0;
          motor1_done = false;
          double brake_vel = 2.0 * ((error1 > 0) ? 1.0 : -1.0);
          position_cmd1.position = NaN;
          position_cmd1.velocity = brake_vel * MOTOR1_DIRECTION;
        } else {
          motor1_stop_count = 0;
          motor1_done = false;
          double vel_magnitude;
          if (abs_error1 > SLOW_ZONE) {
            vel_magnitude = MAX_VELOCITY;
          } else {
            vel_magnitude = max(3.0, MAX_VELOCITY * (abs_error1 / SLOW_ZONE));
          }
          double vel1 = (error1 > 0) ? vel_magnitude : -vel_magnitude;
          position_cmd1.position = NaN;
          position_cmd1.velocity = vel1 * MOTOR1_DIRECTION;
        }
      }
      
      // Motor 2 control logic
      if (got2) {
        double current_ext2 = moteus2.last_result().values.abs_position;
        
        // Store reading for stall detection
        last_ext2[reading_index] = current_ext2;
        
        // Check if encoder is stuck (4 consecutive identical readings within epsilon)
        if (reading_index >= 3 && !motor2_done) {
          const double EPSILON = 1e-6;
          bool stuck2 = (abs(last_ext2[0] - last_ext2[1]) < EPSILON && 
                        abs(last_ext2[1] - last_ext2[2]) < EPSILON && 
                        abs(last_ext2[2] - last_ext2[3]) < EPSILON);
          if (stuck2 && abs(target_ext2 - current_ext2) > TOLERANCE) {
            Serial.println(F(""));
            Serial.println(F("!!! ENCODER STALL DETECTED - MOTOR 2 !!!"));
            Serial.print(F("Encoder stuck at: "));
            Serial.print(current_ext2, 6);
            Serial.println(F(" (4 consecutive identical readings)"));
            Serial.println(F("STOPPING ALL MOTORS FOR SAFETY..."));
            
            displayError("M2 ENCODER STUCK!");
            
            moteus1.SetStop();
            moteus2.SetStop();
            commandRunning = false;
            interruptCommand = false;
            
            // Reset STOP key LED
            setKeyColor(KEY_STOP, COLOR_DIM_RED);
            return;
          }
        }
        
        double error2 = target_ext2 - current_ext2;
        
        // Handle wrap-around
        if (error2 > 0.5) {
          error2 = error2 - 1.0;
        } else if (error2 < -0.5) {
          error2 = error2 + 1.0;
        }
        
        double abs_error2 = abs(error2);
        
        // Check if we overshot
        bool overshot = (prev_error2 * error2 < 0) && (abs_error2 > 0.4);
        prev_error2 = error2;
        
        if (abs_error2 < TOLERANCE) {
          motor2_stop_count++;
          if (motor2_stop_count > 2) {
            motor2_done = true;
          }
          position_cmd2.position = NaN;
          position_cmd2.velocity = 0.0;
        } else if (overshot || abs_error2 < BRAKE_ZONE) {
          motor2_stop_count = 0;
          motor2_done = false;
          double brake_vel = 2.0 * ((error2 > 0) ? 1.0 : -1.0);
          position_cmd2.position = NaN;
          position_cmd2.velocity = brake_vel * MOTOR2_DIRECTION;
        } else {
          motor2_stop_count = 0;
          motor2_done = false;
          double vel_magnitude;
          if (abs_error2 > SLOW_ZONE) {
            vel_magnitude = MAX_VELOCITY;
          } else {
            vel_magnitude = max(3.0, MAX_VELOCITY * (abs_error2 / SLOW_ZONE));
          }
          double vel2 = (error2 > 0) ? vel_magnitude : -vel_magnitude;
          position_cmd2.position = NaN;
          position_cmd2.velocity = vel2 * MOTOR2_DIRECTION;
        }
      }
      
      // Send velocity commands
      moteus1.SetPosition(position_cmd1, &position_fmt, &query_fmt);
      moteus2.SetPosition(position_cmd2, &position_fmt, &query_fmt);
      
      // Update reading index for stall detection (circular buffer)
      reading_index = (reading_index + 1) % 4;
      
      loop_count++;
      
      // Print status every 20 loops
      if (loop_count % 20 == 0) {
        double display_error1 = target_ext1 - moteus1.last_result().values.abs_position;
        if (display_error1 > 0.5) display_error1 -= 1.0;
        else if (display_error1 < -0.5) display_error1 += 1.0;
        
        double display_error2 = target_ext2 - moteus2.last_result().values.abs_position;
        if (display_error2 > 0.5) display_error2 -= 1.0;
        else if (display_error2 < -0.5) display_error2 += 1.0;
        
        Serial.print(F("M1: ext="));
        Serial.print(moteus1.last_result().values.abs_position, 4);
        Serial.print(F(" err="));
        Serial.print(display_error1, 4);
        Serial.print(F(" vel="));
        Serial.print(position_cmd1.velocity, 2);
        Serial.print(F(" done="));
        Serial.print(motor1_done);
        
        Serial.print(F(" | M2: ext="));
        Serial.print(moteus2.last_result().values.abs_position, 4);
        Serial.print(F(" err="));
        Serial.print(display_error2, 4);
        Serial.print(F(" vel="));
        Serial.print(position_cmd2.velocity, 2);
        Serial.print(F(" done="));
        Serial.println(motor2_done);
      }
      
      // Exit when both motors done
      if (motor1_done && motor2_done) {
        Serial.println(F("Both motors reached target!"));
        displayDebug("Target reached!");
        for (int i = 0; i < 5; i++) {
          moteus1.SetStop();
          moteus2.SetStop();
          unsigned long stop_start = millis();
          while (millis() - stop_start < 10) {}
        }
        commandRunning = false;
        break;
      }
    }
    
    // Check timeout
    if (current_time - start_time > TIMEOUT_MS) {
      Serial.println(F("Timeout!"));
      displayDebug("Timeout!");
      commandRunning = false;
      moteus1.SetStop();
      moteus2.SetStop();
      break;
    }
  }
}