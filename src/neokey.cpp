#include "neokey.h"
#include "motor_control.h"
#include "display.h"
#include "xbee.h"
#include <Arduino.h>

// External declarations from xbee.cpp
extern bool xbeeControlActive;
extern float xbeeTargetM1;
extern float xbeeTargetM2;
extern float xbeeMaxVelM1;
extern float xbeeMaxVelM2;
extern bool m1_settled;
extern bool m2_settled;

// Create NeoKey object on Wire (I2C bus 0)
Adafruit_NeoKey_1x4 neokey;

// Last key states to detect changes
uint8_t lastKeyState = 0;

void initNeoKey() {
  // Initialize Wire (I2C bus 0) if not already done
  Wire.begin();
  Wire.setClock(100000);  // Set to 100kHz for more reliable communication
  delay(100);  // Give I2C bus time to stabilize
  
  Serial.println(F("Attempting NeoKey initialization..."));
  
  // Try to detect device on I2C first
  Wire.beginTransmission(0x30);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print(F("NeoKey not detected at 0x30, I2C error: "));
    Serial.println(error);
    Serial.println(F("Check wiring on Wire (default I2C):"));
    Serial.println(F("  SDA -> Wire SDA pin"));
    Serial.println(F("  SCL -> Wire SCL pin"));
    Serial.println(F("  VCC -> 5V"));
    Serial.println(F("  GND -> GND"));
    return;
  }
  
  Serial.println(F("NeoKey detected on I2C, initializing..."));
  
  // Initialize NeoKey on Wire (I2C bus 0) at default address 0x30
  if (!neokey.begin(0x30)) {
    Serial.println(F("NeoKey not found on I2C bus 0"));
    Serial.println(F("Check wiring on Wire (default I2C):"));
    Serial.println(F("  SDA -> Wire SDA pin"));
    Serial.println(F("  SCL -> Wire SCL pin"));
    Serial.println(F("  VCC -> 5V"));
    Serial.println(F("  GND -> GND"));
    return;
  }
  
  Serial.println(F("NeoKey initialized successfully"));
  
  // Set initial colors - dim to show keys are ready
  setKeyColor(KEY_POS_0, COLOR_DIM_GREEN);    // Green for position 0
  setKeyColor(KEY_POS_025, COLOR_DIM_BLUE);   // Blue for position 0.25
  setKeyColor(KEY_POS_075, COLOR_DIM_CYAN);   // Cyan for position 0.75
  setKeyColor(KEY_STOP, COLOR_DIM_RED);       // Red for stop
}

void setKeyColor(uint8_t key, uint32_t color) {
  neokey.pixels.setPixelColor(key, color);
  neokey.pixels.show();
}

void updateNeoKey() {
  // Read current button states
  uint8_t buttons = neokey.read();
  
  // Check for button presses (detect rising edge)
  uint8_t pressed = buttons & ~lastKeyState;
  lastKeyState = buttons;
  
  // Debug: print which buttons are pressed
  if (pressed) {
    Serial.print(F("NeoKey pressed bits: "));
    Serial.println(pressed, BIN);
  }
  
  if (pressed) {
    // Key 1 (index 0): Default position (0) with speed 1 (10 rev/s)
    if (pressed & (1 << KEY_POS_0)) {
      Serial.println(F("NeoKey1: Default position (0) with speed 1"));
      displayDebug("K1: Pos 0");
      setKeyColor(KEY_POS_0, COLOR_GREEN);
      
      // Set XBee control to position 0 with speed 1 (10 rev/s)
      xbeeTargetM1 = 0.0;
      xbeeTargetM2 = 0.0;
      xbeeMaxVelM1 = 10.0;  // Speed 1 = 10 rev/s
      xbeeMaxVelM2 = 10.0;
      m1_settled = false;
      m2_settled = false;
      xbeeControlActive = true;
      
      Serial.print(F("[NeoKey1] Set targets: M1="));
      Serial.print(xbeeTargetM1, 4);
      Serial.print(F(" M2="));
      Serial.print(xbeeTargetM2, 4);
      Serial.print(F(" vel="));
      Serial.print(xbeeMaxVelM1, 1);
      Serial.print(F(" active="));
      Serial.println(xbeeControlActive ? "YES" : "NO");
      
      setKeyColor(KEY_POS_0, COLOR_DIM_GREEN);
      return;  // Exit after processing
    }
    
    // Key 2 (index 1): Both motors to position 0.25
    if (pressed & (1 << KEY_POS_025)) {
      Serial.println(F("NeoKey: Motors -> 0.25"));
      displayDebug("K: Pos 0.25");
      setKeyColor(KEY_POS_025, COLOR_BLUE);
      moveToEncoderPosition(0.25, 0.25);
      setKeyColor(KEY_POS_025, COLOR_DIM_BLUE);
      return;  // Exit after processing
    }
    
    // Key 3 (index 2): Both motors to position 0.75
    if (pressed & (1 << KEY_POS_075)) {
      Serial.println(F("NeoKey: Motors -> 0.75"));
      displayDebug("K: Pos 0.75");
      setKeyColor(KEY_POS_075, COLOR_CYAN);
      moveToEncoderPosition(0.75, 0.75);
      setKeyColor(KEY_POS_075, COLOR_DIM_CYAN);
      return;  // Exit after processing
    }
    
    // Key 4 (index 3): EMERGENCY STOP
    if (pressed & (1 << KEY_STOP)) {
      Serial.println(F("NeoKey4: EMERGENCY STOP"));
      displayDebug("K4: STOP!");
      setKeyColor(KEY_STOP, COLOR_RED);
      
      // Stop XBee control
      xbeeControlActive = false;
      m1_settled = false;
      m2_settled = false;
      
      // Set interrupt to stop any running command
      // interruptCommand = true;
      // commandRunning = false;
      
      // DISABLED: Stop motors immediately (Workshop mode - keep motors engaged)
      Serial.println(F("NeoKey STOP button pressed but IGNORED (Workshop Mode)"));
      // moteus1.SetStop();
      // moteus2.SetStop();
      
      // Flash red LED
      delay(100);
      setKeyColor(KEY_STOP, COLOR_DIM_RED);
      delay(100);
      setKeyColor(KEY_STOP, COLOR_RED);
      delay(100);
      setKeyColor(KEY_STOP, COLOR_DIM_RED);
      return;  // Exit after processing
    }
    
    // If we get here, an unknown key was pressed
    Serial.print(F("Unknown key pressed, bits: "));
    Serial.println(pressed, BIN);
  }
}
