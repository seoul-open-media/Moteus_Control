#include "display.h"
#include "config.h"
#include <ACAN2517FD.h>
#include <Moteus.h>
using Moteus = MoteusController<ACAN2517FD>;

// Forward declare motor objects from main.cpp
extern Moteus moteus1;
extern Moteus moteus2;

// XBee control state from xbee.cpp
extern bool xbeeControlActive;
extern float xbeeTargetM1;
extern float xbeeTargetM2;

// Motor running state from motor_control.cpp
extern volatile bool commandRunning;

// Cached motor values (updated in updateXBeeControl)
extern float cachedM1Pos;
extern float cachedM2Pos;
extern float cachedM1Temp;
extern float cachedM2Temp;

// Create display object using Wire1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

// Debug message buffer (stores last 6 messages)
#define MAX_DEBUG_MESSAGES 6
String debugMessages[MAX_DEBUG_MESSAGES];
int debugMessageIndex = 0;
unsigned long lastDebugTime = 0;
bool showDebugScreen = false;

void initDisplay() {
  // Initialize Wire1 for I2C communication
  Wire1.begin();
  Wire1.setClock(400000); // Set I2C frequency to 400kHz
  
  delay(100); // Give display time to power up
  
  // Initialize display with I2C address
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    Serial.println(F("Check I2C connections on Wire1"));
    Serial.println(F("Expected address: 0x3C"));
    // Don't halt - allow motor control to work even without display
    return;
  }
  
  // Clear the buffer
  display.clearDisplay();
  
  // Display startup message
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Moteus Control"));
  display.println(F("Initializing..."));
  display.display();
  
  Serial.println(F("OLED display initialized successfully"));
}

void updateDisplay() {
  // Check if display was initialized successfully
  static bool displayInitialized = true;
  if (!displayInitialized) {
    return;
  }
  
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();
  
  // Update display at 10Hz (every 100ms)
  if (currentTime - lastUpdateTime < 100) {
    return;
  }
  lastUpdateTime = currentTime;
  
  // Debug: print occasionally to confirm display is updating
  static unsigned long lastDisplayDebug = 0;
  if (currentTime - lastDisplayDebug > 2000) {
    Serial.println(F("[Display] Updating..."));
    lastDisplayDebug = currentTime;
  }
  
  // Auto-return to normal display after 3 seconds of debug display
  if (showDebugScreen && (currentTime - lastDebugTime > 3000)) {
    showDebugScreen = false;
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Show debug screen if active
  if (showDebugScreen) {
    display.setCursor(0, 0);
    display.println(F("DEBUG LOG:"));
    display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
    
    // Display last 6 debug messages
    int yPos = 12;
    for (int i = 0; i < MAX_DEBUG_MESSAGES; i++) {
      int idx = (debugMessageIndex + i) % MAX_DEBUG_MESSAGES;
      if (debugMessages[idx].length() > 0) {
        display.setCursor(0, yPos);
        display.print(debugMessages[idx]);
        yPos += 9;
      }
    }
    display.display();
    return;
  }
  
  // Read motor 2 position and temperature directly from last result
  const auto& motor1 = moteus1.last_result().values;
  const auto& motor2 = moteus2.last_result().values;
  float m2_pos  = motor2.abs_position;
  float m1_temp = motor1.motor_temperature;
  float m2_temp = motor2.motor_temperature;
  
  // Convert raw encoder (0~1, center=0.5) to degrees (-90~+90)
  float m2_deg = (m2_pos - ENCODER_CENTER) * 360.0;
  
  // Debug: print motor values to serial
  static unsigned long lastMotorDebug = 0;
  if (currentTime - lastMotorDebug > 2000) {
    Serial.print(F("[Display] M2="));
    Serial.print(m2_deg, 0);
    Serial.print(F("deg "));
    Serial.print(m2_temp, 0);
    Serial.print(F("C State="));
    Serial.println(xbeeControlActive ? "ENGAGED" : "DISENGAGED");
    lastMotorDebug = currentTime;
  }
  
  // Header: Robot ID + Motor2
  // Layout (128x64):
  //  y= 0: "#4  Motor2"          (size 1, 8px)
  //  y= 9: ───────────────────   (divider)
  //  y=11: Pos: [big degrees]    (label y=15, number y=11, size 2, 16px → y=11..26)
  //  y=29: Tgt: [big degrees]    (label y=33, number y=29, size 2, 16px → y=29..44)
  //  y=49: T:XXC       ENGAGED   (size 1, 8px → no overlap)
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print(F("#"));
  display.print(ROBOT_ID);
  display.print(F("  Motor2"));

  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  // Row 1: Current position (size-2 number, size-1 label)
  display.setCursor(0, 15);
  display.setTextSize(1);
  display.print(F("Pos:"));
  display.setCursor(30, 11);
  display.setTextSize(2);
  display.print((int)m2_deg);
  display.setTextSize(1);
  display.print(F("deg"));

  // Row 2: Target position (size-2 number, size-1 label)
  display.setCursor(0, 33);
  display.setTextSize(1);
  display.print(F("Tgt:"));
  display.setCursor(30, 29);
  display.setTextSize(2);
  if (xbeeControlActive) {
    float targ2_deg = (xbeeTargetM2 - ENCODER_CENTER) * 360.0;

    // Debug print targets
    static unsigned long lastTargDebug = 0;
    if (currentTime - lastTargDebug > 2000) {
      Serial.print(F("[Display] Target M2="));
      Serial.print(xbeeTargetM2, 4);
      Serial.print(F(" ("));
      Serial.print(targ2_deg, 0);
      Serial.println(F("deg)"));
      lastTargDebug = currentTime;
    }

    display.print((int)targ2_deg);
    display.setTextSize(1);
    display.print(F("deg"));
  } else {
    display.print(F(" --"));
  }

  // Row 3: Temperature (left) + Status (right) — different x, no overlap
  display.setTextSize(1);
  display.setCursor(0, 49);
  display.print(F("T:"));
  display.print((int)m2_temp);
  display.print(F("C"));

  display.setCursor(68, 49);
  if (m1_temp > 50.0 || m2_temp > 50.0) {
    display.print(F("  WARM!"));
  } else if (commandRunning) {
    display.print(F("MOVING "));
  } else if (xbeeControlActive) {
    display.print(F("ENGAGED"));
  } else {
    display.print(F("DISENGD"));
  }
  
  // Critical temperature warning with inverted display
  static bool wasInverted = false;
  static unsigned long lastTempPrint = 0;
  bool shouldInvert = (m1_temp > 60.0 || m2_temp > 60.0);
  
  if (shouldInvert) {
    display.setCursor(85, 0);
    display.print(F("HOT!!!"));
    display.invertDisplay(true);
    
    // Print to serial when entering critical temp or every 5 seconds
    unsigned long now = millis();
    if (!wasInverted || (now - lastTempPrint > 5000)) {
      Serial.println(F("!!! DISPLAY INVERTED: Temperature > 60°C !!!"));
      Serial.print(F("Time: "));
      Serial.print(now);
      Serial.print(F(" ms | M1 temp: "));
      Serial.print(m1_temp, 1);
      Serial.print(F("°C | M2 temp: "));
      Serial.print(m2_temp, 1);
      Serial.println(F("°C"));
      lastTempPrint = now;
    }
    wasInverted = true;
  } else {
    display.invertDisplay(false);
    if (wasInverted) {
      // Print when leaving inverted state
      Serial.println(F("Display normal: Temperature < 60°C"));
    }
    wasInverted = false;
  }
  
  display.display();
}

void displayStatus(const char* message) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("STATUS:"));
  display.setCursor(0, 12);
  display.println(message);
  display.display();
}

void displayError(const char* error) {
  // Print to serial FIRST before any display operations
  Serial.println(F("!!! displayError() CALLED !!!"));
  Serial.print(F("Error message: "));
  Serial.println(error);
  
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("ERROR!"));
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.println(error);
  display.display();
  display.invertDisplay(true);
  
  Serial.println(F("Display inverted via displayError()"));
}

void displayDebug(const char* message) {
  // Add message to circular buffer
  debugMessages[debugMessageIndex] = String(message);
  debugMessageIndex = (debugMessageIndex + 1) % MAX_DEBUG_MESSAGES;
  
  // Set flag to show debug screen
  showDebugScreen = true;
  lastDebugTime = millis();
  
  // Also print to serial
  Serial.print(F("DEBUG: "));
  Serial.println(message);
}

void scanI2C() {
  Serial.println(F("\nScanning I2C bus (Wire1)..."));
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
    
    if (error == 0) {
      Serial.print(F("I2C device found at address 0x"));
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print(F("Unknown error at address 0x"));
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (nDevices == 0) {
    Serial.println(F("No I2C devices found"));
    Serial.println(F("Check wiring on Wire1:"));
    Serial.println(F("  SDA -> Wire1 SDA pin"));
    Serial.println(F("  SCL -> Wire1 SCL pin"));
    Serial.println(F("  VCC -> 3.3V or 5V"));
    Serial.println(F("  GND -> GND"));
  }
  else {
    Serial.print(F("Found "));
    Serial.print(nDevices);
    Serial.println(F(" device(s)"));
  }
}
