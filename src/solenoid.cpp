#include "solenoid.h"
#include "config.h"

// State tracking for non-blocking timing
static unsigned long em1_trigger_time = 0;
static unsigned long em2_trigger_time = 0;
static unsigned long em3_trigger_time = 0;
static unsigned long sol_trigger_time = 0;

static bool em1_active = false;
static bool em2_active = false;
static bool em3_active = false;
static bool sol_active = false;

static const unsigned long PULSE_DURATION_MS = 50;
static const unsigned long BRAKE_DURATION_MS = 200;  // 200ms for motor brake

static unsigned long em1_duration = PULSE_DURATION_MS;
static unsigned long em2_duration = PULSE_DURATION_MS;
static unsigned long em3_duration = PULSE_DURATION_MS;
static unsigned long sol_duration = PULSE_DURATION_MS;

void initSolenoid() {
  pinMode(EM1_PIN, OUTPUT);
  pinMode(EM2_PIN, OUTPUT);
  pinMode(EM3_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  
  // Set all to LOW initially
  analogWrite(EM1_PIN, 0);
  analogWrite(EM2_PIN, 0);
  analogWrite(EM3_PIN, 0);
  analogWrite(SOLENOID_PIN, 0);
  
  Serial.println(F("[Solenoid] Initialized: EM1(pin3), EM2(pin4), EM3(pin6), SOL(pin5)"));
}

void updateSolenoid() {
  unsigned long now = millis();
  
  // Check EM1 timing
  if (em1_active && (now - em1_trigger_time >= em1_duration)) {
    analogWrite(EM1_PIN, 0);
    em1_active = false;
    em1_duration = PULSE_DURATION_MS;  // Reset to default
    Serial.println(F("[Solenoid] EM1 OFF"));
  }
  
  // Check EM2 timing
  if (em2_active && (now - em2_trigger_time >= em2_duration)) {
    analogWrite(EM2_PIN, 0);
    em2_active = false;
    em2_duration = PULSE_DURATION_MS;  // Reset to default
    Serial.println(F("[Solenoid] EM2 OFF"));
  }
  
  // Check EM3 timing
  if (em3_active && (now - em3_trigger_time >= em3_duration)) {
    analogWrite(EM3_PIN, 0);
    em3_active = false;
    em3_duration = PULSE_DURATION_MS;  // Reset to default
    Serial.println(F("[Solenoid] EM3 OFF"));
  }
  
  // Check Solenoid timing
  if (sol_active && (now - sol_trigger_time >= sol_duration)) {
    analogWrite(SOLENOID_PIN, 0);
    sol_active = false;
    sol_duration = PULSE_DURATION_MS;  // Reset to default
    Serial.println(F("[Solenoid] SOLENOID OFF"));
  }
}

void triggerElectromagnets() {
  unsigned long now = millis();
  
  // Turn on EM1, EM2, EM3 with full PWM (255)
  analogWrite(EM1_PIN, 255);
  em1_active = true;
  em1_trigger_time = now;
  
  analogWrite(EM2_PIN, 255);
  em2_active = true;
  em2_trigger_time = now;
  
  analogWrite(EM3_PIN, 255);
  em3_active = true;
  em3_trigger_time = now;
  
  Serial.println(F("[Solenoid] EM1+EM2+EM3 ON (50ms)"));
}

void triggerSolenoid() {
  unsigned long now = millis();
  
  // Turn on solenoid with full PWM (255)
  analogWrite(SOLENOID_PIN, 255);
  sol_active = true;
  sol_trigger_time = now;
  
  Serial.println(F("[Solenoid] SOLENOID ON (50ms)"));
}

// Type A control functions
void triggerTypeA_1() {
  // EM1, EM2, EM3 on
  unsigned long now = millis();
  
  analogWrite(EM1_PIN, 255);
  em1_active = true;
  em1_trigger_time = now;
  
  analogWrite(EM2_PIN, 255);
  em2_active = true;
  em2_trigger_time = now;
  
  analogWrite(EM3_PIN, 255);
  em3_active = true;
  em3_trigger_time = now;
  
  Serial.println(F("[Solenoid] TypeA-1: EM1+EM2+EM3 ON (50ms)"));
}

void triggerTypeA_2() {
  // Solenoid on
  unsigned long now = millis();
  
  analogWrite(SOLENOID_PIN, 255);
  sol_active = true;
  sol_trigger_time = now;
  
  Serial.println(F("[Solenoid] TypeA-2: SOLENOID ON (50ms)"));
}

void triggerTypeA_3() {
  // All on
  unsigned long now = millis();
  
  analogWrite(EM1_PIN, 255);
  em1_active = true;
  em1_trigger_time = now;
  
  analogWrite(EM2_PIN, 255);
  em2_active = true;
  em2_trigger_time = now;
  
  analogWrite(EM3_PIN, 255);
  em3_active = true;
  em3_trigger_time = now;
  
  analogWrite(SOLENOID_PIN, 255);
  sol_active = true;
  sol_trigger_time = now;
  
  Serial.println(F("[Solenoid] TypeA-3: ALL ON (50ms)"));
}

// Type B control functions
void triggerTypeB_1() {
  // EM1, EM2 on
  unsigned long now = millis();
  
  analogWrite(EM1_PIN, 255);
  em1_active = true;
  em1_trigger_time = now;
  
  analogWrite(EM2_PIN, 255);
  em2_active = true;
  em2_trigger_time = now;
  
  Serial.println(F("[Solenoid] TypeB-1: EM1+EM2 ON (50ms)"));
}

void triggerTypeB_2() {
  // Solenoid, EM3 on
  unsigned long now = millis();
  
  analogWrite(SOLENOID_PIN, 255);
  sol_active = true;
  sol_trigger_time = now;
  
  analogWrite(EM3_PIN, 255);
  em3_active = true;
  em3_trigger_time = now;
  
  Serial.println(F("[Solenoid] TypeB-2: SOLENOID+EM3 ON (50ms)"));
}

void triggerTypeB_3() {
  // All on
  unsigned long now = millis();
  
  analogWrite(EM1_PIN, 255);
  em1_active = true;
  em1_trigger_time = now;
  
  analogWrite(EM2_PIN, 255);
  em2_active = true;
  em2_trigger_time = now;
  
  analogWrite(EM3_PIN, 255);
  em3_active = true;
  em3_trigger_time = now;
  
  analogWrite(SOLENOID_PIN, 255);
  sol_active = true;
  sol_trigger_time = now;
  
  Serial.println(F("[Solenoid] TypeB-3: ALL ON (50ms)"));
}

void triggerMotorBrake() {
  // Turn on all electromagnets for 100ms to act as motor brake
  unsigned long now = millis();
  
  analogWrite(EM1_PIN, 255);
  em1_active = true;
  em1_trigger_time = now;
  em1_duration = BRAKE_DURATION_MS;  // 1000ms for brake
  Serial.print(F("[Solenoid] EM1 PIN "));
  Serial.print(EM1_PIN);
  Serial.println(F(" = 255"));
  
  analogWrite(EM2_PIN, 255);
  em2_active = true;
  em2_trigger_time = now;
  em2_duration = BRAKE_DURATION_MS;  // 1000ms for brake
  Serial.print(F("[Solenoid] EM2 PIN "));
  Serial.print(EM2_PIN);
  Serial.println(F(" = 255"));
  
  analogWrite(EM3_PIN, 255);
  em3_active = true;
  em3_trigger_time = now;
  em3_duration = BRAKE_DURATION_MS;  // 1000ms for brake
  Serial.print(F("[Solenoid] EM3 PIN "));
  Serial.print(EM3_PIN);
  Serial.println(F(" = 255"));
  
  Serial.println(F("[Solenoid] MOTOR BRAKE: EM1+EM2+EM3 ON (200ms)"));
}
