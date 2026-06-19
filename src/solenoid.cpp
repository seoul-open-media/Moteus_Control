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

static unsigned long em1_duration = PULSE_DURATION_MS;
static unsigned long em2_duration = PULSE_DURATION_MS;
static unsigned long em3_duration = PULSE_DURATION_MS;
static unsigned long sol_duration = PULSE_DURATION_MS;

// Sequential EM trigger schedule (0 = not scheduled)
static unsigned long em2_scheduled_time = 0;
static unsigned long em3_scheduled_time = 0;
static const unsigned long EM_SEQ_INTERVAL_MS = 10;   // 10ms between each EM

// Encoder-based brake monitoring
static bool brake_mode_active = false;  // Is brake in encoder-monitoring mode?
static double prev_ext1 = 0.0;
static double prev_ext2 = 0.0;
static unsigned long last_encoder_check = 0;
static unsigned long brake_start_time = 0;
static int stable_check_count = 0;  // Count consecutive stable checks

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
  
  // If brake mode is active, check encoder changes instead of time
  if (brake_mode_active) {
    // Check if it's time to evaluate encoder changes
    if (now - last_encoder_check >= BRAKE_CHECK_INTERVAL_MS) {
      double ext1_change = abs(prev_ext1);
      double ext2_change = abs(prev_ext2);
      double max_change = max(ext1_change, ext2_change);
      
      // Safety: maximum brake duration
      unsigned long brake_elapsed = now - brake_start_time;
      
      // Don't release brake before minimum duration
      if (brake_elapsed < MIN_BRAKE_DURATION_MS) {
        Serial.print(F("[Solenoid] Brake held (min time, elapsed="));
        Serial.print(brake_elapsed);
        Serial.print(F("ms, change="));
        Serial.print(max_change, 5);
        Serial.println(F(")"));
        prev_ext1 = 0.0;
        prev_ext2 = 0.0;
        last_encoder_check = now;
        return;
      }
      
      if (max_change < ENCODER_CHANGE_THRESHOLD) {
        // Motor might be stopped - increment stable count
        stable_check_count++;
        Serial.print(F("[Solenoid] Stable check "));
        Serial.print(stable_check_count);
        Serial.print(F("/"));
        Serial.print(BRAKE_STABLE_CHECKS);
        Serial.print(F(" (change="));
        Serial.print(max_change, 5);
        Serial.print(F(", elapsed="));
        Serial.print(brake_elapsed);
        Serial.println(F("ms)"));
        
        if (stable_check_count >= BRAKE_STABLE_CHECKS) {
          // Motor confirmed stopped - release brake
          Serial.print(F("[Solenoid] Motor stopped ("));
          Serial.print(stable_check_count);
          Serial.print(F(" stable checks, elapsed="));
          Serial.print(brake_elapsed);
          Serial.println(F("ms) - Releasing brake"));
          stopMotorBrake();
        }
      } else if (brake_elapsed >= MAX_BRAKE_DURATION_MS) {
        // Safety timeout
        Serial.print(F("[Solenoid] Brake timeout ("));
        Serial.print(brake_elapsed);
        Serial.println(F("ms) - Releasing brake"));
        stopMotorBrake();
      } else {
        // Motor still moving - reset stable count and continue brake
        stable_check_count = 0;
        Serial.print(F("[Solenoid] Motor moving (change="));
        Serial.print(max_change, 5);
        Serial.print(F(", elapsed="));
        Serial.print(brake_elapsed);
        Serial.println(F("ms) - Brake held"));
      }
      
      // Reset for next check
      prev_ext1 = 0.0;
      prev_ext2 = 0.0;
      last_encoder_check = now;
    }
    return;  // Skip normal timing checks when in brake mode
  }
  
  // Normal timing-based checks for other solenoid functions

  // Check scheduled sequential EM triggers
  if (em2_scheduled_time > 0 && now >= em2_scheduled_time) {
    analogWrite(EM2_PIN, 255);
    em2_active = true;
    em2_trigger_time = now;
    em2_scheduled_time = 0;
    Serial.println(F("[Solenoid] EM2 ON (sequential)"));
  }
  if (em3_scheduled_time > 0 && now >= em3_scheduled_time) {
    analogWrite(EM3_PIN, 255);
    em3_active = true;
    em3_trigger_time = now;
    em3_scheduled_time = 0;
    Serial.println(F("[Solenoid] EM3 ON (sequential)"));
  }

  // Check EM1 timing (skip if in brake mode)
  if (em1_active && !brake_mode_active && (now - em1_trigger_time >= em1_duration)) {
    analogWrite(EM1_PIN, 0);
    em1_active = false;
    em1_duration = PULSE_DURATION_MS;  // Reset to default
    Serial.println(F("[Solenoid] EM1 OFF"));
  }
  
  // Check EM2 timing (skip if in brake mode)
  if (em2_active && !brake_mode_active && (now - em2_trigger_time >= em2_duration)) {
    analogWrite(EM2_PIN, 0);
    em2_active = false;
    em2_duration = PULSE_DURATION_MS;  // Reset to default
    Serial.println(F("[Solenoid] EM2 OFF"));
  }
  
  // Check EM3 timing (skip if in brake mode)
  if (em3_active && !brake_mode_active && (now - em3_trigger_time >= em3_duration)) {
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

  // EM1 fires immediately
  analogWrite(EM1_PIN, 255);
  em1_active = true;
  em1_trigger_time = now;

  // EM2 scheduled after 100ms
  em2_scheduled_time = now + EM_SEQ_INTERVAL_MS;

  // EM3 scheduled after 200ms
  em3_scheduled_time = now + EM_SEQ_INTERVAL_MS * 2;

  Serial.println(F("[Solenoid] EM1 ON now, EM2+EM3 scheduled (+100ms, +200ms)"));
}

void triggerSolenoid() {
  unsigned long now = millis();
  
  // Turn on solenoid with full PWM (255)
  analogWrite(SOLENOID_PIN, 255);
  sol_active = true;
  sol_trigger_time = now;
  
  Serial.println(F("[Solenoid] SOLENOID ON (50ms)"));
}

void triggerTypeA_2() {
  // Solenoid on
  unsigned long now = millis();
  
  analogWrite(SOLENOID_PIN, 255);
  sol_active = true;
  sol_trigger_time = now;
  
  Serial.println(F("[Solenoid] TypeA-2: SOLENOID ON (50ms)"));
}

void triggerMotorBrake() {
  unsigned long now = millis();

  // EM1 fires immediately
  analogWrite(EM1_PIN, 255);
  em1_active = true;
  em1_trigger_time = now;
  Serial.println(F("[Solenoid] BRAKE EM1 ON now"));

  // EM2 scheduled after 100ms
  em2_scheduled_time = now + EM_SEQ_INTERVAL_MS;
  Serial.println(F("[Solenoid] BRAKE EM2 scheduled +100ms"));

  // EM3 scheduled after 200ms
  em3_scheduled_time = now + EM_SEQ_INTERVAL_MS * 2;
  Serial.println(F("[Solenoid] BRAKE EM3 scheduled +200ms"));

  // Enable encoder-based brake monitoring
  brake_mode_active = true;
  brake_start_time = now;
  last_encoder_check = now;
  prev_ext1 = 0.0;
  prev_ext2 = 0.0;
  stable_check_count = 0;

  Serial.print(F("[Solenoid] MOTOR BRAKE started (min "));
  Serial.print(MIN_BRAKE_DURATION_MS);
  Serial.println(F("ms)"));
}

void updateBrakeEncoders(double ext1, double ext2) {
  // Accumulate encoder changes since last check
  // This is called from motor control loop
  if (brake_mode_active) {
    prev_ext1 += abs(ext1);
    prev_ext2 += abs(ext2);
  }
}

void stopMotorBrake() {
  // Manually stop the brake
  analogWrite(EM1_PIN, 0);
  em1_active = false;
  em1_duration = PULSE_DURATION_MS;
  Serial.println(F("[Solenoid] EM1 OFF"));
  
  analogWrite(EM2_PIN, 0);
  em2_active = false;
  em2_duration = PULSE_DURATION_MS;
  Serial.println(F("[Solenoid] EM2 OFF"));
  
  analogWrite(EM3_PIN, 0);
  em3_active = false;
  em3_duration = PULSE_DURATION_MS;
  Serial.println(F("[Solenoid] EM3 OFF"));
  
  brake_mode_active = false;
  prev_ext1 = 0.0;
  prev_ext2 = 0.0;
  stable_check_count = 0;  // Reset stable counter
}
