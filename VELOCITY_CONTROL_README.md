# Velocity Control Implementation

## Overview
This document describes the working velocity control implementation for the 13-robot XBee wireless control system.

## Working Configuration

### Pure Data Patch
- **File**: `Host/Ahae_sequencer/Ahae_sequencer_R2_01.pd`
- **Protocol**: 28-byte XBee messages (0xFF 0xFF header + 13 robots × 2 bytes)
- **Format**: Each robot controlled by 4 digits `VXXY`
  - `V` (digit1): Velocity control (0-9)
  - `XX` (digit2): Motor 1 position (0-9)
  - `Y` (digit3): Motor 2 position (0-9)

### Firmware Configuration
- **Branch**: `Ahae-control-01`
- **File**: `src/xbee.cpp`
- **Robot ID**: Configurable in `src/config.h` (currently set to 2)

## Velocity Mapping

The velocity control uses a custom mapping optimized to prevent motor stalling while providing good speed range:

| Digit | Velocity (rev/s) | Notes |
|-------|------------------|-------|
| 0 | 0 | Motor holds position |
| 1 | 20 | Minimum speed to overcome friction |
| 2 | 30 | |
| 3 | 40 | |
| 4 | 50 | |
| 5 | 60 | |
| 6 | 70 | |
| 7 | 80 | |
| 8 | 90 | |
| 9 | 100 | Maximum speed |

**Key Design Decision**: Minimum non-zero velocity is 20 rev/s (not 5 rev/s) because lower speeds caused motors to stall mid-movement due to insufficient power to overcome friction and inertia.

## Position Mapping

Motor positions are controlled by digits 0-9:

- **0**: Hold current position (no movement)
- **1-4**: Negative range (-0.25 to 0 revolutions / -90° to 0°)
  - 1 → -0.25 rev (-90°)
  - 2 → -0.167 rev (-60°)
  - 3 → -0.083 rev (-30°)
  - 4 → 0 rev (0°)
- **5**: Center position (0 revolutions)
- **6-9**: Positive range (0 to 0.25 revolutions / 0° to 90°)
  - 6 → 0.083 rev (30°)
  - 7 → 0.167 rev (60°)
  - 8 → 0.25 rev (90°)
  - 9 → 0.25 rev (90°)

Position range is constrained to ±0.25 revolutions (±90°) for safety.

## Control Algorithm

### PD Velocity Control
The system uses PD (Proportional-Derivative) control with velocity commands:

```cpp
// P gains scale linearly with velocity to maintain consistent behavior
vel_scale = max_vel / 400.0;
Kp_far = max_vel * 2.0;    // Far from target
Kp_near = max_vel * 1.0;   // Close to target
Kd = max(5.0, 5.0 * vel_scale);

// Calculate velocity based on position error
if (error > 0.03 rev) {
    velocity = error * Kp_far - current_velocity * Kd;
} else {
    velocity = error * Kp_near - current_velocity * Kd * 1.5;
}

// Constrain to velocity limit
velocity = constrain(velocity, -max_vel, max_vel);
```

### Motor Command
```cpp
cmd.position = NaN;              // Don't use position mode
cmd.velocity = calculated_vel;    // Direct velocity control
cmd.velocity_limit = max_vel;     // Maximum velocity from mapping
cmd.accel_limit = max_vel * 0.3;  // Acceleration scales with velocity
```

## Implementation Details

### Code Locations
1. **Velocity mapping**: `src/xbee.cpp` lines ~126 and ~193
   ```cpp
   const float vel_map[10] = {0, 20, 30, 40, 50, 60, 70, 80, 90, 100};
   float velocity = (digit1 <= 9) ? vel_map[digit1] : 100.0;
   ```

2. **Position mapping**: `src/xbee.cpp` lines ~132-154
   - Separate logic for each motor
   - Digit 0 preserves current target

3. **PD control loop**: `src/xbee.cpp` `updateXBeeControl()` function
   - Runs at 100Hz (10ms update rate)
   - Queries current position and velocity
   - Calculates error with wrap-around handling
   - Applies PD gains scaled to velocity

### Robot ID Configuration
Each robot must have a unique ID (1-13) set in `src/config.h`:
```cpp
static const uint8_t ROBOT_ID = 2;  // Change this for each robot!
```

## XBee Communication

### Message Format
```
[0xFF] [0xFF] [R1_MSB] [R1_LSB] [R2_MSB] [R2_LSB] ... [R13_MSB] [R13_LSB]
```

- **Header**: Two 0xFF bytes
- **Data**: 26 bytes (13 robots × 2 bytes)
- **Encoding**: Each robot's 4 digits packed into 2 bytes as 16-bit value

### Example
For Robot #2 with V=5, M1=7, M2=3 (digits: 5073):
```
Value = 5073
MSB = 0x13 (19)
LSB = 0xD1 (209)
```

### Broadcast Mode
First byte = 0x00 triggers broadcast mode where all robots execute the same command from the first robot's data.

## Testing & Debugging

### Serial Output
The firmware outputs debug information at 115200 baud:
```
[XBee] Valid header found!
[XBee] Robot 2 digits: v=5 m1=7 m2=3
[Vel] max_vel_m1=60.0 max_vel_m2=60.0
M1: pos=0.167 vel=60.0
M2: pos=-0.083 vel=60.0
```

### Pure Data Setup
1. Open `Host/Ahae_sequencer/Ahae_sequencer_R2_01.pd`
2. Configure XBee serial port
3. Set robot number (1-13)
4. Control with sliders:
   - Velocity: 0-9 (0=stop, 9=fastest)
   - Motor 1 position: 0-9
   - Motor 2 position: 0-9

## Troubleshooting

### Motors Stalling
- **Symptom**: Motors stop mid-movement at low velocities
- **Solution**: Implemented - minimum velocity is now 20 rev/s instead of 5 rev/s
- **Code**: Check `vel_map` array has values ≥20 for digits 1-9

### Velocity Not Changing
- **Previous Issue**: PD gains saturating at all velocities
- **Solution**: Linear gain scaling `Kp = max_vel * 2.0` instead of fixed high values
- **Verify**: Serial output should show different `max_vel` values for different velocity digits

### Wrong Robot Responding
- **Check**: ROBOT_ID in `src/config.h` matches intended robot number
- **Verify**: Serial output shows correct `[XBee] Robot X` message

## Version History

- **2024-11-19**: Working velocity control implementation
  - Custom velocity mapping (0→0, 1→20...9→100 rev/s)
  - Linear PD gain scaling
  - Robot ID 2 configured
  - Compatible with Ahae_sequencer_R2_01.pd
