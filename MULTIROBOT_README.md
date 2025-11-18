# XBee Multi-Robot Control System (13 Robots)

## Overview
Control 13 robots simultaneously using XBee wireless communication. Each robot has 2 motors with position control via a compact 28-byte protocol.

## Protocol Format
```
[0xFF 0xFF] [MSB1 LSB1] [MSB2 LSB2] ... [MSB13 LSB13]
```

### Structure:
- **Bytes 1-2**: Header `0xFF 0xFF` (255 255)
- **Bytes 3-4**: Robot 1 data (MSB, LSB)
- **Bytes 5-6**: Robot 2 data (MSB, LSB)
- **...**: Continue for robots 3-13
- **Bytes 27-28**: Robot 13 data (MSB, LSB)

**Total: 28 bytes** (2 header + 26 data bytes)

### Data Encoding (Per Robot)
Each robot receives 2 bytes = 16 bits = 4 decimal digits `VXXY`:
- **V (digit 1)**: Velocity parameter (0-9) → maps to 50-500 rev/s
- **XX (digits 2-3)**: Motor 1 position (0-9 each)
- **Y (digit 4)**: Motor 2 position (0-9)

**Position Mapping:**
- `0`: Stay at current position (no movement)
- `1-4`: Negative range (-0.25 to 0 revolutions / -90° to 0°)
  - `1` = -0.25 rev (-90°)
  - `2` = -0.167 rev (-60°)
  - `3` = -0.083 rev (-30°)
  - `4` = 0.0 rev (0°)
- `5`: Center position (0.0 rev / 0°)
- `6-9`: Positive range (0 to +0.25 revolutions / 0° to +90°)
  - `6` = 0.0 rev (0°)
  - `7` = 0.083 rev (30°)
  - `8` = 0.167 rev (60°)
  - `9` = 0.25 rev (90°)

## Configuration

### Setting Robot ID
Edit `src/config.h` and change the `ROBOT_ID`:
```cpp
static const uint8_t ROBOT_ID = 1;  // Change for each robot (1-13)
```

**IMPORTANT**: Each robot MUST have a unique ID (1-13)

### Recommended XBee Setup
- **Type**: XBee 802.15.4 (Series 1) or XBee 3 802.15.4
- **Mode**: Transparent/AT mode
- **Baud**: 115200
- **Network**: 
  - Host (Pure Data): Coordinator
  - Robots: End devices with unique 16-bit addresses

## Pure Data Control

### Protocol Example
To send a command where Robot 1 gets velocity=3, M1=7, M2=7:
```
Value: 3770
MSB: 14 (3770 / 256 = 14)
LSB: 186 (3770 % 256 = 186)
Send: [0xFF 0xFF] [14 186] [data for robots 2-13...]
```

### XbeeArduino.pd Patch
Main control interface with:
- 4 number boxes per robot (velocity, motor1, motor2, reserved)
- Automatic encoding to MSB/LSB format
- 115200 baud serial communication
- Broadcasts to all 13 robots simultaneously

### Broadcast Mode
If all 26 data bytes are `0xFF`, all robots execute the same command from bytes 3-4.

## Control Patches

### Ahae_control_01.pd
- Simple single robot testing interface
- 4 digit input boxes
- Useful for protocol verification

### Ahae_control_13robots.pd  
- Full 13-robot control interface
- Individual number boxes for each robot's 4 parameters
- Real-time encoding and transmission
- Visual feedback of sent values

## Features

### Position Control
- Range: -90° to +90° (-0.25 to +0.25 revolutions)
- 180° offset applied automatically
- Position constraints enforced

### Velocity Parameter
- **Range**: Digit 0-9 maps to 50-500 rev/s
- **Note**: Due to PD controller with velocity-scaled gains, different velocity settings may not produce visibly different speeds
- **Behavior**: Motors reach target positions accurately but movement time is similar across velocity settings
- **Technical**: Calculated velocity is proportional to velocity limit, causing similar percentage utilization
- **Recommendation**: Use velocity parameter for tuning but don't expect dramatic speed differences

### Safety Features
- Emergency STOP (broadcast to all robots)
- Kill button stops motors and disables control
- Position limits prevent over-rotation

## Manufacturing Setup

For each of 13 robots:

1. **Flash firmware** with default ROBOT_ID = 1
2. **Change ROBOT_ID** in config.h (1-13)
3. **Re-upload** firmware
4. **Configure XBee**:
   - Set same PAN ID for all units
   - Set baud to 115200
   - Transparent/AT mode
5. **Label robot** with its ID number
6. **Test** with Ahae_control_13robots.pd

## Network Architecture

```
Pure Data Host (XBee)
        |
        └─ Broadcast 28 bytes ──> All 13 Robots
                                   |
                                   ├─ Robot #1 reads bytes 3-4
                                   ├─ Robot #2 reads bytes 5-6
                                   ├─ Robot #3 reads bytes 7-8
                                   ⋮
                                   └─ Robot #13 reads bytes 27-28
```

Each robot filters the 28-byte message and extracts only its own 2 bytes based on ROBOT_ID.

## Choreography Examples

### All Robots Same Position
Set all robots to velocity=7, M1=position 7, M2=position 7:
```
Value: 7777
All robots: [0xFF 0xFF] [30 81] [30 81] ... [30 81] (repeat 13 times)
```

### Wave Pattern
Robot 1: position 2, Robot 2: position 4, Robot 3: position 6, etc.
```
[0xFF 0xFF] [MSB1 LSB1] [MSB2 LSB2] [MSB3 LSB3] ...
```
Update values over time to create wave motion.

### Individual Control
Each robot gets unique positions - create complex patterns:
```
Robot 1: 7227 (v=7, m1=2, m2=2, r=7)
Robot 2: 7779 (v=7, m1=7, m2=7, r=9)
Robot 3: 7552 (v=7, m1=5, m2=5, r=2)
...
```

### Sequencing
Use Pure Data metro objects to send different 28-byte messages at timed intervals for choreographed sequences.

## Troubleshooting

### Robot not responding
- Check ROBOT_ID in config.h (must be 1-13)
- Verify XBee baud rate is 115200
- Check power and CAN bus connections
- Monitor serial output for "[XBee] Valid header found!"
- Verify 28-byte message is being sent

### Wrong robot responds
- Each robot extracts data at index = (ROBOT_ID - 1) × 2
- Robot 1 reads bytes 3-4
- Robot 2 reads bytes 5-6, etc.
- Check ROBOT_ID is unique (1-13)

### Position digit 0 doesn't hold position
- Digit 0 = stay at current position (no movement command)
- Motor only holds if already at target
- Use digit 5 to explicitly command center (0.0 rev)

### Motors move but velocity doesn't change speed
- **This is expected behavior** with current PD controller
- Velocity parameter sets limits but PD gains scale proportionally
- Motors reach target accurately but movement time is similar
- To achieve true speed variation, would require non-scaled P gains (may cause overshoot)

## Performance

- **Protocol**: 28 bytes total (2 header + 26 data)
- **Baud rate**: 115200
- **Message time**: ~2.5ms per 28-byte packet
- **Control rate**: 100Hz per robot (10ms loop)
- **XBee latency**: ~5-10ms
- **Total latency**: ~15-20ms from PD to motor command
- **Effective update rate**: Can send new commands at ~20Hz (50ms intervals)

## Technical Details

### PD Control Algorithm
- **P gains**: Scale with velocity limit (`Kp = max_vel × 2.0`)
- **D gains**: Velocity damping (`Kd = max(2.0, 1.5 × vel_scale)`)
- **Acceleration**: Constant 800 rev/s² for all velocities
- **Deadband**: 0.01 revolutions (~3.6°)
- **Brake zone**: 0.03 revolutions (~10.8°)
- **Control mode**: Velocity mode (position=NaN, velocity=calculated)

### Why Velocity Scaling Doesn't Change Visual Speed
When P gain scales with velocity limit, the calculated velocity becomes:
```
vel = error × (max_vel × 2.0) = proportional to max_vel
```
This means at any velocity setting, the calculated velocity is a similar percentage of the limit, resulting in similar movement times.

To achieve true speed variation would require:
- Constant P gain (not velocity-scaled) → risk of overshoot at low speeds
- Position mode with velocity_limit → but this didn't reach exact positions reliably

Current implementation prioritizes **accurate position control** over variable speed control.

## Version History

- v3.0: 13-robot compact protocol with 4-digit encoding per robot
- v2.0: Multi-robot system with robot ID filtering  
- v1.0: Single robot XBee control with velocity
