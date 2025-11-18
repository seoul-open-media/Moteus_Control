# XBee Multi-Robot Control System

## Overview
Control up to 100 robots simultaneously using XBee wireless communication. Each robot has 2 motors with independent position and velocity control.

## Protocol Format
```
255 robot_id motor_num pos_MSB pos_LSB velocity
```

### Bytes:
- **Byte 1**: `255` (0xFF) - Header
- **Byte 2**: `robot_id` - Robot ID (0 = broadcast to ALL, 1-100 = specific robot)
- **Byte 3**: `motor_num` - Motor number (1 or 2, or 4 for STOP)
- **Byte 4**: `pos_MSB` - Position high byte
- **Byte 5**: `pos_LSB` - Position low byte  
- **Byte 6**: `velocity` - Velocity (0-255 maps to 1-100 rev/s)

## Configuration

### Setting Robot ID
Edit `src/config.h` and change the `ROBOT_ID`:
```cpp
static const uint8_t ROBOT_ID = 1;  // Change for each robot (1-100)
```

**IMPORTANT**: Each robot MUST have a unique ID (1-100)

### Recommended XBee Setup
- **Type**: XBee 802.15.4 (Series 1) or XBee 3 802.15.4
- **Mode**: Transparent/AT mode
- **Baud**: 115200
- **Network**: 
  - Host (Pure Data): Coordinator
  - Robots: End devices with unique 16-bit addresses

## Pure Data Control

### Broadcast Commands (All Robots)
Use `robot_id = 0` to send commands to all robots simultaneously:
```
255 0 1 pos_MSB pos_LSB vel  → All robots Motor 1
255 0 2 pos_MSB pos_LSB vel  → All robots Motor 2
255 0 4                       → STOP all robots
```

### Individual Robot Commands
Use `robot_id = 1-100` to control specific robots:
```
255 5 1 pos_MSB pos_LSB vel   → Robot #5 Motor 1
255 42 2 pos_MSB pos_LSB vel  → Robot #42 Motor 2
255 99 4                       → STOP Robot #99
```

## Control Patches

### moteus_xbee_unified.pd
- Single robot control (for testing)
- Use when ROBOT_ID = 1

### moteus_xbee_multirobot.pd  
- Multi-robot control interface
- Broadcast controls (red sliders) - affect all robots
- Individual controls (yellow ID selector) - target specific robot
- Quick presets for choreography

## Features

### Position Control
- Range: -90° to +90° (-0.25 to +0.25 revolutions)
- 180° offset applied automatically
- Position constraints enforced

### Velocity Control
- Range: 1-100 rev/s (configurable up to 500 rev/s)
- Independent per motor
- Square root gain scaling for smooth response
- Anti-overshoot tuning at slow speeds

### Safety Features
- Emergency STOP (broadcast to all robots)
- Kill button stops motors and disables control
- Position limits prevent over-rotation

## Manufacturing Setup

For each of 100 robots:

1. **Flash firmware** with default ROBOT_ID = 1
2. **Change ROBOT_ID** in config.h (1-100)
3. **Re-upload** firmware
4. **Configure XBee**:
   - Set unique 16-bit address
   - Set same PAN ID for all units
   - Set baud to 115200
5. **Label robot** with its ID number
6. **Test** with moteus_xbee_multirobot.pd

## Network Architecture

```
Pure Data Host (XBee Coordinator)
        |
        ├─ Broadcast (ID=0) ──> All Robots
        |
        ├─ Robot #1 (ID=1)
        ├─ Robot #2 (ID=2)
        ├─ Robot #3 (ID=3)
        ⋮
        └─ Robot #100 (ID=100)
```

## Choreography Examples

### Synchronized Movement
```
255 0 1 127 255 51  → All robots M1 to +0.25 at 50 rev/s
```

### Wave Pattern
Send individual commands with timing:
```
255 1 1 pos vel
delay 50ms
255 2 1 pos vel
delay 50ms
255 3 1 pos vel
...
```

### Groups
Group robots by ID ranges:
- Robots 1-25: Group A
- Robots 26-50: Group B
- Robots 51-75: Group C
- Robots 76-100: Group D

Send individual group commands from Pure Data.

## Troubleshooting

### Robot not responding
- Check ROBOT_ID matches command
- Verify XBee address configuration
- Check power and CAN bus connections
- Monitor serial output: "Robot X M1: pos=..."

### All robots respond to individual command
- ROBOT_ID = 0 in config.h (should be 1-100)
- XBee not filtering addresses

### Latency issues
- Reduce number of simultaneous commands
- Increase speedlim in Pure Data (50ms recommended)
- Check XBee signal strength

## Performance

- **Command latency**: ~10-20ms
- **Control rate**: 100Hz (per robot)
- **Update rate from PD**: 20Hz per robot (50ms speedlim)
- **Network capacity**: 100 robots tested
- **Range**: 100m indoor (XBee 802.15.4 with wire antenna)

## Version History

- v2.0: Multi-robot system with robot ID filtering
- v1.0: Single robot XBee control with velocity
