# Moteus Motor Control System

## Overview
This project controls 2 Moteus R4.11 motor controllers using a CANBed FD board. The system uses external AS5600 encoders connected to the AUX2 port for position feedback and implements velocity-based control with safety features.

## Hardware
- **Motor Controllers**: 2x Moteus R4.11
- **Communication**: CANBed FD (Longan Labs) via CAN bus at 1Mbit
- **Encoders**: AS5600 magnetic encoders on AUX2 port (21:1 gear ratio)
- **Microcontroller**: Teensy (teensymm board)

## File Structure

### `config.h`
Configuration file containing all system constants:
- **Hardware Pins**: MCP2517 SPI pins (CS, SCK, SDI, SDO)
- **Motor Settings**: 
  - Motor IDs (1 and 2)
  - Direction multipliers (1.0 = normal, -1.0 = reversed)
- **Control Parameters**:
  - `TOLERANCE = 0.05` - Position accuracy (external encoder revolutions)
  - `MAX_VELOCITY = 15.0` - Maximum speed (rev/s)
  - `SLOW_ZONE = 0.10` - Distance to start slowing down
  - `BRAKE_ZONE = 0.06` - Distance for active braking
  - `LOOP_PERIOD_MS = 10` - Control loop period (100Hz)
  - `TIMEOUT_MS = 15000` - Movement timeout (15 seconds)

### `motor_control.h`
Header file declaring all motor control functions:
- `moveToEncoderPosition()` - Main position control
- `oscillateMotors()` - Test oscillation mode
- `printDiagnostics()` - Basic status output
- `printDiagnosticsAll()` - Detailed periodic diagnostics
- `checkMotorTemperature()` - Temperature safety monitor

### `motor_control.cpp`
Implementation of all motor control logic:

#### **printDiagnostics()**
- Prints basic status: external encoder position and motor temperature
- Compact single-line output for both motors

#### **checkMotorTemperature()**
- Runs every 250ms to monitor motor temperature
- If either motor exceeds 60°C:
  - Stops both motors immediately
  - Prints critical alert message
  - Halts program in infinite loop (requires reset)
- Safety feature to prevent motor damage

#### **printDiagnosticsAll()**
- Runs automatically every 100ms when enabled
- Queries and displays all available motor data:
  - Mode (operating mode number)
  - Position (motor position in revolutions)
  - Velocity (speed in rev/s)
  - Torque (in Nm)
  - Q Current and D Current (in Amps)
  - Abs Position (external encoder reading)
  - Motor Temperature (°C)
  - Voltage (V)
  - Fault code
- Enable by uncommenting `printDiagnosticsAll()` in main loop

#### **oscillateMotors()**
- Test function for motor movement
- Uses sine wave velocity commands
- Motors move in opposite phases (180° offset)
- Parameters:
  - Period: 4 seconds per cycle
  - Max velocity: 0.5 rev/s
- Type "stop" in serial to exit

#### **moveToEncoderPosition(target_ext1, target_ext2)**
The main position control function with sophisticated features:

**Control Algorithm:**
1. **Initialization**
   - Queries current encoder positions
   - Calculates initial errors
   - Sets velocity/acceleration limits (20 rev/s, 10 rev/s²)
   - Sets maximum torque to 6 Nm

2. **Control Loop (100Hz)**
   - Reads encoder positions
   - Calculates position error
   - Handles wrap-around (encoder goes 0→1 or 1→0)
   - Determines control action based on error magnitude

3. **Three Control Modes:**
   - **Within Tolerance (< 0.05 rev)**:
     - Sets velocity to 0
     - Requires 2 consecutive readings to confirm stopped
   - **Braking Zone (< 0.06 rev or overshot)**:
     - Gentle braking at 2.0 rev/s toward target
     - Prevents overshoot
   - **Normal Movement**:
     - Full speed (15 rev/s) when far from target (> 0.10 rev)
     - Proportional slowdown in slow zone (0.06-0.10 rev)
     - Minimum velocity 3.0 rev/s to maintain momentum

4. **Wrap-around Handling**
   - Encoders read 0-1 revolution
   - If error > 0.5, wraps backward (error - 1.0)
   - If error < -0.5, wraps forward (error + 1.0)
   - Always takes shortest path

5. **Overshoot Detection**
   - Tracks previous error sign
   - Detects sign change (error crossed zero)
   - Only triggers if error > 0.4 (ignores wrap-around)
   - Activates braking mode

6. **Safety Features**
   - 15-second timeout
   - Continuous temperature monitoring
   - Sends multiple stop commands when done
   - Status printing every 200ms (20 control loops)

### `main.cpp`
Main program file:

#### **Setup()**
1. Initializes serial communication (115200 baud)
2. Configures SPI bus
3. Sets up CAN bus (1Mbit, custom FIFO sizes for limited RAM)
4. Configures position format (16-bit integers for efficiency)
5. Sets initial velocity/acceleration limits
6. Sends stop commands to clear any faults
7. Prints initial motor state

#### **Loop()**
1. Checks motor temperature (every 250ms)
2. Waits for serial commands
3. Processes commands:

**Serial Commands:**
- `"1"` - Move Motor 1 to encoder position -0.25 (Motor 2 stays)
- `"2"` - Move Motor 1 to encoder position 0.25 (Motor 2 stays)
- `"3"` - Move Motor 2 to encoder position -0.25 (Motor 1 stays)
- `"4"` - Move Motor 2 to encoder position 0.25 (Motor 1 stays)
- `"osc"` - Start oscillation test mode

Each command:
1. Queries current positions
2. Preserves one motor's position
3. Moves the other motor to target
4. Uses `moveToEncoderPosition()` for smooth control

## Control Strategy

### Velocity Control vs Position Control
This system uses **velocity control** instead of traditional position control:
- Sets `position = NaN` (no position target)
- Commands velocity directly based on position error
- Allows fine-grained control over movement
- Better handling of encoder wrap-around

### Why This Approach?
1. **Wrap-around handling**: Position mode struggles with 0/1 boundary
2. **Smooth deceleration**: Proportional velocity control
3. **Overshoot prevention**: Active braking when close
4. **Fast movement**: Can command full speed when far from target

## Safety Features

### Temperature Monitoring
- Checks every 250ms
- Limit: 60°C
- Actions: Stop motors + halt program
- Requires physical reset to recover

### Movement Safety
- 15-second timeout on all movements
- Overshoot detection and correction
- Requires multiple consecutive readings to confirm stop
- Both motors stop if one fails

### Error Handling
- CAN bus error detection
- Fault status monitoring
- Mode 11 detection (stuck at limits)

## Typical Operation Sequence

1. **Power On**
   - System initializes
   - Motors stop and clear faults
   - Initial state printed

2. **Send Command** (e.g., "1")
   - System queries current positions
   - Calculates movement needed
   - Starts velocity control loop

3. **Movement Phase**
   - Fast movement (15 rev/s) when far
   - Slows down approaching target
   - Brakes gently when very close
   - Multiple readings confirm arrival

4. **Completion**
   - Prints "Both motors reached target!"
   - Sends multiple stop commands
   - Ready for next command

## Tuning Parameters

To adjust behavior, modify values in `config.h`:

- **Speed**: Change `MAX_VELOCITY` (currently 15.0 rev/s)
- **Smoothness**: Adjust `SLOW_ZONE` (larger = earlier slowdown)
- **Precision**: Modify `TOLERANCE` (smaller = more accurate)
- **Responsiveness**: Change `LOOP_PERIOD_MS` (smaller = faster updates)

## Troubleshooting

### Motor doesn't move
- Check temperature (may be halted)
- Verify CAN bus connection
- Check motor mode (should not be 11)
- Ensure encoder is connected to AUX2

### Motor oscillates
- Check `MOTOR_DIRECTION` settings in config.h
- Both should be 1.0 unless physically reversed
- Verify encoder direction matches motor direction

### Overshoot
- Reduce `MAX_VELOCITY`
- Increase `SLOW_ZONE` for earlier deceleration
- Increase `BRAKE_ZONE` for gentler stopping

### Temperature halt
- Allow motors to cool
- Reduce duty cycle
- Check for mechanical binding
- Reset microcontroller to resume

## Technical Notes

### Encoder Reading
- AS5600 provides absolute position (0-1 revolution)
- Connected via AUX2 port on Moteus
- 21:1 gear ratio between motor and encoder
- Readings in external encoder revolutions

### CAN Bus Configuration
- 1Mbit bus speed
- Custom FIFO sizes for ATmega32u4 (limited RAM)
- No BRS (Bit Rate Switching) for Arduino compatibility

### Memory Optimization
- Uses 16-bit integer format for CAN messages
- Flash strings with F() macro to save RAM
- Static variables for timing (no global pollution)
