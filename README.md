# Moteus Motor Control System

## Overview
This project controls 2 Moteus R4.11 motor controllers using a CANBed FD board. The system uses external AS5600 encoders connected to the AUX2 port for position feedback and implements velocity-based control with safety features.

## Hardware
- **Motor Controllers**: 2x Moteus R4.11
- **Communication**: CANBed FD (Longan Labs) via CAN bus at 1Mbit
- **Encoders**: AS5600 magnetic encoders on AUX2 port (21:1 gear ratio)
- **Microcontroller**: Teensy (teensymm board)
- **Display**: Adafruit 128x64 OLED (SSD1306) on I2C bus 1 (Wire1)
- **Controls**: Adafruit NeoKey 1x4 on I2C bus 0 (Wire)

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
- **Display Settings**:
  - OLED screen dimensions (128x64)
  - I2C address 0x3C on Wire1
  - Update rate: 10Hz

### `display.h` / `display.cpp`
OLED display management system:
- **Real-time Display**: Shows motor position and temperature at 10Hz
- **Layout**:
  - Line 0: "MOTEUS CONTROL" title
  - Line 12: M1: [position]r [temp]C
  - Line 25: M2: [position]r [temp]C
- **Debug Message System**: 
  - Circular buffer of last 6 messages
  - Auto-displays for 3 seconds
  - Use `displayDebug("message")` to add messages
- **Temperature Warnings**:
  - "WARM!" displayed at 50°C
  - "HOT!!!" with inverted display at 60°C
- **I2C Configuration**: Wire1 at 400kHz, address 0x3C

### `neokey.h` / `neokey.cpp`
Physical button control with Adafruit NeoKey 1x4:
- **Button Mappings**:
  - Key 0 (GREEN): Move both motors to position 0.0
  - Key 1 (BLUE): Move both motors to position 0.25
  - Key 2 (CYAN): Move both motors to position 0.75
  - Key 3 (RED): Emergency STOP - halts motors immediately
- **LED Feedback**:
  - Dim colors when ready
  - Bright when pressed
  - Flashing red for STOP button
- **Emergency Stop**: Works even during motor movement (checked every 50ms)
- **I2C Configuration**: Wire (bus 0) at 100kHz, address 0x30

### `motor_control.h` / `motor_control.cpp`
Header file declaring all motor control functions:
- `moveToEncoderPosition()` - Main position control with interrupt support
- `oscillateMotors()` - Test oscillation mode with interrupt support
- `printDiagnostics()` - Basic status output
- `printDiagnosticsAll()` - Detailed periodic diagnostics
- `checkMotorTemperature()` - Temperature safety monitor

**Interrupt System**:
- Global flags: `interruptCommand`, `commandRunning`, `pendingCommand`
- Serial commands can interrupt ongoing movements
- NeoKey STOP button interrupts movements
- Pending commands execute immediately after interrupt
- NeoKey checked every 50ms during motor movements

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
2. Initializes OLED display on Wire1
3. Runs I2C scanner to detect devices
4. Initializes NeoKey on Wire
5. Configures SPI bus
6. Sets up CAN bus (1Mbit, custom FIFO sizes for limited RAM)
7. Configures position format (16-bit integers for efficiency)
8. Sets initial velocity/acceleration limits
9. Sends stop commands to clear any faults
10. Prints initial motor state

#### **Loop()**
1. Queries motor positions and temperatures (every 50ms / 20Hz)
2. Updates OLED display (every 100ms / 10Hz)
3. Updates NeoKey (checks for button presses)
4. Checks for pending commands (from interrupts)
5. Checks motor temperature (every 250ms)
6. Processes serial commands

**Serial Commands:**
- `"1"` - Move Motor 1 to encoder position -0.25 (Motor 2 stays)
- `"2"` - Move Motor 1 to encoder position 0.25 (Motor 2 stays)
- `"3"` - Move Motor 2 to encoder position -0.25 (Motor 1 stays)
- `"4"` - Move Motor 2 to encoder position 0.25 (Motor 1 stays)
- `"osc"` - Start oscillation test mode
- Any command interrupts ongoing movement

**NeoKey Physical Controls:**
- **Key 0** (Green LED): Move both motors to position 0.0
- **Key 1** (Blue LED): Move both motors to position 0.25
- **Key 2** (Cyan LED): Move both motors to position 0.75
- **Key 3** (Red LED): Emergency STOP - halts motors immediately
- STOP button works even during movement (50ms response time)

Each command:
1. Queries current positions
2. Preserves one motor's position (serial) or moves both (NeoKey)
3. Moves motor(s) to target
4. Uses `moveToEncoderPosition()` for smooth control
5. Can be interrupted by new serial command or STOP button

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

### Command Interrupt System
The system supports real-time command interrupts:
- **Serial Commands**: New command stops current movement and executes immediately
- **NeoKey STOP**: Emergency stop button works even during movement
- **Implementation**:
  - `interruptCommand` flag checked every 10ms in control loop
  - `commandRunning` flag prevents false interrupts
  - `pendingCommand` stores new command for immediate execution
  - NeoKey checked every 50ms inside motor control functions
- **Response Time**: < 50ms to interrupt and start new movement

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
   - OLED display shows "MOTEUS CONTROL"
   - I2C scanner detects devices
   - NeoKey LEDs light up (dim colors)
   - Motors stop and clear faults
   - Initial state printed to serial and display

2. **Send Command** (e.g., serial "1" or press NeoKey button)
   - Display shows command being processed
   - System queries current positions
   - Calculates movement needed
   - Sets `commandRunning = true`
   - Starts velocity control loop

3. **Movement Phase**
   - Fast movement (15 rev/s) when far
   - Slows down approaching target
   - Brakes gently when very close
   - Display updates position/temperature at 10Hz
   - NeoKey checked every 50ms for STOP button
   - Serial checked every 10ms for interrupt commands
   - Multiple readings confirm arrival

4. **Completion**
   - Prints "Both motors reached target!"
   - Sends multiple stop commands
   - Sets `commandRunning = false`
   - Display returns to normal status view
   - NeoKey LEDs reset to dim
   - Ready for next command

5. **Emergency Stop** (anytime)
   - Press red NeoKey button
   - Motors halt within 50ms
   - LED flashes red
   - System ready for new command immediately

## Tuning Parameters

To adjust behavior, modify values in `config.h`:

- **Speed**: Change `MAX_VELOCITY` (currently 15.0 rev/s)
- **Smoothness**: Adjust `SLOW_ZONE` (larger = earlier slowdown)
- **Precision**: Modify `TOLERANCE` (smaller = more accurate)
- **Responsiveness**: Change `LOOP_PERIOD_MS` (smaller = faster updates)

## Troubleshooting

### Motor doesn't move
- Check temperature on OLED display (may be halted at 60°C)
- Verify CAN bus connection
- Check motor mode (should not be 11)
- Ensure encoder is connected to AUX2
- Check if STOP button was pressed (red LED)

### Motor oscillates
- Check `MOTOR_DIRECTION` settings in config.h
- Both should be 1.0 unless physically reversed
- Verify encoder direction matches motor direction

### Overshoot
- Reduce `MAX_VELOCITY`
- Increase `SLOW_ZONE` for earlier deceleration
- Increase `BRAKE_ZONE` for gentler stopping

### Temperature halt
- OLED will show "HOT!!!" warning
- Allow motors to cool
- Reduce duty cycle
- Check for mechanical binding
- Reset microcontroller to resume

### OLED display not working
- Check I2C scanner output in serial monitor
- Verify device at address 0x3C on Wire1
- Check Wire1 SDA/SCL connections
- Ensure display library is installed

### NeoKey not responding
- Check I2C scanner for device at 0x30 on Wire
- Verify Wire (bus 0) SDA/SCL connections
- Look for initialization messages in serial monitor
- Check for I2C address conflicts

### STOP button doesn't work during movement
- This should work - STOP is checked every 50ms
- Check serial monitor for "STOP button pressed during movement!"
- Verify NeoKey library is up to date
- Ensure no I2C communication errors

### Display shows old values
- Motors are queried every 50ms automatically
- If values frozen, check CAN bus communication
- Verify motor controllers are responding
- Check for timeout errors in serial monitor

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

### I2C Bus Architecture
- **Wire (I2C bus 0)**: NeoKey 1x4 at 100kHz
  - Address: 0x30
  - Slower clock for reliable Seesaw communication
- **Wire1 (I2C bus 1)**: OLED display at 400kHz
  - Address: 0x3C
  - Faster clock for display updates
- **Separation Reason**: Prevents I2C conflicts and timing issues
- **I2C Scanner**: Runs at startup to verify device detection

### Memory Optimization
- Uses 16-bit integer format for CAN messages
- Flash strings with F() macro to save RAM
- Static variables for timing (no global pollution)
- Circular buffer for debug messages (6 messages max)

### Update Rates
- **Motor Control Loop**: 100Hz (10ms period)
- **Motor Queries**: 20Hz (50ms period)
- **Display Update**: 10Hz (100ms period)
- **Temperature Check**: 4Hz (250ms period)
- **NeoKey Check in Loop**: Every iteration (~1ms)
- **NeoKey Check During Movement**: 20Hz (50ms period)

### Libraries Used
- `Moteus` v1.0.2 - Motor controller communication
- `ACAN2517FD` - MCP2517 CAN controller driver
- `Adafruit_SSD1306` v2.5.7 - OLED display driver
- `Adafruit_GFX` v1.11.3 - Graphics library
- `Adafruit_NeoPixel` v1.12.0 - LED control for NeoKey
- `Adafruit_Seesaw` v1.7.0 - NeoKey I2C interface
