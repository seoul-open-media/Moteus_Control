# XBee Wireless Control with Pure Data

## Hardware Setup

### XBee Connection to Teensy
- XBee TX → Teensy Serial1 RX
- XBee RX → Teensy Serial1 TX  
- XBee VCC → 3.3V
- XBee GND → GND

**Note**: XBee operates at 3.3V logic levels. Teensy 2.0 is 5V, so you may need a level shifter.

### XBee Configuration
Both XBee modules (on PC and Teensy) should be configured with:
- Baud rate: 115200
- API mode: Disabled (Transparent mode)
- Same PAN ID
- Coordinator (PC) and Router (Teensy) or both as AT mode

## Command Protocol

### From Pure Data to Teensy

#### Single Motor Control
```
M1 0.5      → Move Motor 1 to position 0.5 revolutions
M2 -0.25    → Move Motor 2 to position -0.25 revolutions
```

#### Both Motors Control
```
BOTH 0.5 0.75    → Move M1 to 0.5, M2 to 0.75
B 0.0 0.25       → Move M1 to 0.0, M2 to 0.25 (short form)
```

#### Control Commands
```
STOP        → Emergency stop (also: S)
OSC         → Start oscillation mode (also: OSCILLATE)
STATUS      → Query current positions and temperatures (also: ?)
```

### From Teensy to Pure Data

#### Acknowledgment
```
ACK: <command>   → Confirms command received
```

#### Status Response
```
STATUS M1:0.5000 T:35.2 M2:0.7500 T:36.1
```

#### Error Messages
```
ERROR: Unknown command: <command>
```

## Pure Data Patch Example

### Basic Setup Objects

1. **comport** object to communicate with XBee:
   - Open correct COM port (check Device Manager on Windows, /dev/ttyUSB* on Linux)
   - Set baud rate: 115200

2. **Message formatting**:
   - Use [symbol] or [makefilename] to format commands
   - Add line ending with [add 10] (newline character)

### Example Patch Structure

```
[hslider]  → [/ 100]  → [- 0.5]  → Motor 1 position (range -0.5 to 0.5)
                          ↓
                    [makefilename M1 %g]
                          ↓
                    [add 10]  (add newline)
                          ↓
                    [comport 1 115200]  (XBee serial port)


[bang]  →  [symbol STOP\n]  →  [comport 1 115200]


[metro 1000]  →  [symbol STATUS\n]  →  [comport 1 115200]
                                             ↓
                                    [print response]
```

### Receiving Status

To parse incoming status messages:
```
[comport 1 115200]
    ↓
[route STATUS ACK ERROR]
    ↓              ↓          ↓
[STATUS msg]  [ACK msg]  [ERROR msg]
```

## Command Examples from Pure Data

### Send Position Commands
```pd
[0.5(  → [makefilename M1 %g] → [add 10] → [comport]   // M1 to 0.5
[0.75( → [makefilename M2 %g] → [add 10] → [comport]   // M2 to 0.75
```

### Send Both Motors
```pd
[0.5 0.75(  → [pack f f] → [makefilename BOTH %g %g] → [add 10] → [comport]
```

### Emergency Stop Button
```pd
[bang( → [symbol STOP\n] → [comport]
```

### Continuous Position Control with Sliders
```pd
// Motor 1 slider (0-100) scaled to (-0.5 to 0.5)
[hslider]
    ↓
[/ 100]     // Convert to 0.0-1.0
    ↓
[- 0.5]     // Shift to -0.5 to 0.5
    ↓
[makefilename M1 %.3f]
    ↓
[add 10]
    ↓
[comport 1 115200]
```

### Status Polling
```pd
[tgl]  → [metro 500]  → [symbol STATUS\n]  → [comport 1 115200]
                                                      ↓
                                              [print response]
                                                      ↓
                                              [route STATUS]
                                                      ↓
                                            [parse positions]
```

## Teensy Serial Ports

- **Serial** (USB): Used for debugging (Serial Monitor)
- **Serial1**: Used for XBee wireless communication
  - Teensy 2.0: Pin 7 (RX), Pin 8 (TX)
  - Teensy LC/3.x/4.x: Pin 0 (RX), Pin 1 (TX)

## Troubleshooting

### No response from Teensy
- Check XBee modules are paired (same PAN ID)
- Verify baud rate matches (115200)
- Check physical connections (TX↔RX crossover)
- Verify XBee module on PC is detected as COM port
- Check XBee power (3.3V, not 5V)

### Commands not executing
- Check command format matches protocol
- Ensure newline character is sent (\n)
- Monitor Serial (USB) output for debug messages
- Send STATUS command to verify communication

### Level Shifting
If using Teensy 2.0 (5V logic):
- Use bi-directional level shifter between Teensy and XBee
- XBee is 3.3V tolerant on TX but needs 3.3V on RX

## Integration with Motor Control

XBee commands integrate with existing system:
- **Command interrupts**: XBee commands can interrupt ongoing movements
- **Emergency stop**: NeoKey button or XBee STOP both work
- **Display feedback**: XBee commands show on OLED display
- **Encoder safety**: Stall detection still active with XBee control
