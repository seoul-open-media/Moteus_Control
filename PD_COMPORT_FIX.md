# Pure Data XBee Control - Quick Fix

## Problem
The `comport` object can't process symbols directly. You're getting "no method for 'M1'" error.

## Solution Options

### Option 1: Use [mrpeach/comport] with [any2bytes]
Install mrpeach externals:
```
File -> Find Externals -> Search "mrpeach" -> Install
```

### Option 2: Simple Symbol Send (Easiest)
The comport expects a different format. Here's the simplest working patch:

```
[M1 0.5(  →  [any2bytes]  →  [list append 10]  →  [comport /dev/ttyUSB_Xbee 115200]
```

### Option 3: Use [str] object (if available)
```
[M1 0.5(  →  [makefilename %s]  →  [str fromsymbol]  →  [list append 10]  →  [comport]
```

## Working Example Patch

Create a new patch with this simple test:

1. Create objects:
   - `[comport /dev/ttyUSB_Xbee 115200]`
   - `[print received]` (connect to comport output)
   - `[send 77 49 32 48 46 53 10(` (this is "M1 0.5\n" in ASCII)

2. The message box `[send 77 49 32 48 46 53 10(` sends:
   - 77 = 'M'
   - 49 = '1'
   - 32 = ' ' (space)
   - 48 = '0'
   - 46 = '.'
   - 53 = '5'
   - 10 = '\n' (newline)

## ASCII Conversion Helper

To send "M1 0.5":
```
M = 77
1 = 49
space = 32
0 = 48
. = 46
5 = 53
\n = 10
```

## Quick Test Commands

Try these message boxes:
- `[send 77 49 32 48 10(` → "M1 0\n"
- `[send 77 50 32 48 46 50 53 10(` → "M2 0.25\n"
- `[send 83 84 79 80 10(` → "STOP\n"

Connect each directly to: `[comport /dev/ttyUSB_Xbee 115200]`

## Recommended: Install mrpeach

The easiest long-term solution:
1. Install mrpeach externals
2. Use `[any2bytes]` to convert symbols to ASCII
3. This handles the conversion automatically

Would you like me to create a simple working patch using the ASCII byte method?
