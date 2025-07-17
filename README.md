# SAMD21 Round-Robin Communication Controller

Arduino sketch implementing a daisy-chained round-robin communication system for SEEEDuino XIAO (SAMD21) boards with DAC and servo control capabilities.

**Current Version**: v0.004

## Hardware Requirements
- **SEEEDuino XIAO** (SAMD21 based)
- **A0 (DAC)** → amplified to Buck_DIM net for LED control
- **D2 (PWM)** → logic level converted to 5V SERVO net (60-120 degrees)
- **D1 (RX_READY)** → interrupt input for receiving commands
- **D3 (TX_READY)** → output to signal next device
- **D6 (TX), D7 (RX)** → Serial1 communication to next/previous device
- **D10** → User LED output (active during servo sweep, DAC > 0)

## Communication Architecture

### Round-Robin Chain
Devices are daisy-chained: Master → Device2 → Device3 → ... → Master

### Device Roles
- **Master Device**: USB connected (always device 001), sends commands via Serial Monitor
- **Slave Devices**: Externally powered, relay and process commands

### Initialization Process
1. Device powers up and waits indefinitely for either:
   - USB Serial connection (becomes Master, ID=001)
   - RX_READY signal (becomes Slave)
2. Upon role determination:
   - Performs servo sweep (60° → 120° → 90°)
   - User LED active during sweep
3. Master initiates chain discovery
4. Each slave increments device count for auto-addressing

### State Machine
- **WAITING_FOR_CHAIN**: Waiting for RX_READY to go HIGH
- **CHAIN_READY**: Normal operation state
- **INIT_IN_PROGRESS**: Master initializing chain
- **PROCESSING**: Command in progress
- **READY**: Ready for next command

## Command Protocol

### Message Format
All messages use prefixed format for easy parsing:

#### System Messages
- `VER:0.004` - Version information
- `CMD:xxx` - Command being sent
- `RCV:xxx` - Command received
- `EOT` - End of transmission
- `ERR:xxx` - Error messages
- `WARN:xxx` - Warning messages
- `INIT:xxx` - Initialization messages
- `SRV:id:angle` - Servo position update
- `DAC:id:value` - DAC value update

#### User Interface Messages
- `UI:xxx` - Human-readable messages
- Filtered by GUI for display purposes

### Command Format: `deviceId,command,value`

### Command Types:

#### 1. Device Control
- `xxx,servo,angle` - Set servo angle (60-120 degrees)
- `xxx,dac,value` - Set DAC output (0-1023)
  - User LED active when DAC > 0
  - Where xxx is device ID (000=all, 001-n=specific device)

#### 2. System Commands
- `help` - Show command help and version
- `status` - Show system status

### Error Messages
- `ERR:INVALID_FORMAT` - Command format error
- `ERR:INVALID_DEVICE:n (max:m)` - Device ID out of range
- `ERR:SERVO_RANGE` - Angle not 60-120
- `ERR:DAC_RANGE` - Value not 0-1023
- `ERR:TIMEOUT` - Command timeout

## Features
- **Smooth servo control** with speed limiting
- **User LED feedback** for servo and DAC operations
- **Robust initialization** with role detection
- **Machine-readable protocol** for GUI integration
- **Comprehensive error reporting**
- **Auto-recovery** from chain issues
- **Hot-swap support** with automatic detection

## Pin Mapping
```
A0  - DAC        D6  - TX
D1  - RX_READY   D7  - RX  
D2  - PWM        D8  - D8
D3  - TX_READY   D9  - D9
D4  - SDA        D10 - User LED
D5  - SCL
```

## Communication Example
```
// Device Control
CMD:002,servo,90
RCV:002,servo,90
SRV:2:90
EOT

// System Command
help
UI:Available Commands:
UI:  Device Control:
UI:    xxx,servo,angle - Set servo angle (60-120)
UI:    xxx,dac,value   - Set DAC value (0-1023)
...

// Error Example
CMD:003,servo,90
ERR:INVALID_DEVICE:3 (max:2)
``` 