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
- **Master Device**: USB connected, sends commands via Serial Monitor
- **Slave Devices**: Externally powered, relay and process commands

### Initialization Process
1. Device powers up and waits indefinitely for either:
   - USB Serial connection (becomes Master)
   - RX_READY signal (becomes Slave)
2. Upon role determination:
   - Performs servo sweep (60° → 120° → 90°)
   - User LED active during sweep
3. Master initiates chain discovery
4. Brief delay ensures all devices ready

### State Machine
- **INIT_WAITING**: Waiting for initialization command
- **INIT_ACTIVE**: Processing initialization sequence  
- **READY**: Ready for commands (master only)
- **PROCESSING**: Master waiting for command to return

## Serial Protocol

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

### Three Command Types:

#### 1. Initialization: `000,init,XXX`
- **Special behavior**: Increments XXX and forwards
- **Master**: Never forwards (except initial startup)
- **Slaves**: Always increment and forward
- **Auto-discovery**: Determines total device count

#### 2. Command All Devices: `000,command,value`
- **All devices**: Process the command
- **All devices**: Forward until returns to master
- **Example**: `000,servo,90` - all servos to 90°

#### 3. Single Device: `NNN,command,value`  
- **Target device only**: Processes command
- **All devices**: Forward until returns to master
- **Master**: Enters PROCESSING state until command returns
- **Example**: `002,servo,90` - only device 2 servo to 90°

## Available Commands

### Device Control
- `servo,angle` - Set servo angle (60-120 degrees)
- `dac,value` - Set DAC output (0-1023)
  - User LED active when DAC > 0

### System Commands  
- `help` - Show command help and version
- `status` - Show device status
- `reinit` - Manual re-initialization
- `dnc` - Execute dance routine

### Dance Routine
The `dnc` command executes a coordinated servo movement sequence:
1. Initial sequence (all devices):
   - Move to 60° → 500ms delay
   - Move to 90° → 500ms delay
   - Move to 120° → 500ms delay
2. Random sequence:
   - 5 random positions (60-120°)
   - 200ms delay between moves
3. Features:
   - Can be interrupted by any serial input
   - Automatically resets to 90° when complete
   - Input during dance is discarded

## Error Handling

### Command Validation
- **Format**: `ERR:FORMAT` - Invalid command format
- **Device Range**: `ERR:ID_MAX:2` - Device ID out of range
- **Value Range**: 
  - `ERR:SERVO_RANGE` - Servo angle not 60-120
  - `ERR:DAC_RANGE` - DAC value not 0-1023

### System Errors
- `ERR:BUSY` - System processing another command
- `ERR:TIMEOUT` - Command round-trip timeout
- `ERR:NO_DEVICES` - No devices detected

## Features
- **Smooth servo control** with speed limiting
- **User LED feedback** for servo and DAC operations
- **Robust initialization** with role detection
- **Machine-readable protocol** for GUI integration
- **Comprehensive error reporting**
- **Auto-recovery** from chain issues
- **Hot-swap support** with automatic detection
- **Coordinated movement** routines (dance)

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
VER:0.004
UI:Master started
UI:Type 'help' or 'status'
CMD:002,servo,90
RCV:002,servo,90
SRV:2:90
EOT
``` 