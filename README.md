# SAMD21 Round-Robin Communication Controller

Arduino sketch implementing a daisy-chained round-robin communication system for SEEEDuino XIAO (SAMD21) boards with DAC and servo control capabilities.

**Current Version**: v0.002

## Hardware Requirements
- **SEEEDuino XIAO** (SAMD21 based)
- **A0 (DAC)** → amplified to Buck_DIM net for LED control
- **D2 (PWM)** → logic level converted to 5V SERVO net
- **D1 (RX_READY)** → interrupt input for receiving commands
- **D3 (TX_READY)** → output to signal next device
- **D6 (TX), D7 (RX)** → Serial1 communication to next/previous device

## Communication Architecture

### Round-Robin Chain
Devices are daisy-chained: Master → Device2 → Device3 → ... → Master

### Device Roles
- **Master Device**: USB connected, sends commands via Serial Monitor
- **Slave Devices**: Externally powered, relay and process commands

### State Machine
- **INIT_WAITING**: Waiting for initialization command
- **INIT_ACTIVE**: Processing initialization sequence  
- **READY**: Ready for commands (master only)
- **PROCESSING**: Master waiting for command to return

## Command Protocol

### Format: `deviceId,command,value`

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
- **Example**: `002,servo,45` - only device 2 servo to 45°

## Available Commands

### Device Control
- `servo,angle` - Set servo angle (0-180 degrees)
- `dac,value` - Set DAC output (0-1023)

### System Commands  
- `help` - Show command help and version
- `status` - Show device status and version
- `reinit` - Manual re-initialization

## Command Validation

### Device Range Validation
- **Automatic validation**: Commands only sent to detected devices
- **Range checking**: Device IDs must be 000 (all) or 001-totalDevices
- **Clear error messages**: Shows valid device range when invalid ID used
- **Initialization awareness**: Blocks commands until system ready

### Error Examples
```
> 003,servo,90
ERROR: Device 3 not found. Valid range: 000 (all), 001-2

> 002,servo,90     // Before initialization
ERROR: No devices detected yet. Wait for initialization to complete.
```

## Auto-Recovery Features

### Smart Auto Re-initialization
- **Problem-based triggering**: Only re-initializes for actual issues
- **Stuck initialization**: Triggers if stuck in init states >5 seconds
- **Zero devices detected**: Triggers if no devices found after init
- **Stable operation**: Does NOT re-initialize during normal waiting
- **Hot-swap support**: Automatically detects device changes

### Command Timeout Protection
- **2.5 second timeout** for PROCESSING state
- **Prevents hanging** on lost commands
- **Automatic state recovery** returns to READY state

## Version Tracking

### Code Versioning
- **Version display**: Shown at startup, help, and status
- **Increment tracking**: Version number increases with each code change
- **Multi-device consistency**: Ensure all devices run same version
- **Debug assistance**: Easy identification of running code version

### Version Commands
```
> help     // Shows version in help header
> status   // Shows version in status header
```

## Pin Mapping
```
A0  - DAC        D6  - TX
D1  - RX_READY   D7  - RX  
D2  - PWM        D8  - D8
D3  - TX_READY   D9  - D9
D4  - SDA        D10 - D10_LED
D5  - SCL
```

## Usage Examples

### Master Device (USB Connected)
```
SAMD21 Controller v0.002 - Round Robin Master Started
Starting device initialization sequence...
Initialization complete. Total devices: 2
Master entering READY state - waiting for Serial commands

> help              // Show commands and version
> status            // Show system status  
> 000,servo,90      // All devices servo to 90°  
> 002,dac,512       // Device 2 DAC to 512
> 003,servo,45      // ERROR: Device 3 not found. Valid range: 000 (all), 001-2
> reinit            // Re-discover devices
```

### Communication Flow Example
```
Master: "002,servo,90" → Device2 → Device3 → Master
         ↓ ignores      ↓ processes ↓ ignores  ↓ "Command completed"
```

### Validation Examples
```
> 000,servo,90      ✅ Valid: Command all devices
> 001,dac,512       ✅ Valid: Device 1 (master)
> 002,servo,45      ✅ Valid: Device 2 exists
> 003,servo,90      ❌ Invalid: Device 3 doesn't exist
> 999,dac,100       ❌ Invalid: Device 999 doesn't exist
```

## Features
- **Version tracking** with automatic display
- **Device range validation** prevents invalid commands
- **Interrupt-driven** communication (RX_READY pin)
- **9600 baud** Serial1 daisy-chain communication  
- **115200 baud** USB Serial Monitor interface
- **Smart command validation** with helpful error messages
- **Intelligent state machine** prevents command conflicts
- **Problem-based auto-recovery** from chain issues
- **Hot-swap device support** with automatic detection
- **Comprehensive error reporting** with valid range display 