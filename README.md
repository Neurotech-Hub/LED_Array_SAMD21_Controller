# SAMD21 Round-Robin Communication Controller

Arduino sketch implementing a daisy-chained round-robin communication system for SEEEDuino XIAO (SAMD21) boards with DAC and servo control capabilities.

**Current Version**: v0.004

## 🚀 What Does This Do?

This project creates a **smart chain of controllers** that can:
- Control LED brightness on multiple devices simultaneously or individually
- Move servo motors to precise positions (60-120 degrees)
- Communicate through a daisy-chain setup (like Christmas lights, but smarter!)
- Automatically detect how many devices are connected
- Provide visual feedback when something goes wrong

Think of it as a "conductor" for an orchestra of LED arrays and servo motors - you give one command, and all devices work together harmoniously.

## 🛠️ Hardware You'll Need

- **SEEEDuino XIAO** (SAMD21 based) - one for each device in your chain
- **LED arrays** connected to the DAC output (A0)
- **Servo motors** (optional) connected to PWM output (D2)
- **Connection cables** for the daisy chain
- **Power supply** for each device (except the master which uses USB)

## 📍 Pin Connections

| Pin | Function | Connection |
|-----|----------|------------|
| A0  | DAC Output | → LED Array Control (amplified) |
| D2  | PWM Output | → Servo Motor (5V logic level) |
| D1  | RX_READY | ← Signal from previous device |
| D3  | TX_READY | → Signal to next device |
| D6  | TX | → Data to next device |
| D7  | RX | ← Data from previous device |
| D10 | User LED | Built-in status indicator |

## 🔗 How the Chain Works

```
[Master Device] → [Device 2] → [Device 3] → [Device 4] → ... → [Back to Master]
    (USB)           (Power)      (Power)      (Power)
```

1. **Master Device**: Connected to your computer via USB, sends commands
2. **Slave Devices**: Receive power externally, relay commands around the chain
3. **Auto-Discovery**: The system automatically figures out how many devices you have

## 🎮 Command Examples

### Control All Devices at Once (Use ID: 000)

```
// Turn on all LEDs to half brightness
000,dac,512

// Move all servos to 90 degrees
000,servo,90

// Turn off all LEDs
000,dac,0
```

### Control Individual Devices

```
// Control Device 1 only
001,servo,120     // Move device 1 servo to 120 degrees
001,dac,1023      // Set device 1 LEDs to full brightness

// Control Device 2 only  
002,servo,60      // Move device 2 servo to 60 degrees
002,dac,0         // Turn off device 2 LEDs

// Control Device 3 only
003,dac,256       // Set device 3 LEDs to quarter brightness
```

### System Commands

```
help              // Show all available commands
status            // Check system status and connected devices
```

## 🚨 Troubleshooting

### Blue Light Stuck On?

If you see the **blue LED staying on continuously**, it means:
- The device is stuck in an error state
- Communication with the chain has been interrupted
- The device needs to be reset

**Solution**: Press the **reset button** on the PCB to restart the device and restore the chain.

### Common Issues and Solutions

| Problem | Indication | Solution |
|---------|------------|----------|
| Device stuck | Blue LED stays on | Press reset button on PCB |
| Chain broken | Commands not reaching all devices | Check all connections, reset devices |
| Wrong device count | Commands fail | Run `status` command to check detection and `reinit` command to fix it |

## 💡 Visual Feedback

Your devices give you helpful visual cues:

- **Blue LED Steady**: Normal operation or device stuck (see troubleshooting)
- **Blue LED During Startup**: Device is performing initial servo sweep (normal)
- **Orange LED**: Blinking in normal state
- **User LED Active**: When DAC output > 0

## 🔧 Setup Process

1. **Connect Hardware**: Wire up your devices according to the pin diagram
2. **Power On**: Connect USB to master device, external power(12v) to slaves
3. **Auto-Discovery**: Devices automatically detect the chain and assign IDs
4. **Test**: Try `status` command to see all detected devices
5. **Control**: Start sending commands!

## 📱 Command Format Guide

All commands follow this simple pattern:
```
[Device ID],[Command],[Value]
```

### Device IDs
- `000` = All devices (broadcast command)
- `001` = First device (master)
- `002` = Second device  
- `003` = Third device
- etc.

### Commands
- `servo` = Control servo motor (values: 60-120 degrees)
- `dac` = Control LED brightness (values: 0-1023)

### Complete Examples
```
// Set all devices to 75% LED brightness
000,dac,768

// Move only device 2's servo to 90 degrees
002,servo,90

// Turn off LEDs on device 1, move its servo to 60 degrees
001,dac,0
001,servo,60
```

## 🔍 System Messages

The controller provides helpful feedback:

### Success Messages
```
CMD:002,servo,90    // Command being sent
RCV:002,servo,90    // Command received
SRV:2:90           // Servo moved to position
DAC:2:512          // DAC set to value
EOT                // End of transmission
```

### Error Messages
```
ERR:INVALID_DEVICE:4 (max:3)  // Device doesn't exist
ERR:SERVO_RANGE               // Angle not 60-120
ERR:DAC_RANGE                 // Value not 0-1023
ERR:TIMEOUT                   // Command failed
```

## 🎯 Quick Start Example

1. Open Arduino Serial Monitor (9600 baud)
2. Type `status` to see your devices
3. Try these commands:
   ```
   000,dac,500        // Half brightness on all LEDs
   000,servo,90       // Center all servos
   001,dac,1023       // Full brightness on device 1 only
   002,servo,120      // Move device 2 servo to max angle
   000,dac,0          // Turn off all LEDs
   ```

## 📋 Technical Specifications

- **Microcontroller**: SAMD21 (SEEEDuino XIAO)
- **Communication**: Serial daisy-chain
- **Servo Range**: 60-120 degrees
- **DAC Range**: 0-1023
- **Max Chain Length**: Limited by power and timing
- **Baud Rate**: 9600

## 🤝 Need Help?

- Check the troubleshooting section above
- Use the `help` command for quick reference
- Use the `status` command to diagnose chain issues

---

*Happy controlling! 🎛️*