# SAMD21 LED Array Controller

Arduino sketch for SEEEDuino XIAO (SAMD21) controlling LED arrays and servo motors.

## Hardware
- **SEEEDuino XIAO** (SAMD21 based)
- **A0 (DAC)** → amplified to Buck_DIM net
- **D2 (PWM)** → logic level converted to 5V SERVO net

## Functionality
- Cycles DAC output (0-1023) for LED dimming control
- Sweeps servo motor (20-160°) 
- Built-in LED status indication
- Serial monitoring at 115200 baud

## Pin Mapping
```
A0  - DAC        D6  - TX
D1  - RX_READY   D7  - RX  
D2  - PWM        D8  - D8
D3  - TX_READY   D9  - D9
D4  - SDA        D10 - D10_LED
D5  - SCL
``` 