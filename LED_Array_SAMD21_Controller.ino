/*
  SAMD21_Controller.ino

  This sketch cycles through all valid 10-bit PWM values (0-1023) on A0 (DAC) using analogWrite
  AND controls a servo motor on D2 (PWM), sweeping through all angles (20-160 degrees)
  for the SEEEDuino XIAO board. It provides visual feedback using the built-in
  LED and serial output for monitoring.

  Hardware:
  - SEEEDuino XIAO (SAMD21 based)
  - A0 (DAC) for analog output - amplified to Buck_DIM net
  - D2 (PWM) for servo control - logic level converted to 5V SERVO net
  - Servo motor connected to SERVO net
  - Built-in LED for status indication

  Pin-Schematic Mapping:
  A0  - DAC
  D1  - RX_READY
  D2  - PWM
  D3  - TX_READY
  D4  - SDA
  D5  - SCL
  D6  - TX
  D7  - RX
  D8  - D8
  D9  - D9
  D10 - D10_LED
*/

#include <Arduino.h>
#include <Servo.h>

// Pin assignments based on schematic naming
const int dacPin = A0;          // A0 (DAC) - amplified to Buck_DIM net
const int pwmPin = D2;          // D2 (PWM) - logic level converted to 5V SERVO net
const int ledPin = LED_BUILTIN; // Built-in LED for status feedback

// Analog output variables
const int maxPwmValue = 1023; // 10-bit PWM maximum value (2^10 - 1)
int currentPwmValue = 0;      // Current PWM value
bool pwmIncreasing = true;    // Direction of PWM sweep

// Servo variables (MG90S safe range to avoid buzzing/damage)
const int maxServoAngle = 160; // Maximum servo angle in degrees (safe for MG90S)
const int minServoAngle = 20;  // Minimum servo angle in degrees (safe for MG90S)
Servo myServo;                 // Servo object
int currentAngle = 20;         // Current servo angle (start at safe minimum)
bool servoIncreasing = true;   // Direction of servo sweep

// Timing variables
unsigned long lastUpdate = 0;            // Last time values were updated
const unsigned long updateInterval = 20; // Update interval in milliseconds

// LED blink variables for status indication
unsigned long lastLedBlink = 0;
const unsigned long ledBlinkInterval = 500; // LED blink interval
bool ledState = false;

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);
    delay(1000);

    // Initialize DAC pin, servo, and LED pin
    pinMode(dacPin, OUTPUT);
    myServo.attach(pwmPin);
    pinMode(ledPin, OUTPUT);

    // Set initial values
    analogWrite(dacPin, currentPwmValue);
    myServo.write(currentAngle);

    Serial.println("SAMD21 Controller - Dual Output Started");
    Serial.println("DAC output (0-1023) on A0 -> Buck_DIM AND Servo sweep (20-160°) on D2 -> 5V SERVO");
    Serial.println("Built-in LED indicates system status");
    Serial.println("----------------------------------------");

    // Initial status
    printStatus();
}

void loop()
{
    // Update both analog output and servo angle at specified interval
    if (millis() - lastUpdate >= updateInterval)
    {
        updateAnalogOutput();
        updateServoAngle();
        lastUpdate = millis();
    }

    // Update status LED
    updateStatusLed();
}

void updateAnalogOutput()
{
    // Update PWM value based on direction
    if (pwmIncreasing)
    {
        currentPwmValue++;
        if (currentPwmValue >= maxPwmValue)
        {
            pwmIncreasing = false;
            Serial.println("Reached maximum DAC value - reversing direction");
        }
    }
    else
    {
        currentPwmValue--;
        if (currentPwmValue <= 0)
        {
            pwmIncreasing = true;
            Serial.println("Reached minimum DAC value - reversing direction");
        }
    }

    // Apply PWM value to DAC pin (amplified to Buck_DIM net)
    analogWrite(dacPin, currentPwmValue);
}

void updateServoAngle()
{
    // Update servo angle based on direction
    if (servoIncreasing)
    {
        currentAngle++;
        if (currentAngle >= maxServoAngle)
        {
            servoIncreasing = false;
            Serial.println("Reached maximum servo angle - reversing direction");
        }
    }
    else
    {
        currentAngle--;
        if (currentAngle <= minServoAngle)
        {
            servoIncreasing = true;
            Serial.println("Reached minimum servo angle - reversing direction");
        }
    }

    // Move servo to current angle (logic level converted to 5V SERVO net)
    myServo.write(currentAngle);

    // Print status every 30 degrees for monitoring
    if (currentAngle % 30 == 0)
    {
        printStatus();
    }
}

void updateStatusLed()
{
    // Blink LED to indicate system is running
    if (millis() - lastLedBlink >= ledBlinkInterval)
    {
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
        lastLedBlink = millis();
    }
}

void printStatus()
{
    // DAC status
    float dacPercentage = (float)currentPwmValue / maxPwmValue * 100.0;
    String dacDirection = pwmIncreasing ? "UP" : "DOWN";

    // Servo status
    float servoPercentage = (float)currentAngle / maxServoAngle * 100.0;
    String servoDirection = servoIncreasing ? "CW" : "CCW";

    Serial.print("DAC: ");
    Serial.print(currentPwmValue);
    Serial.print("/");
    Serial.print(maxPwmValue);
    Serial.print(" (");
    Serial.print(dacPercentage, 1);
    Serial.print("%, ");
    Serial.print(dacDirection);
    Serial.print(") | Servo: ");
    Serial.print(currentAngle);
    Serial.print("° (");
    Serial.print(servoPercentage, 1);
    Serial.print("%, ");
    Serial.print(servoDirection);
    Serial.print(") | Buck_DIM: ~");
    Serial.print((currentPwmValue * 3.3) / 1023.0, 2);
    Serial.println("V");
}