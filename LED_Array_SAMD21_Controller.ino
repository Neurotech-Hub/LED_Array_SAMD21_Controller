/*
  SAMD21_Controller.ino - Round Robin Communication System

  This sketch implements a daisy-chained round-robin communication system
  for SEEEDuino XIAO boards with DAC and servo control capabilities.

  Hardware:
  - SEEEDuino XIAO (SAMD21 based)
  - A0 (DAC) for analog output - amplified to Buck_DIM net
  - D2 (PWM) for servo control - logic level converted to 5V SERVO net
  - D1 (RX_READY) - interrupt input for receiving commands
  - D3 (TX_READY) - output to signal next device
  - D6 (TX) - Serial1 transmit to next device
  - D7 (RX) - Serial1 receive from previous device

  Pin-Schematic Mapping:
  A0  - DAC        D6  - TX
  D1  - RX_READY   D7  - RX
  D2  - PWM        D8  - D8
  D3  - TX_READY   D9  - D9
  D4  - SDA        D10 - D10_LED
  D5  - SCL
*/

#include <Arduino.h>
#include <Servo.h>

// Version tracking - increment with each code change
const String CODE_VERSION = "0.004";

// Pin assignments based on schematic naming
const int dacPin = A0;          // A0 (DAC) - amplified to Buck_DIM net
const int pwmPin = D2;          // D2 (PWM) - logic level converted to 5V SERVO net
const int ledPin = LED_BUILTIN; // Built-in LED for status feedback
const int rxReadyPin = D1;      // D1 (RX_READY) - interrupt input
const int txReadyPin = D3;      // D3 (TX_READY) - output to next device
const int txPin = D6;           // D6 (TX) - Serial1 transmit
const int rxPin = D7;           // D7 (RX) - Serial1 receive
const int userLedPin = D10;     // D10 - User LED output

// Device states
enum DeviceState
{
    INIT_WAITING, // Waiting for initialization
    INIT_ACTIVE,  // Processing initialization
    READY,        // Ready for commands
    PROCESSING    // Processing a command
};

// Command structure
struct Command
{
    int deviceId;
    String command;
    String value;
};

// Global variables
DeviceState currentState = INIT_WAITING;
bool isMasterDevice = false;
int myDeviceId = 0;
int totalDevices = 0;
Servo myServo;
volatile bool rxReadyTriggered = false;
unsigned long lastActivity = 0;
unsigned long stateTimeout = 0;
String pendingCommand = "";         // Track master's pending command
unsigned long initCompleteTime = 0; // Track when initialization completed

// Communication timeouts and delays
const unsigned long SERIAL_TIMEOUT = 2000; // USB detection
const unsigned long RX_TIMEOUT = 100;      // 100ms for receiving data
const unsigned long CHAIN_TIMEOUT = 5000;  // 5s for chain break detection
const unsigned long TX_DELAY = 2;          // 2ms setup time for TX_READY signal
const unsigned long TX_SETUP = 1;          // 1ms setup time before sending data

// Servo control variables
int currentServoPos = 90; // Track current position
int targetServoPos = 90;  // Track target position
unsigned long lastServoUpdate = 0;
const unsigned long SERVO_UPDATE_INTERVAL = 20; // 20ms between position updates
const int SERVO_SPEED = 3;                      // Degrees per update (lower = smoother but slower)

void updateServo()
{
    // Only update at the specified interval
    if (millis() - lastServoUpdate < SERVO_UPDATE_INTERVAL)
    {
        return;
    }

    // Move toward target position at controlled speed
    if (currentServoPos < targetServoPos)
    {
        currentServoPos = min(currentServoPos + SERVO_SPEED, targetServoPos);
        myServo.write(currentServoPos);
    }
    else if (currentServoPos > targetServoPos)
    {
        currentServoPos = max(currentServoPos - SERVO_SPEED, targetServoPos);
        myServo.write(currentServoPos);
    }

    lastServoUpdate = millis();
}

void sweepServo()
{
    // Turn on user LED during sweep
    digitalWrite(userLedPin, HIGH);

    // Sweep to 120
    setServo(120);
    while (currentServoPos != 120)
    {
        updateServo();
        delay(1); // Give other processes time to run
    }

    delay(250); // Pause at max

    // Sweep to 60
    setServo(60);
    while (currentServoPos != 60)
    {
        updateServo();
        delay(1); // Give other processes time to run
    }

    delay(250); // Pause at min

    // Return to center (90)
    setServo(90);
    while (currentServoPos != 90)
    {
        updateServo();
        delay(1); // Give other processes time to run
    }

    // Turn off user LED after sweep
    digitalWrite(userLedPin, LOW);
}

void setup()
{
    // Initialize pins
    pinMode(dacPin, OUTPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(rxReadyPin, INPUT_PULLUP);
    pinMode(txReadyPin, OUTPUT);
    pinMode(userLedPin, OUTPUT);
    digitalWrite(txReadyPin, HIGH); // TX_READY idle high
    digitalWrite(userLedPin, LOW);  // Start with user LED off

    // Initialize servo
    myServo.attach(pwmPin);
    myServo.write(90); // Center position
    currentServoPos = 90;
    targetServoPos = 90;
    lastServoUpdate = millis();

    // Initialize DAC
    analogWrite(dacPin, 0);

    // Initialize Serial1 for device communication first
    Serial1.begin(9600);

    // Setup interrupt for RX_READY before waiting for Serial
    attachInterrupt(digitalPinToInterrupt(rxReadyPin), onRxReadyInterrupt, FALLING);

    // Initialize USB Serial
    Serial.begin(115200);

    // Reset RX trigger flag before waiting
    rxReadyTriggered = false;

    // Wait forever until either:
    // 1. USB Serial connection (we're master)
    // 2. RX_READY interrupt (we're slave)
    while (true)
    {
        if (Serial)
        {
            isMasterDevice = true;
            break;
        }
        if (rxReadyTriggered)
        {
            isMasterDevice = false;
            break;
        }
        // Blink LED while waiting
        digitalWrite(ledPin, (millis() / 500) % 2);
    }

    // Perform servo sweep to indicate role determination complete
    sweepServo();

    if (isMasterDevice)
    {
        delay(1000); // Give other devices time to boot
        Serial.println("VER:" + CODE_VERSION);
        Serial.println("UI:Master started");
        Serial.println("UI:Type 'help' or 'status'");

        // Start initialization sequence
        startInitialization();
    }
    else
    {
        // Slave device - wait for initialization
        if (Serial)
        {
            Serial.println("VER:" + CODE_VERSION);
            Serial.println("UI:Slave started");
        }
        currentState = INIT_WAITING;
    }

    lastActivity = millis();
}

void loop()
{
    // Handle RX_READY interrupt
    if (rxReadyTriggered)
    {
        rxReadyTriggered = false;
        handleIncomingData();
    }

    // Update servo position if needed
    updateServo();

    // Master device: Handle Serial Monitor commands
    if (isMasterDevice && Serial.available())
    {
        handleSerialCommand();
    }

    // Check for timeouts - only re-initialize if there's an actual problem
    if (isMasterDevice)
    {
        // Only auto re-initialize in these problem scenarios:
        // 1. Stuck in initialization states for too long
        // 2. System never completed initial initialization
        bool shouldReinitialize = false;
        String reason = "";

        if ((currentState == INIT_WAITING || currentState == INIT_ACTIVE) &&
            millis() - lastActivity > CHAIN_TIMEOUT)
        {
            shouldReinitialize = true;
            reason = "TIMEOUT:INIT_STUCK";
        }
        else if (totalDevices == 0 && currentState == READY &&
                 millis() - lastActivity > CHAIN_TIMEOUT)
        {
            shouldReinitialize = true;
            reason = "TIMEOUT:NO_DEVICES";
        }

        if (shouldReinitialize)
        {
            if (Serial)
            {
                Serial.println("ERR:" + reason);
            }
            restartInitialization();
        }
        else if (millis() - lastActivity > CHAIN_TIMEOUT)
        {
            // Just reset the activity timer - no re-initialization needed
            lastActivity = millis();
        }
    }

    // Check for PROCESSING state timeout (command never returned)
    if (isMasterDevice && currentState == PROCESSING && millis() - lastActivity > (CHAIN_TIMEOUT / 2))
    {
        if (Serial)
        {
            Serial.println("ERR:TIMEOUT:CMD:" + pendingCommand);
        }
        currentState = READY;
        pendingCommand = "";
        lastActivity = millis();
    }

    // Blink LED to show activity
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500)
    {
        digitalWrite(ledPin, !digitalRead(ledPin));
        lastBlink = millis();
    }

    // Master device periodic tasks - DISABLED for debugging
    // Automatic test commands can interfere with initialization
    /*
    if (isMasterDevice && currentState == READY && totalDevices > 0)
    {
        // Only send test commands after sufficient delay from initialization
        if (initCompleteTime > 0 && (millis() - initCompleteTime) > 3000)
        {
            // Example: Send test commands periodically
            static unsigned long lastTest = 0;
            if (millis() - lastTest > 20000)  // Every 20 seconds
            {
                sendTestCommand();
                lastTest = millis();
            }
        }
    }
    */
}

void onRxReadyInterrupt()
{
    rxReadyTriggered = true;
}

void handleIncomingData()
{
    String receivedData = "";
    unsigned long startTime = millis();

    // Wait for complete data with timeout
    while (Serial1.available() == 0 && (millis() - startTime) < RX_TIMEOUT)
    {
        delay(1);
    }

    if (Serial1.available())
    {
        receivedData = Serial1.readStringUntil('\n');
        receivedData.trim();

        if (receivedData.length() > 0)
        {
            // Parse command to check if it's an init command
            Command cmd;
            bool isInit = false;
            if (parseCommand(receivedData, cmd))
            {
                isInit = (cmd.command == "init");
            }

            // For init commands, process first then forward
            if (isInit)
            {
                processReceivedData(receivedData);
                lastActivity = millis();
            }
            // For all other commands, forward first then process
            else
            {
                if (!isMasterDevice)
                {
                    forwardCommand(receivedData);
                }
                processReceivedData(receivedData);
                lastActivity = millis();
            }
        }
    }
}

void processReceivedData(String data)
{
    Command cmd;
    if (parseCommand(data, cmd))
    {
        // Master device handling
        if (isMasterDevice)
        {
            Serial.println("RCV:" + data);

            // Check if this is master's own command coming back
            bool isMyCommandReturning = (currentState == PROCESSING && data == pendingCommand);

            if (isMyCommandReturning)
            {
                // Master's command returned - exit PROCESSING state
                currentState = READY;
                pendingCommand = "";
                Serial.println("EOT");
                return;
            }
        }

        // Handle initialization commands (000,init,XXX)
        if (cmd.command == "init")
        {
            // Master in READY state - completely ignore stray init commands
            if (currentState == READY)
            {
                if (isMasterDevice && Serial)
                {
                    Serial.println("WARN:STRAY_INIT");
                }
                return; // Don't process or forward
            }

            // Master in PROCESSING state - ignore init commands
            if (currentState == PROCESSING)
            {
                if (isMasterDevice && Serial)
                {
                    Serial.println("WARN:BUSY");
                }
                return; // Don't process or forward
            }

            if (currentState == INIT_WAITING)
            {
                // Slave device: increment and forward
                myDeviceId = cmd.value.toInt() + 1;
                currentState = INIT_ACTIVE;

                String nextInit = "000,init," + String(myDeviceId);
                forwardCommand(nextInit);

                currentState = READY;

                if (isMasterDevice && Serial)
                {
                    Serial.println("INIT:DEV:" + String(myDeviceId));
                }

                return;
            }
            else if (isMasterDevice && currentState == INIT_ACTIVE)
            {
                // Master receives final count
                totalDevices = cmd.value.toInt();
                currentState = READY;
                initCompleteTime = millis(); // Record when initialization completed
                lastActivity = millis();     // Reset timeout counter

                if (Serial)
                {
                    Serial.println("INIT:TOTAL:" + String(totalDevices));
                    delay(1000);
                    Serial.println("UI:Ready for commands");
                }

                return;
            }
            else if (!isMasterDevice && currentState == READY)
            {
                // Slave in READY state receiving init command - this means re-initialization
                if (Serial)
                {
                    Serial.println("INIT:RESET");
                }
                currentState = INIT_WAITING;
                myDeviceId = 0; // Reset device ID

                // Now process as normal init
                myDeviceId = cmd.value.toInt() + 1;
                currentState = INIT_ACTIVE;

                String nextInit = "000,init," + String(myDeviceId);
                forwardCommand(nextInit);

                currentState = READY;

                return;
            }

            // This point should never be reached for init commands
            if (Serial)
            {
                Serial.println("ERR:UNKNOWN_STATE");
            }
            return;
        }

        // Ignore non-init commands during initialization
        if (currentState == INIT_WAITING || currentState == INIT_ACTIVE)
        {
            if (isMasterDevice && Serial)
            {
                Serial.println("WARN:NOT_READY:" + data);
            }
            return;
        }

        // Process command if:
        // 1. It's a broadcast command (000)
        // 2. OR it's specifically for this device
        if (cmd.deviceId == 0 || cmd.deviceId == myDeviceId)
        {
            processCommand(cmd);
        }
    }
}

bool parseCommand(String data, Command &cmd)
{
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);

    if (firstComma == -1 || secondComma == -1)
    {
        return false;
    }

    cmd.deviceId = data.substring(0, firstComma).toInt();
    cmd.command = data.substring(firstComma + 1, secondComma);
    cmd.value = data.substring(secondComma + 1);

    return true;
}

void processCommand(Command &cmd)
{
    if (cmd.command == "servo")
    {
        int angle = cmd.value.toInt();
        setServo(angle);

        if (isMasterDevice && Serial)
        {
            Serial.println("SRV:" + String(myDeviceId) + ":" + String(angle));
        }
    }
    else if (cmd.command == "dac")
    {
        int value = cmd.value.toInt();
        setDAC(value);

        if (isMasterDevice && Serial)
        {
            Serial.println("DAC:" + String(myDeviceId) + ":" + String(value));
        }
    }
}

void forwardCommand(String data)
{
    // Signal next device with proper setup time
    digitalWrite(txReadyPin, LOW);
    delay(TX_SETUP); // Give receiving device time to prepare

    // Send data
    Serial1.println(data);
    Serial1.flush();

    // Hold signal for minimum time
    delay(TX_DELAY);

    // Release signal
    digitalWrite(txReadyPin, HIGH);
}

void startInitialization()
{
    currentState = INIT_ACTIVE;
    myDeviceId = 1; // Master is device 1

    // Send initialization command
    String initCommand = "000,init,001";
    forwardCommand(initCommand);
}

void restartInitialization()
{
    if (Serial)
    {
        Serial.println("INIT:RESTART:PREV_COUNT:" + String(totalDevices));
    }

    // Reset device state
    currentState = INIT_ACTIVE;
    myDeviceId = 1;       // Master is device 1
    totalDevices = 0;     // Reset device count
    initCompleteTime = 0; // Reset initialization timer
    pendingCommand = "";  // Clear any pending commands

    // Add longer delay to allow any stray commands to clear the system
    delay(1000);

    // Send initialization command
    String initCommand = "000,init,001";
    forwardCommand(initCommand);

    if (Serial)
    {
        Serial.println("INIT:RESTART:SENT");
    }
}

void sendTestCommand()
{
    if (totalDevices > 1)
    {
        // Send servo command to device 2
        String testCommand = "002,servo,45";
        forwardCommand(testCommand);

        if (Serial)
        {
            Serial.println("Sent test command to device 2");
        }
    }
}

void danceRoutine()
{
    if (!isMasterDevice)
        return; // Only master can initiate dance

    Serial.println("UI:Dance started");

    // Initial sequence
    sendCommandAndWait("000,servo,60");
    delay(500);
    sendCommandAndWait("000,servo,90");
    delay(500);
    sendCommandAndWait("000,servo,120");
    delay(500);

    // Random dance for 5 iterations or until serial input
    for (int i = 0; i < 5 && !Serial.available(); i++)
    {
        int randomAngle = random(60, 121); // Random angle between 60-120
        sendCommandAndWait("000,servo," + String(randomAngle));
        delay(200);
    }

    // Clear any pending serial input
    while (Serial.available())
    {
        Serial.read();
    }

    // Reset all servos to center
    sendCommandAndWait("000,servo,90");
    delay(200);

    Serial.println("UI:Dance complete");
}

void handleSerialCommand()
{
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0)
    {
        return;
    }

    // Display help if user types "help"
    if (input.equalsIgnoreCase("help"))
    {
        printHelp();
        return;
    }

    // Display device status if user types "status"
    if (input.equalsIgnoreCase("status"))
    {
        printDeviceStatus();
        return;
    }

    // Manual re-initialization if user types "reinit"
    if (input.equalsIgnoreCase("reinit"))
    {
        Serial.println("UI:Manual re-initialization requested");
        restartInitialization();
        return;
    }

    // Dance routine if user types "dnc"
    if (input.equalsIgnoreCase("dnc"))
    {
        danceRoutine();
        return;
    }

    // Validate command format
    if (validateCommand(input))
    {
        // Check if master is already processing a command
        if (currentState == PROCESSING)
        {
            Serial.println("ERR:BUSY");
            return;
        }

        Serial.println("CMD:" + input);

        // Set master to PROCESSING state and track pending command
        currentState = PROCESSING;
        pendingCommand = input;

        forwardCommand(input);
        lastActivity = millis();
    }
    else
    {
        Serial.println("UI:Type 'help' for valid commands");
    }
}

bool validateCommand(String command)
{
    // Check basic format: should have exactly 2 commas
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    int thirdComma = command.indexOf(',', secondComma + 1);

    if (firstComma == -1 || secondComma == -1 || thirdComma != -1)
    {
        if (Serial)
            Serial.println("ERR:FORMAT");
        return false;
    }

    // Extract parts
    String deviceIdStr = command.substring(0, firstComma);
    String commandStr = command.substring(firstComma + 1, secondComma);
    String valueStr = command.substring(secondComma + 1);

    // Validate device ID (should be 3 digits)
    if (deviceIdStr.length() != 3)
    {
        if (Serial)
            Serial.println("ERR:ID_LEN");
        return false;
    }

    for (int i = 0; i < 3; i++)
    {
        if (!isDigit(deviceIdStr[i]))
        {
            if (Serial)
                Serial.println("ERR:ID_NUM");
            return false;
        }
    }

    int deviceId = deviceIdStr.toInt();
    if (deviceId < 0 || deviceId > 999)
    {
        if (Serial)
            Serial.println("ERR:ID_RANGE");
        return false;
    }

    // Validate device ID is within detected device range
    if (totalDevices == 0)
    {
        if (Serial)
            Serial.println("ERR:NO_DEVICES");
        return false;
    }

    if (deviceId > 0 && deviceId > totalDevices)
    {
        if (Serial)
            Serial.println("ERR:ID_MAX:" + String(totalDevices));
        return false;
    }

    // Validate command string (should not be empty)
    if (commandStr.length() == 0)
    {
        if (Serial)
            Serial.println("ERR:CMD_EMPTY");
        return false;
    }

    // Validate known commands
    if (commandStr != "servo" && commandStr != "dac" && commandStr != "init")
    {
        if (Serial)
            Serial.println("WARN:CMD_UNKNOWN:" + commandStr);
        // Allow it to proceed anyway for future extensibility
    }

    // Validate value for specific commands
    if (commandStr == "servo")
    {
        int angle = valueStr.toInt();
        if (angle < 60 || angle > 120)
        {
            if (Serial)
                Serial.println("ERR:SERVO_RANGE");
            return false;
        }
    }
    else if (commandStr == "dac")
    {
        int value = valueStr.toInt();
        if (value < 0 || value > 1023)
        {
            if (Serial)
                Serial.println("ERR:DAC_RANGE");
            return false;
        }
    }

    return true;
}

void printHelp()
{
    Serial.println("VER:" + CODE_VERSION);
    Serial.println("FMT:deviceId,command,value");
    Serial.println("DEV:000=all," + String(totalDevices) + "=max");
    Serial.println("CMD:servo=60-120,dac=0-1023");
    Serial.println("SYS:help,status,reinit,dnc");
}

void printDeviceStatus()
{
    Serial.println("VER:" + CODE_VERSION);
    Serial.println("ID:" + String(myDeviceId));
    Serial.println("TOTAL:" + String(totalDevices));
    Serial.println("STATE:" + String(currentState));
    if (pendingCommand.length() > 0)
    {
        Serial.println("PENDING:" + pendingCommand);
    }
    Serial.println("LAST:" + String((millis() - lastActivity) / 1000));
}

// Simple control functions
// Modified setServo to use smooth movement
void setServo(int angle)
{
    angle = constrain(angle, 60, 120);
    targetServoPos = angle;
}

void setDAC(int value)
{
    value = constrain(value, 0, 1023);
    analogWrite(dacPin, value);

    // Control user LED based on DAC value
    digitalWrite(userLedPin, value > 0 ? HIGH : LOW);
}

// Helper function to send command and wait for EOT
void sendCommandAndWait(String command)
{
    if (Serial)
    {
        Serial.println("CMD:" + command);
    }

    // Set master to PROCESSING state and track pending command
    currentState = PROCESSING;
    pendingCommand = command;

    forwardCommand(command);
    lastActivity = millis();

    // Wait for command to complete (EOT)
    while (currentState == PROCESSING)
    {
        if (rxReadyTriggered)
        {
            rxReadyTriggered = false;
            handleIncomingData();
        }

        // Check for timeout
        if (millis() - lastActivity > (CHAIN_TIMEOUT / 2))
        {
            if (Serial)
            {
                Serial.println("ERR:TIMEOUT:" + pendingCommand);
            }
            currentState = READY;
            pendingCommand = "";
            break;
        }
    }
}