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
const String CODE_VERSION = "0.002";

// Pin assignments based on schematic naming
const int dacPin = A0;          // A0 (DAC) - amplified to Buck_DIM net
const int pwmPin = D2;          // D2 (PWM) - logic level converted to 5V SERVO net
const int ledPin = LED_BUILTIN; // Built-in LED for status feedback
const int rxReadyPin = D1;      // D1 (RX_READY) - interrupt input
const int txReadyPin = D3;      // D3 (TX_READY) - output to next device
const int txPin = D6;           // D6 (TX) - Serial1 transmit
const int rxPin = D7;           // D7 (RX) - Serial1 receive

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

// Communication timeouts
const unsigned long SERIAL_TIMEOUT = 2000; // USB detection
const unsigned long RX_TIMEOUT = 100;      // 100ms for receiving data
const unsigned long CHAIN_TIMEOUT = 5000;  // 5s for chain break detection
const unsigned long TX_DELAY = 10;         // 10ms before transmission

void setup()
{
    // Initialize pins
    pinMode(dacPin, OUTPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(rxReadyPin, INPUT_PULLUP);
    pinMode(txReadyPin, OUTPUT);
    digitalWrite(txReadyPin, HIGH); // TX_READY idle high

    // Initialize servo
    myServo.attach(pwmPin);
    myServo.write(90); // Center position

    // Initialize DAC
    analogWrite(dacPin, 0);

    // Check if we're the master device (USB connected)
    delay(SERIAL_TIMEOUT); // devices need time to establish USB, looping for Serial is not reliable
    Serial.begin(115200);

    isMasterDevice = (Serial);

    // Initialize Serial1 for device communication
    Serial1.begin(9600);

    // Setup interrupt for RX_READY
    attachInterrupt(digitalPinToInterrupt(rxReadyPin), onRxReadyInterrupt, FALLING);

    if (isMasterDevice)
    {
        delay(1000); // Give other devices time to boot
        Serial.print("SAMD21 Controller v");
        Serial.print(CODE_VERSION);
        Serial.println(" - Round Robin Master Started");
        Serial.println("Starting device initialization sequence...");
        Serial.println("Auto re-initialization enabled (5s timeout)");
        Serial.println("");
        Serial.println("=== Serial Monitor Commands ===");
        Serial.println("Type 'help' for command list");
        Serial.println("Type 'status' for device status");
        Serial.println("Command format: deviceId,command,value");
        Serial.println("Example: 002,servo,90");
        Serial.println("Use 000 to command all devices");
        Serial.println("===============================");
        Serial.println("");

        // Start initialization sequence
        startInitialization();
    }
    else
    {
        // Slave device - wait for initialization
        Serial.print("SAMD21 Controller v");
        Serial.print(CODE_VERSION);
        Serial.println(" - Round Robin Slave Started");
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
            reason = "Stuck in initialization state";
        }
        else if (totalDevices == 0 && currentState == READY &&
                 millis() - lastActivity > CHAIN_TIMEOUT)
        {
            shouldReinitialize = true;
            reason = "No devices detected after initialization";
        }

        if (shouldReinitialize)
        {
            if (Serial)
            {
                Serial.print("Auto re-initialization triggered: ");
                Serial.println(reason);
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
            Serial.print("WARNING: Command timeout - resetting to READY state. Lost command: ");
            Serial.println(pendingCommand);
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
            processReceivedData(receivedData);
            lastActivity = millis();
        }
    }
}

void processReceivedData(String data)
{
    Command cmd;
    if (parseCommand(data, cmd))
    {
        if (isMasterDevice && Serial)
        {
            Serial.print("Received (State:");
            Serial.print(currentState == INIT_WAITING ? "INIT_WAITING" : currentState == INIT_ACTIVE ? "INIT_ACTIVE"
                                                                     : currentState == READY         ? "READY"
                                                                                                     : "PROCESSING");
            Serial.print("): ");
            Serial.println(data);
        }

        // Case 1: Handle initialization commands (000,init,XXX)
        if (cmd.command == "init")
        {
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
                    Serial.print("Device initialized as ID: ");
                    Serial.println(myDeviceId);
                }

                // Slave has processed and forwarded - don't forward again
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
                    Serial.print("Initialization complete. Total devices: ");
                    Serial.println(totalDevices);
                    Serial.println("Master entering READY state - waiting for Serial commands");
                }

                // Master has completed initialization - don't forward
                return;
            }
            else if (isMasterDevice && currentState == READY)
            {
                // Master in READY state - completely ignore stray init commands
                if (Serial)
                {
                    Serial.println("WARNING: Ignoring stray init command - system already initialized");
                }
                return; // Don't process or forward
            }
            else if (isMasterDevice && currentState == PROCESSING)
            {
                // Master in PROCESSING state - ignore init commands
                if (Serial)
                {
                    Serial.println("WARNING: Ignoring init command while processing user command");
                }
                return; // Don't process or forward
            }
            else if (!isMasterDevice && currentState == READY)
            {
                // Slave in READY state receiving init command - this means re-initialization
                // Reset to INIT_WAITING and process normally
                if (Serial)
                {
                    Serial.println("Slave resetting for re-initialization");
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
                Serial.println("ERROR: Unhandled init command state");
            }
            return;
        }

        // Ignore non-init commands during initialization
        if (currentState == INIT_WAITING || currentState == INIT_ACTIVE)
        {
            if (isMasterDevice && Serial)
            {
                Serial.print("WARNING: Ignoring non-init command during initialization: ");
                Serial.println(data);
            }
            return;
        }

        // Check if this is master's own command coming back
        bool isMyCommandReturning = (isMasterDevice && currentState == PROCESSING && data == pendingCommand);

        if (isMyCommandReturning)
        {
            // Master's command returned - exit PROCESSING state
            currentState = READY;
            pendingCommand = "";

            if (Serial)
            {
                Serial.println("Command completed round trip - returning to READY state");
            }
        }

        // Case 2 & 3: Handle servo/dac commands
        // Process command if it's for this device (specific ID) or all devices (000)
        if (cmd.deviceId == myDeviceId || cmd.deviceId == 0)
        {
            processCommand(cmd);
        }

        // Forward logic:
        // - Slaves always forward (except init commands handled above)
        // - Master only forwards if NOT its own command returning
        if (!isMasterDevice || !isMyCommandReturning)
        {
            forwardCommand(data);
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
            Serial.print("Device ");
            Serial.print(myDeviceId);
            Serial.print(" servo set to: ");
            Serial.println(angle);
        }
    }
    else if (cmd.command == "dac")
    {
        int value = cmd.value.toInt();
        setDAC(value);

        if (isMasterDevice && Serial)
        {
            Serial.print("Device ");
            Serial.print(myDeviceId);
            Serial.print(" DAC set to: ");
            Serial.println(value);
        }
    }
}

void forwardCommand(String data)
{
    // Signal next device
    digitalWrite(txReadyPin, LOW);
    delay(TX_DELAY);

    // Send data
    Serial1.println(data);
    Serial1.flush();

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
        Serial.print("=== Re-initializing Device Chain v");
        Serial.print(CODE_VERSION);
        Serial.println(" ===");
        Serial.print("Previous device count: ");
        Serial.println(totalDevices);
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
        Serial.println("Re-initialization command sent");
        Serial.println("===================================");
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
        Serial.println("Manual re-initialization requested");
        restartInitialization();
        return;
    }

    // Validate command format
    if (validateCommand(input))
    {
        // Check if master is already processing a command
        if (currentState == PROCESSING)
        {
            Serial.println("ERROR: Master is processing another command. Wait for completion.");
            return;
        }

        Serial.print("Sending command: ");
        Serial.println(input);

        // Set master to PROCESSING state and track pending command
        currentState = PROCESSING;
        pendingCommand = input;

        forwardCommand(input);
        lastActivity = millis();
    }
    else
    {
        Serial.println("ERROR: Invalid command format or parameters");
        Serial.println("Use: deviceId,command,value");
        Serial.println("Example: 002,servo,90");
        if (totalDevices > 0)
        {
            Serial.print("Valid device IDs: 000 (all), 001-");
            Serial.println(totalDevices);
        }
        else
        {
            Serial.println("System initializing - please wait...");
        }
        Serial.println("Type 'help' for more information");
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
        return false;
    }

    // Extract parts
    String deviceIdStr = command.substring(0, firstComma);
    String commandStr = command.substring(firstComma + 1, secondComma);
    String valueStr = command.substring(secondComma + 1);

    // Validate device ID (should be 3 digits)
    if (deviceIdStr.length() != 3)
    {
        return false;
    }

    for (int i = 0; i < 3; i++)
    {
        if (!isDigit(deviceIdStr[i]))
        {
            return false;
        }
    }

    int deviceId = deviceIdStr.toInt();
    if (deviceId < 0 || deviceId > 999)
    {
        return false;
    }

    // Validate device ID is within detected device range
    if (totalDevices == 0)
    {
        Serial.println("ERROR: No devices detected yet. Wait for initialization to complete.");
        return false;
    }

    if (deviceId > 0 && deviceId > totalDevices)
    {
        Serial.print("ERROR: Device ");
        Serial.print(deviceId);
        Serial.print(" not found. Valid range: 000 (all), 001-");
        Serial.println(totalDevices);
        return false;
    }

    // Validate command string (should not be empty)
    if (commandStr.length() == 0)
    {
        return false;
    }

    // Validate known commands
    if (commandStr != "servo" && commandStr != "dac" && commandStr != "init")
    {
        Serial.print("WARNING: Unknown command '");
        Serial.print(commandStr);
        Serial.println("'. Valid commands: servo, dac, init");
        // Allow it to proceed anyway for future extensibility
    }

    // Validate value for specific commands
    if (commandStr == "servo")
    {
        int angle = valueStr.toInt();
        if (angle < 0 || angle > 180)
        {
            Serial.println("ERROR: Servo angle must be 0-180 degrees");
            return false;
        }
    }
    else if (commandStr == "dac")
    {
        int value = valueStr.toInt();
        if (value < 0 || value > 1023)
        {
            Serial.println("ERROR: DAC value must be 0-1023");
            return false;
        }
    }

    return true;
}

void printHelp()
{
    Serial.print("=== SAMD21 Round Robin Controller v");
    Serial.print(CODE_VERSION);
    Serial.println(" Help ===");
    Serial.println("Command Format: deviceId,command,value");
    Serial.println("");
    Serial.println("Device IDs:");
    Serial.println("  000 - Command all devices");
    Serial.print("  001-");
    Serial.print(totalDevices, DEC);
    Serial.println(" - Specific device numbers");
    Serial.println("");
    Serial.println("Commands:");
    Serial.println("  servo,angle - Set servo angle (0-180 degrees)");
    Serial.println("  dac,value   - Set DAC value (0-1023)");
    Serial.println("");
    Serial.println("Examples:");
    Serial.println("  002,servo,90  - Set device 2 servo to 90 degrees");
    Serial.println("  003,dac,512   - Set device 3 DAC to 512");
    Serial.println("  000,servo,45  - Command ALL devices servo to 45 degrees");
    Serial.println("");
    Serial.println("Special Commands:");
    Serial.println("  help   - Show this help");
    Serial.println("  status - Show device status");
    Serial.println("  reinit - Re-initialize device chain");
    Serial.println("===========================================");
}

void printDeviceStatus()
{
    Serial.print("=== Device Status v");
    Serial.print(CODE_VERSION);
    Serial.println(" ===");
    Serial.print("Master Device ID: ");
    Serial.println(myDeviceId);
    Serial.print("Total Devices: ");
    Serial.println(totalDevices);
    Serial.print("Current State: ");

    switch (currentState)
    {
    case INIT_WAITING:
        Serial.println("INIT_WAITING");
        break;
    case INIT_ACTIVE:
        Serial.println("INIT_ACTIVE");
        break;
    case READY:
        Serial.println("READY");
        break;
    case PROCESSING:
        Serial.println("PROCESSING");
        if (pendingCommand.length() > 0)
        {
            Serial.print("Pending command: ");
            Serial.println(pendingCommand);
        }
        break;
    default:
        Serial.println("UNKNOWN");
        break;
    }

    Serial.print("Last Activity: ");
    Serial.print((millis() - lastActivity) / 1000);
    Serial.println(" seconds ago");
    Serial.println("====================");
}

// Simple control functions
void setServo(int angle)
{
    angle = constrain(angle, 0, 180);
    myServo.write(angle);
}

void setDAC(int value)
{
    value = constrain(value, 0, 1023);
    analogWrite(dacPin, value);
}