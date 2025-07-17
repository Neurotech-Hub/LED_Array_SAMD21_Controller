// SAMD21 Round - Robin Communication Controller

#include <Arduino.h>
#include <Servo.h>

// Version tracking
const String CODE_VERSION = "0.004";

// Pin assignments
const int dacPin = A0;          // DAC output
const int pwmPin = D2;          // Servo control
const int ledPin = LED_BUILTIN; // Status LED
const int rxReadyPin = D1;      // Chain ready input
const int txReadyPin = D3;      // Chain ready output
const int userLedPin = D10;     // User LED
// Serial1 uses D6 (TX) and D7 (RX)

// Command structure
struct Command
{
    int deviceId;
    String command;
    String value;
};

// Simplified states
enum DeviceState
{
    WAITING_FOR_CHAIN, // Waiting for rxReady to go HIGH
    CHAIN_READY,       // Chain is ready, normal operation
    INIT_IN_PROGRESS,  // Only used by master during initialization
    PROCESSING,        // Master is processing a command
    READY              // Master is ready to receive new commands
};

// Global variables
DeviceState currentState = WAITING_FOR_CHAIN;
bool isMasterDevice = false;
int myDeviceId = 0;
int totalDevices = 0;
Servo myServo;
int currentServoPos = 90;
int targetServoPos = 90;
String pendingCommand = ""; // Track master's current command

// Function declarations
void updateServo();
void processCommand(String data);
bool parseCommand(String data, Command &cmd);
void updateStatusLED();
void waitForChain();
void reinitializeDevices();
void printHelp();
void printStatus();
void sendCommandAndWait(String command); // Add function declaration

void setup()
{
    // Initialize pins
    pinMode(dacPin, OUTPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(rxReadyPin, INPUT_PULLDOWN);
    pinMode(txReadyPin, OUTPUT);
    pinMode(userLedPin, OUTPUT);

    // Initialize outputs
    digitalWrite(txReadyPin, LOW); // Start with txReady LOW
    digitalWrite(userLedPin, LOW);

    // Initialize servo
    myServo.attach(pwmPin);
    myServo.write(90);

    // Initialize Serial interfaces
    Serial.begin(115200); // USB
    Serial1.begin(9600);  // Device chain

    // Wait for either Serial (master) or rxReadyPin (slave)
    while (!isMasterDevice)
    {
        if (Serial)
        {
            isMasterDevice = true;
            myDeviceId = 1; // Master is always device 1
            Serial.println("\nSAMD21 Controller Starting...");
            Serial.println("VER:" + CODE_VERSION);
            Serial.println("DEBUG: Setting txReady HIGH to start chain");
            digitalWrite(txReadyPin, HIGH);
            break;
        }
        if (digitalRead(rxReadyPin) == HIGH)
        {
            isMasterDevice = false;
            break;
        }
        digitalWrite(ledPin, (millis() / 500) % 2); // Blink while waiting
        delay(10);                                  // Small delay to prevent tight loop
    }

    // Master must wait for rxReady before proceeding
    if (isMasterDevice)
    {
        Serial.println("DEBUG: Master waiting for rxReady...");
        while (digitalRead(rxReadyPin) == LOW)
        {
            digitalWrite(ledPin, (millis() / 500) % 2);
            delay(10);
        }
        Serial.println("DEBUG: Chain connected to master!");
    }

    // Initial chain connection
    waitForChain();
}

void loop()
{
    // Always mirror rxReady to txReady for slaves
    if (!isMasterDevice)
    {
        digitalWrite(txReadyPin, digitalRead(rxReadyPin));
    }

    // Update LED status
    updateStatusLED();

    // If rxReady is LOW, chain is broken
    if (digitalRead(rxReadyPin) == LOW)
    {
        if (currentState != WAITING_FOR_CHAIN)
        {
            currentState = WAITING_FOR_CHAIN;
            Serial1.flush(); // Clear any pending data
            if (isMasterDevice)
            {
                Serial.println("DEBUG: rxReady went LOW - chain broken!");
                Serial.println("DEBUG: Attempting to re-establish chain...");
                pendingCommand = ""; // Clear any pending command
            }
            // Reset device state
            myDeviceId = isMasterDevice ? 1 : 0;
            totalDevices = 0;
            // Wait for chain to reconnect
            waitForChain();
        }
        return;
    }

    // Master: Handle USB Serial commands
    if (isMasterDevice && Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.length() > 0)
        {
            // Handle system commands directly on master
            if (command == "help")
            {
                printHelp();
                return;
            }
            else if (command == "status")
            {
                printStatus();
                return;
            }
            else if (command == "reinit")
            {
                reinitializeDevices();
                return;
            }

            // Parse and validate regular command
            Command cmd;
            if (!parseCommand(command, cmd))
            {
                Serial.println("ERR:INVALID_FORMAT");
                Serial.println("UI:Use format: deviceId,command,value");
                Serial.println("UI:Example: 001,servo,90 or 000,dac,512");
                return;
            }

            // Validate command type
            if (cmd.command != "servo" && cmd.command != "dac" && cmd.command != "init")
            {
                Serial.println("ERR:INVALID_COMMAND:" + cmd.command);
                Serial.println("UI:Valid commands: servo, dac");
                Serial.println("UI:Example: 001,servo,90 or 002,dac,512");
                return;
            }

            // Validate device ID range (except for broadcast)
            if (cmd.deviceId != 0)
            {
                if (cmd.deviceId > totalDevices)
                {
                    Serial.println("ERR:INVALID_DEVICE:" + String(cmd.deviceId) + " (max:" + String(totalDevices) + ")");
                    return;
                }
            }

            // Validate command values
            if (cmd.command == "servo")
            {
                int angle = cmd.value.toInt();
                if (angle < 60 || angle > 120)
                {
                    Serial.println("ERR:SERVO_RANGE:" + String(angle));
                    Serial.println("UI:Servo angle must be 60-120 degrees");
                    return;
                }
            }
            else if (cmd.command == "dac")
            {
                int value = cmd.value.toInt();
                if (value < 0 || value > 1023)
                {
                    Serial.println("ERR:DAC_RANGE:" + String(value));
                    Serial.println("UI:DAC value must be 0-1023");
                    return;
                }
            }

            // Send valid command to chain
            pendingCommand = command;  // Track the command we're sending
            currentState = PROCESSING; // Set state to PROCESSING while waiting for command return
            Serial1.println(command);
            Serial.println("DEBUG: Sent to chain: " + command);
        }
    }

    // All devices: Process chain communication (Serial1)
    if (Serial1.available())
    {
        String data = Serial1.readStringUntil('\n');
        data.trim();

        if (data.length() > 0)
        {
            if (isMasterDevice)
            {
                Serial.println("DEBUG: Received from chain: " + data);
            }
            processCommand(data);
        }
    }

    // Handle servo movement
    updateServo();
}

void processCommand(String data)
{
    Command cmd;
    if (!parseCommand(data, cmd))
    {
        if (isMasterDevice)
        {
            Serial.println("DEBUG: Failed to parse chain command: " + data);
        }
        return;
    }

    if (Serial)
    {
        Serial.println("DEBUG: Device " + String(myDeviceId) + " processing: " + data);
    }

    // Handle initialization command
    if (cmd.command == "init")
    {
        if (isMasterDevice)
        {
            if (currentState == INIT_IN_PROGRESS)
            {
                // Received final count
                totalDevices = cmd.value.toInt();
                currentState = CHAIN_READY;
                Serial.println("DEBUG: Init complete - found " + String(totalDevices) + " devices");
                Serial.println("INIT:TOTAL:" + String(totalDevices));
                Serial.println("UI:Ready for commands");
            }
            else
            {
                Serial.println("DEBUG: Ignoring init command in state: " + String(currentState));
            }
        }
        else
        {
            // Slave: take ID and increment
            myDeviceId = cmd.value.toInt() + 1;
            // Forward incremented count
            String nextInit = "000,init," + String(myDeviceId);
            if (Serial)
            {
                Serial.println("DEBUG: Slave taking ID " + String(myDeviceId));
                Serial.println("DEBUG: Forwarding: " + nextInit);
            }
            Serial1.println(nextInit);
            if (Serial)
            {
                Serial.println("INIT:DEV:" + String(myDeviceId));
            }
        }
        return;
    }

    // Process command if:
    // 1. It's a broadcast (000)
    // 2. OR it matches our device ID exactly
    bool isForUs = (cmd.deviceId == 0) || (cmd.deviceId == myDeviceId);

    if (isForUs)
    {
        if (Serial)
        {
            Serial.println("DEBUG: Device " + String(myDeviceId) + " executing command");
        }

        // Handle servo command
        if (cmd.command == "servo")
        {
            int angle = cmd.value.toInt();
            angle = constrain(angle, 60, 120);
            targetServoPos = angle;
            if (Serial)
            {
                Serial.println("SRV:" + String(myDeviceId) + ":" + String(angle));
            }
        }
        // Handle DAC command
        else if (cmd.command == "dac")
        {
            int value = cmd.value.toInt();
            value = constrain(value, 0, 1023);
            analogWrite(dacPin, value);
            digitalWrite(userLedPin, value > 0 ? HIGH : LOW);
            if (Serial)
            {
                Serial.println("DAC:" + String(myDeviceId) + ":" + String(value));
            }
        }
    }

    // Always forward unless we're master and it's our command returning
    if (!isMasterDevice || (isMasterDevice && data != pendingCommand))
    {
        if (Serial)
        {
            Serial.println("DEBUG: Device " + String(myDeviceId) + " forwarding: " + data);
        }
        Serial1.println(data);
    }
    else if (isMasterDevice && data == pendingCommand)
    {
        currentState = READY;  // Reset state when command returns
        pendingCommand = "";   // Clear pending command
        Serial.println("EOT"); // Command made it around the chain
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

void updateServo()
{
    if (currentServoPos < targetServoPos)
    {
        currentServoPos++;
        myServo.write(currentServoPos);
    }
    else if (currentServoPos > targetServoPos)
    {
        currentServoPos--;
        myServo.write(currentServoPos);
    }
}

void updateStatusLED()
{
    switch (currentState)
    {
    case WAITING_FOR_CHAIN:
        // Fast blink while waiting
        digitalWrite(ledPin, (millis() / 500) % 2);
        break;

    case INIT_IN_PROGRESS:
        // Slow blink during init
        digitalWrite(ledPin, (millis() / 1000) % 2);
        break;

    case CHAIN_READY:
        // Solid on when ready
        digitalWrite(ledPin, HIGH);
        break;
    }
}

void waitForChain()
{
    if (isMasterDevice)
    {
        Serial.println("DEBUG: Waiting for rxReady to go HIGH...");
    }

    // Wait for chain to be ready (rxReady HIGH)
    while (digitalRead(rxReadyPin) == LOW)
    {
        // Blink LED while waiting
        digitalWrite(ledPin, (millis() / 500) % 2);
        if (isMasterDevice && (millis() % 1000 == 0))
        { // Print every second
            Serial.println("DEBUG: rxReady still LOW");
            delay(200); // Prevent message flood
        }
    }

    // Chain is ready
    currentState = CHAIN_READY;
    if (isMasterDevice)
    {
        Serial.println("DEBUG: rxReady is HIGH - chain connected!");
        // Start initialization
        currentState = INIT_IN_PROGRESS;
        Serial.println("DEBUG: Starting device initialization");
        Serial.println("DEBUG: Sending initial command: 000,init,001");
        // Send initial device count
        Serial1.println("000,init,001");
    }
}

void reinitializeDevices()
{
    if (!isMasterDevice)
    {
        Serial.println("ERR:ONLY_MASTER_CAN_REINIT");
        return;
    }
    
    Serial.println("UI:Restarting device initialization...");
    
    // Reset device count and state
    totalDevices = 0;
    pendingCommand = "";
    currentState = INIT_IN_PROGRESS;
    
    // Clear any pending Serial1 data
    Serial1.flush();
    while (Serial1.available())
    {
        Serial1.read();
    }
    
    // Small delay to let slaves settle
    delay(100);
    
    // Check chain connection
    if (digitalRead(rxReadyPin) == LOW)
    {
        Serial.println("ERR:CHAIN_NOT_CONNECTED");
        Serial.println("UI:Chain not connected - check physical connections");
        currentState = WAITING_FOR_CHAIN;
        return;
    }
    
    // Start fresh initialization
    Serial.println("DEBUG: Starting fresh device initialization");
    Serial.println("STATE:Initializing");
    Serial.println("DEBUG: Sending initial command: 000,init,001");
    Serial1.println("000,init,001");
}

void printHelp()
{
    Serial.println("VER:" + CODE_VERSION);
    Serial.println("UI:Available Commands:");
    Serial.println("UI:  Device Control:");
    Serial.println("UI:    xxx,servo,angle - Set servo angle (60-120)");
    Serial.println("UI:    xxx,dac,value   - Set DAC value (0-1023)");
    Serial.println("UI:  Where xxx is:");
    Serial.println("UI:    000 = all devices");
    Serial.println("UI:    001-" + String(totalDevices) + " = specific device");
    Serial.println("UI:  System Commands:");
    Serial.println("UI:    help   - Show this help");
    Serial.println("UI:    status - Show system status");
    Serial.println("UI:    reinit - Restart device initialization");
}

void printStatus()
{
    Serial.println("VER:" + CODE_VERSION);
    Serial.println("TOTAL:" + String(totalDevices));
    
    // Print meaningful state names
    String stateName;
    switch (currentState)
    {
        case WAITING_FOR_CHAIN:
            stateName = "Waiting for Chain";
            break;
        case CHAIN_READY:
            stateName = "Ready";
            break;
        case INIT_IN_PROGRESS:
            stateName = "Initializing";
            break;
        case PROCESSING:
            stateName = "Processing";
            break;
        case READY:
            stateName = "Ready";
            break;
        default:
            stateName = "Unknown";
            break;
    }
    Serial.println("STATE:" + stateName);
    
    if (pendingCommand.length() > 0)
    {
        Serial.println("PENDING:" + pendingCommand);
    }
}

void sendCommandAndWait(String command)
{
    if (!isMasterDevice)
        return; // Only master can send commands

    // Send the command
    pendingCommand = command;
    currentState = PROCESSING;
    Serial1.println(command);
    Serial.println("CMD:" + command);

    // Wait for command to complete (return to master)
    unsigned long startTime = millis();
    while (currentState == PROCESSING)
    {
        // Call loop to handle incoming data
        loop();

        // Timeout after 2 seconds
        if (millis() - startTime > 2000)
        {
            Serial.println("ERR:TIMEOUT");
            currentState = READY;
            pendingCommand = "";
            break;
        }
    }
}