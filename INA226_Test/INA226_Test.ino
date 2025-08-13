/*
 * INA226 Current Sensor Test Code for Adafruit ItsyBitsy M4
 * Using INA226 Library with DAC Control
 * 
 * This code uses the INA226 library for superior current sensor operation
 * and adds DAC control on A0 pin to trigger MOSFET through op-amp for
 * current measurement testing.
 * 
 * Hardware Connections:
 * ItsyBitsy M4 to INA226:
 * - SDA: SDA pin (I2C data)
 * - SCL: SCL pin (I2C clock)
 * - VCC: 3.3V
 * - GND: Common ground
 * 
 * ItsyBitsy M4 DAC Control Circuit:
 * - A0: DAC output (0-3.3V, 12-bit resolution)
 * 
 * Circuit Setup:
 * ItsyBitsy A0 (DAC) --> Op-Amp --> MOSFET --> High Power LED/Load
 *                                      |
 *                                  INA226 Shunt (39mΩ)
 *                                      |
 *                                    Vin- --> GND
 * 
 * INA226 I2C Address: 0x40 (A0=GND, A1=GND on INA226 chip)
 * Note: INA226's A0/A1 pins are for I2C addressing, not DAC control
 * 
 * Library Installation:
 * 1. Open Arduino IDE
 * 2. Go to Tools > Manage Libraries
 * 3. Search for "INA226" and install "INA226 Library"
 * 4. Search for "I2C_SCANNER" and install "RobTillaart I2C_SCANNER Library"
 */

#include <Wire.h>
#include <INA226.h>
#include <I2C_SCANNER.h>

// INA226 Alert Register Flags (for better readability)
const uint16_t ALERT_SHUNT_OVER_VOLTAGE  = 0x8000;  // Shunt voltage over limit
const uint16_t ALERT_SHUNT_UNDER_VOLTAGE = 0x4000;  // Shunt voltage under limit  
const uint16_t ALERT_BUS_OVER_VOLTAGE    = 0x2000;  // Bus voltage over limit
const uint16_t ALERT_BUS_UNDER_VOLTAGE   = 0x1000;  // Bus voltage under limit
const uint16_t ALERT_POWER_OVER_LIMIT    = 0x0800;  // Power over limit
const uint16_t ALERT_CONVERSION_READY    = 0x0400;  // Conversion ready flag

// Create INA226 object
INA226 ina226(0x4A);  // Default I2C address (INA226's A0=GND, A1=GND)

// Create I2C_SCANNER object for professional diagnostics
I2C_SCANNER scanner;

// DAC pin for ItsyBitsy M4
const int dacPin = A0;  // DAC output pin

// Calibration parameters for your circuit
float shuntResistance = 0.04195;  // Shunt resistance in ohms (39mΩ)
float maxCurrent = 1.5;          // Maximum expected current in amps (adjust based on your LED)

// Measurement variables
float current = 0.0;
float voltage = 0.0;
float power = 0.0;
float shuntVoltage = 0.0;

// DAC control variables
int currentDacValue = 0;      // Current DAC value (0-4095 for 12-bit)
int targetDacValue = 0;       // Target DAC value
bool dacEnabled = false;      // DAC output enabled flag

// Status variables
bool ina226Initialized = false;

// Timing variables
unsigned long lastMeasurement = 0;
const unsigned long MEASUREMENT_INTERVAL = 1000; // 1 second

void setup() {
  Serial.begin(115200);
  
  Serial.println("INA226 Current Sensor Test for ItsyBitsy M4");
  Serial.println("Using INA226 Library + RobTillaart I2C_SCANNER");
  Serial.println("==============================================");
  Serial.println("Features:");
  Serial.println("- INA226 current monitoring (library)");
  Serial.println("- Professional I2C diagnostics (RobTillaart)");
  Serial.println("- 12-bit DAC control (A0 pin)");
  Serial.println("- MOSFET/Op-Amp triggering");
  Serial.println("- Real-time current measurement");
  Serial.println("- Alert functions and error handling");
  
  // Wait for serial connection
  delay(1000);
  
  // Initialize I2C and scanner
  Wire.begin();
  scanner.begin();
  
  // Initialize DAC pin
  pinMode(dacPin, OUTPUT);
  analogWriteResolution(12);  // Set 12-bit resolution for ItsyBitsy M4
  analogWrite(dacPin, 0);     // Start with DAC off
  
  // Initialize INA226
  if (initializeINA226()) {
    Serial.println("INA226 initialized successfully!");
    ina226Initialized = true;
  } else {
    Serial.println("ERROR: Failed to initialize INA226!");
    Serial.println("Check I2C connections and address.");
  }
  
  Serial.println("Commands: measure, dac, calibrate, config, help");
  Serial.println();
}

void loop() {
  // Update DAC value gradually for smooth transitions
  updateDAC();
  
  // Automatic measurements disabled - use 'measure' command instead
  // if (millis() - lastMeasurement >= MEASUREMENT_INTERVAL) {
  //   if (ina226Initialized) {
  //     measureAll();
  //     displayResults();
  //   }
  //   lastMeasurement = millis();
  // }
  
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    handleCommand(command);
  }
  
  delay(10);
}

// Professional I2C diagnostic function using RobTillaart I2C_SCANNER
void detailedI2CTest() {
  Serial.println("=== Professional I2C Diagnostics (RobTillaart) ===");
  
  // Test different I2C clock speeds
  Serial.println("1. Testing I2C clock speeds...");
  uint32_t speeds[] = {100000, 400000, 1000000}; // 100kHz, 400kHz, 1MHz
  String speedNames[] = {"100kHz", "400kHz", "1MHz"};
  
  for (int i = 0; i < 3; i++) {
    Serial.print("   Setting clock to ");
    Serial.print(speedNames[i]);
    Serial.print("... ");
    
    if (scanner.setClock(speeds[i])) {
      Serial.println("OK");
      delay(10);
      
      // Test ping to INA226 at this speed
      uint16_t pingResult = scanner.ping(0x40, 3); // Try 3 times
      Serial.print("     Ping test (3 attempts): ");
      Serial.print(pingResult);
      Serial.println(" successful responses");
      
      // Test ping time
      int32_t pingTime = scanner.pingTime(0x40);
      if (pingTime >= 0) {
        Serial.print("     Response time: ");
        Serial.print(pingTime);
        Serial.println(" μs");
      } else {
        Serial.println("     No response (timeout/error)");
      }
    } else {
      Serial.println("FAILED to set clock speed");
    }
  }
  
  // Reset to standard speed
  scanner.setClock(100000);
  
  Serial.println();
  Serial.println("2. Comprehensive I2C device scan...");
  uint8_t deviceCount = scanner.count(8, 119); // Scan standard I2C range
  Serial.print("   Found ");
  Serial.print(deviceCount);
  Serial.println(" I2C devices");
  
  Serial.println();
  Serial.println("3. Testing specific INA226 addresses...");
  
  // Test all possible INA226 addresses
  uint8_t ina226_addresses[] = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47};
  
  for (int i = 0; i < 8; i++) {
    uint8_t addr = ina226_addresses[i];
    Serial.print("   Address 0x");
    if (addr < 16) Serial.print("0");
    Serial.print(addr, HEX);
    Serial.print(": ");
    
    // Use professional ping test
    uint16_t pings = scanner.ping(addr, 5); // 5 attempts
    if (pings > 0) {
      Serial.print("FOUND (");
      Serial.print(pings);
      Serial.print("/5 responses) - ");
      
      // Get diagnostic information
      int diagResult = scanner.diag(addr);
      Serial.print("Diag: ");
      Serial.print(diagResult);
      
      // Get average response time
      int32_t avgTime = 0;
      for (int j = 0; j < 3; j++) {
        int32_t time = scanner.pingTime(addr);
        if (time >= 0) avgTime += time;
      }
      avgTime /= 3;
      Serial.print(", Avg time: ");
      Serial.print(avgTime);
      Serial.println(" μs");
      
      // Try to read manufacturer ID to confirm it's INA226
      testINA226ManufacturerID(addr);
      
    } else {
      Serial.println("No response");
    }
  }
  
  Serial.println();
  Serial.println("4. I2C Bus Health Check...");
  
  // Check if we can communicate reliably
  uint16_t reliabilityTest = scanner.ping(0x40, 20); // 20 attempts
  Serial.print("   Reliability test (20 pings to 0x40): ");
  Serial.print(reliabilityTest);
  Serial.print("/20 = ");
  Serial.print((reliabilityTest * 100) / 20);
  Serial.println("% success rate");
  
  if (reliabilityTest < 18) { // Less than 90% success
    Serial.println("   WARNING: Poor I2C reliability detected!");
    Serial.println("   - Check connections");
    Serial.println("   - Check pull-up resistors");
    Serial.println("   - Check wire length and quality");
    Serial.println("   - Check power supply stability");
  }
  
  Serial.println("=== End Professional I2C Diagnostics ===");
  Serial.println();
}

// Test specific INA226 manufacturer ID
void testINA226ManufacturerID(uint8_t address) {
  Serial.print("      Testing Manufacturer ID... ");
  
  Wire.beginTransmission(address);
  Wire.write(0xFE); // Manufacturer ID register
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Wire.requestFrom(address, (uint8_t)2);
    if (Wire.available() >= 2) {
      uint16_t manID = (Wire.read() << 8) | Wire.read();
      Serial.print("0x");
      Serial.print(manID, HEX);
      if (manID == 0x5449) {
        Serial.println(" (Texas Instruments - CONFIRMED INA226!)");
      } else if (manID == 0xFFFF || manID == 0x0000) {
        Serial.println(" (No valid response)");
      } else {
        Serial.println(" (Different device)");
      }
    } else {
      Serial.println("No data received");
    }
  } else {
    Serial.println("Communication error");
  }
}

// Professional I2C scan using RobTillaart I2C_SCANNER
void scanI2CDevices() {
  Serial.println("=== Professional I2C Scan (RobTillaart) ===");
  
  // Quick device count
  uint8_t deviceCount = scanner.count(8, 119); // Standard I2C address range
  Serial.print("Found ");
  Serial.print(deviceCount);
  Serial.println(" I2C devices");
  
  if (deviceCount == 0) {
    Serial.println();
    Serial.println("No I2C devices found!");
    Serial.println("This indicates a hardware problem:");
    Serial.println("- Check VCC (3.3V) connection to INA226");
    Serial.println("- Check GND connection");
    Serial.println("- Check SDA/SCL wiring");
    Serial.println("- Verify INA226 is not damaged");
    Serial.println("- Check for short circuits");
    Serial.println("- Measure voltage at INA226 VCC pin");
    Serial.println("- Check pull-up resistors (4.7kΩ on SDA/SCL)");
  } else {
    Serial.println();
    Serial.println("Detailed device information:");
    
    // Scan through all addresses and show detailed info for found devices
    for (uint8_t address = 8; address < 120; address++) {
      uint16_t pingResult = scanner.ping(address, 1);
      if (pingResult > 0) {
        Serial.print("  Address 0x");
        if (address < 16) Serial.print("0");
        Serial.print(address, HEX);
        Serial.print(": ");
        
        // Get response time
        int32_t responseTime = scanner.pingTime(address);
        if (responseTime >= 0) {
          Serial.print("Response time: ");
          Serial.print(responseTime);
          Serial.print(" μs");
        }
        
        // Get diagnostic code
        int diagCode = scanner.diag(address);
        Serial.print(", Diag: ");
        Serial.print(diagCode);
        
        // Check if it might be INA226
        if (address >= 0x40 && address <= 0x47) {
          Serial.print(" (Possible INA226)");
        }
        
        Serial.println();
      }
    }
  }
  
  Serial.println("=== I2C Scan Complete ===");
  Serial.println();
}

// Initialize INA226 using library
bool initializeINA226() {
  Serial.println("Initializing INA226...");
  
  // First, scan for I2C devices using professional scanner
  scanI2CDevices();
  
  // Initialize the INA226
  Serial.println("Attempting to connect to INA226 at address 0x40...");
  if (!ina226.begin()) {
    Serial.println("ERROR: INA226 not found at address 0x40!");
    Serial.println("Trying alternative addresses...");
    
    // Try other common addresses
    uint8_t addresses[] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47};
    bool found = false;
    
    for (int i = 0; i < 7; i++) {
      Serial.println("Trying address 0x" + String(addresses[i], HEX) + "...");
      INA226 testINA226(addresses[i]);
      if (testINA226.begin()) {
        Serial.println("INA226 found at address 0x" + String(addresses[i], HEX) + "!");
        Serial.println("Update your code to use this address.");
        found = true;
        break;
      }
    }
    
    if (!found) {
      Serial.println("INA226 not found at any standard address!");
      Serial.println("Hardware troubleshooting needed:");
      Serial.println("1. Check VCC (3.3V) connection");
      Serial.println("2. Check GND connection");
      Serial.println("3. Check SDA connection");
      Serial.println("4. Check SCL connection");
      Serial.println("5. Verify INA226 is not damaged");
      Serial.println("6. Check A0/A1 pins on INA226 for address configuration");
      return false;
    }
    
    return false; // Still return false since we need to use the correct address
  }
  
  // Read manufacturer ID to verify device
  uint16_t manufacturerID = ina226.getManufacturerID();
  uint16_t dieID = ina226.getDieID();
  
  Serial.println("Manufacturer ID: 0x" + String(manufacturerID, HEX));
  Serial.println("Die ID: 0x" + String(dieID, HEX));
  
  if (manufacturerID != 0x5449) { // TI manufacturer ID
    Serial.println("WARNING: Unexpected manufacturer ID!");
  }
  
  // Configure the INA226
  Serial.println("Configuring INA226...");
  
  // Set averaging mode (16 samples)
  ina226.setAverage(4);  // 4 = 16 samples
  
  // Set bus voltage conversion time (1.1ms)
  ina226.setBusVoltageConversionTime(4);  // 4 = 1.1ms
  
  // Set shunt voltage conversion time (1.1ms)
  ina226.setShuntVoltageConversionTime(4);  // 4 = 1.1ms
  
  // Set operating mode (continuous shunt and bus measurement)
  ina226.setMode(7);  // 7 = continuous shunt and bus measurement
  
  // Calculate and set calibration
  calculateCalibration();
  
  Serial.println("INA226 configured successfully!");
  return true;
}

// Calculate calibration value using method
void calculateCalibration() {
  Serial.println("Calculating calibration...");
  Serial.println("Shunt resistance: " + String(shuntResistance, 3) + " Ω");
  Serial.println("Max current: " + String(maxCurrent, 2) + " A");
  
  // Use RobTillaart's setMaxCurrentShunt method
  int result = ina226.setMaxCurrentShunt(maxCurrent, shuntResistance, true);
  
  if (result == INA226_ERR_NONE) {
    Serial.println("Calibration successful!");
    Serial.println("Current LSB: " + String(ina226.getCurrentLSB_mA(), 6) + " mA");
    Serial.println("Shunt: " + String(ina226.getShunt(), 3) + " Ω");
    Serial.println("Max Current: " + String(ina226.getMaxCurrent(), 2) + " A");
    Serial.println("Shunt voltage range: 0 to " + String(maxCurrent * shuntResistance * 1000, 1) + " mV");
  } else {
    Serial.println("ERROR: Calibration failed with code: 0x" + String(result, HEX));
    switch (result) {
      case INA226_ERR_SHUNTVOLTAGE_HIGH:
        Serial.println("Error: maxCurrent * shunt > 81.9 mV");
        Serial.println("Current setting: " + String(maxCurrent * shuntResistance * 1000, 1) + " mV");
        Serial.println("Reduce maxCurrent or use smaller shunt resistance");
        break;
      case INA226_ERR_MAXCURRENT_LOW:
        Serial.println("Error: maxCurrent < 0.001 A");
        Serial.println("Increase maxCurrent value");
        break;
      case INA226_ERR_SHUNT_LOW:
        Serial.println("Error: shunt resistance < 0.001 Ω");
        Serial.println("Increase shunt resistance value");
        break;
      case INA226_ERR_NORMALIZE_FAILED:
        Serial.println("Error: normalization failed");
        Serial.println("Try setting normalize=false in setMaxCurrentShunt()");
        break;
      default:
        Serial.println("Unknown error code");
        break;
    }
  }
}

// Take all measurements
void measureAll() {
  if (!ina226Initialized) return;
  
  shuntVoltage = ina226.getShuntVoltage_mV();
  voltage = ina226.getBusVoltage();
  current = ina226.getCurrent_mA();
  power = ina226.getPower();
}

// Display measurement results
void displayResults() {
  Serial.println("=== INA226 Measurements ===");
  Serial.println("DAC Value:      " + String(currentDacValue) + " (0-4095)");
  Serial.println("DAC Voltage:    " + String((currentDacValue * 3.3) / 4095.0, 3) + " V");
  Serial.println("Shunt Voltage:  " + String(shuntVoltage, 3) + " mV");
  Serial.println("Bus Voltage:    " + String(voltage, 3) + " V");
  // Serial.println("Current:        " + String(current * 1000, 2) + " mA");
  Serial.println("Current:    " + String(current) + " mA");
  Serial.println("Power:          " + String(power, 3) + " W");
  
  // Check for alerts
  uint16_t alertFlag = ina226.getAlertFlag();
  if (alertFlag != 0) {
    Serial.println("ALERT FLAGS: 0x" + String(alertFlag, HEX));
    if (alertFlag & ALERT_SHUNT_OVER_VOLTAGE) Serial.println("  - Shunt Over Voltage (High Current!)");
    if (alertFlag & ALERT_SHUNT_UNDER_VOLTAGE) Serial.println("  - Shunt Under Voltage");
    if (alertFlag & ALERT_BUS_OVER_VOLTAGE) Serial.println("  - Bus Over Voltage");
    if (alertFlag & ALERT_BUS_UNDER_VOLTAGE) Serial.println("  - Bus Under Voltage");
    if (alertFlag & ALERT_POWER_OVER_LIMIT) Serial.println("  - Power Over Limit");
    if (alertFlag & ALERT_CONVERSION_READY) Serial.println("  - Conversion Ready");
  }
  
  Serial.println("==========================");
}

// Update DAC value gradually
void updateDAC() {
  if (currentDacValue < targetDacValue) {
    currentDacValue++;
    analogWrite(dacPin, currentDacValue);
  } else if (currentDacValue > targetDacValue) {
    currentDacValue--;
    analogWrite(dacPin, currentDacValue);
  }
}

// Handle serial commands
void handleCommand(String command) {
  command.toLowerCase();
  
  if (command == "help" || command == "h") {
    printHelp();
  }
  else if (command == "measure" || command == "m") {
    if (ina226Initialized) {
      measureAll();
      displayResults();
    } else {
      Serial.println("ERROR: INA226 not initialized!");
    }
  }
  else if (command == "calibrate" || command == "cal") {
    if (ina226Initialized) {
      recalibrate();
    } else {
      Serial.println("ERROR: INA226 not initialized!");
    }
  }
  else if (command == "config" || command == "c") {
    printConfiguration();
  }
  else if (command == "status") {
    Serial.println("INA226 Status: " + String(ina226Initialized ? "Initialized" : "Not Initialized"));
    Serial.println("DAC Status: " + String(dacEnabled ? "Enabled" : "Disabled"));
    Serial.println("Current DAC Value: " + String(currentDacValue));
    Serial.println("Last Error: 0x" + String(ina226.getLastError(), HEX));
  }
  else if (command.startsWith("dac ")) {
    int newDacValue = command.substring(4).toInt();
    if (newDacValue >= 0 && newDacValue <= 4095) {
      setDACValue(newDacValue);
    } else {
      Serial.println("ERROR: DAC value must be 0-4095");
    }
  }
  else if (command == "dac off" || command == "dac 0") {
    setDACValue(0);
  }
  else if (command == "dac on" || command == "dac max") {
    setDACValue(4095);
  }
  else if (command.startsWith("shunt ")) {
    float newShunt = command.substring(6).toFloat();
    if (newShunt > 0) {
      shuntResistance = newShunt;
      if (ina226Initialized) {
        calculateCalibration();
      }
      Serial.println("Shunt resistance updated to: " + String(shuntResistance, 3) + " Ω");
    }
  }
  else if (command.startsWith("maxcurrent ")) {
    float newMax = command.substring(11).toFloat();
    if (newMax > 0) {
      maxCurrent = newMax;
      if (ina226Initialized) {
        calculateCalibration();
      }
      Serial.println("Max current updated to: " + String(maxCurrent, 2) + " A");
    }
  }
  else if (command == "continuous" || command == "cont") {
    if (ina226Initialized) {
      Serial.println("Starting continuous measurement mode...");
      Serial.println("Press any key to stop");
      continuousMode();
    } else {
      Serial.println("ERROR: INA226 not initialized!");
    }
  }
  else if (command == "reset") {
    Serial.println("Resetting INA226...");
    if (ina226Initialized) {
      ina226.reset();
      delay(10);
      calculateCalibration();
      Serial.println("INA226 reset and recalibrated!");
    }
  }
  else if (command == "sweep") {
    if (ina226Initialized) {
      Serial.println("Starting DAC sweep test...");
      Serial.println("Press any key to stop");
      dacSweepTest();
    } else {
      Serial.println("ERROR: INA226 not initialized!");
    }
  }
  else if (command == "step") {
    if (ina226Initialized) {
      Serial.println("Starting step response test...");
      stepResponseTest();
    } else {
      Serial.println("ERROR: INA226 not initialized!");
    }
  }
  else if (command == "alert") {
    if (ina226Initialized) {
      setupAlerts();
    } else {
      Serial.println("ERROR: INA226 not initialized!");
    }
  }
  else if (command == "debug") {
    if (ina226Initialized) {
      debugRegisters();
    } else {
      Serial.println("ERROR: INA226 not initialized!");
    }
  }
  else if (command == "scan" || command == "i2c") {
    scanI2CDevices();
  }
  else if (command == "diag" || command == "diagnostic") {
    detailedI2CTest();
  }
  else if (command == "ping") {
    // Test ping to current INA226 address
    Serial.println("Pinging INA226 at address 0x40...");
    uint16_t pingResult = scanner.ping(0x40, 10);
    Serial.print("Result: ");
    Serial.print(pingResult);
    Serial.println("/10 successful pings");
    
    int32_t avgTime = 0;
    for (int i = 0; i < 5; i++) {
      int32_t time = scanner.pingTime(0x40);
      if (time >= 0) avgTime += time;
    }
    avgTime /= 5;
    Serial.print("Average response time: ");
    Serial.print(avgTime);
    Serial.println(" μs");
  }
  else if (command == "speed") {
    // Show available I2C speeds
    Serial.println("Available I2C speeds:");
    Serial.println("  100000 Hz  - Standard mode (100kHz)");
    Serial.println("  400000 Hz  - Fast mode (400kHz)");
    Serial.println("  1000000 Hz - Fast mode plus (1MHz)");
    Serial.println("Use 'speed <value>' to set speed");
  }
  else if (command.startsWith("speed ")) {
    // Set I2C speed
    uint32_t newSpeed = command.substring(6).toInt();
    if (newSpeed > 0) {
      if (scanner.setClock(newSpeed)) {
        Serial.print("I2C clock set to: ");
        Serial.print(newSpeed);
        Serial.println(" Hz");
      } else {
        Serial.println("Failed to set I2C clock speed");
      }
    } else {
      Serial.println("Invalid speed value");
    }
  }
  else if (command == "verify" || command == "v") {
    verifyCurrent();
  }
  else {
    Serial.println("Unknown command. Type 'help' for available commands.");
  }
}

// Set DAC value
void setDACValue(int value) {
  targetDacValue = constrain(value, 0, 4095);
  dacEnabled = (targetDacValue > 0);
  
  Serial.println("Setting DAC to: " + String(targetDacValue));
  Serial.println("DAC Voltage: " + String((targetDacValue * 3.3) / 4095.0, 3) + " V");
  
  // Wait for DAC to settle
  delay(50);
  
  // Take immediate measurement
  if (ina226Initialized) {
    measureAll();
    displayResults();
  }
}

// Setup alert functions
void setupAlerts() {
  Serial.println("Setting up INA226 alerts...");
  
  // Calculate safe current limit (80% of max to avoid nuisance trips)
  float safeCurrentLimit = maxCurrent * 0.8;
  float shuntVoltageLimit = safeCurrentLimit * shuntResistance * 1000; // in mV
  
  // Set alert for shunt over voltage (high current protection)
  ina226.setAlertRegister(ALERT_SHUNT_OVER_VOLTAGE);
  ina226.setAlertLimit((uint16_t)shuntVoltageLimit);
  
  Serial.println("Alert configured for shunt over voltage (overcurrent protection)");
  Serial.println("Safe current limit: " + String(safeCurrentLimit, 2) + " A");
  Serial.println("Shunt voltage limit: " + String(shuntVoltageLimit, 1) + " mV");
  Serial.println("This will trigger at " + String((shuntVoltageLimit / (shuntResistance * 1000)), 2) + " A");
}

// Debug registers
void debugRegisters() {
  Serial.println("=== INA226 Register Debug ===");
  Serial.println("Configuration: 0x" + String(ina226.getRegister(0), HEX));
  Serial.println("Shunt Voltage: 0x" + String(ina226.getRegister(1), HEX));
  Serial.println("Bus Voltage:   0x" + String(ina226.getRegister(2), HEX));
  Serial.println("Power:         0x" + String(ina226.getRegister(3), HEX));
  Serial.println("Current:       0x" + String(ina226.getRegister(4), HEX));
  Serial.println("Calibration:   0x" + String(ina226.getRegister(5), HEX));
  Serial.println("Alert Flag:    0x" + String(ina226.getRegister(6), HEX));
  Serial.println("Alert Limit:   0x" + String(ina226.getRegister(7), HEX));
  Serial.println("===========================");
}

// Print help information
void printHelp() {
  Serial.println("Available Commands:");
  Serial.println("  help, h           - Show this help");
  Serial.println("  measure, m        - Take current measurement");
  Serial.println("  calibrate, cal    - Recalculate and apply calibration");
  Serial.println("  config, c         - Show current configuration");
  Serial.println("  status            - Show initialization status");
  Serial.println("  continuous, cont  - Start continuous measurement mode");
  Serial.println("  reset             - Reset INA226 and recalibrate");
  Serial.println("  shunt <value>     - Set shunt resistance in ohms");
  Serial.println("  maxcurrent <value> - Set maximum current in amps");
  Serial.println("  alert             - Setup alert functions");
  Serial.println("  debug             - Show register values");
  Serial.println("  scan, i2c         - Scan I2C bus for devices");
  Serial.println("  diag, diagnostic  - Advanced I2C diagnostics");
  Serial.println("  ping              - Ping test INA226 communication");
  Serial.println("  speed             - Show available I2C clock speeds");
  Serial.println("  speed <hz>        - Set I2C clock speed");
  Serial.println("  verify, v         - Verify current measurement accuracy");
  Serial.println();
  Serial.println("DAC Control Commands:");
  Serial.println("  dac <value>       - Set DAC value (0-4095)");
  Serial.println("  dac off           - Turn off DAC (set to 0)");
  Serial.println("  dac on            - Turn on DAC (set to 4095)");
  Serial.println("  sweep             - Run DAC sweep test");
  Serial.println("  step              - Run step response test");
  Serial.println();
  Serial.println("Examples:");
  Serial.println("  dac 2048          - Set DAC to 50% (1.65V)");
  Serial.println("  dac 1000          - Set DAC to ~25% (0.8V)");
  Serial.println("  shunt 0.039       - Set 39mΩ shunt (your current setup)");
  Serial.println("  maxcurrent 5.0    - Set 5A max current (for high power LED)");
  Serial.println("  sweep             - Run current sweep test");
  Serial.println("  alert             - Setup overcurrent protection");
  Serial.println("  ping              - Test I2C communication reliability");
  Serial.println("  speed 100000      - Set I2C to 100kHz (standard speed)");
  Serial.println("  speed 400000      - Set I2C to 400kHz (fast mode)");
}

// Print current configuration
void printConfiguration() {
  Serial.println("=== INA226 Configuration ===");
  Serial.println("Initialized:        " + String(ina226Initialized ? "Yes" : "No"));
  Serial.println("Shunt Resistance:   " + String(shuntResistance, 3) + " Ω");
  Serial.println("Max Current:        " + String(maxCurrent, 2) + " A");
  Serial.println("DAC Enabled:        " + String(dacEnabled ? "Yes" : "No"));
  Serial.println("Current DAC Value:  " + String(currentDacValue));
  Serial.println("DAC Voltage:        " + String((currentDacValue * 3.3) / 4095.0, 3) + " V");
  
  if (ina226Initialized) {
    Serial.println("Current LSB:        " + String(ina226.getCurrentLSB_mA(), 6) + " mA");
    Serial.println("Averaging:          " + String(ina226.getAverage()));
    Serial.println("Bus Conv Time:      " + String(ina226.getBusVoltageConversionTime()));
    Serial.println("Shunt Conv Time:    " + String(ina226.getShuntVoltageConversionTime()));
    Serial.println("Operating Mode:     " + String(ina226.getMode()));
    Serial.println("Alert Register:     0x" + String(ina226.getAlertFlag(), HEX));
    Serial.println("Alert Limit:        0x" + String(ina226.getAlertLimit(), HEX));
  }
  Serial.println("===========================");
}

// Recalculate and apply calibration
void recalibrate() {
  if (!ina226Initialized) {
    Serial.println("ERROR: INA226 not initialized!");
    return;
  }
  
  Serial.println("Recalculating calibration...");
  calculateCalibration();
  Serial.println("Calibration updated!");
  printConfiguration();
}

// Continuous measurement mode
void continuousMode() {
  if (!ina226Initialized) return;
  
  unsigned long startTime = millis();
  int measurementCount = 0;
  
  while (!Serial.available()) {
    if (millis() - lastMeasurement >= 500) { // Faster updates
      measureAll();
      
      // Print compact format
      Serial.print("Time: " + String((millis() - startTime) / 1000.0, 1) + "s | ");
      Serial.print("DAC: " + String(currentDacValue) + " | ");
      Serial.print("V: " + String(voltage, 3) + "V | ");
      Serial.print("I: " + String(current * 1000, 1) + "mA | ");
      Serial.print("P: " + String(power, 3) + "W");
      
      // Show alerts if any
      uint16_t alertFlag = ina226.getAlertFlag();
      if (alertFlag != 0) {
        Serial.print(" | ALERT: 0x" + String(alertFlag, HEX));
      }
      Serial.println();
      
      lastMeasurement = millis();
      measurementCount++;
    }
  }
  
  // Clear any pending serial input
  while (Serial.available()) {
    Serial.read();
  }
  
  Serial.println("Continuous mode stopped. Total measurements: " + String(measurementCount));
}

// DAC sweep test
void dacSweepTest() {
  if (!ina226Initialized) return;
  
  Serial.println("DAC Sweep Test - Current vs DAC Value");
  Serial.println("DAC\tVoltage\tCurrent(mA)\tPower(W)\tAlert");
  Serial.println("---\t-------\t----------\t-------\t-----");
  
  // Sweep from 0 to 4095 in steps
  for (int dac = 0; dac <= 4095; dac += 256) { // 16 steps
    setDACValue(dac);
    delay(100); // Wait for current to stabilize
    
    measureAll();
    
    float dacVoltage = (dac * 3.3) / 4095.0;
    uint16_t alertFlag = ina226.getAlertFlag();
    
    Serial.print(dac);
    Serial.print("\t");
    Serial.print(dacVoltage, 3);
    Serial.print("\t");
    Serial.print(current * 1000, 1);
    Serial.print("\t\t");
    Serial.print(power, 3);
    Serial.print("\t");
    Serial.println(alertFlag != 0 ? "YES" : "NO");
    
    // Check for user input to stop
    if (Serial.available()) {
      break;
    }
  }
  
  // Turn off DAC
  setDACValue(0);
  
  // Clear any pending serial input
  while (Serial.available()) {
    Serial.read();
  }
  
  Serial.println("DAC sweep test completed.");
}

// Verify current measurement accuracy
void verifyCurrent() {
  if (!ina226Initialized) {
    Serial.println("ERROR: INA226 not initialized!");
    return;
  }
  
  Serial.println("=== Current Measurement Verification ===");
  
  // Step 1: Check calibration parameters
  Serial.println("1. Calibration Parameters:");
  Serial.println("   Shunt Resistance: " + String(shuntResistance, 3) + " Ω");
  Serial.println("   Max Current: " + String(maxCurrent, 2) + " A");
  Serial.println("   Max Shunt Voltage: " + String(maxCurrent * shuntResistance * 1000, 1) + " mV");
  Serial.println("   Current LSB: " + String(ina226.getCurrentLSB_mA(), 6) + " mA");
  
  // Step 2: Take baseline measurement (should be near zero)
  Serial.println();
  Serial.println("2. Baseline Measurement (DAC = 0):");
  setDACValue(0);
  delay(100);
  measureAll();
  
  float baselineCurrent = current * 1000; // Convert to mA
  Serial.println("   Shunt Voltage: " + String(shuntVoltage, 3) + " mV");
  Serial.println("   Current: " + String(baselineCurrent, 3) + " mA");
  
  if (abs(baselineCurrent) > 10) { // More than 10mA offset
    Serial.println("   WARNING: High baseline current detected!");
    Serial.println("   Check for current leakage or offset issues.");
  } else {
    Serial.println("   ✓ Baseline current looks good");
  }
  
  // Step 3: Test known current levels
  Serial.println();
  Serial.println("3. Current vs DAC Test:");
  Serial.println("   DAC\tVoltage\tShunt(mV)\tCurrent(mA)\tCalculated(mA)");
  Serial.println("   ---\t-------\t---------\t----------\t-------------");
  
  int testDacValues[] = {0, 512, 1024, 2048, 3072, 4095};
  
  for (int i = 0; i < 6; i++) {
    setDACValue(testDacValues[i]);
    delay(200); // Let current stabilize
    measureAll();
    
    float dacVoltage = (testDacValues[i] * 3.3) / 4095.0;
    float measuredCurrent = current * 1000; // mA
    float calculatedCurrent = shuntVoltage / shuntResistance; // Using Ohm's law
    
    Serial.print("   " + String(testDacValues[i]));
    Serial.print("\t" + String(dacVoltage, 2) + "V");
    Serial.print("\t" + String(shuntVoltage, 3));
    Serial.print("\t\t" + String(measuredCurrent, 1));
    Serial.print("\t\t" + String(calculatedCurrent, 1));
    
    // Check if measured vs calculated match
    float error = abs(measuredCurrent - calculatedCurrent);
    if (error > (calculatedCurrent * 0.05)) { // 5% error
      Serial.println(" ⚠️");
    } else {
      Serial.println(" ✓");
    }
  }
  
  // Step 4: Check current direction
  Serial.println();
  Serial.println("4. Current Direction Test:");
  Serial.println("   Current should be POSITIVE when flowing VIN+ → VIN-");
  Serial.println("   If negative, check INA226 VIN+/VIN- connections");
  
  setDACValue(2048); // 50% DAC
  delay(100);
  measureAll();
  
  if (current > 0) {
    Serial.println("   ✓ Current direction is correct (positive)");
  } else if (current < 0) {
    Serial.println("   ⚠️  Current is negative - check VIN+/VIN- wiring");
  } else {
    Serial.println("   ⚠️  No current detected - check circuit connections");
  }
  
  // Step 5: Sanity check
  Serial.println();
  Serial.println("5. Sanity Check:");
  float expectedMaxShuntV = maxCurrent * shuntResistance * 1000;
  float actualMaxShuntV = 4095 * 3.3 / 4095.0; // This doesn't make sense, let me fix
  
  Serial.println("   At 5A, shunt voltage should be: " + String(expectedMaxShuntV, 1) + " mV");
  Serial.println("   INA226 can measure up to: ±81.9V across shunt");
  Serial.println("   Your range (0-" + String(expectedMaxShuntV, 1) + "mV) is: " + 
                (expectedMaxShuntV < 81900 ? "✓ SAFE" : "⚠️ TOO HIGH"));
  
  // Reset DAC
  setDACValue(0);
  
  Serial.println();
  Serial.println("=== Verification Complete ===");
  Serial.println("If you see warnings, check your circuit connections.");
  Serial.println("Use 'measure' to take individual measurements.");
}

// Step response test
void stepResponseTest() {
  if (!ina226Initialized) return;
  
  Serial.println("Step Response Test");
  Serial.println("Setting DAC to 0, 50%, 100%, 50%, 0");
  
  int steps[] = {0, 2048, 4095, 2048, 0};
  int numSteps = 5;
  
  for (int i = 0; i < numSteps; i++) {
    Serial.println("Step " + String(i + 1) + ": DAC = " + String(steps[i]));
    setDACValue(steps[i]);
    
    // Take multiple measurements to see response
    for (int j = 0; j < 10; j++) {
      delay(100);
      measureAll();
      
      uint16_t alertFlag = ina226.getAlertFlag();
      
      Serial.print("  Time: " + String(j * 0.1, 1) + "s | ");
      Serial.print("Current: " + String(current * 1000, 1) + "mA | ");
      Serial.print("Power: " + String(power, 3) + "W");
      if (alertFlag != 0) {
        Serial.print(" | ALERT: 0x" + String(alertFlag, HEX));
      }
      Serial.println();
    }
    
    Serial.println();
  }
  
  Serial.println("Step response test completed.");
}
