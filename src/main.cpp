#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Library includes
#include "SystemConfig.h"
#include "Logger.h"
#include "RS03Motor.h"
#include "SpektrumSatelliteReader.h"
#include "RCInputManager.h"
#include "DisplayManager.h"
#include "MotorController.h"
#include "FeatherM4CanInterface.h"
#include <CANSAME5x.h>

// ----- Global Objects -----
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
CANSAME5x CAN;
SpektrumSatelliteReader spektrumReader(SPEKTRUM_SERIAL, 12, MIN_PULSE, MAX_PULSE, MID_PULSE);

// Hardware interfaces
FeatherM4CanInterface canBus;
RS03Motor motor1(canBus, MOTOR_ID_1, MASTER_ID);
RS03Motor motor2(canBus, MOTOR_ID_2, MASTER_ID);

// High-level managers  
RCInputManager rcInput(spektrumReader);
MotorController motorController(motor1, motor2, rcInput, pixels);

// ----- System Status Variables -----
static bool displayAvailable = false;
static bool rcAvailable = false;
static bool canAvailable = false;
static bool motorControllerAvailable = false;
static bool motorsReady = false;

// ----- Setup -----
void setup() {
    // Initialize serial communication with timeout
    Serial.begin(115200);
    // Wait for serial connection with 5 second timeout
    unsigned long serialStartTime = millis();
    while (!Serial && (millis() - serialStartTime < 5000)) {
        delay(10); // Wait for serial connection, but not forever
    }
    
    // Initialize logger
    Logger::init();
    Logger::info("Simple RC Channel Reader - Starting");
    
    // Initialize OLED display (non-blocking)
    if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        displayAvailable = true;
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("RC Channel Reader");
        display.println("Initializing...");
        display.display();
        Logger::info("OLED display initialized");
    } else {
        Logger::error("SSD1306 allocation failed - continuing without display");
        if (Serial) {
            Serial.println("OLED Display initialization failed - continuing without display");
        }
    }
    
    // Initialize NeoPixel
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(50, 50, 0)); // Yellow while initializing
    pixels.show();
    Logger::info("NeoPixel initialized");
    
    // Initialize RC input (non-blocking)
    if (spektrumReader.begin()) {
        rcAvailable = true;
        Logger::info("Spektrum reader initialized");
    } else {
        Logger::error("Spektrum reader initialization failed - continuing without RC input");
        if (Serial) {
            Serial.println("RC Receiver initialization failed - continuing without RC input");
        }
        pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red for error
        pixels.show();
        
        if (displayAvailable) {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("RC Init Failed!");
            display.println("Motors disabled");
            display.display();
        }
    }
    
    // Initialize CAN bus (non-blocking)
    pinMode(PIN_CAN_STANDBY, OUTPUT);
    digitalWrite(PIN_CAN_STANDBY, false);
    pinMode(PIN_CAN_BOOSTEN, OUTPUT);
    digitalWrite(PIN_CAN_BOOSTEN, true);
    
    if (CAN.begin(1000000)) {
        canAvailable = true;
        Logger::info("CAN initialized at 1 Mbit/s");
    } else {
        Logger::error("Starting CAN failed - motors will not function");
        if (Serial) {
            Serial.println("CAN bus initialization failed - motors will not function");
        }
        pixels.setPixelColor(0, pixels.Color(50, 0, 0));
        pixels.show();
    }
    
    // Initialize motor controller (non-blocking)
    if (canAvailable && motorController.begin()) {
        motorControllerAvailable = true;
        Logger::info("Motor controller initialized");
    } else {
        Logger::error("Motor controller initialization failed - motors will not function");
        if (Serial) {
            Serial.println("Motor controller initialization failed - motors will not function");
        }
        pixels.setPixelColor(0, pixels.Color(50, 0, 0));
        pixels.show();
        
        if (displayAvailable) {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Motor Init Failed!");
            display.display();
        }
    }
    
    // Motor setup sequence with proper timing (only if motor controller is available)
    if (motorControllerAvailable) {
        Logger::info("Starting motor setup sequence...");
        
        // Step 1: Reset any faults and enable motors
        Logger::info("Step 1: Enabling motors and clearing faults");
        motor1.resetFaults();
        motor2.resetFaults();
        delay(100);
        
        motor1.enable();
        motor2.enable();
        delay(200);  // Give motors time to enable
    
    // Step 2: Configure zero flag and set mechanical zero positions  
    Logger::info("Step 2: Configuring zero flag for proper position reference");
    // Set zero flag to 1 for -π to π range (more suitable for position control)
    if (!motor1.setZeroFlag(1)) {
        Logger::warning("Failed to set zero flag for motor1");
    }
    if (!motor2.setZeroFlag(1)) {
        Logger::warning("Failed to set zero flag for motor2");
    }
    delay(200);
    
    Logger::info("Setting mechanical zero positions");
    bool zero1Success = motor1.setMechanicalZero();
    bool zero2Success = motor2.setMechanicalZero();
    
    if (!zero1Success) {
        Logger::error("Failed to set mechanical zero for motor1!");
    }
    if (!zero2Success) {
        Logger::error("Failed to set mechanical zero for motor2!");
    }
    
    delay(500);  // Allow time for zero setting to complete
    
    // Step 3: Set motors to position mode (after mechanical zero is set)
    Logger::info("Step 3: Setting position mode");
    if (!motor1.setModePositionPP(25.0f, 200.0f, 40.0f)) {
        Logger::error("Failed to set motor1 to position mode!");
    }
    if (!motor2.setModePositionPP(25.0f, 200.0f, 40.0f)) {
        Logger::error("Failed to set motor2 to position mode!");
    }
    delay(500);  // Allow time for mode setting to complete
    
    // Step 4: Enable active reporting for position feedback
    Logger::info("Step 4: Enabling active reporting");
    
    // Enable active reporting for motor1 with fallback
    bool reportingEnabled1 = motor1.setActiveReporting(true);
    if (!reportingEnabled1) {
        Logger::warning("Library method failed, trying direct approach for active reporting for motor1");
        
        // Type 5 message for active reporting command (from old working version)
        uint32_t reportingId1 = ((static_cast<uint32_t>(0x05) << 24) | 
                                (static_cast<uint32_t>(MASTER_ID) << 8) | 
                                static_cast<uint32_t>(MOTOR_ID_1)) & 0x1FFFFFFF;
        
        uint8_t reportingData[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x01 = enable reporting
        
        if (CAN.beginExtendedPacket(reportingId1)) {
            CAN.write(reportingData, 8);
            if (CAN.endPacket()) {
                Logger::info("Direct active reporting command sent for motor1");
                reportingEnabled1 = true;
            } else {
                Logger::error("Failed to send direct active reporting command for motor1");
            }
        } else {
            Logger::error("Failed to begin direct active reporting packet for motor1");
        }
    }
    
    // Enable active reporting for motor2 with fallback
    bool reportingEnabled2 = motor2.setActiveReporting(true);
    if (!reportingEnabled2) {
        Logger::warning("Library method failed, trying direct approach for active reporting for motor2");
        
        // Type 5 message for active reporting command (from old working version)
        uint32_t reportingId2 = ((static_cast<uint32_t>(0x05) << 24) | 
                                (static_cast<uint32_t>(MASTER_ID) << 8) | 
                                static_cast<uint32_t>(MOTOR_ID_2)) & 0x1FFFFFFF;
        
        uint8_t reportingData[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x01 = enable reporting
        
        if (CAN.beginExtendedPacket(reportingId2)) {
            CAN.write(reportingData, 8);
            if (CAN.endPacket()) {
                Logger::info("Direct active reporting command sent for motor2");
                reportingEnabled2 = true;
            } else {
                Logger::error("Failed to send direct active reporting command for motor2");
            }
        } else {
            Logger::error("Failed to begin direct active reporting packet for motor2");
        }
    }
    
    if (reportingEnabled1 && reportingEnabled2) {
        Logger::info("Active reporting enabled for both motors");
    } else {
        Logger::warning("Active reporting setup had issues - some motors may not report feedback");
    }
    
    delay(200);
    
    // Step 5: Move both motors to position 0 (should already be there after zero setting)
    Logger::info("Step 5: Moving to zero position");
    motor1.setPosition(0.0f);
    motor2.setPosition(0.0f);
    delay(200);
    
        // Verify setup
        if (zero1Success && zero2Success) {
            Logger::info("Motor setup completed successfully");
            motorsReady = true;
        } else {
            Logger::warning("Motor setup completed with errors - check motor connections");
            motorsReady = false;
        }
    } else {
        Logger::warning("Motor controller not available - motors will not function");
        motorsReady = false;
    }
    
    // System ready indication
    if (motorsReady && rcAvailable) {
        pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green indicates fully ready
        Logger::info("Setup complete. Motors ready for RC control.");
    } else if (motorsReady) {
        pixels.setPixelColor(0, pixels.Color(0, 25, 25)); // Cyan indicates motors ready but no RC
        Logger::info("Setup complete. Motors ready but no RC input.");
    } else {
        pixels.setPixelColor(0, pixels.Color(25, 25, 0)); // Yellow indicates limited functionality
        Logger::info("Setup complete with limited functionality - check connections.");
    }
    pixels.show();
    
    // Update display with current status
    if (displayAvailable) {
        display.clearDisplay();
        display.setCursor(0, 0);
        if (motorsReady && rcAvailable) {
            display.println("Motor Control Ready");
            display.println("Zero set first");
            display.println("Ch4: Mode 1,2,3");
            display.println("SwA: Zero");
            display.println("M1: Reversed");
        } else if (motorsReady) {
            display.println("Motors Ready");
            display.println("No RC Input");
            display.println("Check RC connection");
        } else {
            display.println("Limited Function");
            display.println("Check connections");
            display.println("Motors disabled");
        }
        display.display();
        delay(1000);
    }
    
    // Print status to serial if available
    if (Serial) {
        if (motorsReady && rcAvailable) {
            Serial.println("Motor Control Ready - Full Functionality");
        } else if (motorsReady) {
            Serial.println("Motors Ready - No RC Input");
        } else {
            Serial.println("Limited Functionality - Check Connections");
        }
        
        Serial.println("FIXED: Updated position constants from -12.5/12.5 to -12.57/12.57 rad");
        Serial.println("FIXED: Corrected mode values and parameter indices from manual");
        Serial.println("FIXED: Zero flag set to 1 for -π to π position range");
        Serial.println("Control Scheme:");
        Serial.println("  Channel 4 Mode 1 = Velocity Mode");
        Serial.println("  Channel 4 Mode 2 = Position Mode (Switch B control)");
        Serial.println("  Channel 4 Mode 3 = Position Mode (Channel 5 control, base=1.9rad)");
        Serial.println("  Switch A = Set mechanical zero");
        Serial.println("  Channel 5 = Velocity control (Mode 1) or Position offset ±2π (Mode 3)");
        Serial.println("  Switch B = Position control: OFF=0rad, ON=3.8rad (Mode 2 only)");
        Serial.println("  Channel 8 = State tracking only (Low/Middle/High)");
        Serial.println("  Motor 1 is REVERSED by default");
        Serial.println("Ch1  Ch2  Ch3  MODE  Control_Mode  Target/Velocity  M1_Pos  M2_Pos  Switches  Ch8State");
    }
}

// ----- Additional Variables for Motor Control -----
static bool inPositionMode = false;
static bool modeInitialized = false;
static float currentTargetPosition = 0.0f;

// ----- Main Loop -----
void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastSerialPrint = 0;
    static unsigned long lastStatsLog = 0;
    static unsigned long lastMotorUpdate = 0;
    unsigned long currentLoopTime = millis();
    
    // Update RC receiver (only if RC is available)
    if (rcAvailable) {
        spektrumReader.update();
    }
    
    // Process CAN messages more aggressively to update motor feedback (only if CAN is available)
    if (canAvailable) {
        // Call multiple times per loop to be more aggressive about capturing feedback
        for (int i = 0; i < 5; i++) {
            int packetSize = CAN.parsePacket();
            if (packetSize) {
                CanFrame frame;
                frame.id = CAN.packetId();
                frame.is_extended = CAN.packetExtended();
                frame.dlc = packetSize;
                
                // Read data bytes
                int j = 0;
                while (CAN.available() && j < 8) {
                    frame.data[j++] = CAN.read();
                }
                
                // Process the frame with both motors to update their feedback
                motor1.processFeedback(frame);
                motor2.processFeedback(frame);
            }
            // Small delay between processing attempts
            delayMicroseconds(100);
        }
    }
    
    // Periodically request motor state directly using Type 0 query for both motors (only if CAN is available)
    if (canAvailable) {
        static unsigned long lastDirectQueryTime = 0;
        if (currentLoopTime - lastDirectQueryTime >= 100) { // Every 100ms
        // Send a Type 0 query to request state from motor 1
        uint32_t queryId1 = ((0x00) << 24) | (MASTER_ID << 16) | MOTOR_ID_1;
        if (CAN.beginExtendedPacket(queryId1 & 0x1FFFFFFF)) {
            // Type 0 has no data
            if (CAN.endPacket()) {
                Logger::debug("Sent Type 0 query to request motor 1 state");
            }
        }
        
        // Send a Type 0 query to request state from motor 2
        uint32_t queryId2 = ((0x00) << 24) | (MASTER_ID << 16) | MOTOR_ID_2;
        if (CAN.beginExtendedPacket(queryId2 & 0x1FFFFFFF)) {
            // Type 0 has no data
            if (CAN.endPacket()) {
                Logger::debug("Sent Type 0 query to request motor 2 state");
            }
        }
        
            lastDirectQueryTime = currentLoopTime;
        }
    }
    
    // PRIORITY: Motor control logic (every 20ms) - only if motors and RC are ready
    // This happens BEFORE display/serial updates to ensure consistent timing
    if (motorsReady && rcAvailable && spektrumReader.isReceiving() && 
        currentLoopTime - lastMotorUpdate >= 20) {
        
        // Get RC channel values and switch states
        int ch5Value = spektrumReader.getChannel(4);  // Channel 5 for velocity control (0-indexed as 4)
        int mode = spektrumReader.getModeFromChannel4();  // Channel 4 mode selection
        auto switches = spektrumReader.getSwitchStatesFromChannel6();  // Switch states
        auto ch8State = spektrumReader.getChannel8State();  // Channel 8 state tracking
        
        // Motor 1 is reversed by default
        bool motor1Reversed = true;
        bool motor2Reversed = false;
        
        // Determine control mode based on Channel 4 mode
        // Mode 1 = Velocity Mode, Mode 2 = Position Mode (Switch B), Mode 3 = Position Mode (Channel 5)
        bool shouldBeInPositionMode = (mode == 2 || mode == 3);
        
        // Switch modes if needed or if switching between different position modes
        static int lastMode = 0;
        if (shouldBeInPositionMode != inPositionMode || mode != lastMode) {
            inPositionMode = shouldBeInPositionMode;
            modeInitialized = false;
            if (mode == 1) {
                Logger::info("Switching to velocity mode (Mode 1)");
            } else if (mode == 2) {
                Logger::info("Switching to position mode with Switch B control (Mode 2)");
            } else if (mode == 3) {
                Logger::info("Switching to position mode with Channel 5 control (Mode 3)");
            }
            lastMode = mode;
        }
        
        // Initialize mode if needed (always control both motors)
        if (!modeInitialized) {
            if (inPositionMode) {
                // Set position mode for both motors
                motor1.setModePositionPP(POSITION_SPEED_LIMIT, POSITION_ACCELERATION, MOTOR_CURRENT_LIMIT);
                motor2.setModePositionPP(POSITION_SPEED_LIMIT, POSITION_ACCELERATION, MOTOR_CURRENT_LIMIT);
                Logger::info("Both motors set to position mode");
            } else {
                // Set velocity mode for both motors
                motor1.setModeVelocity();
                motor2.setModeVelocity();
                Logger::info("Both motors set to velocity mode");
            }
            modeInitialized = true;
            delay(100);  // Give time for mode change
        }
        
        // Handle zero position command using Switch A
        static bool lastSwitchAState = false;
        if (switches.switchA && !lastSwitchAState) {
            // Rising edge detection - switch just turned ON
            Logger::info("Switch A pressed - Setting mechanical zero position");
            motor1.setMechanicalZero();
            motor2.setMechanicalZero();
            currentTargetPosition = 0.0f;
        }
        lastSwitchAState = switches.switchA;
        
        // Execute motor commands based on mode
        if (modeInitialized) {
            if (inPositionMode) {
                if (mode == 2) {
                    // Mode 2: Position mode - use Switch B to control position (0 or 3.8 radians)
                    if (switches.switchB) {
                        currentTargetPosition = 3.8f;  // Switch B ON = 3.8 radians
                    } else {
                        currentTargetPosition = 0.0f;  // Switch B OFF = 0 radians
                    }
                } else if (mode == 3) {
                    // Mode 3: Position mode - use Channel 5 to control position around 1.9 rad base with ±2π range
                    float basePosition = 1.9f;
                    float maxOffset = 2.0f * PI;  // ±2π radians
                    
                    // Map Channel 5 from pulse range to position offset
                    float positionOffset = 0.0f;
                    if (ch5Value >= (MID_PULSE - DEAD_ZONE) && ch5Value <= (MID_PULSE + DEAD_ZONE)) {
                        positionOffset = 0.0f;  // Dead zone - no offset from base position
                    } else if (ch5Value < MID_PULSE) {
                        // Map lower half to negative offset
                        positionOffset = map(ch5Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -maxOffset * 1000, 0) * 0.001f;
                    } else {
                        // Map upper half to positive offset
                        positionOffset = map(ch5Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, maxOffset * 1000) * 0.001f;
                    }
                    
                    currentTargetPosition = basePosition + positionOffset;
                }
                
                // Send position commands to both motors with direction control
                float motor1TargetPosition = motor1Reversed ? -currentTargetPosition : currentTargetPosition;
                float motor2TargetPosition = motor2Reversed ? -currentTargetPosition : currentTargetPosition;
                motor1.setPosition(motor1TargetPosition);
                motor2.setPosition(motor2TargetPosition);
                
            } else {
                // Mode 1: Velocity mode - use Channel 5 for velocity control with dead zone
                float targetVelocity = 0.0f;
                if (ch5Value >= (MID_PULSE - DEAD_ZONE) && ch5Value <= (MID_PULSE + DEAD_ZONE)) {
                    targetVelocity = 0.0f;  // Dead zone
                } else if (ch5Value < MID_PULSE) {
                    targetVelocity = map(ch5Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -MAX_VELOCITY * 1000, 0) * 0.001f;
                } else {
                    targetVelocity = map(ch5Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, MAX_VELOCITY * 1000) * 0.001f;
                }
                
                // Send velocity commands to both motors with direction control
                float motor1TargetVelocity = motor1Reversed ? -targetVelocity : targetVelocity;
                float motor2TargetVelocity = motor2Reversed ? -targetVelocity : targetVelocity;
                motor1.setVelocity(motor1TargetVelocity);
                motor2.setVelocity(motor2TargetVelocity);
                
                // Update target position for display (in velocity mode, show current velocity as target)
                currentTargetPosition = targetVelocity;
            }
        }
        
        lastMotorUpdate = currentLoopTime;
    }
    
    // Handle RC signal status and LED updates (less critical timing)
    bool hasValidRCSignal = rcAvailable && spektrumReader.isReceiving();
    
    if (!hasValidRCSignal) {
        // No signal or RC not available - show error
        pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red
        pixels.show();
        
        // Update display when no signal/RC (less frequently to avoid blocking motor updates)
        if (displayAvailable && currentLoopTime - lastUpdate >= 500) {
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            if (!rcAvailable) {
                display.println("RC Not Available");
                display.println("Motors Disabled");
                display.println("Check RC connection");
            } else {
                display.println("RC Channel Reader");
                display.println("");
                display.println("NO SIGNAL");
                display.println("Check RC connection");
            }
            display.display();
            lastUpdate = currentLoopTime;
        }
        
        // Print status to serial (less frequently)
        if (Serial && currentLoopTime - lastSerialPrint >= 2000) {
            if (!rcAvailable) {
                Serial.println("RC INPUT NOT AVAILABLE - Motors disabled for safety");
            } else {
                Serial.println("NO RC SIGNAL");
            }
            lastSerialPrint = currentLoopTime;
        }
        
        // Use shorter delay to maintain responsiveness for when signal returns
        delay(10);
        return;
    }
    
    // We have valid RC signal - show green LED
    pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green
    pixels.show();
    
    // Read all channels (up to 12) for display/serial output
    int numChannelsToShow = min(12, spektrumReader.getChannelCount());
    int channels[12];
    for (int i = 0; i < numChannelsToShow; i++) {
        channels[i] = spektrumReader.getChannel(i);
    }
    
    // Get decoded special channels for display/serial output
    int mode = spektrumReader.getModeFromChannel4();
    auto switches = spektrumReader.getSwitchStatesFromChannel6();
    auto ch8State = spektrumReader.getChannel8State();
    
    // Update OLED display every 150ms (only if display is available)
    if (displayAvailable && currentLoopTime - lastUpdate >= 150) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print("Motor Control (");
        display.print(numChannelsToShow);
        display.println(")");
        
        // Show first 3 channels
        for (int i = 0; i < min(3, numChannelsToShow); i++) {
            display.print("Ch");
            display.print(i + 1);
            display.print(":");
            display.println(channels[i]);
        }
        
        // Show MODE instead of channel 4
        if (numChannelsToShow > 3) {
            display.print("MODE:");
            if (mode > 0) {
                display.println(mode);
            } else {
                display.println("?");
            }
        }
        
        // Show current mode based on Channel 4
        display.print("Mode:");
        if (mode == 1) {
            display.print("VEL (1)");
        } else if (mode == 2) {
            display.print("POS (2)");
        } else if (mode == 3) {
            display.print("POS (3)");
        } else {
            display.print("? (");
            display.print(mode);
            display.print(")");
        }
        display.println();
        
        // Show control source
        if (mode == 1) {
            display.println("Ctrl: Channel 5");
        } else if (mode == 2) {
            display.println("Ctrl: Switch B");
        } else if (mode == 3) {
            display.println("Ctrl: Channel 5");
        } else {
            display.println("Ctrl: Unknown");
        }
        
        // Show target value (position in POS mode, velocity in VEL mode)
        if (inPositionMode) {
            display.print("Tgt:");
            display.print(currentTargetPosition, 1);
            display.println("rad");
        } else {
            display.print("Vel:");
            display.print(currentTargetPosition, 1);
            display.println("r/s");
        }
        
        // Show actual motor positions (with error handling)
        float motor1Pos = motor1.getLastFeedback().position;
        float motor2Pos = motor2.getLastFeedback().position;
        display.print("M1:");
        display.print(motor1Pos, 1);
        display.print(" M2:");
        display.println(motor2Pos, 1);
        
        // Show switch states and direction control
        if (numChannelsToShow > 5) {
            display.print("SW A:");
            display.print(switches.switchA ? "ON " : "OFF");
            display.print(" B:");
            display.println(switches.switchB ? "ON" : "OFF");
            
            // Show Channel 8 state
            display.print("Ch8: ");
            switch (ch8State) {
                case SpektrumSatelliteReader::CH8_LOW:
                    display.println("Low");
                    break;
                case SpektrumSatelliteReader::CH8_MIDDLE:
                    display.println("Middle");
                    break;
                case SpektrumSatelliteReader::CH8_HIGH:
                    display.println("High");
                    break;
                default:
                    display.println("Unknown");
                    break;
            }
        }
        
        display.display();
        lastUpdate = currentLoopTime;
    }
    
    // Print to serial console every 100ms (only if serial is available)
    if (Serial && currentLoopTime - lastSerialPrint >= 100) {
        // Print channels 1-3
        for (int i = 0; i < min(3, numChannelsToShow); i++) {
            Serial.print(channels[i]);
            Serial.print("  ");
        }
        
        // Print MODE instead of channel 4
        if (numChannelsToShow > 3) {
            Serial.print("MODE:");
            if (mode > 0) {
                Serial.print(mode);
            } else {
                Serial.print("?");
            }
            Serial.print("  ");
        }
        
        // Print mode information
        Serial.print("Mode:");
        if (mode == 1) {
            Serial.print("VEL(1)");
        } else if (mode == 2) {
            Serial.print("POS(2)");
        } else if (mode == 3) {
            Serial.print("POS(3)");
        } else {
            Serial.print("?(");
            Serial.print(mode);
            Serial.print(")");
        }
        Serial.print("  ");
        
        // Print target value (position or velocity)
        if (inPositionMode) {
            Serial.print("Target:");
            Serial.print(currentTargetPosition, 1);
            Serial.print("rad  ");
        } else {
            Serial.print("Velocity:");
            Serial.print(currentTargetPosition, 1);
            Serial.print("r/s  ");
        }
        
        // Get and print actual motor positions
        float motor1Pos = motor1.getLastFeedback().position;
        float motor2Pos = motor2.getLastFeedback().position;
        Serial.print("M1:");
        Serial.print(motor1Pos, 2);
        Serial.print("rad  M2:");
        Serial.print(motor2Pos, 2);
        Serial.print("rad  ");
        
        // Print switch states and direction control
        if (numChannelsToShow > 5) {
            Serial.print("A:");
            Serial.print(switches.switchA ? "ON" : "OFF");
            Serial.print(",B:");
            Serial.print(switches.switchB ? "ON" : "OFF");
            Serial.print("  ");
            
            // Print Channel 8 state
            Serial.print("Ch8:");
            Serial.print(spektrumReader.getChannel(7));
            Serial.print("(");
            switch (ch8State) {
                case SpektrumSatelliteReader::CH8_LOW:
                    Serial.print("Low");
                    break;
                case SpektrumSatelliteReader::CH8_MIDDLE:
                    Serial.print("Middle");
                    break;
                case SpektrumSatelliteReader::CH8_HIGH:
                    Serial.print("High");
                    break;
                default:
                    Serial.print("Unknown");
                    break;
            }
            Serial.print(")  ");
        }
        
        // Print remaining channels
        for (int i = 6; i < numChannelsToShow; i++) {
            Serial.print(channels[i]);
            if (i < numChannelsToShow - 1) {
                Serial.print("  ");
            }
        }
        
        Serial.println();
        lastSerialPrint = currentLoopTime;
    }
    
    // Print frame statistics every 5 seconds for debugging (only if serial is available)
    if (Serial && currentLoopTime - lastStatsLog >= 5000) {
        auto stats = spektrumReader.getFrameStats();
        Serial.println();
        Serial.println("=== FRAME STATISTICS ===");
        Serial.print("Total Frames: "); Serial.println(stats.totalFrames);
        Serial.print("Valid Frames: "); Serial.println(stats.validFrames);
        Serial.print("Format Samples: "); Serial.println(stats.formatDetectSamples);
        Serial.print("Format: "); Serial.println(stats.is11Bit ? "11-bit" : "10-bit");
        Serial.print("Detected Channels: "); Serial.println(stats.detectedChannels);
        Serial.print("Last Frame: "); Serial.print(millis() - stats.lastFrameTime); Serial.println("ms ago");
        
        // Calculate frame rate
        if (stats.totalFrames > 0 && stats.lastFrameTime > 0) {
            float frameRate = 1000.0 * stats.validFrames / stats.lastFrameTime;
            Serial.print("Frame Rate: "); Serial.print(frameRate, 1); Serial.println(" Hz");
        }
        
        Serial.println("========================");
        Serial.println();
        lastStatsLog = currentLoopTime;
    }
    
    // Small delay to prevent tight looping
    delay(10);
} 