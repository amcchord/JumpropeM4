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
        Serial.println("  Channel 4 Mode 2 = Position Mode (Switch B control, Ch5 nudges zero ±0.2rad)");
        Serial.println("  Channel 4 Mode 3 = Position Mode (SwA=-0.1rad, SwB=3.9rad, Ch5=1.9±2π rad)");
        Serial.println("  Channel 4 Mode 4 = Position Mode (Fixed: M1=-4.4rad, M2=-0.6rad)");
        Serial.println("  Channel 4 Mode 5 = Position Mode (Individual motor control)");
        Serial.println("  Channel 4 Mode 6 = Emergency Stop and Clear Errors");
        Serial.println("  Switch A = Set mechanical zero (Mode 6), or Position -0.1rad (Mode 3)");
        Serial.println("  Channel 5 = Velocity control (Mode 1), Zero nudging (Mode 2), or Position 1.9±2π rad (Mode 3, no switches)");
        Serial.println("  Channel 7 = Motor 1 position ±7rad (Mode 5 only)");
        Serial.println("  Switch B = Position control: OFF=0rad, ON=3.8rad (Mode 2), or Position 3.9rad (Mode 3)");
        Serial.println("  Channel 9 = Motor 2 position ±7rad (Mode 5 only)");
        Serial.println("  Channel 8 = State tracking only (Low/Middle/High)");
        Serial.println("  Motor 1 is REVERSED by default");
        Serial.println("Mode  M1_Desired  M2_Desired  ButtonA  ButtonB");
    }
}

// ----- Additional Variables for Motor Control -----
static bool inPositionMode = false;
static bool modeInitialized = false;
static float currentTargetPosition = 0.0f;
static float nudgeableZeroPosition = 0.0f;  // Adjustable zero position (±0.2 rad max)
static unsigned long lastNudgeTime = 0;
static bool wasNudging = false;

// ----- Main Loop -----
void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastSerialPrint = 0;
    static unsigned long lastStatsLog = 0;
    static unsigned long lastMotorUpdate = 0;
    
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
        unsigned long currentTime = millis();
        if (currentTime - lastDirectQueryTime >= 100) { // Every 100ms
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
        
            lastDirectQueryTime = currentTime;
        }
    }
    
    // Check if we're receiving signal (only if RC is available)
    if (rcAvailable && !spektrumReader.isReceiving()) {
        // No signal - show error
        pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red
        pixels.show();
        
        // Update display every 200ms when no signal
        if (displayAvailable && millis() - lastUpdate >= 200) {
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("Motor Control");
            display.println("Mode: NO SIGNAL");
            display.println("M1 Des: STOP");
            display.println("M2 Des: STOP");
            display.println("Btn A: ---  B: ---");
            display.display();
            lastUpdate = millis();
        }
        
        // Print to serial every 1000ms when no signal
        if (Serial && millis() - lastSerialPrint >= 1000) {
            Serial.println("NO RC SIGNAL");
            lastSerialPrint = millis();
        }
        
        delay(50);
        return;
    }
    
    // If RC is not available, skip RC-based control but continue with other functions
    if (!rcAvailable) {
        // Update display to show RC not available
        if (displayAvailable && millis() - lastUpdate >= 1000) {
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("Motor Control");
            display.println("Mode: NO RC");
            display.println("M1 Des: STOP");
            display.println("M2 Des: STOP");
            display.println("Btn A: ---  B: ---");
            display.display();
            lastUpdate = millis();
        }
        
        if (Serial && millis() - lastSerialPrint >= 2000) {
            Serial.println("RC INPUT NOT AVAILABLE - Motors disabled for safety");
            lastSerialPrint = millis();
        }
        
        delay(100);
        return;
    }
    
    // We have signal - show green LED
    pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green
    pixels.show();
    
    // We only need specific channels for control, not display
    
    // Get decoded special channels (needed for display and motor control)
    int mode = 0;
    SpektrumSatelliteReader::SwitchStates switches = {false, false};
    SpektrumSatelliteReader::Channel8State ch8State = SpektrumSatelliteReader::CH8_LOW;
    
    if (rcAvailable) {
        mode = spektrumReader.getModeFromChannel4();  // Channel 4 mode selection
        switches = spektrumReader.getSwitchStatesFromChannel6();  // Switch states
        ch8State = spektrumReader.getChannel8State();  // Channel 8 state tracking
    }
    
            // Motor control logic (every 20ms) - only if motors are ready
        if (motorsReady && millis() - lastMotorUpdate >= 20) {
            // Get RC channel values
            int ch5Value = spektrumReader.getChannel(4);  // Channel 5 for velocity control (0-indexed as 4)
            int ch7Value = spektrumReader.getChannel(6);  // Channel 7 for Motor 1 control (0-indexed as 6)
            int ch9Value = spektrumReader.getChannel(8);  // Channel 9 for Motor 2 control (0-indexed as 8)
        
        // Motor 1 is reversed by default
        bool motor1Reversed = true;
        bool motor2Reversed = false;
        
        // Determine control mode based on Channel 4 mode
        // Mode 1 = Velocity Mode, Mode 2 = Position Mode (Switch B), Mode 3 = Position Mode (Channel 5)
        // Mode 4 = Position Mode (Fixed positions: M1=-4.4rad, M2=-0.6rad)
        // Mode 5 = Position Mode (Individual motor control - Ch7 for M1, Ch9 for M2)
        // Mode 6 = Emergency Stop and Clear Errors
        bool shouldBeInPositionMode = (mode == 2 || mode == 3 || mode == 4 || mode == 5);
        
        // Handle Mode 6 - Emergency Stop and Clear Errors
        static bool mode6Executed = false;
        static int lastMode6Time = 0;
        static bool wasInMode6 = false;
        
        if (mode == 6) {
            // Execute Mode 6 actions only once per mode entry or every 2 seconds while in mode
            if (!mode6Executed || (millis() - lastMode6Time > 2000)) {
                Logger::info("Mode 6: Clearing errors and stopping motors");
                
                // Clear all motor faults
                motor1.resetFaults();
                motor2.resetFaults();
                delay(100);
                
                // Stop motors by setting velocity to 0
                motor1.setModeVelocity();
                motor2.setModeVelocity();
                delay(100);
                motor1.setVelocity(0.0f);
                motor2.setVelocity(0.0f);
                
                mode6Executed = true;
                lastMode6Time = millis();
                Logger::info("Motors stopped and errors cleared");
            }
            wasInMode6 = true;
        } else {
            // Reset Mode 6 execution flag when exiting Mode 6
            if (wasInMode6) {
                Logger::info("Exiting Mode 6 - preparing for normal operation");
                
                // Re-enable motors and clear any remaining faults
                motor1.resetFaults();
                motor2.resetFaults();
                delay(100);
                motor1.enable();
                motor2.enable();
                delay(100);
                
                mode6Executed = false;
                modeInitialized = false;  // Force re-initialization of the new mode
                wasInMode6 = false;
                
                Logger::info("Motors re-enabled and ready for mode transition");
            }
        }
        
        // Switch modes if needed or if switching between different position modes
        static int lastMode = 0;
        if ((shouldBeInPositionMode != inPositionMode || mode != lastMode) && mode != 6) {
            inPositionMode = shouldBeInPositionMode;
            modeInitialized = false;
            if (mode == 1) {
                Logger::info("Switching to velocity mode (Mode 1)");
            } else if (mode == 2) {
                Logger::info("Switching to position mode with Switch B control (Mode 2)");
            } else if (mode == 3) {
                Logger::info("Switching to position mode with Channel 5 control (Mode 3)");
            } else if (mode == 4) {
                Logger::info("Switching to position mode with fixed positions (Mode 4)");
            } else if (mode == 5) {
                Logger::info("Switching to position mode with individual motor control (Mode 5)");
            }
        }
        
        // Update lastMode for all modes (including Mode 6) to track transitions
        if (mode != lastMode) {
            lastMode = mode;
        }
        
        // Initialize mode if needed (always control both motors) - skip for Mode 6
        if (!modeInitialized && mode != 6) {
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
        
        // Handle zero position command using Switch A (only in Mode 6)
        static bool lastSwitchAState = false;
        if (switches.switchA && !lastSwitchAState && mode == 6) {
            // Rising edge detection - switch just turned ON, and we're in Mode 6
            Logger::info("Switch A pressed in Mode 6 - Setting mechanical zero position and resetting nudgeable zero");
            motor1.setMechanicalZero();
            motor2.setMechanicalZero();
            currentTargetPosition = 0.0f;
            nudgeableZeroPosition = 0.0f;  // Reset the nudgeable zero position
        }
        lastSwitchAState = switches.switchA;
        
        // Execute motor commands based on mode (skip for Mode 6 - motors already stopped)
        if (modeInitialized && mode != 6) {
            if (inPositionMode) {
                if (mode == 2) {
                    // Mode 2: Position mode - use Switch B to control position (nudgeable zero or 3.8 radians)
                    // Use Channel 5 to nudge the zero position
                    
                    // Calculate Channel 5 percentage (0-100%)
                    float ch5Percentage = (float)(ch5Value - MIN_PULSE) / (MAX_PULSE - MIN_PULSE) * 100.0f;
                    
                    // Check if we should nudge (above 65% or below 35%)
                    bool shouldNudgeUp = (ch5Percentage > 65.0f);
                    bool shouldNudgeDown = (ch5Percentage < 35.0f);
                    bool shouldNudge = shouldNudgeUp || shouldNudgeDown;
                    
                    // Handle nudging timing
                    unsigned long currentTime = millis();
                    if (shouldNudge) {
                        if (!wasNudging) {
                            // Just started nudging - record the time
                            lastNudgeTime = currentTime;
                            wasNudging = true;
                        } else if (currentTime - lastNudgeTime >= 100) {
                            // We've been nudging for 100ms - apply the nudge
                            float nudgeAmount = 0.01f;
                            if (shouldNudgeDown) {
                                nudgeAmount = -0.01f;
                            }
                            
                            // Apply nudge with limits (±0.2 radians)
                            float newZeroPosition = nudgeableZeroPosition + nudgeAmount;
                            if (newZeroPosition >= -0.2f && newZeroPosition <= 0.2f) {
                                nudgeableZeroPosition = newZeroPosition;
                                Logger::info("Nudged zero position to " + String(nudgeableZeroPosition, 3) + " rad");
                            }
                            
                            // Reset timing for next nudge
                            lastNudgeTime = currentTime;
                        }
                    } else {
                        wasNudging = false;
                    }
                    
                    // Set target position based on Switch B, offset by nudgeable zero
                    if (switches.switchB) {
                        currentTargetPosition = 3.8f - nudgeableZeroPosition;  // Switch B ON = 3.8 radians + offset
                    } else {
                        currentTargetPosition = nudgeableZeroPosition;  // Switch B OFF = nudgeable zero position
                    }
                } else if (mode == 3) {
                    // Mode 3: Position mode - Switch A = -0.1 rad, Switch B = 3.9 rad, or Channel 5 control
                    
                    // Check for switch overrides first
                    if (switches.switchA) {
                        currentTargetPosition = -0.1f;  // Switch A = -0.1 radians
                    } else if (switches.switchB) {
                        currentTargetPosition = 3.9f;   // Switch B = 3.9 radians
                    } else {
                        // No switches pressed - use Channel 5 to control position around 1.9 rad base with ±2π range
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
                } else if (mode == 4) {
                    // Mode 4: Position mode - fixed positions (M1=-4.4rad, M2=-0.6rad)
                    currentTargetPosition = 0.0f;  // Not used in Mode 4, individual targets are used
                    
                    // Send individual position commands to motors with direction control
                    float motor1TargetPosition = motor1Reversed ? -(-4.4f) : -4.4f;  // M1 target: -4.4 rad
                    float motor2TargetPosition = motor2Reversed ? -(-0.6f) : -0.6f;   // M2 target: -0.6 rad
                    motor1.setPosition(motor1TargetPosition);
                    motor2.setPosition(motor2TargetPosition);
                } else if (mode == 5) {
                    // Mode 5: Position mode - use Channel 7 for Motor 1 and Channel 9 for Motor 2 (individual control)
                    // Map channels from pulse range to -7 to +7 radians
                    float motor1Position = 0.0f;
                    float motor2Position = 0.0f;
                    
                    // Map Channel 7 to Motor 1 position (-7 to +7 radians)
                    if (ch7Value >= (MID_PULSE - DEAD_ZONE) && ch7Value <= (MID_PULSE + DEAD_ZONE)) {
                        motor1Position = 0.0f;  // Dead zone - neutral position
                    } else if (ch7Value < MID_PULSE) {
                        motor1Position = map(ch7Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -7000, 0) * 0.001f;
                    } else {
                        motor1Position = map(ch7Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 7000) * 0.001f;
                    }
                    
                    // Map Channel 9 to Motor 2 position (-7 to +7 radians)
                    if (ch9Value >= (MID_PULSE - DEAD_ZONE) && ch9Value <= (MID_PULSE + DEAD_ZONE)) {
                        motor2Position = 0.0f;  // Dead zone - neutral position
                    } else if (ch9Value < MID_PULSE) {
                        motor2Position = map(ch9Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -7000, 0) * 0.001f;
                    } else {
                        motor2Position = map(ch9Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 7000) * 0.001f;
                    }
                    
                    // Send individual position commands to motors with direction control
                    float motor1TargetPosition = motor1Reversed ? -motor1Position : motor1Position;
                    float motor2TargetPosition = motor2Reversed ? -motor2Position : motor2Position;
                    motor1.setPosition(motor1TargetPosition);
                    motor2.setPosition(motor2TargetPosition);
                    
                    // Store positions for display (use the actual commanded positions)
                    currentTargetPosition = 0.0f;  // Not used in Mode 5, individual targets are used
                }
                
                // Send position commands to both motors with direction control (for modes 2 and 3)
                if (mode == 2 || mode == 3) {
                    float motor1TargetPosition = motor1Reversed ? -currentTargetPosition : currentTargetPosition;
                    float motor2TargetPosition = motor2Reversed ? -currentTargetPosition : currentTargetPosition;
                    motor1.setPosition(motor1TargetPosition);
                    motor2.setPosition(motor2TargetPosition);
                }
                
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
        
        lastMotorUpdate = millis();
    }
    
    // Update OLED display every 150ms (only if display is available)
    if (displayAvailable && millis() - lastUpdate >= 150) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("Motor Control");
        
        // Show current mode
        display.print("Mode: ");
        if (mode == 1) {
            display.println("VEL (1)");
        } else if (mode == 2) {
            display.println("POS (2)");
        } else if (mode == 3) {
            display.println("POS (3)");
        } else if (mode == 4) {
            display.println("FIX (4)");
        } else if (mode == 5) {
            display.println("IND (5)");
        } else if (mode == 6) {
            display.println("STOP (6)");
        } else {
            display.print("? (");
            display.print(mode);
            display.println(")");
        }
        
        // Calculate individual motor targets (accounting for direction reversal)
        float motor1Target = 0.0f;
        float motor2Target = 0.0f;
        bool motor1Reversed = true;
        bool motor2Reversed = false;
        
        if (mode == 6) {
            motor1Target = 0.0f;
            motor2Target = 0.0f;
        } else if (mode == 4) {
            // Mode 4: Fixed positions (M1=-4.4rad, M2=-0.6rad)
            motor1Target = motor1Reversed ? -(-4.4f) : -4.4f;  // M1 target: -4.4 rad
            motor2Target = motor2Reversed ? -(-0.6f) : -0.6f;   // M2 target: -0.6 rad
        } else if (mode == 5) {
            // Mode 5: Individual motor control - calculate from channel values
            int ch7Value = spektrumReader.getChannel(6);  // Channel 7 for Motor 1
            int ch9Value = spektrumReader.getChannel(8);  // Channel 9 for Motor 2
            
            // Map Channel 7 to Motor 1 position
            float motor1Position = 0.0f;
            if (ch7Value >= (MID_PULSE - DEAD_ZONE) && ch7Value <= (MID_PULSE + DEAD_ZONE)) {
                motor1Position = 0.0f;
            } else if (ch7Value < MID_PULSE) {
                motor1Position = map(ch7Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -7000, 0) * 0.001f;
            } else {
                motor1Position = map(ch7Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 7000) * 0.001f;
            }
            
            // Map Channel 9 to Motor 2 position
            float motor2Position = 0.0f;
            if (ch9Value >= (MID_PULSE - DEAD_ZONE) && ch9Value <= (MID_PULSE + DEAD_ZONE)) {
                motor2Position = 0.0f;
            } else if (ch9Value < MID_PULSE) {
                motor2Position = map(ch9Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -7000, 0) * 0.001f;
            } else {
                motor2Position = map(ch9Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 7000) * 0.001f;
            }
            
            motor1Target = motor1Reversed ? -motor1Position : motor1Position;
            motor2Target = motor2Reversed ? -motor2Position : motor2Position;
        } else if (inPositionMode) {
            motor1Target = motor1Reversed ? -currentTargetPosition : currentTargetPosition;
            motor2Target = motor2Reversed ? -currentTargetPosition : currentTargetPosition;
        } else {
            motor1Target = motor1Reversed ? -currentTargetPosition : currentTargetPosition;
            motor2Target = motor2Reversed ? -currentTargetPosition : currentTargetPosition;
        }
        
        // Show Motor 1 desired value
        display.print("M1 Des: ");
        if (mode == 6) {
            display.println("STOP");
        } else if (inPositionMode) {
            display.print(motor1Target, 1);
            display.println(" rad");
        } else {
            display.print(motor1Target, 1);
            display.println(" r/s");
        }
        
        // Show Motor 2 desired value
        display.print("M2 Des: ");
        if (mode == 6) {
            display.println("STOP");
        } else if (inPositionMode) {
            display.print(motor2Target, 1);
            display.println(" rad");
        } else {
            display.print(motor2Target, 1);
            display.println(" r/s");
        }
        
        // Show switch states
        display.print("Btn A: ");
        display.print(switches.switchA ? "ON" : "OFF");
        display.print("  B: ");
        display.print(switches.switchB ? "ON" : "OFF");
        
        // Show nudgeable zero position in Mode 2
        if (mode == 2) {
            display.print(" Z:");
            display.print(nudgeableZeroPosition, 2);
        }
        display.println();
        
        display.display();
        lastUpdate = millis();
    }
    
    // Print to serial console every 100ms (only if serial is available)
    if (Serial && millis() - lastSerialPrint >= 100) {
        // Print mode information
        Serial.print("Mode:");
        if (mode == 1) {
            Serial.print("VEL(1)");
        } else if (mode == 2) {
            Serial.print("POS(2)");
        } else if (mode == 3) {
            Serial.print("POS(3)");
        } else if (mode == 4) {
            Serial.print("FIX(4)");
        } else if (mode == 5) {
            Serial.print("IND(5)");
        } else if (mode == 6) {
            Serial.print("STOP(6)");
        } else {
            Serial.print("?(");
            Serial.print(mode);
            Serial.print(")");
        }
        Serial.print("  ");
        
        // Calculate individual motor targets (accounting for direction reversal)
        float motor1Target = 0.0f;
        float motor2Target = 0.0f;
        bool motor1Reversed = true;
        bool motor2Reversed = false;
        
        if (mode == 6) {
            motor1Target = 0.0f;
            motor2Target = 0.0f;
        } else if (mode == 4) {
            // Mode 4: Fixed positions (M1=-4.4rad, M2=-0.6rad)
            motor1Target = motor1Reversed ? -(-4.4f) : -4.4f;  // M1 target: -4.4 rad
            motor2Target = motor2Reversed ? -(-0.6f) : -0.6f;   // M2 target: -0.6 rad
        } else if (mode == 5) {
            // Mode 5: Individual motor control - calculate from channel values
            int ch7Value = spektrumReader.getChannel(6);  // Channel 7 for Motor 1
            int ch9Value = spektrumReader.getChannel(8);  // Channel 9 for Motor 2
            
            // Map Channel 7 to Motor 1 position
            float motor1Position = 0.0f;
            if (ch7Value >= (MID_PULSE - DEAD_ZONE) && ch7Value <= (MID_PULSE + DEAD_ZONE)) {
                motor1Position = 0.0f;
            } else if (ch7Value < MID_PULSE) {
                motor1Position = map(ch7Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -7000, 0) * 0.001f;
            } else {
                motor1Position = map(ch7Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 7000) * 0.001f;
            }
            
            // Map Channel 9 to Motor 2 position
            float motor2Position = 0.0f;
            if (ch9Value >= (MID_PULSE - DEAD_ZONE) && ch9Value <= (MID_PULSE + DEAD_ZONE)) {
                motor2Position = 0.0f;
            } else if (ch9Value < MID_PULSE) {
                motor2Position = map(ch9Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -7000, 0) * 0.001f;
            } else {
                motor2Position = map(ch9Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 7000) * 0.001f;
            }
            
            motor1Target = motor1Reversed ? -motor1Position : motor1Position;
            motor2Target = motor2Reversed ? -motor2Position : motor2Position;
        } else if (inPositionMode) {
            motor1Target = motor1Reversed ? -currentTargetPosition : currentTargetPosition;
            motor2Target = motor2Reversed ? -currentTargetPosition : currentTargetPosition;
        } else {
            motor1Target = motor1Reversed ? -currentTargetPosition : currentTargetPosition;
            motor2Target = motor2Reversed ? -currentTargetPosition : currentTargetPosition;
        }
        
        // Print Motor 1 desired value
        Serial.print("M1_Des:");
        if (mode == 6) {
            Serial.print("STOP");
        } else if (inPositionMode) {
            Serial.print(motor1Target, 2);
            Serial.print("rad");
        } else {
            Serial.print(motor1Target, 2);
            Serial.print("r/s");
        }
        Serial.print("  ");
        
        // Print Motor 2 desired value
        Serial.print("M2_Des:");
        if (mode == 6) {
            Serial.print("STOP");
        } else if (inPositionMode) {
            Serial.print(motor2Target, 2);
            Serial.print("rad");
        } else {
            Serial.print(motor2Target, 2);
            Serial.print("r/s");
        }
        Serial.print("  ");
        
        // Print button states
        Serial.print("BtnA:");
        Serial.print(switches.switchA ? "ON" : "OFF");
        Serial.print("  BtnB:");
        Serial.print(switches.switchB ? "ON" : "OFF");
        
        // Show nudgeable zero position in Mode 2
        if (mode == 2) {
            Serial.print("  Zero:");
            Serial.print(nudgeableZeroPosition, 3);
            Serial.print("rad");
        }
        
        Serial.println();
        lastSerialPrint = millis();
    }
    
    // Print frame statistics every 5 seconds for debugging (only if serial is available)
    if (Serial && millis() - lastStatsLog >= 5000) {
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
        lastStatsLog = millis();
    }
    
    // Small delay to prevent tight looping
    delay(10);
} 