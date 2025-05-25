#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "rs03_motor.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath> // For M_PI and fabs()
#include "CPPMReader.h" // Include CPPM reader
#include "FeatherM4CanInterface.h" // Include the new header file
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ----- Pin and Hardware Configuration -----
#define PIN            8    // NeoPixel data pin
#define NUMPIXELS      1    // Number of NeoPixels
#define CPPM_PIN       9    // Pin for CPPM signal input
#define MOTOR_ID_1     127  // First motor ID
#define MOTOR_ID_2     126  // Second motor ID
#define MASTER_ID      0xFD // Default Master ID for commands

// ----- CPPM Configuration -----
#define MIN_PULSE      1000 // 1000us = -20 rad/s
#define MID_PULSE      1500 // 1500us = dead zone
#define MAX_PULSE      2000 // 2000us = 20 rad/s
#define DEAD_ZONE      50   // ±50us dead zone around 1500us
#define CPPM_CHANNEL   0    // First channel (0-based index) for motor speed control
#define CURRENT_CHANNEL 1   // Channel 2 (zero-indexed as 1) for current limit control
#define MODE_CHANNEL   2    // Channel 3 (zero-indexed as 2) for mode control
#define POS_CHANNEL    0    // Channel 4 (zero-indexed as 3) for position control
#define SELECT_CHANNEL 4    // Channel 5 (zero-indexed as 4) for motor selection
#define ZERO_CHANNEL   5    // Channel 6 (zero-indexed as 5) for position zeroing
#define CPPM_CHANNELS  8    // Total number of CPPM channels

// ----- Mode Selection Thresholds -----
#define MODE_THRESHOLD 1800 // Above 1800us = position mode, below = velocity mode
#define ZERO_THRESHOLD 1800 // Above 1800us = set position to zero
#define SELECT_HIGH    1800 // Above 1800us = motor 1 only
#define SELECT_LOW     1300 // Below 1300us = motor 2 only
                            // Between 1300-1800 = both motors, motor 2 reversed

// ----- Motor Configuration -----
#define MOTOR_CURRENT_LIMIT 40.0f   // Current limit in amperes
#define MOTOR_ACCELERATION  200.0f  // Acceleration limit in rad/s² for velocity mode
#define POSITION_SPEED_LIMIT 25.0f  // Speed limit for position mode in rad/s
#define POSITION_ACCELERATION 200.0f // Acceleration limit in rad/s² for position mode
#define MAX_VELOCITY        25.0f   // Maximum motor velocity in rad/s
#define MIN_POSITION        -6.28f   // Minimum position in radians
#define MAX_POSITION        6.28f    // Maximum position in radians

// ----- Timing Configuration -----
#define MOTOR_UPDATE_RATE_MS 20     // 20ms = 50 updates per second (increased from 50ms)
#define DEBUG_PRINT_INTERVAL 500    // Print debug info every 500ms
#define FEEDBACK_INTERVAL    200    // Fetch and report motor feedback every 200ms (decreased from 1000ms)

// ----- Debug Levels -----
#define LOG_ERROR   1
#define LOG_INFO    2
#define LOG_DEBUG   3
#define LOG_VERBOSE 4
#define LOG_WARNING 1  // Same level as LOG_ERROR since it's important but not fatal

// Set current debug level
const int DEBUG_LEVEL = LOG_INFO;

// ----- OLED Display Configuration -----
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET     -1    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // I2C address of the OLED display (typically 0x3C or 0x3D)
#define DISPLAY_UPDATE_RATE_MS 150  // Update display every 150ms

// ----- Global Objects -----
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
CANSAME5x CAN;
CPPMReader cppmReader(CPPM_PIN, 8, MIN_PULSE, MAX_PULSE, MID_PULSE);
unsigned long lastMotorUpdateTime = 0;
unsigned long lastFeedbackTime = 0;
unsigned long lastDisplayUpdateTime = 0;
bool motorsInitialized = false;
bool inPositionMode = false;
bool modeInitialized = false;
float currentPosition = 0.0f;

// Added variables for zero position management
unsigned long lastZeroSetTime = 0;
bool holdingZeroPosition = false;
unsigned long zeroHoldStartTime = 0;

// Define a structure to hold raw feedback data for each motor
struct MotorRawFeedback {
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    float temperature = 0.0f;
    float current = 0.0f;
    float currentLimit = 0.0f;
    bool received = false;
    unsigned long timestamp = 0;
};

// Global variables to store raw feedback for each motor
MotorRawFeedback motor1_feedback;
MotorRawFeedback motor2_feedback;

// Track current RC-controlled current limit
float rcCurrentLimit = MOTOR_CURRENT_LIMIT;

// Motor selection enum for clarity
enum MotorSelection {
    MOTOR_1_ONLY,     // Motor 1 only (ID 127)
    MOTOR_2_ONLY,     // Motor 2 only (ID 126)
    BOTH_MOTORS       // Both motors, with motor 2 reversed
};

// Current motor selection state
MotorSelection currentMotorSelection = MOTOR_1_ONLY;

// ----- Function Prototypes -----
void setAllPixelsColor(int red, int green, int blue);
void log(int level, const String& message);
float mapPulseToVelocity(int pulseWidth);
float mapPulseToPosition(int pulseWidth);
void initializePositionMode(RS03Motor& motor);
void initializeVelocityMode(RS03Motor& motor);
void fetchAndReportMotorFeedback(RS03Motor& motor, const String& motorLabel);
bool queryParameter(uint16_t index, float &value, uint8_t motor_id);
void processCanMessages();

// Function to update the OLED display with motor feedback
void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    
    // Display mode and motor selection at the top
    display.println(inPositionMode ? "MODE: POSITION" : "MODE: VELOCITY");
    
    String motorSelectionText;
    switch (currentMotorSelection) {
        case MOTOR_1_ONLY: motorSelectionText = "MOTOR: M1 ONLY"; break;
        case MOTOR_2_ONLY: motorSelectionText = "MOTOR: M2 ONLY"; break;
        case BOTH_MOTORS: motorSelectionText = "MOTORS: M1 & M2"; break;
    }
    display.println(motorSelectionText);
    
    // Draw a line separator below the header
    display.drawLine(0, 16, 127, 16, SSD1306_WHITE);
    
    // Setup column headers
    display.setCursor(0, 18);
    display.print("MOTOR 1");
    display.setCursor(75, 18);
    display.print("MOTOR 2");
    
    // Draw a vertical separator between columns
    display.drawLine(64, 16, 64, 63, SSD1306_WHITE);
    
    // Set up data labels
    display.setCursor(0, 28);
    display.print("Pos:");
    display.setCursor(75, 28);
    display.print("Pos:");
    
    display.setCursor(0, 38);
    display.print("Vel:");
    display.setCursor(75, 38);
    display.print("Vel:");
    
    display.setCursor(0, 48);
    display.print("Cur:");
    display.setCursor(75, 48);
    display.print("Cur:");
    
    // Display Motor 1 data values (right-aligned in left column)
    if (motor1_feedback.received) {
        // Position
        String posStr = String(motor1_feedback.position, 2);
        int posWidth = posStr.length() * 6; // approx 6 pixels per character
        display.setCursor(59 - posWidth, 28);
        display.print(posStr);
        
        // Velocity
        String velStr = String(motor1_feedback.velocity, 2);
        int velWidth = velStr.length() * 6;
        display.setCursor(59 - velWidth, 38);
        display.print(velStr);
        
        // Current
        String curStr = String(motor1_feedback.current, 2);
        int curWidth = curStr.length() * 6;
        display.setCursor(59 - curWidth, 48);
        display.print(curStr);
    } else {
        display.setCursor(30, 38);
        display.print("NO DATA");
    }
    
    // Display Motor 2 data values (right-aligned in right column)
    if (motor2_feedback.received) {
        // Position
        String posStr = String(motor2_feedback.position, 2);
        int posWidth = posStr.length() * 6;
        display.setCursor(124 - posWidth, 28);
        display.print(posStr);
        
        // Velocity
        String velStr = String(motor2_feedback.velocity, 2);
        int velWidth = velStr.length() * 6;
        display.setCursor(124 - velWidth, 38);
        display.print(velStr);
        
        // Current
        String curStr = String(motor2_feedback.current, 2);
        int curWidth = curStr.length() * 6;
        display.setCursor(124 - curWidth, 48);
        display.print(curStr);
    } else {
        display.setCursor(95, 38);
        display.print("NO DATA");
    }
    
    display.display();
}

// ----- Concrete CAN Interface for Feather M4 CAN -----
// Class has been moved to FeatherM4CanInterface.h/cpp

FeatherM4CanInterface canBus;
RS03Motor motor1(canBus, MOTOR_ID_1, MASTER_ID);
RS03Motor motor2(canBus, MOTOR_ID_2, MASTER_ID);

// ----- Utility Functions -----

// Structured logging function
void log(int level, const String& message) {
    if (level <= DEBUG_LEVEL) {
        String prefix;
        switch (level) {
            case LOG_ERROR:   prefix = "[ERROR] "; break;
            case LOG_INFO:    prefix = "[INFO] "; break;
            case LOG_DEBUG:   prefix = "[DEBUG] "; break;
            case LOG_VERBOSE: prefix = "[VERBOSE] "; break;
            default:          prefix = "[LOG] "; break;
        }
        Serial.println(prefix + message);
        if (level == LOG_ERROR) {
            Serial.flush(); // Only flush for errors
        }
    }
}

// Map RC pulse width to motor velocity
float mapPulseToVelocity(int pulseWidth) {
    // Dead zone around center (1500us)
    if (pulseWidth >= (MID_PULSE - DEAD_ZONE) && pulseWidth <= (MID_PULSE + DEAD_ZONE)) {
        return 0.0f;
    }
    
    // Map pulse width to velocity
    if (pulseWidth < MID_PULSE) {
        // Map 1000-1450us to -MAX_VELOCITY-0 rad/s
        return map(pulseWidth, MIN_PULSE, MID_PULSE - DEAD_ZONE, -MAX_VELOCITY, 0) * 1.0f;
    } else {
        // Map 1550-2000us to 0-MAX_VELOCITY rad/s
        return map(pulseWidth, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, MAX_VELOCITY) * 1.0f;
    }
}

// Map RC pulse width to current limit
float mapPulseToCurrentLimit(int pulseWidth) {
    // Map pulse width from 1000-2000us to 0-MOTOR_CURRENT_LIMIT
    // Linear mapping with minimum 0.0A as a safety measure
    const float MIN_CURRENT = 0.0f; // Minimum current limit even at 1000us
    
    if (pulseWidth <= MIN_PULSE) {
        return MIN_CURRENT;
    } else if (pulseWidth >= MAX_PULSE) {
        return MOTOR_CURRENT_LIMIT;
    } else {
        return MIN_CURRENT + (MOTOR_CURRENT_LIMIT - MIN_CURRENT) * 
               (pulseWidth - MIN_PULSE) / (MAX_PULSE - MIN_PULSE);
    }
}

// Map RC pulse width to motor position
float mapPulseToPosition(int pulseWidth) {
    // Linear mapping from pulse width to position
    return map(pulseWidth, MIN_PULSE, MAX_PULSE, MIN_POSITION * 1000, MAX_POSITION * 1000) * 0.001f;
}

// Set all pixels to the same color
void setAllPixelsColor(int red, int green, int blue) {
    pixels.setPixelColor(0, pixels.Color(red, green, blue));
    pixels.show();
}

// Initialize position mode with proper parameters for specified motor
void initializePositionMode(RS03Motor& motor) {
    log(LOG_INFO, "Initializing position mode (PP) with proper limits for motor ID " + String(motor.getMotorId()));
    
    // Set the motor to Profile Position (PP) mode with speed and acceleration limits
    if (!motor.setModePositionPP(POSITION_SPEED_LIMIT, POSITION_ACCELERATION, MOTOR_CURRENT_LIMIT)) {
        log(LOG_ERROR, "Failed to set position mode (PP) for motor ID " + String(motor.getMotorId()) + "!");
        return;
    }
    
    log(LOG_INFO, "Position mode (PP) set for motor ID " + String(motor.getMotorId()) + 
                  " with speed limit: " + String(POSITION_SPEED_LIMIT) + 
                  " rad/s, acceleration: " + String(POSITION_ACCELERATION) + 
                  " rad/s², current limit: " + String(MOTOR_CURRENT_LIMIT) + " A");
}

// Initialize velocity mode with proper parameters for specified motor
void initializeVelocityMode(RS03Motor& motor) {
    log(LOG_INFO, "Initializing velocity mode for motor ID " + String(motor.getMotorId()));
    
    // Set the motor to velocity mode
    if (!motor.setModeVelocity()) {
        log(LOG_ERROR, "Failed to set velocity mode for motor ID " + String(motor.getMotorId()) + "!");
        return;
    }

    // Set current limit and acceleration for velocity mode
    bool success = true;
    if (!motor.setParameterFloat(INDEX_LIMIT_CUR, MOTOR_CURRENT_LIMIT)) {
        log(LOG_ERROR, "Failed to set current limit for velocity mode for motor ID " + String(motor.getMotorId()) + "!");
        success = false;
    }
    
    if (!motor.setParameterFloat(0x7022, MOTOR_ACCELERATION)) {
        log(LOG_ERROR, "Failed to set acceleration limit for velocity mode for motor ID " + String(motor.getMotorId()) + "!");
        success = false;
    }
    
    if (success) {
        log(LOG_INFO, "Velocity mode set for motor ID " + String(motor.getMotorId()) + 
                      " with current limit: " + String(MOTOR_CURRENT_LIMIT) + 
                      " A, acceleration: " + String(MOTOR_ACCELERATION) + " rad/s²");
    }
}

// Helper function to initialize motors
void initializeMotor(RS03Motor& motor) {
    log(LOG_INFO, "Initializing motor with ID: " + String(motor.getMotorId()));
    
    // Reset any faults
    log(LOG_INFO, "Resetting motor faults for ID " + String(motor.getMotorId()) + "...");
    if (!motor.resetFaults()) {
        log(LOG_ERROR, "Failed to reset motor faults for ID " + String(motor.getMotorId()) + "!");
    } else {
        log(LOG_INFO, "Motor faults reset for ID " + String(motor.getMotorId()));
    }
    delay(100);
    
    // Enable the motor
    log(LOG_INFO, "Enabling motor ID " + String(motor.getMotorId()) + "...");
    if (!motor.enable()) {
        log(LOG_ERROR, "Failed to enable motor ID " + String(motor.getMotorId()) + "!");
    } else {
        log(LOG_INFO, "Motor ID " + String(motor.getMotorId()) + " enabled");
    }
    delay(100);
    
    // Enable active reporting from the motor
    log(LOG_INFO, "Enabling active reporting from motor ID " + String(motor.getMotorId()) + "...");
    
    bool reportingEnabled = motor.setActiveReporting(true);
    
    if (!reportingEnabled) {
        log(LOG_WARNING, "Library method failed, trying direct approach for active reporting for motor ID " + String(motor.getMotorId()));
        
        // Type 5 message for active reporting command
        uint32_t reportingId = ((static_cast<uint32_t>(0x05) << 24) | 
                              (static_cast<uint32_t>(MASTER_ID) << 8) | 
                              static_cast<uint32_t>(motor.getMotorId())) & 0x1FFFFFFF;
        
        uint8_t reportingData[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x01 = enable reporting
        
        if (CAN.beginExtendedPacket(reportingId)) {
            CAN.write(reportingData, 8);
            if (CAN.endPacket()) {
                log(LOG_INFO, "Direct active reporting command sent for motor ID " + String(motor.getMotorId()));
                reportingEnabled = true;
            } else {
                log(LOG_ERROR, "Failed to send direct active reporting command for motor ID " + String(motor.getMotorId()));
            }
        } else {
            log(LOG_ERROR, "Failed to begin direct active reporting packet for motor ID " + String(motor.getMotorId()));
        }
    }
    
    if (reportingEnabled) {
        log(LOG_INFO, "Active reporting enabled for motor ID " + String(motor.getMotorId()));
    } else {
        log(LOG_ERROR, "Failed to enable active reporting for motor ID " + String(motor.getMotorId()));
    }
}

// Initialize both motors in the same control mode
void initializeMotorsForMode(bool positionMode) {
    if (positionMode) {
        log(LOG_INFO, "Initializing all motors in position mode");
        initializePositionMode(motor1);
        initializePositionMode(motor2);
    } else {
        log(LOG_INFO, "Initializing all motors in velocity mode");
        initializeVelocityMode(motor1);
        initializeVelocityMode(motor2);
    }
    modeInitialized = true;
}

// Fetch and report motor feedback from the specified motor
void fetchAndReportMotorFeedback(RS03Motor& motor, const String& motorLabel) {
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    float temperature = 0.0f;
    float vbus = 0.0f;
    float current = 0.0f;
    float mode_float = 0.0f;
    int mode = 0;
    bool success = false;
    
    // Parameter indices
    const uint16_t INDEX_POSITION = 0x7000;
    const uint16_t INDEX_VELOCITY = 0x7001;
    const uint16_t INDEX_TORQUE = 0x7002;
    const uint16_t INDEX_TEMPERATURE = 0x7007;
    const uint16_t INDEX_VBUS = 0x701C;
    const uint16_t INDEX_MODE = 0x7005;
    const uint16_t INDEX_CURRENT = 0x701A;
    
    // Get the motor's ID
    uint8_t motorId = motor.getMotorId();
    
    log(LOG_INFO, "Querying feedback from " + motorLabel + " (ID " + String(motorId) + ")");
    
    // Check if we have recent raw feedback from Type 2 messages for this motor
    bool useRawFeedback = false;
    unsigned long currentTime = millis();
    
    // Get reference to the appropriate feedback structure
    MotorRawFeedback* rawFeedback = nullptr;
    if (motorId == MOTOR_ID_1) {
        rawFeedback = &motor1_feedback;
    } else if (motorId == MOTOR_ID_2) {
        rawFeedback = &motor2_feedback;
    }
    
    // Check if we have recent raw feedback (less than 500ms old)
    if (rawFeedback && rawFeedback->received && (currentTime - rawFeedback->timestamp < 500)) {
        position = rawFeedback->position;
        velocity = rawFeedback->velocity;
        torque = rawFeedback->torque;
        temperature = rawFeedback->temperature;
        useRawFeedback = true;
        success = true;
        log(LOG_DEBUG, "Using raw feedback data from Type 2 messages for " + motorLabel);
    }
    
    // Even if we have raw feedback, we still need to query these parameters
    // as they're not included in Type 2 messages
    
    // Parameter index: 0x701C = VBUS (always query this)
    if (queryParameter(INDEX_VBUS, vbus, motorId)) {
        success = true;
        log(LOG_DEBUG, "Direct VBUS query success for " + motorLabel + ": " + String(vbus));
    }
    
    // Parameter index: 0x7005 = Mode (always query this)
    if (queryParameter(INDEX_MODE, mode_float, motorId)) {
        mode = (int)mode_float;
        success = true;
        log(LOG_DEBUG, "Direct mode query success for " + motorLabel + ": " + String(mode));
    }
    
    // Parameter index: 0x701A = Current (IQF) (always query this)
    if (queryParameter(INDEX_CURRENT, current, motorId)) {
        success = true;
        log(LOG_DEBUG, "Direct current query success for " + motorLabel + ": " + String(current));
        
        // Store in feedback structure
        if (rawFeedback) {
            rawFeedback->current = current;
        }
    }
    
    // Query current limit (index 0x7022)
    float currentLimit = 0.0f;
    if (queryParameter(0x7022, currentLimit, motorId)) {
        log(LOG_DEBUG, "Direct current limit query success for " + motorLabel + ": " + String(currentLimit));
        
        // Store in feedback structure
        if (rawFeedback) {
            rawFeedback->currentLimit = currentLimit;
        }
    }
    
    // If we don't have raw feedback, query all remaining parameters directly
    if (!useRawFeedback) {
        // Position
        if (queryParameter(INDEX_POSITION, position, motorId)) {
            success = true;
            log(LOG_DEBUG, "Direct position query success for " + motorLabel + ": " + String(position));
        }
        
        // Velocity
        if (queryParameter(INDEX_VELOCITY, velocity, motorId)) {
            success = true;
            log(LOG_DEBUG, "Direct velocity query success for " + motorLabel + ": " + String(velocity));
        }
        
        // Torque
        if (queryParameter(INDEX_TORQUE, torque, motorId)) {
            success = true;
            log(LOG_DEBUG, "Direct torque query success for " + motorLabel + ": " + String(torque));
        }
        
        // Temperature (if not already obtained from raw feedback)
        if (!useRawFeedback) {
            if (queryParameter(INDEX_TEMPERATURE, temperature, motorId)) {
                success = true;
                log(LOG_DEBUG, "Direct temperature query success for " + motorLabel + ": " + String(temperature));
            }
        }
    }
    
    // If direct querying failed, use last feedback from the motor
    if (!success) {
        log(LOG_WARNING, "Failed to query parameters directly for " + motorLabel + ", using last feedback");
        RS03Motor::Feedback feedback = motor.getLastFeedback();
        position = feedback.position;
        velocity = feedback.velocity;
        torque = feedback.torque;
        temperature = feedback.temperature;
        mode = feedback.mode;
    }
    
    // Report data
    log(LOG_INFO, "-------- " + motorLabel + " FEEDBACK --------");
    log(LOG_INFO, "Position: " + String(position) + " rad");
    log(LOG_INFO, "Velocity: " + String(velocity) + " rad/s");
    log(LOG_INFO, "Torque:   " + String(torque) + " Nm");
    log(LOG_INFO, "Current:  " + String(current) + " A");
    log(LOG_INFO, "Temp:     " + String(temperature) + " °C");
    log(LOG_INFO, "Mode:     " + String(mode));
    
    if (vbus > 0.0f) {
        log(LOG_INFO, "VBUS:     " + String(vbus) + " V");
    }
    
    // Report errors if any
    if (motor.hasErrors()) {
        uint16_t error_flags = motor.getLastFeedback().error_flags;
        log(LOG_ERROR, motorLabel + " Error flags: 0x" + String(error_flags, HEX));
        log(LOG_ERROR, motorLabel + " Error description: " + String(motor.getErrorText().c_str()));
    }
    
    log(LOG_INFO, "--------------------------------");
}

// Helper function to query a parameter from the motor
bool queryParameter(uint16_t index, float &value, uint8_t motor_id) {
    // Create a Type 17 frame to read the parameter
    uint32_t readId = ((0x11) << 24) | (MASTER_ID << 8) | motor_id;
    uint8_t readData[8] = {0};
    // Parameter index in little-endian
    readData[0] = index & 0xFF;        // Low byte of index
    readData[1] = (index >> 8) & 0xFF; // High byte of index
    
    if (DEBUG_LEVEL >= LOG_DEBUG) {
        log(LOG_DEBUG, "Querying parameter 0x" + String(index, HEX) + " from motor ID " + String(motor_id) + " - ID: 0x" + String(readId, HEX));
    }
    
    // Send the request
    if (!CAN.beginExtendedPacket(readId & 0x1FFFFFFF)) {
        log(LOG_ERROR, "Failed to begin extended packet for motor ID " + String(motor_id));
        return false;
    }
    
    CAN.write(readData, 8);
    if (!CAN.endPacket()) {
        log(LOG_ERROR, "Failed to send packet for motor ID " + String(motor_id));
        return false;
    }
    
    // Wait for response
    unsigned long startTime = millis();
    while (millis() - startTime < 100) { // 100ms timeout
        int packetSize = CAN.parsePacket();
        if (packetSize) {
            uint32_t id = CAN.packetId();
            uint8_t responseType = (id >> 24) & 0x1F;
            uint8_t responseMotorId = (id >> 8) & 0xFF; // Extract motor ID from the response
            
            // Read the data payload
            uint8_t data[8] = {0};
            for (int i = 0; i < packetSize && i < 8; i++) {
                data[i] = CAN.read();
            }
            
            // Check for the Type 0x11 response from the specific motor
            if (responseType == 0x11 && responseMotorId == motor_id) {
                uint16_t responseIndex = data[0] | (data[1] << 8);
                if (responseIndex == index) {
                    // The value is stored as a float in the last 4 bytes
                    memcpy(&value, &data[4], sizeof(float));
                    
                    if (DEBUG_LEVEL >= LOG_DEBUG) {
                        log(LOG_DEBUG, "Got response for param 0x" + String(index, HEX) + " from motor ID " + 
                            String(motor_id) + ": " + String(value));
                    }
                    
                    return true;
                }
            }
        }
        delay(1);
    }
    
    if (DEBUG_LEVEL >= LOG_DEBUG) {
        log(LOG_DEBUG, "Timeout waiting for parameter 0x" + String(index, HEX) + 
                  " from motor ID " + String(motor_id));
    }
    
    return false;
}

// Process incoming CAN messages
void processCanMessages() {
    // Check for any available CAN messages
    int packetSize = CAN.parsePacket();
    if (packetSize) {
        // Create a CAN frame to hold the data
        CanFrame frame;
        frame.id = CAN.packetId();
        frame.is_extended = CAN.packetExtended();
        frame.dlc = packetSize;
        
        // Read data bytes
        int i = 0;
        while (CAN.available() && i < 8) {
            frame.data[i++] = CAN.read();
        }
        
        // Check specifically for Type 2 (feedback) messages
        uint8_t msgType = (frame.id >> 24) & 0x1F;
        if (msgType == 0x02 && frame.dlc == 8) {
            // Extract source motor ID (for Type 2 messages it's in the upper byte of data field 2)
            uint8_t sourceMotorId = (frame.id >> 8) & 0xFF;
            
            // Extract Type 2 feedback data according to format:
            // Bytes 0-1: Current position (16-bit)
            // Bytes 2-3: Current velocity (16-bit)
            // Bytes 4-5: Current torque (16-bit)
            // Bytes 6-7: Temperature value (16-bit)
            
            uint16_t pos_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
            uint16_t vel_raw = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
            uint16_t torque_raw = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
            uint16_t temp_raw = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];
            
            // Apply conversions using RS03Motor constants
            float position = motor1.uintToFloat(pos_raw, P_MIN, P_MAX, 16);
            float velocity = motor1.uintToFloat(vel_raw, V_MIN, V_MAX, 16);
            float torque = motor1.uintToFloat(torque_raw, T_MIN, T_MAX, 16);
            float temperature = static_cast<float>(temp_raw) / 10.0f;
            
            // Store in the appropriate motor's feedback structure
            if (sourceMotorId == MOTOR_ID_1) {
                motor1_feedback.position = position;
                motor1_feedback.velocity = velocity;
                motor1_feedback.torque = torque;
                motor1_feedback.temperature = temperature;
                motor1_feedback.received = true;
                motor1_feedback.timestamp = millis();
                
                if (DEBUG_LEVEL >= LOG_VERBOSE) {
                    log(LOG_VERBOSE, "Type 2 feedback from Motor 1 (ID " + String(sourceMotorId) + 
                                     "): Pos=" + String(position, 4) +
                                     " Vel=" + String(velocity, 4) +
                                     " Torq=" + String(torque, 4) +
                                     " Temp=" + String(temperature, 1));
                }
            } 
            else if (sourceMotorId == MOTOR_ID_2) {
                motor2_feedback.position = position;
                motor2_feedback.velocity = velocity;
                motor2_feedback.torque = torque;
                motor2_feedback.temperature = temperature;
                motor2_feedback.received = true;
                motor2_feedback.timestamp = millis();
                
                if (DEBUG_LEVEL >= LOG_VERBOSE) {
                    log(LOG_VERBOSE, "Type 2 feedback from Motor 2 (ID " + String(sourceMotorId) + 
                                     "): Pos=" + String(position, 4) +
                                     " Vel=" + String(velocity, 4) +
                                     " Torq=" + String(torque, 4) +
                                     " Temp=" + String(temperature, 1));
                }
            }
            else {
                if (DEBUG_LEVEL >= LOG_VERBOSE) {
                    log(LOG_VERBOSE, "Type 2 feedback from unknown Motor ID " + String(sourceMotorId));
                }
            }
        }
        
        // Process the frame with both motors
        bool processed1 = motor1.processFeedback(frame);
        bool processed2 = motor2.processFeedback(frame);
        
        // Log frame details for debugging
        if (DEBUG_LEVEL >= LOG_VERBOSE) {
            String frameDetails = "CAN frame: ID=0x" + String(frame.id, HEX) + 
                                  ", Type=0x" + String((frame.id >> 24) & 0x1F, HEX) +
                                  ", DLC=" + String(frame.dlc) + ", Data=[";
            
            for (int j = 0; j < frame.dlc; j++) {
                if (frame.data[j] < 0x10) frameDetails += "0"; // Pad with leading zero
                frameDetails += String(frame.data[j], HEX);
                if (j < frame.dlc - 1) frameDetails += " ";
            }
            frameDetails += "]";
            
            log(LOG_VERBOSE, frameDetails);
        }
        
        if (processed1 || processed2) {
            log(LOG_VERBOSE, "Processed motor feedback frame");
        }
    }
}

// ----- Setup and Main Loop -----

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
    log(LOG_INFO, "RS03 Dual Motor Control with RC Input - Starting");
    
    // Initialize the OLED display
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        log(LOG_ERROR, "SSD1306 OLED display initialization failed");
    } else {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("RS03 Dual Motor");
        display.println("Control System");
        display.println("");
        display.println("Initializing...");
        display.display();
        log(LOG_INFO, "SSD1306 OLED display initialized");
    }
    
    // Initialize the NeoPixel
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    setAllPixelsColor(50, 0, 50); // Purple while initializing
    log(LOG_INFO, "NeoPixel initialized");
    
    // Initialize CAN bus
    pinMode(PIN_CAN_STANDBY, OUTPUT);
    digitalWrite(PIN_CAN_STANDBY, false);
    pinMode(PIN_CAN_BOOSTEN, OUTPUT);
    digitalWrite(PIN_CAN_BOOSTEN, true);
    
    if (!CAN.begin(1000000)) {
        log(LOG_ERROR, "Starting CAN failed!");
        setAllPixelsColor(50, 0, 0);  // Red indicates error
        while(1);
    }
    log(LOG_INFO, "CAN initialized at 1 Mbit/s");

    // Initialize CPPM reader
    if (cppmReader.begin()) {
        log(LOG_INFO, "CPPM Reader initialized on pin " + String(CPPM_PIN));
    } else {
        log(LOG_ERROR, "Failed to initialize CPPM Reader!");
        setAllPixelsColor(50, 0, 50);
    }

    // Initialize motors with their IDs
    log(LOG_INFO, "Initializing motors with IDs: " + String(MOTOR_ID_1) + " and " + String(MOTOR_ID_2));
    
    // Initialize both motors
    initializeMotor(motor1);
    initializeMotor(motor2);
    
    // Set the motors to velocity mode initially
    initializeMotorsForMode(false);  // false = velocity mode
    inPositionMode = false;
    
    motorsInitialized = true;
    setAllPixelsColor(0, 50, 0); // Green indicates ready
    log(LOG_INFO, "Setup complete. Ready for CPPM control.");
    log(LOG_INFO, "Channel 1 (index 0): 1000us=-20rad/s, 1500us=stop, 2000us=20rad/s");
    log(LOG_INFO, "Channel 2 (index 1): 1000us=1A, 2000us=" + String(MOTOR_CURRENT_LIMIT) + "A current limit (in velocity mode)");
    log(LOG_INFO, "Channel 3 (index 2): >1800us=position mode, <1800us=velocity mode");
    log(LOG_INFO, "Channel 4 (index 3): 1000us=-4rad, 2000us=+4rad (in position mode)");
    log(LOG_INFO, "Channel 5 (index 4): >1800us=motor 1, <1300us=motor 2, 1300-1800us=both (motor 2 reversed)");
    log(LOG_INFO, "Channel 6 (index 5): >1800us=set current position as zero");
    
    // Update display with ready status
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("System Ready!");
    display.println("");
    display.println("Waiting for");
    display.println("CPPM control input");
    display.display();
    
    delay(50);
}

void loop() {
    static bool firstLoop = true;
    if (firstLoop) {
        log(LOG_INFO, "Main loop started");
        firstLoop = false;
    }
    
    // Update CPPM reader and check if signal is still valid
    cppmReader.update();

    // Process any incoming CAN messages to update feedback
    // Call multiple times per loop to be more aggressive about capturing feedback
    for (int i = 0; i < 5; i++) {
        processCanMessages();
        // Small delay between processing attempts
        delayMicroseconds(100);
    }

    // Periodically request motor state directly using Type 0 query for both motors
    static unsigned long lastDirectQueryTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastDirectQueryTime >= 100) { // Every 100ms
        // Send a Type 0 query to request state from both motors based on current selection
        if (currentMotorSelection == MOTOR_1_ONLY || currentMotorSelection == BOTH_MOTORS) {
            uint32_t queryId1 = ((0x00) << 24) | (MASTER_ID << 16) | MOTOR_ID_1;
            if (CAN.beginExtendedPacket(queryId1 & 0x1FFFFFFF)) {
                // Type 0 has no data
                if (CAN.endPacket()) {
                    if (DEBUG_LEVEL >= LOG_VERBOSE) {
                        log(LOG_VERBOSE, "Sent Type 0 query to request motor 1 state");
                    }
                }
            }
        }
        
        if (currentMotorSelection == MOTOR_2_ONLY || currentMotorSelection == BOTH_MOTORS) {
            uint32_t queryId2 = ((0x00) << 24) | (MASTER_ID << 16) | MOTOR_ID_2;
            if (CAN.beginExtendedPacket(queryId2 & 0x1FFFFFFF)) {
                // Type 0 has no data
                if (CAN.endPacket()) {
                    if (DEBUG_LEVEL >= LOG_VERBOSE) {
                        log(LOG_VERBOSE, "Sent Type 0 query to request motor 2 state");
                    }
                }
            }
        }
        
        lastDirectQueryTime = currentTime;
    }

    static unsigned long lastChannelPrint = 0;
    
    // Print all channel values once per second
    if (currentTime - lastChannelPrint >= 1000) {
        String channelValues = "CPPM Channels: ";
        for (int i = 0; i < CPPM_CHANNELS; i++) {
            channelValues += String(i + 1) + "=" + String(cppmReader.getChannel(i)) + "us ";
        }
        log(LOG_DEBUG, channelValues);
        lastChannelPrint = currentTime;
    }

    // Handle CPPM control of the motors
    if (motorsInitialized) {
        static unsigned long lastDebugPrint = 0;
        unsigned long currentTime = millis();
        
        // Check which motor(s) to control based on SELECT_CHANNEL (Channel 5)
        int selectChannelValue = cppmReader.getChannel(SELECT_CHANNEL);
        MotorSelection newMotorSelection;
        
        if (selectChannelValue > SELECT_HIGH) {
            newMotorSelection = MOTOR_1_ONLY;
        } else if (selectChannelValue < SELECT_LOW) {
            newMotorSelection = MOTOR_2_ONLY;
        } else {
            newMotorSelection = BOTH_MOTORS;
        }
        
        // Log if the motor selection changed
        if (newMotorSelection != currentMotorSelection) {
            String selectionText;
            switch (newMotorSelection) {
                case MOTOR_1_ONLY: selectionText = "Motor 1 only (ID " + String(MOTOR_ID_1) + ")"; break;
                case MOTOR_2_ONLY: selectionText = "Motor 2 only (ID " + String(MOTOR_ID_2) + ")"; break;
                case BOTH_MOTORS: selectionText = "Both motors (Motor 2 reversed)"; break;
            }
            log(LOG_INFO, "Motor selection changed: " + selectionText);
            currentMotorSelection = newMotorSelection;
        }
        
        // Check if we need to switch control modes
        int modeChannelValue = cppmReader.getChannel(MODE_CHANNEL);
        bool shouldBeInPositionMode = modeChannelValue > MODE_THRESHOLD;
        
        // Only switch modes if the mode has changed
        if (shouldBeInPositionMode != inPositionMode) {
            inPositionMode = shouldBeInPositionMode;
            modeInitialized = false; // Need to re-initialize the mode
            
            // Don't actually change modes yet, wait for the next loop iteration
            log(LOG_INFO, inPositionMode ? "Will switch to position mode" : "Will switch to velocity mode");
        }
        
        // Initialize the appropriate mode if needed
        if (!modeInitialized) {
            initializeMotorsForMode(inPositionMode);
        }
        
        // Check if we need to zero the position
        int zeroChannelValue = cppmReader.getChannel(ZERO_CHANNEL);
        
        // Check if we're in a zero hold period and should exit
        if (holdingZeroPosition) {
            // Exit zero hold if either:
            // 1. More than 1 second has passed since we started holding zero, or
            // 2. The desired target position from RC is within ±0.1 rad
            int positionPulseWidth = cppmReader.getChannel(POS_CHANNEL);
            float desiredPosition = mapPulseToPosition(positionPulseWidth);
            
            if ((currentTime - zeroHoldStartTime > 1000) || (fabs(desiredPosition) < 0.1f)) {
                log(LOG_INFO, "Exiting zero hold position mode");
                holdingZeroPosition = false;
            } else {
                // While holding zero, force the target position to remain at 0
                currentPosition = 0.0f;
                
                // Set the position to zero for the active motor(s)
                if (currentMotorSelection == MOTOR_1_ONLY || currentMotorSelection == BOTH_MOTORS) {
                    if (!motor1.setPosition(0.0f)) {
                        log(LOG_ERROR, "Failed to hold zero position for motor 1!");
                    }
                }
                
                if (currentMotorSelection == MOTOR_2_ONLY || currentMotorSelection == BOTH_MOTORS) {
                    if (!motor2.setPosition(0.0f)) {
                        log(LOG_ERROR, "Failed to hold zero position for motor 2!");
                    }
                }
            }
        }
        
        // Only allow zeroing if not in hold mode and enough time has passed since last zero
        if (zeroChannelValue > ZERO_THRESHOLD && 
            !holdingZeroPosition && (currentTime - lastZeroSetTime >= 500)) {
            
            if (inPositionMode) {
                log(LOG_INFO, "Setting current position as zero in position mode");
            } else {
                log(LOG_INFO, "Setting current position as zero in velocity mode");
            }
            
            // Set mechanical zero based on current motor selection
            bool zeroSuccess = true;
            
            if (currentMotorSelection == MOTOR_1_ONLY || currentMotorSelection == BOTH_MOTORS) {
                if (!motor1.setMechanicalZero()) {
                    log(LOG_ERROR, "Failed to set mechanical zero for motor 1!");
                    zeroSuccess = false;
                }
            }
            
            if (currentMotorSelection == MOTOR_2_ONLY || currentMotorSelection == BOTH_MOTORS) {
                if (!motor2.setMechanicalZero()) {
                    log(LOG_ERROR, "Failed to set mechanical zero for motor 2!");
                    zeroSuccess = false;
                }
            }
            
            if (zeroSuccess) {
                // Reset our internal position tracking
                currentPosition = 0.0f;
                
                // Only enter zero hold mode if we're in position mode
                if (inPositionMode) {
                    // Start holding the zero position
                    holdingZeroPosition = true;
                    zeroHoldStartTime = currentTime;
                    log(LOG_INFO, "Entering zero hold position mode");
                }
                
                // Always update the last zero set time
                lastZeroSetTime = currentTime;
            }
        }
        
        // Update motor at the specified rate
        if (currentTime - lastMotorUpdateTime >= MOTOR_UPDATE_RATE_MS) {
            // If no signal is detected, default to center position (stop/neutral)
            if (!cppmReader.isReceiving()) {
                log(LOG_ERROR, "No CPPM signal detected!");
                // Set motors to safe state
                motor1.setVelocity(0);
                motor2.setVelocity(0);
                setAllPixelsColor(50, 50, 0); // Yellow for no signal
            } else {
                if (inPositionMode && modeInitialized) {
                    // Position Mode - use channel 4 for position control
                    int positionPulseWidth = cppmReader.getChannel(POS_CHANNEL);
                    float targetPosition = mapPulseToPosition(positionPulseWidth);
                    
                    // Skip setting new target position if we're holding zero
                    if (!holdingZeroPosition) {
                        currentPosition = targetPosition;
                    }
                    
                    // Print debug info occasionally
                    if (currentTime - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
                        log(LOG_INFO, "Position Mode: RC Pulse: " + String(positionPulseWidth) + 
                                      "us, Target Position: " + String(holdingZeroPosition ? 0.0f : targetPosition) + " rad");
                        lastDebugPrint = currentTime;
                    }
                    
                    // Calculate position to set (0 if holding zero, otherwise target position)
                    float positionToSet = holdingZeroPosition ? 0.0f : targetPosition;
                    
                    // Set position for the appropriate motor(s)
                    bool success = true;
                    
                    // For motor 1
                    if (currentMotorSelection == MOTOR_1_ONLY || currentMotorSelection == BOTH_MOTORS) {
                        if (!motor1.setPosition(positionToSet)) {
                            log(LOG_ERROR, "Failed to set position for motor 1!");
                            success = false;
                        }
                    }
                    
                    // For motor 2 (with reversed direction when in BOTH_MOTORS mode)
                    if (currentMotorSelection == MOTOR_2_ONLY || currentMotorSelection == BOTH_MOTORS) {
                        float motor2Position = positionToSet;
                        
                        // In BOTH_MOTORS mode, reverse the direction for motor 2
                        if (currentMotorSelection == BOTH_MOTORS) {
                            motor2Position = -positionToSet;
                        }
                        
                        if (!motor2.setPosition(motor2Position)) {
                            log(LOG_ERROR, "Failed to set position for motor 2!");
                            success = false;
                        }
                    }
                    
                    // Set LED color based on position and success
                    if (success) {
                        if (holdingZeroPosition) {
                            // Pulsing white color for zero hold mode
                            int pulseIntensity = (millis() / 100) % 50; // 0-49 pulsing effect
                            setAllPixelsColor(pulseIntensity, pulseIntensity, pulseIntensity);
                        } else if (positionToSet > 0) {
                            // Positive position - cyan with intensity based on position
                            int intensity = map(abs(positionToSet * 1000), 0, MAX_POSITION * 1000, 0, 50);
                            setAllPixelsColor(0, intensity, intensity);
                        } else if (positionToSet < 0) {
                            // Negative position - magenta with intensity based on position
                            int intensity = map(abs(positionToSet * 1000), 0, MAX_POSITION * 1000, 0, 50);
                            setAllPixelsColor(intensity, 0, intensity);
                        } else {
                            // Zero position - white
                            setAllPixelsColor(20, 20, 20);
                        }
                    } else {
                        setAllPixelsColor(50, 50, 0); // Yellow for command error
                    }
                    
                } else if (!inPositionMode && modeInitialized) {
                    // Velocity Mode - use channel 1 for velocity control
                    int velocityPulseWidth = cppmReader.getChannel(CPPM_CHANNEL);
                    float targetVelocity = mapPulseToVelocity(velocityPulseWidth);
                    
                    // Get current limit from channel 2
                    int currentPulseWidth = cppmReader.getChannel(CURRENT_CHANNEL);
                    float currentLimit = mapPulseToCurrentLimit(currentPulseWidth);
                    
                    // Store the current limit for feedback display
                    rcCurrentLimit = currentLimit;
                    
                    // Print debug info occasionally
                    if (currentTime - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
                        log(LOG_INFO, "Velocity Mode: RC Pulse: " + String(velocityPulseWidth) + 
                                      "us, Velocity: " + String(targetVelocity) + " rad/s, " +
                                      "Current Limit: " + String(currentLimit) + " A");
                        lastDebugPrint = currentTime;
                    }
                    
                    // Track success for LED color
                    bool success = true;
                    
                    // Set velocity for the appropriate motor(s)
                    if (currentMotorSelection == MOTOR_1_ONLY || currentMotorSelection == BOTH_MOTORS) {
                        if (!motor1.setVelocityWithLimits(targetVelocity, currentLimit, MOTOR_ACCELERATION)) {
                            log(LOG_ERROR, "Failed to set velocity for motor 1!");
                            success = false;
                        }
                    }
                    
                    // For motor 2 (with reversed direction when in BOTH_MOTORS mode)
                    if (currentMotorSelection == MOTOR_2_ONLY || currentMotorSelection == BOTH_MOTORS) {
                        float motor2Velocity = targetVelocity;
                        
                        // In BOTH_MOTORS mode, reverse the direction for motor 2
                        if (currentMotorSelection == BOTH_MOTORS) {
                            motor2Velocity = -targetVelocity;
                        }
                        
                        if (!motor2.setVelocityWithLimits(motor2Velocity, currentLimit, MOTOR_ACCELERATION)) {
                            log(LOG_ERROR, "Failed to set velocity for motor 2!");
                            success = false;
                        }
                    }
                    
                    // Set LED color based on velocity and success
                    if (success) {
                        if (targetVelocity > 0) {
                            // Forward - green intensity based on speed
                            int intensity = map(abs(targetVelocity * 1000), 0, MAX_VELOCITY * 1000, 0, 50);
                            setAllPixelsColor(0, intensity, 0);
                        } else if (targetVelocity < 0) {
                            // Reverse - red intensity based on speed
                            int intensity = map(abs(targetVelocity * 1000), 0, MAX_VELOCITY * 1000, 0, 50);
                            setAllPixelsColor(intensity, 0, 0);
                        } else {
                            // Stopped - blue
                            setAllPixelsColor(0, 0, 20);
                        }
                    } else {
                        setAllPixelsColor(50, 50, 0); // Yellow for command error
                    }
                }
            }
            
            lastMotorUpdateTime = currentTime;
        }
        
        // Fetch and report motor feedback in both position and velocity modes
        if (currentTime - lastFeedbackTime >= FEEDBACK_INTERVAL) {
            // Report feedback based on current motor selection
            if (currentMotorSelection == MOTOR_1_ONLY) {
                fetchAndReportMotorFeedback(motor1, "MOTOR 1");
            } else if (currentMotorSelection == MOTOR_2_ONLY) {
                fetchAndReportMotorFeedback(motor2, "MOTOR 2");
            } else { // BOTH_MOTORS
                fetchAndReportMotorFeedback(motor1, "MOTOR 1");
                fetchAndReportMotorFeedback(motor2, "MOTOR 2");
            }
            
            lastFeedbackTime = currentTime;
        }
        
        // Update the OLED display
        if (currentTime - lastDisplayUpdateTime >= DISPLAY_UPDATE_RATE_MS) {
            updateDisplay();
            lastDisplayUpdateTime = currentTime;
        }
    } else {
        // Motors not initialized
        unsigned long currentTime = millis();
        static unsigned long lastErrorPrint = 0;
        
        if (currentTime - lastErrorPrint >= 1000) {
            log(LOG_ERROR, "Motors not initialized!");
            lastErrorPrint = currentTime;
            
            // Update display with error
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("ERROR:");
            display.println("Motors not");
            display.println("initialized!");
            display.display();
        }
        setAllPixelsColor(50, 0, 0); // Red if motors not initialized
    }
    
    // Small delay to prevent tight looping
    delay(5); // Decreased from 10ms
}