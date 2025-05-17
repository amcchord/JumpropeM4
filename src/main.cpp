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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ----- Pin and Hardware Configuration -----
#define PIN            8    // NeoPixel data pin
#define NUMPIXELS      1    // Number of NeoPixels
#define CPPM_PIN       9    // Pin for CPPM signal input
#define MOTOR_ID       127  // Fixed motor ID
#define MASTER_ID      0xFD // Default Master ID for commands

// ----- CPPM Configuration -----
#define MIN_PULSE      1000 // 1000us = -20 rad/s
#define MID_PULSE      1500 // 1500us = dead zone
#define MAX_PULSE      2000 // 2000us = 20 rad/s
#define DEAD_ZONE      50   // ±50us dead zone around 1500us
#define CPPM_CHANNEL   0    // First channel (0-based index) for motor speed control
#define MODE_CHANNEL   2    // Channel 3 (zero-indexed as 2) for mode control
#define POS_CHANNEL    3    // Channel 4 (zero-indexed as 3) for position control
#define ZERO_CHANNEL   5    // Channel 6 (zero-indexed as 5) for position zeroing
#define CPPM_CHANNELS  8    // Total number of CPPM channels

// ----- Mode Selection Thresholds -----
#define MODE_THRESHOLD 1800 // Above 1800us = position mode, below = velocity mode
#define ZERO_THRESHOLD 1800 // Above 1800us = set position to zero

// ----- Motor Configuration -----
#define MOTOR_CURRENT_LIMIT 40.0f   // Current limit in amperes
#define MOTOR_ACCELERATION  200.0f  // Acceleration limit in rad/s² for velocity mode
#define POSITION_SPEED_LIMIT 25.0f  // Speed limit for position mode in rad/s
#define POSITION_ACCELERATION 20.0f // Acceleration limit in rad/s² for position mode
#define MAX_VELOCITY        25.0f   // Maximum motor velocity in rad/s
#define MIN_POSITION        -4.0f   // Minimum position in radians
#define MAX_POSITION        4.0f    // Maximum position in radians

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

// ----- Global Objects -----
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
CANSAME5x CAN;
CPPMReader cppmReader(CPPM_PIN, 8, MIN_PULSE, MAX_PULSE, MID_PULSE);
unsigned long lastMotorUpdateTime = 0;
unsigned long lastFeedbackTime = 0;
bool motorInitialized = false;
bool inPositionMode = false;
bool modeInitialized = false;
float currentPosition = 0.0f;

// Added variables for zero position management
unsigned long lastZeroSetTime = 0;
bool holdingZeroPosition = false;
unsigned long zeroHoldStartTime = 0;

// Global variables to store raw feedback from CAN messages
float g_raw_position = 0.0f;
float g_raw_velocity = 0.0f;
float g_raw_torque = 0.0f;
float g_raw_temperature = 0.0f;
bool g_raw_feedback_received = false;
unsigned long g_last_raw_feedback_time = 0;

// ----- Function Prototypes -----
void setAllPixelsColor(int red, int green, int blue);
void log(int level, const String& message);
float mapPulseToVelocity(int pulseWidth);
float mapPulseToPosition(int pulseWidth);
void initializePositionMode();
void initializeVelocityMode();
void fetchAndReportMotorFeedback();
void processCanMessages();
bool queryParameter(uint16_t index, float &value);

// ----- Concrete CAN Interface for Feather M4 CAN -----
// Class has been moved to FeatherM4CanInterface.h/cpp

FeatherM4CanInterface canBus;
RS03Motor motor(canBus, MOTOR_ID, MASTER_ID);

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

// Initialize position mode with proper parameters
void initializePositionMode() {
    log(LOG_INFO, "Initializing position mode (PP) with proper limits");
    
    // Set the motor to Profile Position (PP) mode with speed and acceleration limits
    if (!motor.setModePositionPP(POSITION_SPEED_LIMIT, POSITION_ACCELERATION, MOTOR_CURRENT_LIMIT)) {
        log(LOG_ERROR, "Failed to set position mode (PP)!");
        return;
    }
    
    log(LOG_INFO, "Position mode (PP) set with speed limit: " + String(POSITION_SPEED_LIMIT) + 
                  " rad/s, acceleration: " + String(POSITION_ACCELERATION) + 
                  " rad/s², current limit: " + String(MOTOR_CURRENT_LIMIT) + " A");
    
    modeInitialized = true;
}

// Initialize velocity mode with proper parameters
void initializeVelocityMode() {
    log(LOG_INFO, "Initializing velocity mode");
    
    // Set the motor to velocity mode
    if (!motor.setModeVelocity()) {
        log(LOG_ERROR, "Failed to set velocity mode!");
        return;
    }

    // Set current limit and acceleration for velocity mode
    bool success = true;
    if (!motor.setParameterFloat(INDEX_LIMIT_CUR, MOTOR_CURRENT_LIMIT)) {
        log(LOG_ERROR, "Failed to set current limit for velocity mode!");
        success = false;
    }
    
    if (!motor.setParameterFloat(0x7022, MOTOR_ACCELERATION)) {
        log(LOG_ERROR, "Failed to set acceleration limit for velocity mode!");
        success = false;
    }
    
    if (success) {
        log(LOG_INFO, "Velocity mode set with current limit: " + String(MOTOR_CURRENT_LIMIT) + 
                      " A, acceleration: " + String(MOTOR_ACCELERATION) + " rad/s²");
    }
    
    modeInitialized = true;
}

// Fetch and report motor feedback
void fetchAndReportMotorFeedback() {
    // We'll directly query each parameter rather than relying on getLastFeedback
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    float temperature = 0.0f;
    float vbus = 0.0f;
    float current = 0.0f;
    int mode = 0;
    bool success = false;
    
    // Check if we have recent raw feedback from Type 2 messages (less than 500ms old)
    if (g_raw_feedback_received && (millis() - g_last_raw_feedback_time < 500)) {
        // Use the raw feedback data
        position = g_raw_position;
        velocity = g_raw_velocity;
        torque = g_raw_torque;
        temperature = g_raw_temperature;
        success = true;
        log(LOG_DEBUG, "Using raw feedback data from Type 2 messages");
    }
    
    // Parameter indices
    const uint16_t INDEX_POSITION = 0x7000;
    const uint16_t INDEX_VELOCITY = 0x7001;
    const uint16_t INDEX_TORQUE = 0x7002;
    const uint16_t INDEX_TEMPERATURE = 0x7007;
    const uint16_t INDEX_VBUS = 0x701C;
    const uint16_t INDEX_MODE = 0x7005;  // Corrected from 0x7030 to 0x7005 (run_mode)
    const uint16_t INDEX_CURRENT = 0x701A; // IQF (iq Filter) - motor current
    
    // Even if we have raw feedback, always query VBUS and Mode
    // as they're not included in Type 2 messages
    
    // Parameter index: 0x701C = VBUS
    if (queryParameter(INDEX_VBUS, vbus)) {
        log(LOG_DEBUG, "Direct VBUS query success: " + String(vbus));
    }
    
    // Parameter index: 0x7005 = Mode
    float mode_float = 0.0f;
    if (queryParameter(INDEX_MODE, mode_float)) {
        mode = (int)mode_float;
        log(LOG_DEBUG, "Direct mode query success: " + String(mode));
    }
    
    // Parameter index: 0x701A = Current (IQF)
    if (queryParameter(INDEX_CURRENT, current)) {
        log(LOG_DEBUG, "Direct current query success: " + String(current));
    }
    
    // If we don't have raw feedback, query all parameters directly
    if (!success) {
        // Directly query all parameters
        // Parameter index: 0x7000 = Position
        if (queryParameter(INDEX_POSITION, position)) {
            success = true;
            log(LOG_DEBUG, "Direct position query success: " + String(position));
        }
        
        // Parameter index: 0x7001 = Velocity
        if (queryParameter(INDEX_VELOCITY, velocity)) {
            success = true;
            log(LOG_DEBUG, "Direct velocity query success: " + String(velocity));
        }
        
        // Parameter index: 0x7002 = Torque
        if (queryParameter(INDEX_TORQUE, torque)) {
            success = true;
            log(LOG_DEBUG, "Direct torque query success: " + String(torque));
        }
        
        // Parameter index: 0x7007 = Temperature
        if (queryParameter(INDEX_TEMPERATURE, temperature)) {
            success = true;
            log(LOG_DEBUG, "Direct temperature query success: " + String(temperature));
        }
    }
    
    // Only if we couldn't get any direct queries or raw feedback, fall back to last feedback
    if (!success) {
        // Get the motor's feedback information
        RS03Motor::Feedback feedback = motor.getLastFeedback();
        position = feedback.position;
        velocity = feedback.velocity;
        torque = feedback.torque;
        temperature = feedback.temperature;
        mode = feedback.mode;
        
        // Note: current may not be available in the feedback structure
        // so we keep the last queried value
    }
    
    // Report position, velocity, and general status
    log(LOG_INFO, "-------- MOTOR FEEDBACK --------");
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
        log(LOG_ERROR, "Error flags: 0x" + String(error_flags, HEX));
        // Convert getErrorText() result to String to avoid compilation issues
        log(LOG_ERROR, "Error description: " + String(motor.getErrorText().c_str()));
    }
    
    log(LOG_INFO, "--------------------------------");
}

// Helper function to query a parameter from the motor
bool queryParameter(uint16_t index, float &value) {
    // Create a Type 17 frame to read the parameter
    uint32_t readId = ((0x11) << 24) | (MASTER_ID << 8) | MOTOR_ID;
    uint8_t readData[8] = {0};
    // Parameter index in little-endian
    readData[0] = index & 0xFF;        // Low byte of index
    readData[1] = (index >> 8) & 0xFF; // High byte of index
    
    if (DEBUG_LEVEL >= LOG_DEBUG) {
        log(LOG_DEBUG, "Querying parameter 0x" + String(index, HEX) + " - ID: 0x" + String(readId, HEX));
    }
    
    // Send the request
    if (!CAN.beginExtendedPacket(readId & 0x1FFFFFFF)) {
        return false;
    }
    
    CAN.write(readData, 8);
    if (!CAN.endPacket()) {
        return false;
    }
    
    // Wait for response
    unsigned long startTime = millis();
    while (millis() - startTime < 100) { // 100ms timeout
        int packetSize = CAN.parsePacket();
        if (packetSize) {
            uint32_t id = CAN.packetId();
            uint8_t responseType = (id >> 24) & 0x1F;
            
            // Read the data payload
            uint8_t data[8] = {0};
            for (int i = 0; i < packetSize && i < 8; i++) {
                data[i] = CAN.read();
            }
            
            // Check for the Type 0x11 response
            if (responseType == 0x11) {
                uint16_t responseIndex = data[0] | (data[1] << 8);
                if (responseIndex == index) {
                    // The value is stored as a float in the last 4 bytes
                    memcpy(&value, &data[4], sizeof(float));
                    return true;
                }
            }
        }
        delay(1);
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
            g_raw_position = motor.uintToFloat(pos_raw, P_MIN, P_MAX, 16);
            g_raw_velocity = motor.uintToFloat(vel_raw, V_MIN, V_MAX, 16);
            g_raw_torque = motor.uintToFloat(torque_raw, T_MIN, T_MAX, 16);
            g_raw_temperature = static_cast<float>(temp_raw) / 10.0f;
            
            g_raw_feedback_received = true;
            g_last_raw_feedback_time = millis();
            
            if (DEBUG_LEVEL >= LOG_VERBOSE) {
                log(LOG_VERBOSE, "Type 2 feedback received: Pos=" + String(g_raw_position, 4) +
                                 " Vel=" + String(g_raw_velocity, 4) +
                                 " Torq=" + String(g_raw_torque, 4) +
                                 " Temp=" + String(g_raw_temperature, 1));
            }
        }
        
        // Process the frame with the motor library as well
        bool processed = motor.processFeedback(frame);
        
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
        
        if (processed) {
            log(LOG_VERBOSE, "Processed motor feedback frame");
        }
    }
}

// ----- Setup and Main Loop -----

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
    log(LOG_INFO, "RS03 Motor Control with RC Input - Starting");
    
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

    // Initialize motor with fixed ID
    log(LOG_INFO, "Using motor ID: " + String(MOTOR_ID));
    
    // Reset any faults
    log(LOG_INFO, "Resetting motor faults...");
    if (!motor.resetFaults()) {
        log(LOG_ERROR, "Failed to reset motor faults!");
    } else {
        log(LOG_INFO, "Motor faults reset");
    }
    delay(200);
    
    // Enable the motor
    log(LOG_INFO, "Enabling motor...");
    if (!motor.enable()) {
        log(LOG_ERROR, "Failed to enable motor!");
    } else {
        log(LOG_INFO, "Motor enabled");
    }
    delay(200);
    
    // Set the motor to velocity mode initially
    initializeVelocityMode();
    inPositionMode = false;
    
    // Set up active reporting from the motor using direct CAN communication
    log(LOG_INFO, "Enabling active reporting from the motor...");
    
    // First try using the library method
    bool reportingEnabled = motor.setActiveReporting(true);
    
    // If that fails, try a direct approach based on main-old.cpp
    if (!reportingEnabled) {
        log(LOG_WARNING, "Library method failed, trying direct approach for active reporting");
        
        // Type 5 message for active reporting command
        uint32_t reportingId = ((static_cast<uint32_t>(0x05) << 24) | 
                              (static_cast<uint32_t>(MASTER_ID) << 8) | 
                              static_cast<uint32_t>(MOTOR_ID)) & 0x1FFFFFFF;
        
        uint8_t reportingData[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x01 = enable reporting
        
        if (CAN.beginExtendedPacket(reportingId)) {
            CAN.write(reportingData, 8);
            if (CAN.endPacket()) {
                log(LOG_INFO, "Direct active reporting command sent");
                reportingEnabled = true;
            } else {
                log(LOG_ERROR, "Failed to send direct active reporting command");
            }
        } else {
            log(LOG_ERROR, "Failed to begin direct active reporting packet");
        }
    }
    
    if (reportingEnabled) {
        log(LOG_INFO, "Active reporting enabled");
    } else {
        log(LOG_ERROR, "Failed to enable active reporting");
    }
    
    // Verify motor communication by reading VBUS
    log(LOG_INFO, "Verifying motor communication by reading VBUS...");
    float vbus = 0.0f;
    bool vbusSuccess = false;
    
    // Create a Type 17 frame to read the VBUS parameter
    uint32_t readId = ((0x11) << 24) | (MASTER_ID << 8) | MOTOR_ID;
    uint8_t readData[8] = {0};
    readData[0] = 0x1C;  // 0x701C = VBUS register (low byte)
    readData[1] = 0x70;  // high byte
    
    // Send the request
    if (CAN.beginExtendedPacket(readId & 0x1FFFFFFF)) {
        CAN.write(readData, 8);
        if (CAN.endPacket()) {
            // Wait for response
            unsigned long startTime = millis();
            while (millis() - startTime < 500 && !vbusSuccess) {
                int packetSize = CAN.parsePacket();
                if (packetSize) {
                    uint32_t id = CAN.packetId();
                    uint8_t responseType = (id >> 24) & 0x1F;
                    
                    // Read the data payload
                    uint8_t data[8] = {0};
                    for (int i = 0; i < packetSize && i < 8; i++) {
                        data[i] = CAN.read();
                    }
                    
                    // Check for the Type 0x11 response
                    if (responseType == 0x11) {
                        uint16_t responseIndex = data[0] | (data[1] << 8);
                        if (responseIndex == 0x701C) {
                            memcpy(&vbus, &data[4], sizeof(float));
                            if (vbus > 5.0f && vbus < 60.0f) {
                                log(LOG_INFO, "VBUS voltage: " + String(vbus) + " V");
                                vbusSuccess = true;
                            }
                        }
                    }
                }
                delay(1);
            }
        }
    }
    
    if (!vbusSuccess) {
        log(LOG_WARNING, "Could not read VBUS voltage. Motor may not be responding correctly.");
    }
    
    motorInitialized = true;
    setAllPixelsColor(0, 50, 0); // Green indicates ready
    log(LOG_INFO, "Setup complete. Ready for CPPM control.");
    log(LOG_INFO, "Channel 1 (index 0): 1000us=-20rad/s, 1500us=stop, 2000us=20rad/s");
    log(LOG_INFO, "Channel 3 (index 2): >1800us=position mode, <1800us=velocity mode");
    log(LOG_INFO, "Channel 4 (index 3): 1000us=-4rad, 2000us=+4rad (in position mode)");
    log(LOG_INFO, "Channel 6 (index 5): >1800us=set current position as zero");
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

    // Periodically request motor state directly using Type 0 query
    static unsigned long lastDirectQueryTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastDirectQueryTime >= 100) { // Every 100ms
        // Send a Type 0 query to request motor state
        uint32_t queryId = ((0x00) << 24) | (MASTER_ID << 16) | MOTOR_ID;
        if (CAN.beginExtendedPacket(queryId & 0x1FFFFFFF)) {
            // Type 0 has no data
            if (CAN.endPacket()) {
                if (DEBUG_LEVEL >= LOG_VERBOSE) {
                    log(LOG_VERBOSE, "Sent Type 0 query to request motor state");
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

    // Handle CPPM control of the motor
    if (motorInitialized) {
        static unsigned long lastDebugPrint = 0;
        unsigned long currentTime = millis();
        
        // Check if we need to switch modes
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
            if (inPositionMode) {
                initializePositionMode();
            } else {
                initializeVelocityMode();
            }
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
                
                // Set the position to zero
                if (!motor.setPosition(0.0f)) {
                    log(LOG_ERROR, "Failed to hold zero position!");
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
            
            // Use the proper method to set mechanical zero (Communication Type 6)
            if (!motor.setMechanicalZero()) {
                log(LOG_ERROR, "Failed to set mechanical zero!");
            } else {
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
                // Set motor to safe state
                if (inPositionMode) {
                    // Hold current position
                } else {
                    // Stop motor
                    motor.setVelocity(0);
                }
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
                    
                    // Set position - note that speed and current limits were set during mode initialization
                    // If holding zero, we force the position to 0, otherwise use the target position
                    float positionToSet = holdingZeroPosition ? 0.0f : targetPosition;
                    if (motor.setPosition(positionToSet)) {
                        // Set pixel color based on position
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
                        if (currentTime - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
                            log(LOG_ERROR, "Failed to set motor position!");
                        }
                        setAllPixelsColor(50, 50, 0); // Yellow for command error
                    }
                    
                } else if (!inPositionMode && modeInitialized) {
                    // Velocity Mode - use channel 1 for velocity control
                    int velocityPulseWidth = cppmReader.getChannel(CPPM_CHANNEL);
                    float targetVelocity = mapPulseToVelocity(velocityPulseWidth);
                    
                    // Print debug info occasionally
                    if (currentTime - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
                        log(LOG_INFO, "Velocity Mode: RC Pulse: " + String(velocityPulseWidth) + 
                                      "us, Velocity: " + String(targetVelocity) + " rad/s");
                        lastDebugPrint = currentTime;
                    }
                    
                    // Set motor velocity with limits
                    if (motor.setVelocityWithLimits(targetVelocity, MOTOR_CURRENT_LIMIT, MOTOR_ACCELERATION)) {
                        // Set pixel color based on velocity
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
                        if (currentTime - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
                            log(LOG_ERROR, "Failed to set motor velocity/limits!");
                        }
                        setAllPixelsColor(50, 50, 0); // Yellow for command error
                    }
                }
            }
            
            lastMotorUpdateTime = currentTime;
        }
        
        // Fetch and report motor feedback in both position and velocity modes
        if (currentTime - lastFeedbackTime >= FEEDBACK_INTERVAL) {
            fetchAndReportMotorFeedback();
            lastFeedbackTime = currentTime;
        }
    } else {
        // Motor not initialized
        unsigned long currentTime = millis();
        static unsigned long lastErrorPrint = 0;
        
        if (currentTime - lastErrorPrint >= 1000) {
            log(LOG_ERROR, "Motor not initialized!");
            lastErrorPrint = currentTime;
        }
        setAllPixelsColor(50, 0, 0); // Red if motor not initialized
    }
    
    // Small delay to prevent tight looping
    delay(5); // Decreased from 10ms
}