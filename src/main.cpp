#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "rs03_motor.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath> // For M_PI if available, otherwise define it
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
#define POSITION_SPEED_LIMIT 10.0f  // Speed limit for position mode in rad/s
#define POSITION_ACCELERATION 2.0f // Acceleration limit in rad/s² for position mode
#define MAX_VELOCITY        25.0f   // Maximum motor velocity in rad/s
#define MIN_POSITION        -4.0f   // Minimum position in radians
#define MAX_POSITION        4.0f    // Maximum position in radians

// ----- Timing Configuration -----
#define MOTOR_UPDATE_RATE_MS 50     // 50ms = 20 updates per second
#define DEBUG_PRINT_INTERVAL 500    // Print debug info every 500ms
#define FEEDBACK_INTERVAL    1000   // How often to fetch and report motor feedback in position mode

// ----- Debug Levels -----
#define LOG_ERROR   1
#define LOG_INFO    2
#define LOG_DEBUG   3
#define LOG_VERBOSE 4

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

// ----- Function Prototypes -----
void setAllPixelsColor(int red, int green, int blue);
void log(int level, const String& message);
float mapPulseToVelocity(int pulseWidth);
float mapPulseToPosition(int pulseWidth);
void initializePositionMode();
void initializeVelocityMode();
void fetchAndReportMotorFeedback();

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
    log(LOG_INFO, "Initializing position mode with proper limits");
    
    // First set the motor to position mode CSP with speed limit
    if (!motor.setModePositionCSP(POSITION_SPEED_LIMIT, MOTOR_CURRENT_LIMIT)) {
        log(LOG_ERROR, "Failed to set position mode!");
        return;
    }
    
    // Set acceleration limit for position mode
    if (!motor.setParameterFloat(0x7022, POSITION_ACCELERATION)) {
        log(LOG_ERROR, "Failed to set acceleration limit for position mode!");
    } else {
        log(LOG_INFO, "Position mode set with speed limit: " + String(POSITION_SPEED_LIMIT) + 
                      " rad/s, current limit: " + String(MOTOR_CURRENT_LIMIT) + 
                      " A, acceleration: " + String(POSITION_ACCELERATION) + " rad/s²");
    }
    
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
    // Get the motor's feedback information
    RS03Motor::Feedback feedback = motor.getLastFeedback();
    
    // Report position, velocity, and general status
    log(LOG_INFO, "-------- MOTOR FEEDBACK --------");
    log(LOG_INFO, "Position: " + String(feedback.position) + " rad");
    log(LOG_INFO, "Velocity: " + String(feedback.velocity) + " rad/s");
    log(LOG_INFO, "Torque:   " + String(feedback.torque) + " Nm");
    log(LOG_INFO, "Temp:     " + String(feedback.temperature) + " °C");
    log(LOG_INFO, "Mode:     " + String(feedback.mode));
    
    // Report errors if any
    if (feedback.error_flags) {
        log(LOG_ERROR, "Error flags: 0x" + String(feedback.error_flags, HEX));
        log(LOG_ERROR, "Error description: " + String(motor.getErrorText().c_str()));
    }
    
    log(LOG_INFO, "--------------------------------");
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

    static unsigned long lastChannelPrint = 0;
    unsigned long currentTime = millis();
    
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
        if (zeroChannelValue > ZERO_THRESHOLD && inPositionMode) {
            log(LOG_INFO, "Setting current position as zero");
            // Get current position and save it as offset
            currentPosition = 0.0f;
            // No direct method to set position offset, use setPosition(0)
            if (!motor.setPosition(0.0f)) {
                log(LOG_ERROR, "Failed to set position to zero!");
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
                    currentPosition = targetPosition;
                    
                    // Print debug info occasionally
                    if (currentTime - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
                        log(LOG_INFO, "Position Mode: RC Pulse: " + String(positionPulseWidth) + 
                                      "us, Target Position: " + String(targetPosition) + " rad");
                        lastDebugPrint = currentTime;
                    }
                    
                    // Set position - note that speed and current limits were set during mode initialization
                    if (motor.setPosition(targetPosition)) {
                        // Set pixel color based on position
                        if (targetPosition > 0) {
                            // Positive position - cyan with intensity based on position
                            int intensity = map(abs(targetPosition * 1000), 0, MAX_POSITION * 1000, 0, 50);
                            setAllPixelsColor(0, intensity, intensity);
                        } else if (targetPosition < 0) {
                            // Negative position - magenta with intensity based on position
                            int intensity = map(abs(targetPosition * 1000), 0, MAX_POSITION * 1000, 0, 50);
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
        
        // Fetch and report motor feedback in position mode
        if (inPositionMode && currentTime - lastFeedbackTime >= FEEDBACK_INTERVAL) {
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
    delay(10);
}