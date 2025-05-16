#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "rs03_motor.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath> // For M_PI if available, otherwise define it
#include "RCPulseReader.h" // Include RC pulse reader
#include "FeatherM4CanInterface.h" // Include the new header file

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ----- Pin and Hardware Configuration -----
#define PIN            8    // NeoPixel data pin
#define NUMPIXELS      1    // Number of NeoPixels
#define RC_CHANNEL     2    // Channel 3 (index 2) for motor speed control
#define RC_PIN         9    // Pin for RC channel input
#define MOTOR_ID       127  // Fixed motor ID
#define MASTER_ID      0xFD // Default Master ID for commands

// ----- RC Pulse Configuration -----
#define MIN_PULSE      1000 // 1000us = -20 rad/s
#define MID_PULSE      1500 // 1500us = dead zone
#define MAX_PULSE      2000 // 2000us = 20 rad/s
#define DEAD_ZONE      50   // ±50us dead zone around 1500us

// ----- Motor Configuration -----
#define MOTOR_CURRENT_LIMIT 40.0f   // Current limit in amperes
#define MOTOR_ACCELERATION  200.0f  // Acceleration limit in rad/s²
#define MAX_VELOCITY        25.0f   // Maximum motor velocity in rad/s

// ----- Timing Configuration -----
#define MOTOR_UPDATE_RATE_MS 50  // 50ms = 20 updates per second
#define DEBUG_PRINT_INTERVAL 500 // Print debug info every 500ms

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
RCPulseReader rcReader;
unsigned long lastMotorUpdateTime = 0;
bool motorInitialized = false;

// ----- Function Prototypes -----
void setAllPixelsColor(int red, int green, int blue);
void log(int level, const String& message);
float mapPulseToVelocity(unsigned long pulseWidth);

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
float mapPulseToVelocity(unsigned long pulseWidth) {
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

// Set all pixels to the same color
void setAllPixelsColor(int red, int green, int blue) {
    pixels.setPixelColor(0, pixels.Color(red, green, blue));
    pixels.show();
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

    // Initialize RC pulse reader
    if (rcReader.attach(RC_CHANNEL, RC_PIN)) {
        log(LOG_INFO, "RC Pulse Reader initialized on pin " + String(RC_PIN) + " for channel " + String(RC_CHANNEL+1));
    } else {
        log(LOG_ERROR, "Failed to initialize RC Pulse Reader!");
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
    
    // Set the motor to velocity mode
    log(LOG_INFO, "Setting motor to velocity mode...");
    if (!motor.setModeVelocity()) {
        log(LOG_ERROR, "Failed to set velocity mode!");
    } else {
        log(LOG_INFO, "Motor set to velocity mode");
    }
    delay(200);
    
    motorInitialized = true;
    setAllPixelsColor(0, 50, 0); // Green indicates ready
    log(LOG_INFO, "Setup complete. Ready for RC control. (1000us=-20rad/s, 1500us=stop, 2000us=20rad/s)");
    delay(50);
}

void loop() {
    static bool firstLoop = true;
    if (firstLoop) {
        log(LOG_INFO, "Main loop started");
        firstLoop = false;
    }
    
    // Update RC pulse reader
    rcReader.update();

    // Handle RC control of the motor
    if (motorInitialized) {
        static unsigned long lastDebugPrint = 0;
        unsigned long currentTime = millis();
        
        // Update motor velocity at the specified rate
        if (currentTime - lastMotorUpdateTime >= MOTOR_UPDATE_RATE_MS) {
            // Read pulse width from RC channel
            unsigned long pulseWidth = rcReader.getPulseWidth(RC_CHANNEL);
            
            // If no pulse is detected, default to center position (stop)
            if (pulseWidth == 0) {
                pulseWidth = MID_PULSE;
            }
            
            // Map pulse width to velocity
            float targetVelocity = mapPulseToVelocity(pulseWidth);
            
            // Print debug info occasionally
            if (currentTime - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
                log(LOG_INFO, "RC Pulse: " + String(pulseWidth) + "us, Velocity: " + String(targetVelocity) + " rad/s");
                lastDebugPrint = currentTime;
            }
            
            // Set motor velocity with limits
            if (motor.setVelocityWithLimits(targetVelocity, MOTOR_CURRENT_LIMIT, MOTOR_ACCELERATION)) {
                // Set pixel color based on velocity
                if (targetVelocity > 0) {
                    // Forward - green intensity based on speed
                    int intensity = map(abs(targetVelocity), 0, MAX_VELOCITY, 0, 50);
                    setAllPixelsColor(0, intensity, 0);
                } else if (targetVelocity < 0) {
                    // Reverse - red intensity based on speed
                    int intensity = map(abs(targetVelocity), 0, MAX_VELOCITY, 0, 50);
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
            
            lastMotorUpdateTime = currentTime;
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