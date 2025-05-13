#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "rs03_motor.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath> // For M_PI if available, otherwise define it
#include "RCPulseReader.h" // Include RC pulse reader

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <CANSAME5x.h>

#define PIN        8  // Which pin on the Arduino is connected to the NeoPixels
#define NUMPIXELS 1  // How many NeoPixels are attached to the Arduino
#define DELAYVAL 500  // Time (in milliseconds) to pause between colors

// RC Pulse Reader setup
#define RC_CHANNEL 2  // Channel 3 (index 2) for motor speed control
#define RC_PIN 9     // Pin for RC channel 3 input
#define MIN_PULSE 1000  // 1000us = -20 rad/s
#define MID_PULSE 1500  // 1500us = dead zone
#define MAX_PULSE 2000  // 2000us = 20 rad/s
#define DEAD_ZONE 50    // ±50us dead zone around 1500us

// Motor update rate
#define MOTOR_UPDATE_RATE_MS 50  // 50ms = 20 updates per second

// Initialize the NeoPixel strip
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Initialize CAN object
CANSAME5x CAN;

// Initialize RC Pulse Reader
RCPulseReader rcReader;

int colorIndex = 0;  // Track which color to display
unsigned long lastCanMsgTime = 0;  // Track when we last sent a CAN message
unsigned long lastMotorUpdateTime = 0; // Track when we last updated the motor

// Function prototype
void setAllPixelsColor(int red, int green, int blue);
void sendTestCanMessages();

// --- Configuration ---
const uint8_t MOTOR_ID = 127; // Fixed motor ID
const uint8_t MASTER_ID = 0xFD; // Default Master ID for commands (usually 0xFD or 0xFE)

// --- Concrete CAN Interface for Feather M4 CAN ---
class FeatherM4CanInterface : public CanInterface {
public:
    FeatherM4CanInterface() = default;

    bool sendFrame(const CanFrame& frame) override {
        // Debug output
        Serial.print("sendFrame called with ID=0x");
        Serial.print(frame.id, HEX);
        Serial.print(", extended=");
        Serial.print(frame.is_extended ? "true" : "false");
        Serial.print(", DLC=");
        Serial.println(frame.dlc);
        Serial.flush(); // Make sure debug info is sent
        
        // Check ID range for extended IDs (29-bit max)
        if (frame.is_extended && frame.id > 0x1FFFFFFF) {
            Serial.print("ERROR: Extended ID 0x");
            Serial.print(frame.id, HEX);
            Serial.println(" exceeds 29-bit maximum");
            Serial.flush();
            return false;
        }
        
        // Use CAN.beginPacket for standard IDs or beginExtendedPacket for extended IDs
        bool beginSuccess;
        if (frame.is_extended) {
            // For extended IDs, ensure we're only using 29 bits
            uint32_t id29bit = frame.id & 0x1FFFFFFF;
            Serial.print("Starting extended packet with ID 0x");
            Serial.println(id29bit, HEX);
            Serial.flush();
            beginSuccess = CAN.beginExtendedPacket(id29bit);
            if (!beginSuccess) {
                Serial.print("Failed to begin extended packet with ID 0x");
                Serial.println(id29bit, HEX);
                Serial.flush();
                return false;
            }
        } else {
            // For standard IDs, ensure we're only using 11 bits
            uint32_t id11bit = frame.id & 0x7FF;
            Serial.print("Starting standard packet with ID 0x");
            Serial.println(id11bit, HEX);
            Serial.flush();
            beginSuccess = CAN.beginPacket(id11bit);
            if (!beginSuccess) {
                Serial.print("Failed to begin packet with ID 0x");
                Serial.println(id11bit, HEX);
                Serial.flush();
                return false;
            }
        }
        
        // Write the data if there is any
        if (frame.dlc > 0) {
            Serial.print("Writing data bytes: ");
            for (int i = 0; i < frame.dlc; i++) {
                Serial.print("0x");
                if (frame.data[i] < 0x10) Serial.print("0");
                Serial.print(frame.data[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
            Serial.flush();
            CAN.write(frame.data, frame.dlc);
        }
        
        // End the packet and transmit
        Serial.println("Calling endPacket()...");
        Serial.flush();
        if (!CAN.endPacket()) {
            Serial.println("Failed to send packet");
            Serial.flush();
            return false;
        }
        
        Serial.println("CAN frame successfully sent");
        Serial.flush();
        return true;
    }

    // Optional: Implement receive if needed for direct polling
    bool receiveFrame(CanFrame& frame, uint32_t timeout_ms) override {
        unsigned long startTime = millis();
        
        // Poll for packets until timeout
        while ((millis() - startTime) < timeout_ms || timeout_ms == 0) {
            int packetSize = CAN.parsePacket();
            if (packetSize) {
                // We've received a packet
                frame.id = CAN.packetId();
                frame.is_extended = CAN.packetExtended();
                frame.dlc = packetSize;
                
                // Read the data
                int i = 0;
                while (CAN.available() && i < 8) {
                    frame.data[i++] = CAN.read();
                }
                
                // Zero out remaining bytes if dlc < 8
                if (frame.dlc < 8) {
                    memset(frame.data + frame.dlc, 0, 8 - frame.dlc);
                }
                
                return true;
            }
            
            // Small delay to prevent tight loop
            if (timeout_ms != 0) {
                delay(1);
            }
        }
        
        return false;
    }
};

// --- Global Objects ---
FeatherM4CanInterface canBus; // Instantiate the concrete CAN interface
// Initialize with fixed motor ID 127
RS03Motor motor(canBus, MOTOR_ID, MASTER_ID);
bool motorInitialized = false; // Flag to indicate if the motor is initialized

// Map RC pulse width to motor velocity
float mapPulseToVelocity(unsigned long pulseWidth) {
    // Dead zone around center (1500us)
    if (pulseWidth >= (MID_PULSE - DEAD_ZONE) && pulseWidth <= (MID_PULSE + DEAD_ZONE)) {
        return 0.0f;
    }
    
    // Map pulse width to velocity
    if (pulseWidth < MID_PULSE) {
        // Map 1000-1450us to -20-0 rad/s
        return map(pulseWidth, MIN_PULSE, MID_PULSE - DEAD_ZONE, -20, 0) * 1.0f;
    } else {
        // Map 1550-2000us to 0-20 rad/s
        return map(pulseWidth, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 20) * 1.0f;
    }
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 5000); // Wait for serial port to connect (max 5s)
  Serial.println("RS03 Motor Control with RC Input - Starting");
  
  // Initialize the NeoPixel library
  pixels.begin();
  pixels.setBrightness(50);  // Set brightness to 50%
  pixels.clear();
  setAllPixelsColor(50, 0, 50); // Purple while initializing
  Serial.println("NeoPixel initialized");
  
  // Initialize CAN bus
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster
  
  // Configure CAN settings: 1 Mbit/s for motor communication
  if (!CAN.begin(1000000)) {
    Serial.println("Starting CAN failed!");
    setAllPixelsColor(50, 0, 0);  // Red indicates error
    while(1); // Halt on CAN error
  }
  Serial.println("CAN initialized successfully at 1 Mbit/s");

  // Initialize RC pulse reader for channel 3
  Serial.println("Attempting to initialize RC Pulse Reader...");
  if (rcReader.attach(RC_CHANNEL, RC_PIN)) {
    Serial.println("RC Pulse Reader initialized on pin " + String(RC_PIN) + " for channel " + String(RC_CHANNEL+1));
  } else {
    Serial.println("Failed to initialize RC Pulse Reader!");
    setAllPixelsColor(50, 0, 50); // Purple indicates RC setup error
  }

  // Initialize motor with fixed ID
  Serial.println("Using fixed motor ID: " + String(MOTOR_ID));
  
  // Reset any faults
  Serial.println("Resetting motor faults...");
  if (!motor.resetFaults()) {
    Serial.println("Failed to reset motor faults!");
  } else {
    Serial.println("Motor faults reset");
  }
  delay(200);
  
  // Enable the motor
  Serial.println("Enabling motor...");
  if (!motor.enable()) {
    Serial.println("Failed to enable motor!");
  } else {
    Serial.println("Motor enabled");
  }
  delay(200);
  
  // Set the motor to velocity mode
  Serial.println("Setting motor to velocity mode...");
  if (!motor.setModeVelocity()) {
    Serial.println("Failed to set velocity mode!");
  } else {
    Serial.println("Motor set to velocity mode");
  }
  delay(200);
  
  // IMPORTANT: DON'T enable active reporting - it causes the system to hang
  // Serial.println("Enabling active reporting...");
  // if (!motor.setActiveReporting(true)) {
  //   Serial.println("Failed to enable active reporting!");
  // } else {
  //   Serial.println("Active reporting enabled");
  // }
  // delay(200);
  
  motorInitialized = true;
  setAllPixelsColor(0, 50, 0); // Green indicates ready
  Serial.println("Setup complete. Motor ready for RC control.");
  Serial.flush();
  delay(20);
  
  Serial.println("RC input: 1000us = -20 rad/s, 1500us = stop, 2000us = 20 rad/s");
  Serial.flush();
  delay(20);
  
  Serial.println("Starting main loop...");
  Serial.flush();
  delay(50);
}

void loop() {
  static bool firstLoop = true;
  if (firstLoop) {
    Serial.println("Loop started successfully!");
    Serial.flush();
    firstLoop = false;
  }
  
  // Debug point 1
  // Serial.println("DEBUG: Start of loop");
  // Serial.flush();
  
  // Skip CAN frame reception entirely - this may be causing the hang
  // CanFrame receivedFrame;
  // if (canBus.receiveFrame(receivedFrame, 0)) { ... }

  // Update RC pulse reader
  rcReader.update();

  // Handle RC control of the motor
  if (motorInitialized) {
    static unsigned long lastDebugPrint = 0;
    unsigned long currentTime = millis();
    
    // Update motor velocity at the specified rate
    if (currentTime - lastMotorUpdateTime >= MOTOR_UPDATE_RATE_MS) {
      // Read pulse width from RC channel 3
      unsigned long pulseWidth = rcReader.getPulseWidth(RC_CHANNEL);
      
      // If no pulse is detected, default to center position (stop)
      if (pulseWidth == 0) {
        pulseWidth = MID_PULSE;
      }
      
      // Map pulse width to velocity
      float targetVelocity = mapPulseToVelocity(pulseWidth);
      
      // Limit velocity for safety
      // if (targetVelocity > 20.0f) targetVelocity = 5.0f;
      // if (targetVelocity < -5.0f) targetVelocity = -5.0f;
      
      // Print debug info occasionally
      if (currentTime - lastDebugPrint >= 500) { // Print every 500ms
        Serial.print("RC Pulse: ");
        Serial.print(pulseWidth);
        Serial.print("us, Velocity: ");
        Serial.print(targetVelocity);
        Serial.println(" rad/s");
        lastDebugPrint = currentTime;
      }
      
      // Set motor velocity
      if (motor.setVelocity(targetVelocity)) {
        // Set pixel color based on velocity
        if (targetVelocity > 0) {
          // Forward - green intensity based on speed
          int intensity = map(abs(targetVelocity), 0, 20, 0, 50);
          setAllPixelsColor(0, intensity, 0);
        } else if (targetVelocity < 0) {
          // Reverse - red intensity based on speed
          int intensity = map(abs(targetVelocity), 0, 20, 0, 50);
          setAllPixelsColor(intensity, 0, 0);
        } else {
          // Stopped - blue
          setAllPixelsColor(0, 0, 20);
        }
      } else {
        if (currentTime - lastDebugPrint >= 500) { // Print errors only every 500ms
          Serial.println("Failed to set motor velocity!");
        }
        setAllPixelsColor(50, 50, 0); // Yellow for command error
      }
      
      lastMotorUpdateTime = currentTime;
    }
  } else {
    // Motor not initialized
    unsigned long currentTime = millis();
    static unsigned long lastErrorPrint = 0;
    
    if (currentTime - lastErrorPrint >= 1000) { // Print only every second
      Serial.println("Motor not initialized!");
      lastErrorPrint = currentTime;
    }
    setAllPixelsColor(50, 0, 0); // Red if motor not initialized
  }
  
  // Small delay to prevent tight looping
  delay(10);
}

// Helper function to set all pixels to the same color
void setAllPixelsColor(int red, int green, int blue) {
  pixels.setPixelColor(0, pixels.Color(red, green, blue));
  pixels.show();  // Send the updated pixel colors to the hardware
}

// Note: Removed sendTestCanMessages() function as it's replaced by motor control
// Note: Removed the C++ style main() function entirely. 

// Note: removed C++ style main function entirely