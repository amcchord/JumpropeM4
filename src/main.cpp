#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "rs03_motor.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath> // For M_PI if available, otherwise define it

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <CANSAME5x.h>

#define PIN        8  // Which pin on the Arduino is connected to the NeoPixels
#define NUMPIXELS 1  // How many NeoPixels are attached to the Arduino
#define DELAYVAL 500  // Time (in milliseconds) to pause between colors

// Initialize the NeoPixel strip
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Initialize CAN object
CANSAME5x CAN;

int colorIndex = 0;  // Track which color to display
unsigned long lastCanMsgTime = 0;  // Track when we last sent a CAN message
const unsigned long CAN_MSG_INTERVAL = 2000;  // Send a CAN message every 2 seconds

// Function prototype
void setAllPixelsColor(int red, int green, int blue);
void sendTestCanMessages();

// --- Configuration ---
// const uint8_t MOTOR_ID = 127; // No longer a fixed constant for operation, discovery will set it.
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
        
        // Check ID range for extended IDs (29-bit max)
        if (frame.is_extended && frame.id > 0x1FFFFFFF) {
            Serial.print("ERROR: Extended ID 0x");
            Serial.print(frame.id, HEX);
            Serial.println(" exceeds 29-bit maximum");
            return false;
        }
        
        // Use CAN.beginPacket for standard IDs or beginExtendedPacket for extended IDs
        bool beginSuccess;
        if (frame.is_extended) {
            // For extended IDs, ensure we're only using 29 bits
            uint32_t id29bit = frame.id & 0x1FFFFFFF;
            beginSuccess = CAN.beginExtendedPacket(id29bit);
            if (!beginSuccess) {
                Serial.print("Failed to begin extended packet with ID 0x");
                Serial.println(id29bit, HEX);
                return false;
            }
        } else {
            // For standard IDs, ensure we're only using 11 bits
            uint32_t id11bit = frame.id & 0x7FF;
            beginSuccess = CAN.beginPacket(id11bit);
            if (!beginSuccess) {
                Serial.print("Failed to begin packet with ID 0x");
                Serial.println(id11bit, HEX);
                return false;
            }
        }
        
        // Write the data if there is any
        if (frame.dlc > 0) {
            CAN.write(frame.data, frame.dlc);
        }
        
        // End the packet and transmit
        if (!CAN.endPacket()) {
            Serial.println("Failed to send packet");
            return false;
        }
        
        Serial.println("CAN frame successfully sent");
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
// Initialize with a temporary/placeholder ID. Discovery will update it.
RS03Motor motor(canBus, 0, MASTER_ID); // Placeholder ID 0
bool motorFound = false; // Flag to indicate if the target motor responded
uint8_t detectedMotorId = 0; // To store and report the ID found

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 5000); // Wait for serial port to connect (max 5s)
  Serial.println("RS03 Motor Control Test Start");
  
  // Initialize the NeoPixel library
  pixels.begin();
  pixels.setBrightness(50);  // Set brightness to 50%
  pixels.clear();
  setAllPixelsColor(50, 0, 50); // Purple while initializing
  
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

  // Add a small delay for peripheral/transceiver stabilization
  delay(100);

  // --- Motor Discovery --- Loop through possible IDs indefinitely until found
  Serial.println("Attempting to discover motor by polling IDs 0-127 (repeating)... ");
  setAllPixelsColor(100, 100, 0); // Yellow during discovery

  const unsigned long pollingTimeoutPerId = 20; // ms to wait for response from each ID

  while (!motorFound) { // Keep looping until a motor is found
    for (uint8_t id_to_test = 0; id_to_test <= 127; ++id_to_test) {
      // Add a small delay between polling different IDs to avoid flooding bus
      // and allow other tasks (like serial printing) to run.
      if (id_to_test > 0) delay(10); // Increased delay to 10ms
      
      // Print status less aggressively inside the while loop
      if (id_to_test % 16 == 0) { // Print every 16 IDs
          Serial.print("."); 
      }
      // Serial.print("Polling ID: ");
      // Serial.print(id_to_test);

      // Construct Type 0 Query Frame
      CanFrame queryFrame;
      // Type 0x00 (Bits 28-24)
      // Data Area 2 (Bits 23-8): master_id_ in upper byte (as per manual for request frame)
      // Destination Address (Bits 7-0): id_to_test
      queryFrame.id = (static_cast<uint32_t>(0x00) << 24) | 
                        (static_cast<uint32_t>(MASTER_ID) << 16) | // Assuming MASTER_ID in bits 16-23 of ID for consistency with how motor might form it
                        (static_cast<uint32_t>(id_to_test));       // Target motor ID in bits 0-7
      queryFrame.is_extended = true;
      queryFrame.dlc = 0; // Type 0 request has no data payload

      // Print frame details before sending
      Serial.print("  -> Sending Frame: ID=0x");
      Serial.print(queryFrame.id, HEX);
      Serial.print(" DLC=");
      Serial.print(queryFrame.dlc);
      // Serial.print(" Data="); // Data is empty

      if (!canBus.sendFrame(queryFrame)) {
          Serial.println(" - Failed to send query command!");
          delay(1000); // Wait 1 second before trying the next ID
          continue; // Try next ID
      } else {
          Serial.println(" - Query sent. Listening...");
      }

      unsigned long pollingStartTime = millis();
      while (millis() - pollingStartTime < pollingTimeoutPerId) {
          CanFrame receivedFrame;
          if (canBus.receiveFrame(receivedFrame, 0)) { // Poll non-blockingly
              if (receivedFrame.is_extended) {
                  // Check if it's a Type 0 Reply (to us, 0xFE)
                  uint8_t msgType = (receivedFrame.id >> 24) & 0x1F;
                  uint8_t destination_in_reply = receivedFrame.id & 0xFF;

                  if (msgType == 0x00 && destination_in_reply == 0xFE) {
                      // Extract responding motor ID from Data Area 2 (bits 23-8) of the reply ID
                      // The manual states for reply: bit23~8 is target motor CAN_ID
                      uint8_t motor_id_from_reply_data_area_2 = (receivedFrame.id >> 8) & 0xFF;

                      Serial.print("  Got Type 0 Reply, Dest=0xFE, From Motor ID (in data_area_2): ");
                      Serial.println(motor_id_from_reply_data_area_2);

                      // Accept the first valid Type 0 reply we get, regardless of which ID was polled.
                      // The ID in the reply IS the ID of the responding motor.
                      detectedMotorId = motor_id_from_reply_data_area_2;
                      Serial.print("\nMotor Found! Responded to Query with its ID: ");
                      Serial.println(detectedMotorId);

                      // Update the motor object to use this discovered ID
                      motor.setMotorId(detectedMotorId);

                      motorFound = true;
                      break; // Exit the inner listening loop immediately
                  }
              }
          }
          delay(1); // Small delay within listen loop
      } // End listening window for current ID
      
      if (motorFound) break; // Exit the for loop if found in the inner loop
      
    } // End for loop polling IDs
    // If motor still not found, the outer while loop repeats
    Serial.println(); // Newline after a full 0-127 cycle without finding motor
  } // End while(!motorFound)

  // This part is only reached when motorFound is true
  Serial.print("Setup complete. Using detected motor ID: ");
  Serial.println(detectedMotorId);
  setAllPixelsColor(0, 0, 50); // Blue indicates running loop with motor tests
  
  // else { ... } // Failure case removed as loop continues until success

  Serial.println("Setup Finished. Entering loop.");
   if (motorFound) {
      setAllPixelsColor(0, 0, 50); // Blue indicates running loop with motor tests
   } else {
       setAllPixelsColor(50, 0, 0); // Red indicates running loop without motor tests
   }
}

unsigned long last_feedback_print_time = 0;
// const unsigned long FEEDBACK_PRINT_INTERVAL = 500; // No longer using interval

// Function to read the VBUS (bus voltage) from the motor
bool readVBUSVoltage(float &voltage) {
    const uint16_t INDEX_VBUS = 0x701C;  // Bus voltage register from rs03_motor.h
    bool success = false;
    voltage = 0.0f;

    Serial.println("Reading VBUS voltage to verify motor communication...");
    
    // Create a Type 17 frame to read the VBUS parameter
    uint32_t readId = ((0x11) << 24) | (MASTER_ID << 8) | detectedMotorId; // Type 17 (0x11) Read Parameter
    
    uint8_t readData[8] = {0};
    // Parameter index in little-endian
    readData[0] = INDEX_VBUS & 0xFF;        // Low byte of index
    readData[1] = (INDEX_VBUS >> 8) & 0xFF; // High byte of index
    
    Serial.print("Sending VBUS Read command - ID: 0x");
    Serial.print(readId, HEX);
    Serial.print(", Index: 0x");
    Serial.println(INDEX_VBUS, HEX);
    
    // Send the request
    if (CAN.beginExtendedPacket(readId & 0x1FFFFFFF)) {
        CAN.write(readData, 8);
        if (CAN.endPacket()) {
            Serial.println("VBUS Read request sent!");
            
            // Wait for response - looking for Type 11 responses or Type 2 feedback messages
            unsigned long startTime = millis();
            int feedbackCount = 0;
            
            while (millis() - startTime < 500) { // 500ms timeout
                int packetSize = CAN.parsePacket();
                if (packetSize) {
                    uint32_t id = CAN.packetId();
                    // Extract the message type
                    uint8_t responseType = (id >> 24) & 0x1F;
                    
                    Serial.print("Received packet with ID: 0x");
                    Serial.print(id, HEX);
                    Serial.print(", Type: 0x");
                    Serial.println(responseType, HEX);
                    
                    // Read the data payload
                    uint8_t data[8] = {0};
                    for (int i = 0; i < packetSize && i < 8; i++) {
                        data[i] = CAN.read();
                    }
                    
                    // Print data bytes
                    Serial.print("Data: ");
                    for (int i = 0; i < 8; i++) {
                        Serial.print("0x");
                        if (data[i] < 0x10) Serial.print("0");
                        Serial.print(data[i], HEX);
                        Serial.print(" ");
                    }
                    Serial.println();
                    
                    // Check for the Type 0x11 response which directly contains the VBUS value
                    if (responseType == 0x11) { 
                        // Extract parameter index to verify this is for VBUS
                        uint16_t responseIndex = data[0] | (data[1] << 8);
                        
                        if (responseIndex == INDEX_VBUS) {
                            // The VBUS value is likely stored as a float in the last 4 bytes
                            // Try both byte orders (in case of endianness issues)
                            
                            // Standard little endian format (common for ARM)
                            float vbus_le;
                            memcpy(&vbus_le, &data[4], sizeof(float));
                            
                            // Check if bytes are reversed (big endian)
                            uint8_t reversed[4] = {data[7], data[6], data[5], data[4]};
                            float vbus_be;
                            memcpy(&vbus_be, reversed, sizeof(float));
                            
                            Serial.print("VBUS Voltage (LE): ");
                            Serial.print(vbus_le);
                            Serial.print(" V, (BE): ");
                            Serial.print(vbus_be);
                            Serial.println(" V");
                            
                            // Pick the more reasonable value (usually 12-48V)
                            if (vbus_le > 5.0f && vbus_le < 60.0f) {
                                voltage = vbus_le;
                            } else if (vbus_be > 5.0f && vbus_be < 60.0f) {
                                voltage = vbus_be;
                            } else {
                                // Try a different approach - treat as raw bytes
                                uint32_t raw_value;
                                memcpy(&raw_value, &data[4], 4);
                                Serial.print("Raw value: 0x");
                                Serial.println(raw_value, HEX);
                                
                                // Extract as 32-bit integer and convert (division by 100 or 1000?)
                                float div100 = raw_value / 100.0f;
                                float div1000 = raw_value / 1000.0f;
                                Serial.print("As int/100: ");
                                Serial.print(div100);
                                Serial.print(" V, As int/1000: ");
                                Serial.println(div1000);
                                
                                if (div100 > 5.0f && div100 < 60.0f) {
                                    voltage = div100;
                                } else if (div1000 > 5.0f && div1000 < 60.0f) {
                                    voltage = div1000;
                                }
                            }
                            
                            success = true;
                            break;
                        }
                    }
                    else if (responseType == 0x02) { // Type 2 is a feedback message
                        feedbackCount++;
                        
                        // Parse the feedback message to extract values
                        if (packetSize == 8) {
                            // Extract Type 2 feedback data according to format:
                            // Bytes 0-1: Current position (16-bit)
                            // Bytes 2-3: Current velocity (16-bit)
                            // Bytes 4-5: Current torque (16-bit)
                            // Bytes 6-7: Temperature value (16-bit)
                            
                            uint16_t pos_raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
                            uint16_t vel_raw = (static_cast<uint16_t>(data[2]) << 8) | data[3];
                            uint16_t torque_raw = (static_cast<uint16_t>(data[4]) << 8) | data[5];
                            uint16_t temp_raw = (static_cast<uint16_t>(data[6]) << 8) | data[7];
                            
                            float position = motor.uintToFloat(pos_raw, P_MIN, P_MAX, 16);
                            float velocity = motor.uintToFloat(vel_raw, V_MIN, V_MAX, 16);
                            float torque = motor.uintToFloat(torque_raw, T_MIN, T_MAX, 16);
                            float temperature = static_cast<float>(temp_raw) / 10.0f;
                            
                            Serial.print("FB Message: Pos="); Serial.print(position, 4);
                            Serial.print(" Vel="); Serial.print(velocity, 4);
                            Serial.print(" Torq="); Serial.print(torque, 4);
                            Serial.print(" Temp="); Serial.println(temperature, 1);
                            
                            // If we've received at least 3 feedback messages but no VBUS data,
                            // consider communication successful but use a default voltage
                            if (feedbackCount >= 3 && !success) {
                                Serial.println("Received multiple feedback messages - motor is communicating!");
                                Serial.println("WARNING: Using default VBUS value, actual value unknown.");
                                success = true;
                                voltage = 24.0f; // Default voltage assumption
                                // Don't break, keep waiting to see if we get a Type 0x11 response
                            }
                        }
                    }
                }
                delay(1);
            }
            
            if (!success) {
                Serial.println("Failed to receive sufficient motor feedback within timeout");
            }
        } else {
            Serial.println("Failed to send VBUS Read request!");
        }
    } else {
        Serial.println("Failed to begin VBUS Read packet!");
    }
    
    return success;
}

void loop() {
  // Check for received CAN messages
  CanFrame receivedFrame;
  if (canBus.receiveFrame(receivedFrame, 0)) { // Poll for received frames
      Serial.println("* CAN Frame Received *" ); // DEBUG: Indicate any frame reception
      // Attempt to process the frame with the motor library
      if (motor.processFeedback(receivedFrame)) {
          // If it was feedback for our motor, print it immediately
          RS03Motor::Feedback feedback = motor.getLastFeedback();
          Serial.print("FB: ID="); Serial.print(static_cast<int>(feedback.motor_id));
          Serial.print(" M="); Serial.print(feedback.mode); // 0:Reset, 1:Cali, 2:Run
          Serial.print(" E=0x"); Serial.print(feedback.error_flags, HEX);
          Serial.print(" Pos="); Serial.print(feedback.position, 4);
          Serial.print(" Vel="); Serial.print(feedback.velocity, 4);
          Serial.print(" Torq="); Serial.print(feedback.torque, 4);
          Serial.print(" Temp="); Serial.println(feedback.temperature, 1);
          setAllPixelsColor(0, 20, 0);
          delay(50); 
      } else {
          Serial.println("  -> Frame ignored by motor handler."); 
      }
  }

  if (motorFound) {
    // Check VBUS voltage to verify motor communication
    float vbusVoltage = 0.0f;
    bool vbusSuccess = readVBUSVoltage(vbusVoltage);
    
    if (vbusSuccess) {
      Serial.print("MOTOR COMMUNICATION VERIFIED! VBUS = ");
      Serial.print(vbusVoltage);
      Serial.println(" V");
      setAllPixelsColor(0, 50, 0); // Green indicates communication verified
      delay(200);
      setAllPixelsColor(0, 0, 50); // Back to blue for normal operation
    } else {
      Serial.println("WARNING: Could not read VBUS voltage. Motor may not be responding.");
      setAllPixelsColor(50, 50, 0); // Yellow indicates communication issue
      delay(500);
    }

    Serial.println("\n--- Starting Motor Test Sequence Based on Log ---");
    setAllPixelsColor(0, 0, 50); 

    Serial.println("Attempting to Reset Faults...");
    if (!motor.resetFaults()) { Serial.println(" -> Failed!"); }
    delay(200); 

    Serial.println("Attempting to Enable motor...");
    if (!motor.enable()) { Serial.println(" -> Failed!"); }
    delay(200);

    // Set the motor to MIT mode, which is essential for Type 1 commands
    Serial.println("Setting motor to MIT mode...");
    if (!motor.setModeMit()) { Serial.println(" -> Failed to set MIT mode!"); }
    delay(200);

    Serial.println("Attempting to Enable Active Reporting...");
    if (!motor.setActiveReporting(true)) { Serial.println(" -> Failed!"); }
    delay(200); 

    // Add the missing initialization commands from the test tool transcript
    Serial.println("Sending command 129: Query/Ping command...");
    uint32_t cmdId129 = ((0x00) << 24) | (0x07EB << 8) | 0xFC; // Type 0, Data area 0x07EB, Motor ID 0xFC
    uint8_t data129[] = {0x00};
    
    Serial.print("CMD 129 Raw Values - ID: 0x");
    Serial.print(cmdId129, HEX);
    Serial.print(" (Masked to 29-bit: 0x");
    Serial.print(cmdId129 & 0x1FFFFFFF, HEX);
    Serial.print("), Data[");
    Serial.print(sizeof(data129));
    Serial.print("]: ");
    for (size_t i = 0; i < sizeof(data129); i++) {
        Serial.print("0x");
        if (data129[i] < 0x10) Serial.print("0");
        Serial.print(data129[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    if (CAN.beginExtendedPacket(cmdId129 & 0x1FFFFFFF)) {
        CAN.write(data129, 1);
        if (CAN.endPacket()) {
            Serial.println("Command 129 sent successfully!");
        } else {
            Serial.println("Failed to send command 129!");
        }
    } else {
        Serial.println("Failed to begin command 129!");
    }
    delay(100);
    
    Serial.println("Sending command 130: Parameter set command...");
    uint32_t cmdId130 = ((0x20) << 24) | (0x07EB << 8) | 0xFC; // Type 0x20, Data area 0x07EB, Motor ID 0xFC
    uint8_t data130[] = {0x00, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    Serial.print("CMD 130 Raw Values - ID: 0x");
    Serial.print(cmdId130, HEX);
    Serial.print(" (Masked to 29-bit: 0x");
    Serial.print(cmdId130 & 0x1FFFFFFF, HEX);
    Serial.print("), Data[");
    Serial.print(sizeof(data130));
    Serial.print("]: ");
    for (size_t i = 0; i < sizeof(data130); i++) {
        Serial.print("0x");
        if (data130[i] < 0x10) Serial.print("0");
        Serial.print(data130[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    if (CAN.beginExtendedPacket(cmdId130 & 0x1FFFFFFF)) {
        CAN.write(data130, 8);
        if (CAN.endPacket()) {
            Serial.println("Command 130 sent successfully!");
        } else {
            Serial.println("Failed to send command 130!");
        }
    } else {
        Serial.println("Failed to begin command 130!");
    }
    delay(100);
    
    Serial.println("Sending command 131: Second query command...");
    uint32_t cmdId131 = ((0x00) << 24) | (0x07E8 << 8) | 0x44; // Type 0, Data area 0x07E8, ID 0x44
    uint8_t data131[] = {0x00};
    
    Serial.print("CMD 131 Raw Values - ID: 0x");
    Serial.print(cmdId131, HEX);
    Serial.print(" (Masked to 29-bit: 0x");
    Serial.print(cmdId131 & 0x1FFFFFFF, HEX);
    Serial.print("), Data[");
    Serial.print(sizeof(data131));
    Serial.print("]: ");
    for (size_t i = 0; i < sizeof(data131); i++) {
        Serial.print("0x");
        if (data131[i] < 0x10) Serial.print("0");
        Serial.print(data131[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    if (CAN.beginExtendedPacket(cmdId131 & 0x1FFFFFFF)) {
        CAN.write(data131, 1);
        if (CAN.endPacket()) {
            Serial.println("Command 131 sent successfully!");
        } else {
            Serial.println("Failed to send command 131!");
        }
    } else {
        Serial.println("Failed to begin command 131!");
    }
    delay(1000); // Slightly longer delay before the JOG command

    // This is based on the log entry at line 132 which seems to be the successful JOG FORWARD command
    // The command appears to be: 41 54 90 07 eb fc 08 05 70 00 00 07 01 86 65
    Serial.println("Sending JOG FORWARD command using EXACT log values...");
    
    // Lets try a few different ID formats to see which one works
    
    // Original format in logs: 0x9007EBFC - but won't fit in 29 bits
    // Our previous attempt:    0x1007EB7F - using 0x10 instead of 0x90
    
    // Version 1: Just the lower 24 bits, 0x10 for type (fits in 29 bits)
    uint32_t jogId1 = 0x1007EB00 | detectedMotorId;
    // Version 2: Using 0x9 instead of 0x90 as command type
    uint32_t jogId2 = 0x907EB00 | detectedMotorId;
    // Version 3: Using 0x90 but truncated to fit in 29 bits (may be invalid)
    uint32_t jogId3 = (0x90 << 20) | (0x07EB << 8) | detectedMotorId;
    // Version 4: Using just the lower 3 bits of command type (0x0)
    uint32_t jogId4 = (0x90 & 0x07) << 24 | (0x07EB << 8) | detectedMotorId;
    
    // Try with jogId3 first
    uint32_t jogId = jogId3;
    
    Serial.println("Trying multiple ID variants to match log format:");
    Serial.print("Version 1 (0x10 type): 0x"); Serial.println(jogId1, HEX);
    Serial.print("Version 2 (0x9 type):  0x"); Serial.println(jogId2, HEX);
    Serial.print("Version 3 (0x90 truncated): 0x"); Serial.println(jogId3, HEX);
    Serial.print("Version 4 (0x90 & 0x07): 0x"); Serial.println(jogId4, HEX);
    Serial.print("Using ID Version 3: 0x"); Serial.println(jogId, HEX);
    
    // Use exact data values from the log
    uint8_t data_fwd[] = {
        0x05, 0x70,  // Position from log: 05 70
        0x00, 0x00,  // Zero values from log
        0x07, 0x01,  // Values from log
        0x86, 0x65   // Velocity from log: 86 65
    };
    
    Serial.print("JOG Raw Values - ID: 0x");
    Serial.print(jogId, HEX);
    Serial.print(" (Masked to 29-bit: 0x");
    Serial.print(jogId & 0x1FFFFFFF, HEX);
    Serial.print("), Data[");
    Serial.print(sizeof(data_fwd));
    Serial.print("]: ");
    for (size_t i = 0; i < sizeof(data_fwd); i++) {
        Serial.print("0x");
        if (data_fwd[i] < 0x10) Serial.print("0");
        Serial.print(data_fwd[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Try each of the jogId variants
    for (int variant = 1; variant <= 4; variant++) {
        uint32_t currentJogId;
        switch (variant) {
            case 1: currentJogId = jogId1; break;
            case 2: currentJogId = jogId2; break;
            case 3: currentJogId = jogId3; break;
            case 4: currentJogId = jogId4; break;
            default: currentJogId = jogId1;
        }
        
        Serial.println("\n==============================================");
        Serial.print("TESTING JOG VARIANT ");
        Serial.print(variant);
        Serial.print(" of 4: 0x");
        Serial.println(currentJogId, HEX);
        Serial.println("==============================================");
        
        // Visual indication on NeoPixel
        switch (variant) {
            case 1: setAllPixelsColor(50, 0, 0); break;  // Red for variant 1
            case 2: setAllPixelsColor(50, 50, 0); break; // Yellow for variant 2
            case 3: setAllPixelsColor(0, 50, 0); break;  // Green for variant 3
            case 4: setAllPixelsColor(0, 0, 50); break;  // Blue for variant 4
        }
        
        // Try direct approach first
        if (CAN.beginExtendedPacket(currentJogId & 0x1FFFFFFF)) {
            CAN.write(data_fwd, 8);
            if (CAN.endPacket()) {
                Serial.print("JOG variant ");
                Serial.print(variant);
                Serial.println(" sent successfully via direct method!");
            } else {
                Serial.print("Failed to end JOG variant ");
                Serial.print(variant);
                Serial.println(" packet!");
            }
        } else {
            Serial.print("Failed to begin JOG variant ");
            Serial.print(variant);
            Serial.println(" packet!");
        }
        
        // Wait longer between variants for observation
        Serial.println("Waiting 1 second to observe if motor moves...");
        delay(1000); // Increased to 1 second

        // Check for any feedback from the motor
        Serial.println("Checking for motor feedback...");
        unsigned long checkStartTime = millis();
        bool feedbackReceived = false;
        
        while (millis() - checkStartTime < 500) { // Check for feedback for 500ms
            int packetSize = CAN.parsePacket();
            if (packetSize) {
                uint32_t id = CAN.packetId();
                uint8_t responseType = (id >> 24) & 0x1F;
                
                Serial.print("Received packet with ID: 0x");
                Serial.print(id, HEX);
                Serial.print(", Type: 0x");
                Serial.println(responseType, HEX);
                
                // Read and print data
                uint8_t data[8] = {0};
                for (int i = 0; i < packetSize && i < 8; i++) {
                    data[i] = CAN.read();
                }
                
                Serial.print("Data: ");
                for (int i = 0; i < 8; i++) {
                    Serial.print("0x");
                    if (data[i] < 0x10) Serial.print("0");
                    Serial.print(data[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                
                feedbackReceived = true;
            }
            delay(1);
        }
        
        if (!feedbackReceived) {
            Serial.println("No feedback received from motor during check period.");
        }
        
        // Additional delay before next variant
        delay(1000); // Wait another second before the next variant
    }
    
    setAllPixelsColor(0, 0, 50); // Back to blue for normal operation
    Serial.println("Waiting 4 seconds for forward motion...");
    delay(4000);
    
    // This is based on log entry at line 133 which appears to be the STOP command
    // The command appears to be: 41 54 90 07 eb fc 08 05 70 00 00 07 00 7f ff
    Serial.println("Sending STOP command using EXACT log values...");
    
    uint8_t data_stop[] = {
        0x05, 0x70,  // Position from log: 05 70
        0x00, 0x00,  // Zero values from log
        0x07, 0x00,  // Values from log 
        0x7F, 0xFF   // Stop velocity from log: 7F FF
    };
    
    Serial.print("STOP Raw Values - Data[");
    Serial.print(sizeof(data_stop));
    Serial.print("]: ");
    for (size_t i = 0; i < sizeof(data_stop); i++) {
        Serial.print("0x");
        if (data_stop[i] < 0x10) Serial.print("0");
        Serial.print(data_stop[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Try each of the jogId variants for STOP too
    for (int variant = 1; variant <= 4; variant++) {
        uint32_t currentStopId;
        switch (variant) {
            case 1: currentStopId = jogId1; break;  // reuse the same ID variants
            case 2: currentStopId = jogId2; break;
            case 3: currentStopId = jogId3; break;
            case 4: currentStopId = jogId4; break;
            default: currentStopId = jogId1;
        }
        
        // Only try the variant that worked for JOG (assuming it was successful)
        Serial.print("Trying STOP with ID variant ");
        Serial.print(variant);
        Serial.print(": 0x");
        Serial.println(currentStopId, HEX);
        
        // Try direct approach
        if (CAN.beginExtendedPacket(currentStopId & 0x1FFFFFFF)) {
            CAN.write(data_stop, 8);
            if (CAN.endPacket()) {
                Serial.print("STOP variant ");
                Serial.print(variant);
                Serial.println(" sent successfully!");
            } else {
                Serial.print("Failed to end STOP variant ");
                Serial.print(variant);
                Serial.println(" packet!");
            }
        } else {
            Serial.print("Failed to begin STOP variant ");
            Serial.print(variant);
            Serial.println(" packet!");
        }
        
        delay(500); // Short delay between variants
    }
    
    Serial.println("Waiting 2 seconds after stopping...");
    delay(2000);
    
    // Try another movement command based on line 134
    // 41 54 90 07 eb fc 08 05 70 00 00 07 01 79 99
    Serial.println("Sending another JOG command using EXACT log values...");
    
    uint8_t data_fwd2[] = {
        0x05, 0x70,  // Position from log: 05 70
        0x00, 0x00,  // Zero values from log
        0x07, 0x01,  // Values from log
        0x79, 0x99   // Second velocity from log: 79 99
    };
    
    Serial.println("Trying sendRawFrame for second JOG command...");
    if (!motor.sendRawFrame(jogId, 8, data_fwd2)) {
        Serial.println(" -> SendRaw Failed!");
        
        // Try direct approach 
        Serial.println("Trying direct CAN approach for second JOG...");
        if (CAN.beginExtendedPacket(jogId)) {
            CAN.write(data_fwd2, 8);
            if (CAN.endPacket()) {
                Serial.println("Direct second JOG transmission successful!");
            } else {
                Serial.println("Direct second JOG end packet failed!");
            }
        } else {
            Serial.println("Direct second JOG begin packet failed!");
        }
    } else {
        Serial.println("Second JOG SendRaw succeeded!");
    }
    
    Serial.println("Waiting 4 seconds for second motion...");
    delay(4000);
    
    // Final stop command based on line 135
    Serial.println("Sending final STOP command...");
    
    Serial.println("Trying sendRawFrame for final STOP command...");
    if (!motor.sendRawFrame(jogId, 8, data_stop)) {
        Serial.println(" -> SendRaw Failed!");
        
        // Try direct approach
        Serial.println("Trying direct CAN approach for final STOP...");
        if (CAN.beginExtendedPacket(jogId)) {
            CAN.write(data_stop, 8);
            if (CAN.endPacket()) {
                Serial.println("Direct final STOP transmission successful!");
            } else {
                Serial.println("Direct final STOP end packet failed!");
            }
        } else {
            Serial.println("Direct final STOP begin packet failed!");
        }
    } else {
        Serial.println("Final STOP SendRaw succeeded!");
    }
    
    Serial.println("--- Motor Test Sequence Complete. Restarting in 5 seconds --- ");
    delay(5000);
    
  } else {
    delay(1000); 
    setAllPixelsColor(50, 0, 0);
  }
  delay(1); 
}

// Helper function to set all pixels to the same color
void setAllPixelsColor(int red, int green, int blue) {
  pixels.setPixelColor(0, pixels.Color(red, green, blue));
  pixels.show();  // Send the updated pixel colors to the hardware
}

// Note: Removed sendTestCanMessages() function as it's replaced by motor control
// Note: Removed the C++ style main() function entirely. 

// Note: removed C++ style main function entirely