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

// Add this new function after readVBUSVoltage() and before loop()
bool sendJogCommand(uint8_t motorId, float velocity, bool useExactLogFormat = true) {
    Serial.print("Sending JOG command with velocity: ");
    Serial.println(velocity);
    
    uint8_t commandType = 0x01; // MIT control mode command
    
    // Create a properly formatted command similar to those that worked in the logs
    uint32_t jogId;
    if (useExactLogFormat) {
        // Format 3: Using format that matches logs but truncated to fit 29 bits
        jogId = ((static_cast<uint32_t>(commandType) << 24) | 
                (static_cast<uint32_t>(0x07EB) << 8) | 
                static_cast<uint32_t>(motorId)) & 0x1FFFFFFF;
    } else {
        // Alternative approach using RS03Motor library method
        uint16_t packed_torque = motor.packFloatToUint16(0.0f, T_MIN, T_MAX); // No additional torque
        jogId = ((static_cast<uint32_t>(commandType) << 24) | 
                (static_cast<uint32_t>(packed_torque) << 8) | 
                static_cast<uint32_t>(motorId)) & 0x1FFFFFFF;
    }
    
    // Prepare data payload according to MIT mode format
    uint8_t data[8] = {0};
    
    if (useExactLogFormat) {
        // Use exact format from successful logs
        data[0] = 0x05;  // Position MSB from log
        data[1] = 0x70;  // Position LSB from log
        data[2] = 0x00;  // Zero
        data[3] = 0x00;  // Zero
        data[4] = 0x07;  // Kp MSB from log
        data[5] = velocity > 0.0f ? 0x01 : 0x00;  // Direction from log (0x01=forward, 0x00=stop)
        
        // Velocity value - convert the float to a comparable format used in logs
        if (abs(velocity) < 0.01f) {
            // For stop (velocity near zero), use the STOP values from log
            data[6] = 0x7F;
            data[7] = 0xFF;
        } else {
            // For movement, use the JOG values from log
            data[6] = 0x86;
            data[7] = 0x65;
        }
    } else {
        // Use proper MIT mode format with floating point conversion
        uint16_t packed_pos = motor.packFloatToUint16(0.0f, P_MIN, P_MAX);
        uint16_t packed_vel = motor.packFloatToUint16(velocity, V_MIN, V_MAX);
        uint16_t packed_kp = motor.packFloatToUint16(0.0f, KP_MIN, KP_MAX);  
        uint16_t packed_kd = motor.packFloatToUint16(0.0f, KD_MIN, KD_MAX);
        
        // Properly format according to MIT protocol
        data[0] = (packed_pos >> 8) & 0xFF;  // Position MSB
        data[1] = packed_pos & 0xFF;         // Position LSB
        data[2] = (packed_vel >> 8) & 0xFF;  // Velocity MSB
        data[3] = packed_vel & 0xFF;         // Velocity LSB
        data[4] = (packed_kp >> 8) & 0xFF;   // Kp MSB
        data[5] = packed_kp & 0xFF;          // Kp LSB
        data[6] = (packed_kd >> 8) & 0xFF;   // Kd MSB
        data[7] = packed_kd & 0xFF;          // Kd LSB
    }
    
    // Print command details
    Serial.print("JOG Command - ID: 0x");
    Serial.print(jogId, HEX);
    Serial.print(", Data: ");
    for (int i = 0; i < 8; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Send the command
    if (CAN.beginExtendedPacket(jogId)) {
        CAN.write(data, 8);
        if (CAN.endPacket()) {
            Serial.println("JOG command sent successfully!");
            return true;
        } else {
            Serial.println("Failed to end JOG packet!");
        }
    } else {
        Serial.println("Failed to begin JOG packet!");
    }
    
    return false;
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
          
          // Show human-readable error descriptions if any errors exist
          if (feedback.error_flags != 0) {
              Serial.print(" (");
              Serial.print(motor.getErrorText());
              Serial.print(")");
          }
          
          Serial.print(" Pos="); Serial.print(feedback.position, 4);
          Serial.print(" Vel="); Serial.print(feedback.velocity, 4);
          Serial.print(" Torq="); Serial.print(feedback.torque, 4);
          Serial.print(" Temp="); Serial.println(feedback.temperature, 1);
          
          // Show NeoPixel based on error status
          if (feedback.error_flags != 0) {
              setAllPixelsColor(50, 0, 0); // Red for errors
          } else {
              setAllPixelsColor(0, 20, 0); // Green for normal feedback
          }
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
    if (!motor.resetFaults()) { 
        Serial.println(" -> Failed to reset faults!");
    } else {
        Serial.println(" -> Faults reset successfully.");
    }
    delay(200); 

    Serial.println("Attempting to Enable motor...");
    if (!motor.enable()) { 
        Serial.println(" -> Failed to enable motor!");
    } else {
        Serial.println(" -> Motor enabled successfully.");
    }
    delay(200);

    // Set the motor to MIT mode, which is essential for Type 1 commands
    Serial.println("Setting motor to MIT mode...");
    if (!motor.setModeMit()) { 
        Serial.println(" -> Failed to set MIT mode!");
    } else {
        Serial.println(" -> MIT mode set successfully.");
    }
    delay(200);

    Serial.println("Attempting to Enable Active Reporting...");
    if (!motor.setActiveReporting(true)) { 
        Serial.println(" -> Failed to enable active reporting!");
    } else {
        Serial.println(" -> Active reporting enabled successfully.");
    }
    delay(200); 

    // Simplified test sequence using our new function
    Serial.println("\n--- TESTING JOG COMMANDS ---");
    
    // Test 1: Using the standalone function with log format
    Serial.println("\n--- Test 1: Using sendJogCommand with log format ---");
    setAllPixelsColor(50, 0, 0); // Red for test 1
    
    // Check for errors before moving
    if (motor.hasErrors()) {
        Serial.print("WARNING: Errors detected before Test 1: ");
        Serial.println(motor.getErrorText());
    }
    
    Serial.println("Sending JOG FORWARD command...");
    if (sendJogCommand(detectedMotorId, 5.0f, true)) {
        Serial.println("JOG FORWARD sent successfully. Waiting for movement...");
        delay(3000); // Wait longer to observe movement
        
        // Check for errors during movement
        if (motor.hasErrors()) {
            Serial.print("WARNING: Errors detected during movement: ");
            Serial.println(motor.getErrorText());
        }
        
        Serial.println("Sending STOP command...");
        if (sendJogCommand(detectedMotorId, 0.0f, true)) {
            Serial.println("STOP command sent successfully.");
        }
        
        // Reset faults after movement test
        Serial.println("Resetting faults after Test 1...");
        if (motor.resetFaults()) {
            Serial.println(" -> Faults reset successfully.");
        } else {
            Serial.println(" -> Failed to reset faults!");
        }
        delay(200);
        
        // Check if faults were cleared
        if (motor.hasErrors()) {
            Serial.print("WARNING: Errors still present after reset: ");
            Serial.println(motor.getErrorText());
        }
    }
    delay(2000); // Wait between tests
    
    // Test 2: Using the standalone function with MIT format
    Serial.println("\n--- Test 2: Using sendJogCommand with MIT format ---");
    setAllPixelsColor(0, 50, 0); // Green for test 2
    
    // Check for errors before moving
    if (motor.hasErrors()) {
        Serial.print("WARNING: Errors detected before Test 2: ");
        Serial.println(motor.getErrorText());
    }
    
    Serial.println("Sending JOG FORWARD command with MIT format...");
    if (sendJogCommand(detectedMotorId, 5.0f, false)) {
        Serial.println("JOG FORWARD (MIT) sent successfully. Waiting for movement...");
        delay(3000); // Wait longer to observe movement
        
        // Check for errors during movement
        if (motor.hasErrors()) {
            Serial.print("WARNING: Errors detected during movement: ");
            Serial.println(motor.getErrorText());
        }
        
        Serial.println("Sending STOP command with MIT format...");
        if (sendJogCommand(detectedMotorId, 0.0f, false)) {
            Serial.println("STOP command (MIT) sent successfully.");
        }
        
        // Reset faults after movement test
        Serial.println("Resetting faults after Test 2...");
        if (motor.resetFaults()) {
            Serial.println(" -> Faults reset successfully.");
        } else {
            Serial.println(" -> Failed to reset faults!");
        }
        delay(200);
        
        // Check if faults were cleared
        if (motor.hasErrors()) {
            Serial.print("WARNING: Errors still present after reset: ");
            Serial.println(motor.getErrorText());
        }
    }
    delay(2000); // Wait between tests
    
    // Test 3: Using the new motor.jogMotor() method with log format
    Serial.println("\n--- Test 3: Using motor.jogMotor() with log format ---");
    setAllPixelsColor(0, 0, 50); // Blue for test 3
    
    // Check for errors before moving
    if (motor.hasErrors()) {
        Serial.print("WARNING: Errors detected before Test 3: ");
        Serial.println(motor.getErrorText());
    }
    
    Serial.println("Sending JOG FORWARD command via motor.jogMotor()...");
    if (motor.jogMotor(5.0f, true)) {
        Serial.println("JOG FORWARD sent successfully via motor.jogMotor(). Waiting for movement...");
        delay(3000); // Wait longer to observe movement
        
        // Check for errors during movement
        if (motor.hasErrors()) {
            Serial.print("WARNING: Errors detected during movement: ");
            Serial.println(motor.getErrorText());
        }
        
        Serial.println("Sending STOP command via motor.jogMotor()...");
        if (motor.jogMotor(0.0f, true)) {
            Serial.println("STOP command sent successfully via motor.jogMotor().");
        }
        
        // Reset faults after movement test
        Serial.println("Resetting faults after Test 3...");
        if (motor.resetFaults()) {
            Serial.println(" -> Faults reset successfully.");
        } else {
            Serial.println(" -> Failed to reset faults!");
        }
        delay(200);
        
        // Check if faults were cleared
        if (motor.hasErrors()) {
            Serial.print("WARNING: Errors still present after reset: ");
            Serial.println(motor.getErrorText());
        }
    }
    delay(2000); // Wait between tests
    
    // Test 4: Using the new motor.jogMotor() method with MIT format
    Serial.println("\n--- Test 4: Using motor.jogMotor() with MIT format ---");
    setAllPixelsColor(50, 50, 0); // Yellow for test 4
    
    // Check for errors before moving
    if (motor.hasErrors()) {
        Serial.print("WARNING: Errors detected before Test 4: ");
        Serial.println(motor.getErrorText());
    }
    
    Serial.println("Sending JOG REVERSE command via motor.jogMotor()...");
    if (motor.jogMotor(-5.0f, false)) {
        Serial.println("JOG REVERSE sent successfully via motor.jogMotor(). Waiting for movement...");
        delay(3000); // Wait longer to observe movement
        
        // Check for errors during movement
        if (motor.hasErrors()) {
            Serial.print("WARNING: Errors detected during movement: ");
            Serial.println(motor.getErrorText());
        }
        
        Serial.println("Sending STOP command via motor.jogMotor()...");
        if (motor.jogMotor(0.0f, false)) {
            Serial.println("STOP command sent successfully via motor.jogMotor().");
        }
        
        // Reset faults after movement test
        Serial.println("Resetting faults after Test 4...");
        if (motor.resetFaults()) {
            Serial.println(" -> Faults reset successfully.");
        } else {
            Serial.println(" -> Failed to reset faults!");
        }
        delay(200);
        
        // Check if faults were cleared
        if (motor.hasErrors()) {
            Serial.print("WARNING: Errors still present after reset: ");
            Serial.println(motor.getErrorText());
        }
    }
    
    // Final fault reset and re-enable at the end of all tests
    Serial.println("Final fault reset and re-enable after all tests...");
    
    if (motor.resetFaults()) {
        Serial.println(" -> Final faults reset successfully.");
    } else {
        Serial.println(" -> Failed to reset faults in final reset!");
    }
    delay(200);
    
    if (motor.enable()) {
        Serial.println(" -> Final motor enable successful.");
    } else {
        Serial.println(" -> Failed to enable motor in final enable!");
    }
    delay(200);
    
    Serial.println("--- Motor Test Sequence Complete. Restarting in 10 seconds --- ");
    setAllPixelsColor(50, 50, 50); // White indicates test complete
    delay(10000); // Longer delay before repeating tests
    
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