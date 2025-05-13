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

// Define Message RAM size for CAN1 (CAN0 not used)
#define CAN0_MESSAGE_RAM_SIZE (0)
#define CAN1_MESSAGE_RAM_SIZE (1728)

#include <ACANFD_FeatherM4CAN.h>

#define PIN        8  // Which pin on the Arduino is connected to the NeoPixels
#define NUMPIXELS 1  // How many NeoPixels are attached to the Arduino
#define DELAYVAL 500  // Time (in milliseconds) to pause between colors

// Initialize the NeoPixel strip
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

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
        CANFDMessage msg_to_send;
        msg_to_send.id = frame.id;
        msg_to_send.ext = frame.is_extended;
        msg_to_send.len = frame.dlc;
        memcpy(msg_to_send.data, frame.data, frame.dlc);

        // Use the global can1 object from ACANFD_FeatherM4CAN library
        const uint32_t status = can1.tryToSendReturnStatusFD(msg_to_send);
        if (status == 0) {
            // Serial.println("CAN frame sent successfully.");
            return true;
        } else {
            Serial.print("Failed to send CAN frame, error: 0x");
            Serial.println(status, HEX);
            return false;
        }
    }

    // Optional: Implement receive if needed for direct polling
    bool receiveFrame(CanFrame& frame, uint32_t /*timeout_ms*/) override {
         CANFDMessage received_msg;
         if (can1.receiveFD0(received_msg)) { // Check FIFO 0
            frame.id = received_msg.id;
            frame.is_extended = received_msg.ext;
            frame.dlc = received_msg.len;
            memcpy(frame.data, received_msg.data, received_msg.len);
            // Zero out remaining bytes if dlc < 8
            if (frame.dlc < 8) {
                memset(frame.data + frame.dlc, 0, 8 - frame.dlc);
            }
            return true;
         } else if (can1.receiveFD1(received_msg)) { // Check FIFO 1 (if used)
             frame.id = received_msg.id;
             frame.is_extended = received_msg.ext;
             frame.dlc = received_msg.len;
             memcpy(frame.data, received_msg.data, received_msg.len);
             if (frame.dlc < 8) {
                 memset(frame.data + frame.dlc, 0, 8 - frame.dlc);
             }
             return true;
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
  // Using 48MHz clock, 1Mbit/s, Factor 1 (assuming CAN FD capable transceiver)
  ACANFD_FeatherM4CAN_Settings settings(ACANFD_FeatherM4CAN_Settings::CLOCK_48MHz, 1000 * 1000, DataBitRateFactor::x1);
  settings.mDriverTransmitFIFOSize = 5; // Example FIFO size
  settings.mDriverReceiveFIFO0Size = 10;
  settings.mDriverReceiveFIFO1Size = 0;

  // For testing without actual CAN devices, you can use loopback mode:
  // settings.mModuleMode = ACANFD_FeatherM4CAN_Settings::INTERNAL_LOOP_BACK;
  // settings.mModuleMode = ACANFD_FeatherM4CAN_Settings::EXTERNAL_LOOP_BACK;

  // Start CAN bus
  const uint32_t errorCode = can1.beginFD(settings);
  
  if (errorCode == 0) {
    Serial.println("CAN initialized successfully at 1 Mbit/s");
    // setAllPixelsColor(0, 50, 0);  // Green indicates success - will be set yellow for discovery
    // delay(500);
  } else {
    Serial.print("Error initializing CAN: 0x");
    Serial.println(errorCode, HEX);
    setAllPixelsColor(50, 0, 0);  // Red indicates error
    while(1); // Halt on CAN error
  }

  // Removed non-compiling checks for isBusOff() and error counters
  Serial.println("CAN configured. Adding delay before discovery...");

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
          // Removed non-compiling checks for isBusOff() and error counters
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
    Serial.println("\n--- Starting Motor RAW FRAME Test (Type 1 with Velocity) ---");
    setAllPixelsColor(0, 0, 50); 

    Serial.println("Attempting to Reset Faults...");
    if (!motor.resetFaults()) { Serial.println(" -> Failed!"); }
    delay(200); 

    Serial.println("Attempting to Enable motor...");
    if (!motor.enable()) { Serial.println(" -> Failed!"); }
    delay(200);

    Serial.println("Attempting to Enable Active Reporting...");
    if (!motor.setActiveReporting(true)) { Serial.println(" -> Failed!"); }
    delay(200); 

    // Construct the Type 1 CAN ID: Type (0x01) | Packed Torque (16-bit) | Motor ID
    // Using a nominal torque of 5.0 Nm for the ID (packs to 0x8AAA).
    uint16_t packed_torque_for_id = motor.packFloatToUint16(5.0f, T_MIN, T_MAX);
    uint32_t type1_can_id = (0x01UL << 24) | (static_cast<uint32_t>(packed_torque_for_id) << 8) | detectedMotorId;

    // Data for Type 1: Pos (2B), Vel (2B), Kp (2B), Kd (2B) - Big Endian
    uint16_t pos_raw = motor.packFloatToUint16(0.0f, P_MIN, P_MAX); // Target position 0
    uint16_t kp_raw  = motor.packFloatToUint16(50.0f, KP_MIN, KP_MAX); // Example Kp
    uint16_t kd_raw  = motor.packFloatToUint16(1.0f, KD_MIN, KD_MAX);  // Example Kd

    // --- JOG FORWARD ---
    Serial.println("Sending RAW Type 1 JOG FORWARD (+1.0 rad/s)...");
    uint16_t vel_fwd_raw = motor.packFloatToUint16(1.0f, V_MIN, V_MAX);
    uint8_t data_fwd[] = {
        (uint8_t)(pos_raw >> 8), (uint8_t)pos_raw,
        (uint8_t)(vel_fwd_raw >> 8), (uint8_t)vel_fwd_raw,
        (uint8_t)(kp_raw >> 8), (uint8_t)kp_raw,
        (uint8_t)(kd_raw >> 8), (uint8_t)kd_raw
    };
    if (!motor.sendRawFrame(type1_can_id, 8, data_fwd)) { Serial.println(" -> SendRaw Failed!"); }
    Serial.println("Waiting 4 seconds...");
    delay(4000); 

    // --- STOP ---
    Serial.println("Sending RAW Type 1 STOP (0.0 rad/s)...");
    // A true stop velocity for floatToUint(0.0, -20, 20, 16) is 32767 (0x7FFF)
    uint16_t vel_stop_raw = 0x7FFF; 
    
    uint8_t data_stop[] = {
        (uint8_t)(pos_raw >> 8), (uint8_t)pos_raw,
        (uint8_t)(vel_stop_raw >> 8), (uint8_t)vel_stop_raw,
        (uint8_t)(kp_raw >> 8), (uint8_t)kp_raw,
        (uint8_t)(kd_raw >> 8), (uint8_t)kd_raw
    };
    if (!motor.sendRawFrame(type1_can_id, 8, data_stop)) { Serial.println(" -> SendRaw Failed!"); }
    Serial.println("Waiting 2 seconds...");
    delay(2000);
    
    Serial.println("--- Motor RAW FRAME Type 1 JOG Test Finished. Restarting Loop --- ");
    delay(2000); 
    
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