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
    setAllPixelsColor(0, 50, 0);  // Green indicates success
    delay(500);
  } else {
    Serial.print("Error initializing CAN: 0x");
    Serial.println(errorCode, HEX);
    setAllPixelsColor(50, 0, 0);  // Red indicates error
    while(1); // Halt on CAN error
  }

  // --- Motor Discovery --- Loop through possible IDs indefinitely until found
  Serial.println("Attempting to discover motor by polling IDs 0-127 (repeating)... ");
  setAllPixelsColor(100, 100, 0); // Yellow during discovery

  const unsigned long pollingTimeoutPerId = 20; // ms to wait for response from each ID

  while (!motorFound) { // Keep looping until a motor is found
    for (uint8_t id_to_test = 0; id_to_test <= 127; ++id_to_test) {
      // Add a small delay between polling different IDs to avoid flooding bus
      // and allow other tasks (like serial printing) to run.
      if (id_to_test > 0) delay(5); 
      
      // Print status less aggressively inside the while loop
      if (id_to_test % 16 == 0) { // Print every 16 IDs
          Serial.print("."); 
      }
      // Serial.print("Polling ID: ");
      // Serial.print(id_to_test);

      // Construct specific enable frame (Type 3, Target id_to_test, Master MASTER_ID)
      CanFrame enableFrame;
      enableFrame.id = (static_cast<uint32_t>(0x03) << 24) | (static_cast<uint32_t>(MASTER_ID) << 8) | id_to_test;
      enableFrame.is_extended = true;
      enableFrame.dlc = 0;

      if (!canBus.sendFrame(enableFrame)) {
          Serial.println(" - Failed to send enable command!");
          continue; // Try next ID
      } else {
          Serial.println(" - Enable sent. Listening...");
      }

      unsigned long pollingStartTime = millis();
      while (millis() - pollingStartTime < pollingTimeoutPerId) {
          CanFrame receivedFrame;
          if (canBus.receiveFrame(receivedFrame, 0)) { // Poll non-blockingly
              if (receivedFrame.is_extended) {
                  // Check if it's a Feedback frame (Type 2)
                  uint8_t msgType = (receivedFrame.id >> 24) & 0x1F;
                  if (msgType == 0x02) {
                      // Extract responding motor ID from bits 8-15
                      uint8_t respondingId = (receivedFrame.id >> 8) & 0xFF;
                      // Check if the response is from the ID we just polled
                      if (respondingId == id_to_test) {
                           detectedMotorId = respondingId;
                           Serial.print("\nMotor Found! Responded with ID: ");
                           Serial.println(detectedMotorId);

                           // Update the motor object to use this discovered ID
                           motor.setMotorId(detectedMotorId);

                           motorFound = true;
                           // goto discovery_done; // Exit the outer loop - No longer needed, while loop will exit
                           break; // Exit the inner listening loop immediately
                      } else {
                           // Optional: Log if another motor responded unexpectedly
                           // Serial.print("  (Ignoring response from ID: "); Serial.print(respondingId); Serial.println(")");
                      }
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
const unsigned long FEEDBACK_PRINT_INTERVAL = 500; // Print feedback every 500ms

void loop() {
  // Check for received CAN messages
  CanFrame receivedFrame;
  if (canBus.receiveFrame(receivedFrame, 0)) { // Poll for received frames
      // Attempt to process the frame with the motor library
      if (motor.processFeedback(receivedFrame)) {
          // If it was feedback for our motor, print it periodically
          unsigned long now = millis();
          if (now - last_feedback_print_time >= FEEDBACK_PRINT_INTERVAL) {
              last_feedback_print_time = now;
              RS03Motor::Feedback feedback = motor.getLastFeedback();
              Serial.print("Feedback: ID="); Serial.print(static_cast<int>(feedback.motor_id));
              Serial.print(" Pos="); Serial.print(feedback.position, 4);
              Serial.print(" Vel="); Serial.print(feedback.velocity, 4);
              Serial.print(" Torq="); Serial.print(feedback.torque, 4);
              Serial.print(" Temp="); Serial.print(feedback.temperature, 1);
              Serial.print(" Err=0x"); Serial.print(feedback.error_flags, HEX);
              Serial.println();
              // Set pixel green briefly on successful feedback decode
              setAllPixelsColor(0, 20, 0);
              delay(50);
              setAllPixelsColor(0, 0, 50); // Back to blue
          }
      } else {
         // Optional: Print messages not processed by the motor instance
         // Serial.print("Received other CAN frame: ID=0x"); Serial.println(receivedFrame.id, HEX);
         // setAllPixelsColor(20, 20, 0); // Yellow for other messages
         // delay(50);
         // setAllPixelsColor(0, 0, 50); // Back to blue
      }
  }

  // Keep Neopixel blue while running
  // (Handled by feedback section now)

  // Add any other loop tasks here
  // --- Repeating Motor Test Sequence ---
  if (motorFound) {
    Serial.println("\n--- Starting Motor Test Sequence ---");

    Serial.println("Attempting to Enable motor...");
    if (motor.enable()) {
        Serial.println("Enable command sent successfully.");
    } else {
        Serial.println("Failed to send enable command!");
    }
    delay(1000); // Wait for motor to potentially enable/initialize

    Serial.println("Attempting to Set Velocity Mode...");
    if (motor.setModeVelocity()) { // Using default limits
        Serial.println("Set Velocity Mode command sent successfully.");
    } else {
        Serial.println("Failed to send set velocity mode command!");
    }
    delay(200); // Allow time for mode change

    float test_velocity = 1.0f; // rad/s
    Serial.print("Attempting to Set velocity to "); Serial.print(test_velocity); Serial.println(" rad/s...");
    if (motor.setVelocity(test_velocity)) {
        Serial.println("Set velocity command sent successfully.");
    } else {
        Serial.println("Failed to send set velocity command!");
    }
    delay(2000); // Run for 2 seconds

    Serial.println("Attempting to Set velocity to 0 rad/s...");
    if (motor.setVelocity(0.0f)) {
        Serial.println("Set velocity command sent successfully.");
    } else {
        Serial.println("Failed to send set velocity command!");
    }
    delay(1000);

    Serial.println("Attempting to Set Position Mode (CSP)...");
    float csp_speed_limit = 3.0f; // rad/s
    float csp_current_limit = 15.0f; // Amps
    if (motor.setModePositionCSP(csp_speed_limit, csp_current_limit)) {
         Serial.println("Set Position Mode CSP command sent successfully.");
    } else {
         Serial.println("Failed to send set position mode CSP commands!");
    }
    delay(200); // Allow time for mode change

    float target_position1 = M_PI / 2.0f; // 90 degrees
    Serial.print("Attempting to Set target position to "); Serial.print(target_position1); Serial.println(" rad...");
    if (motor.setPosition(target_position1)) {
        Serial.println("Set position command sent successfully.");
    } else {
        Serial.println("Failed to send set position command!");
    }
    delay(3000); // Wait 3 seconds for potential movement

    float target_position2 = -M_PI / 2.0f; // -90 degrees
    Serial.print("Attempting to Set target position to "); Serial.print(target_position2); Serial.println(" rad...");
    if (motor.setPosition(target_position2)) {
        Serial.println("Set position command sent successfully.");
    } else {
        Serial.println("Failed to send set position command!");
    }
    delay(3000); // Wait 3 seconds for potential movement

    Serial.println("Attempting to Set target position to 0 rad...");
    if (motor.setPosition(0.0f)) {
        Serial.println("Set position command sent successfully.");
    } else {
        Serial.println("Failed to send set position command!");
    }
    delay(3000); // Wait 3 seconds for potential movement

    Serial.println("--- Motor Test Sequence Finished. Restarting Loop ---");
    delay(1000); // Pause before restarting the sequence
  } else {
    // Optional: Add behavior if motor wasn't found, e.g., blink red LED
    // Or just do nothing if feedback monitoring is sufficient
    delay(1000); // Prevent loop from spinning too fast if motor not found
    // Keep LED Red
    setAllPixelsColor(50, 0, 0);
  }
}

// Helper function to set all pixels to the same color
void setAllPixelsColor(int red, int green, int blue) {
  pixels.setPixelColor(0, pixels.Color(red, green, blue));
  pixels.show();  // Send the updated pixel colors to the hardware
}

// Note: Removed sendTestCanMessages() function as it's replaced by motor control
// Note: Removed the C++ style main() function entirely. 

// Note: removed C++ style main function entirely