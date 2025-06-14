#include "RS03Motor.h"
#include <Arduino.h>
#include "Logger.h"
#include <Arduino.h> // Include for Serial object
#include <cmath>   // For std::isnan, std::isinf, round. Not for clamp.
#include <cstring> // For memcpy
#include <algorithm> // For std::min and std::max, if available and used instead of manual clamp
#include <string>

// Helper function for printing frames
void print_frame_details(const char* label, const CanFrame& frame) {
    // Use LOG_DEBUG level - this will only print if the debug level in main.cpp
    // is set to LOG_DEBUG (3) or higher
    if (Serial) { 
        String frameDetails = String(label) + 
                              " ID=0x" + String(frame.id, HEX) + 
                              " DLC=" + String(frame.dlc) + 
                              " Data=[";
        
        for (int i = 0; i < frame.dlc; ++i) {
            if (frame.data[i] < 0x10) frameDetails += "0"; // Pad with leading zero
            frameDetails += String(frame.data[i], HEX);
            if (i < frame.dlc - 1) frameDetails += " ";
        }
        frameDetails += "]";
        
        Logger::debug(frameDetails);
    }
}

// Manual clamp implementation as std::clamp might not be available (C++17)
namespace {
    template <typename T>
    const T& manual_clamp(const T& value, const T& low, const T& high) {
        return std::max(low, std::min(value, high)); // Requires <algorithm>
        // Or, without <algorithm>:
        // return value < low ? low : (value > high ? high : value);
    }
}

// --- Helper Functions ---

// Convert float to unsigned int for CAN packing
// From https://github.com/mjbots/moteus/blob/master/docs/reference.md#can-frame-format
// Adapted slightly
int RS03Motor::floatToUint(float x, float x_min, float x_max, int bits) {
    if (std::isnan(x)) {
        // The spec doesn't define this, but 0 is probably the safest.
        x = 0.0f;
    }
    float span = x_max - x_min;
    float offset = x_min;
    x = manual_clamp(x, x_min, x_max);
    return static_cast<int>((x - offset) * ((1 << bits) - 1) / span);
}

// Convert unsigned int from CAN unpacking back to float
float RS03Motor::uintToFloat(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    x_int = manual_clamp(x_int, 0, (1 << bits) - 1);
    return offset + span * static_cast<float>(x_int) / ((1 << bits) - 1);
}

// Specific version for packing 16-bit values as used in Type 1 feedback/commands
uint16_t RS03Motor::packFloatToUint16(float x, float x_min, float x_max) {
    return static_cast<uint16_t>(floatToUint(x, x_min, x_max, 16));
}


// --- Constructor ---

RS03Motor::RS03Motor(CanInterface& can_interface, uint8_t motor_id, uint8_t master_id)
    : can_(can_interface), motor_id_(motor_id), master_id_(master_id) {
    last_feedback_.motor_id = motor_id_; // Initialize feedback struct
}

// --- CAN Frame Handling ---

// Creates the extended CAN ID based on the manual's description:
// Type (bits 28-24) | Data Area 2 (bits 23-8) | Destination Address (bits 7-0)
CanFrame RS03Motor::createFrame(uint8_t type, uint16_t data_field_2, uint8_t dest_id) {
    CanFrame frame;
    frame.is_extended = true;
    frame.dlc = 8; // Default, can be overridden if needed
    // Ensure type is within 5 bits (0-31)
    frame.id = ((static_cast<uint32_t>(type & 0x1F) << 24) |
                (static_cast<uint32_t>(data_field_2) << 8) |
                static_cast<uint32_t>(dest_id));
    return frame;
}

bool RS03Motor::sendFrame(const CanFrame& frame) {
    return can_.sendFrame(frame);
}

// --- Basic Control ---

bool RS03Motor::enable() {
    // Communication Type 3: Motor Enabled to Run
    CanFrame frame = createFrame(3, master_id_, motor_id_);
    frame.dlc = 0; // Use DLC 0 as per description
    print_frame_details("SEND enable:", frame); // <<< PRINT
    return sendFrame(frame);
}

bool RS03Motor::disable() {
    // Communication Type 4: Motor Stops Running (Reset/Disable)
    CanFrame frame = createFrame(4, master_id_, motor_id_);
    frame.dlc = 8;
    std::memset(frame.data, 0, sizeof(frame.data)); // Clear data field
    print_frame_details("SEND disable:", frame); // <<< PRINT
    return sendFrame(frame);
}

bool RS03Motor::resetFaults() {
    // Communication Type 4: Motor Stops Running
    CanFrame frame = createFrame(4, master_id_, motor_id_);
    frame.dlc = 8;
    std::memset(frame.data, 0, sizeof(frame.data));
    frame.data[0] = 1; // Set clear fault flag
    print_frame_details("SEND resetFaults:", frame); // <<< PRINT
    return sendFrame(frame);
}

bool RS03Motor::setMechanicalZero() {
    // Communication Type 6: Set Motor Mechanical Zero
    CanFrame frame = createFrame(6, master_id_, motor_id_);
    frame.dlc = 8;
    std::memset(frame.data, 0, sizeof(frame.data));
    frame.data[0] = 1; // Set to 1 to set mechanical zero
    print_frame_details("SEND setMechZero:", frame);
    bool success = sendFrame(frame);
    
    // After setting mechanical zero, save parameters to flash (Type 22)
    if (success) {
        delay(100); // Give time for zero setting to complete
        CanFrame saveFrame = createFrame(0x16, master_id_, motor_id_); // Type 22
        saveFrame.dlc = 8;
        std::memset(saveFrame.data, 0, sizeof(saveFrame.data));
        print_frame_details("SEND saveParams:", saveFrame);
        success &= sendFrame(saveFrame);
    }
    
    return success;
}

bool RS03Motor::setZeroFlag(uint8_t flag) {
    // Set zero flag bit: 0=0-2π, 1=-π-π
    print_frame_details(" -> setZeroFlag calls:", CanFrame{});
    return setParameterUint8(INDEX_ZERO_STA, flag);
}


// --- Mode Setting ---

bool RS03Motor::setModeVelocity() {
    print_frame_details(" -> setModeVelocity calls:", CanFrame{}); // Indicate function call
    return setParameterUint8(INDEX_RUN_MODE, MODE_VELOCITY);
}

bool RS03Motor::setModePositionCSP(float speed_limit, float current_limit) {
    print_frame_details(" -> setModePosCSP calls:", CanFrame{}); // Indicate function call
    bool success = true;
    // Order matters? Manual isn't clear. Set mode last.
    success &= setParameterFloat(INDEX_LIMIT_CUR, current_limit);
    success &= setParameterFloat(INDEX_LIMIT_SPD, speed_limit);
    success &= setParameterUint8(INDEX_RUN_MODE, MODE_POS_CSP);
    return success;
}

bool RS03Motor::setModePositionPP(float speed, float acceleration, float current_limit) {
    print_frame_details(" -> setModePosPP calls:", CanFrame{}); // Indicate function call
    bool success = true;
    
    // Set all the necessary parameters for PP mode
    success &= setParameterFloat(INDEX_LIMIT_CUR, current_limit);    // Current limit
    success &= setParameterFloat(INDEX_VEL_MAX_PP, speed);           // PP mode speed
    success &= setParameterFloat(INDEX_ACC_SET_PP, acceleration);    // PP mode acceleration
    
    // Set mode to PP mode last
    success &= setParameterUint8(INDEX_RUN_MODE, MODE_POS_PP);
    
    return success;
}

bool RS03Motor::setModeMit() {
    print_frame_details(" -> setModeMit calls:", CanFrame{}); // Indicate function call
    return setParameterUint8(INDEX_RUN_MODE, MODE_MIT); // MODE_MIT is 0
}

// --- Commands (depend on current mode) ---

bool RS03Motor::setVelocity(float speed_rad_s) {
    print_frame_details(" -> setVelocity calls:", CanFrame{}); // Indicate function call
    return setParameterFloat(INDEX_SPD_REF, speed_rad_s);
}

bool RS03Motor::setPosition(float position_rad) {
    print_frame_details(" -> setPosition calls:", CanFrame{}); // Indicate function call
    return setParameterFloat(INDEX_LOC_REF, position_rad);
}

bool RS03Motor::setMitCommand(float position_rad, float velocity_rad_s, float kp, float kd, float torque_nm) {
    // Communication Type 1: Operation control mode motor control instruction
    // ID: Type (0x1) | Packed Torque (16-bit) | Target Motor ID
    // Data: Packed Angle (16-bit), Packed Velocity (16-bit), Packed Kp (16-bit), Packed Kd (16-bit)

    uint16_t packed_torque = packFloatToUint16(torque_nm, T_MIN, T_MAX);
    CanFrame frame = createFrame(0x01, packed_torque, motor_id_); // Type 1, torque in data_field_2
    frame.dlc = 8;

    uint16_t packed_pos = packFloatToUint16(position_rad, P_MIN, P_MAX);
    uint16_t packed_vel = packFloatToUint16(velocity_rad_s, V_MIN, V_MAX);
    uint16_t packed_kp = packFloatToUint16(kp, KP_MIN, KP_MAX);
    uint16_t packed_kd = packFloatToUint16(kd, KD_MIN, KD_MAX);

    // Data: Byte0~1: target Angle (big endian in manual example, but usually CAN data is little endian by convention?)
    // Manual sample code: txMsg.tx_data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8; 
    // This indicates Big Endian for the manual's example. Let's follow that.
    frame.data[0] = (packed_pos >> 8) & 0xFF; // Pos MSB
    frame.data[1] = packed_pos & 0xFF;        // Pos LSB
    frame.data[2] = (packed_vel >> 8) & 0xFF; // Vel MSB
    frame.data[3] = packed_vel & 0xFF;        // Vel LSB
    frame.data[4] = (packed_kp >> 8) & 0xFF;  // Kp MSB
    frame.data[5] = packed_kp & 0xFF;         // Kp LSB
    frame.data[6] = (packed_kd >> 8) & 0xFF;  // Kd MSB
    frame.data[7] = packed_kd & 0xFF;         // Kd LSB

    print_frame_details("SEND MIT Cmd (T1):", frame); 
    return sendFrame(frame);
}


// --- Parameter Access ---

bool RS03Motor::setParameterFloat(uint16_t index, float value) {
    // Communication Type 18: Single Parameter Write
    CanFrame frame = createFrame(0x12, master_id_, motor_id_);
    frame.dlc = 8;
    // Index (Bytes 0-1, Little Endian)
    frame.data[0] = index & 0xFF;
    frame.data[1] = (index >> 8) & 0xFF;
    // Bytes 2-3 are 0x00
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    // Data (Bytes 4-7, Little Endian for float)
    memcpy(&frame.data[4], &value, sizeof(float));
    print_frame_details("  SEND setParamF:", frame); // <<< PRINT (Indented)
    return sendFrame(frame);
}

bool RS03Motor::setParameterUint8(uint16_t index, uint8_t value) {
    CanFrame frame = createFrame(0x12, master_id_, motor_id_);
    frame.dlc = 8;
    frame.data[0] = index & 0xFF;
    frame.data[1] = (index >> 8) & 0xFF;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    // Data (Byte 4)
    frame.data[4] = value;
    frame.data[5] = 0x00; // Pad remaining bytes
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    print_frame_details("  SEND setParamU8:", frame); // <<< PRINT (Indented)
    return sendFrame(frame);
}

bool RS03Motor::setParameterUint16(uint16_t index, uint16_t value) {
    CanFrame frame = createFrame(0x12, master_id_, motor_id_);
    frame.dlc = 8;
    frame.data[0] = index & 0xFF;
    frame.data[1] = (index >> 8) & 0xFF;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    // Data (Bytes 4-5, Little Endian)
    memcpy(&frame.data[4], &value, sizeof(uint16_t));
    frame.data[6] = 0x00; // Pad remaining bytes
    frame.data[7] = 0x00;
    print_frame_details("  SEND setParamU16:", frame); // <<< PRINT (Indented)
    return sendFrame(frame);
}

bool RS03Motor::setParameterUint32(uint16_t index, uint32_t value) {
    CanFrame frame = createFrame(0x12, master_id_, motor_id_);
    frame.dlc = 8;
    frame.data[0] = index & 0xFF;
    frame.data[1] = (index >> 8) & 0xFF;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    // Data (Bytes 4-7, Little Endian)
    memcpy(&frame.data[4], &value, sizeof(uint32_t));
    print_frame_details("  SEND setParamU32:", frame); // <<< PRINT (Indented)
    return sendFrame(frame);
}


// --- Feedback Handling ---

bool RS03Motor::processFeedback(const CanFrame& frame) {
    // Check if it's an extended frame with DLC 8
    if (!frame.is_extended || frame.dlc != 8) {
        return false;
    }

    // Extract components from the 29-bit ID
    uint8_t type = (frame.id >> 24) & 0x1F;
    uint16_t source_id = (frame.id >> 8) & 0xFFFF; // This holds motor ID and potentially status bits
    uint8_t dest_id = frame.id & 0xFF;

    // Check if it's a feedback frame (Type 2) for us from our motor
    if (type == 0x02 && dest_id == master_id_) {
        uint8_t current_motor_id = (source_id >> 8) & 0xFF; // Extract motor ID from source_id upper byte
        if (current_motor_id == motor_id_) {
            // Extract status and error bits from source_id (lower byte + more?)
            // ID format: Bit28~bit24=type(0x2) | bit23~8=data_area_2 | bit7~0=dest_id(master)
            // Data Area 2 description:
            // bit15~8: CAN ID of the current motor
            // bit21~16: fault information (0 none 1 has)
            //   bit21: uncalibrated? (Manual has typo, maybe gridlock related?)
            //   bit20: Gridlock overload fault
            //   bit19: magnetic coding fault
            //   bit18: overtemperature
            //   bit17: overcurrent
            //   bit16: undervoltage fault
            // bit22~23: Mode status
            //   0: Reset mode [reset]
            //   1: Cali mode [calibration]
            //   2: Motor mode [Run]
            // Note: The bit numbers (16-23) seem to apply to the *entire* 29-bit ID, not just the data_area_2 field. Let's assume they mean bits within the data_area_2 field (bits 8-23 of the full ID).
            uint16_t data_area_2 = (frame.id >> 8) & 0xFFFF;
            last_feedback_.motor_id = (data_area_2 >> 8) & 0xFF; // Should match motor_id_
            last_feedback_.error_flags = (data_area_2 >> 0) & 0b111111; // Extract bits 16-21 -> bits 0-5 of data_area_2
            uint8_t mode_status_bits = (data_area_2 >> 6) & 0b11; // Extract bits 22-23 -> bits 6-7 of data_area_2
            last_feedback_.mode = mode_status_bits; // Store the raw mode status bits

            // Unpack data bytes
            // Byte0~1: Current Angle [0~65535] -> (-4π~4π) -> (-12.57 to 12.57) rad
            // Byte2~3: Current angular velocity [0~65535] -> (-20rad/s~20rad/s)
            // Byte4~5: Current torque [0~65535] -> (-60Nm~60Nm)
            // Byte6~7: Current temperature: Temp(Celsius) *10
            uint16_t pos_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
            uint16_t vel_raw = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
            uint16_t tor_raw = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
            uint16_t temp_raw = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];

            last_feedback_.position = uintToFloat(pos_raw, P_MIN, P_MAX, 16);
            last_feedback_.velocity = uintToFloat(vel_raw, V_MIN, V_MAX, 16);
            last_feedback_.torque = uintToFloat(tor_raw, T_MIN, T_MAX, 16);
            last_feedback_.temperature = static_cast<float>(temp_raw) / 10.0f;

            return true; // Successfully processed feedback
        }
    }
    // TODO: Handle other message types if needed (e.g., Type 0 for ID response, Type 17 for param read response, Type 21 for fault feedback)

    return false; // Not a feedback frame or not for this motor based on current motor_id_
}

RS03Motor::Feedback RS03Motor::getLastFeedback() const {
    return last_feedback_;
}

void RS03Motor::setMotorId(uint8_t new_id) {
    motor_id_ = new_id;
}

bool RS03Motor::setActiveReporting(bool enable_reporting) {
    // Communication Type 24: Motor actively reports frames
    CanFrame frame = createFrame(0x18, master_id_, motor_id_);
    frame.dlc = 8;
    std::memset(frame.data, 0, sizeof(frame.data));
    frame.data[0] = enable_reporting ? 0x01 : 0x00;
    print_frame_details("SEND setActiveRep:", frame); // <<< PRINT
    return sendFrame(frame);
}

bool RS03Motor::sendRawFrame(uint32_t id, uint8_t dlc, const uint8_t* data_bytes, bool is_extended) {
    // For debugging
    Serial.print("Sending raw frame with ID: 0x");
    Serial.print(id, HEX);
    Serial.print(", DLC: ");
    Serial.print(dlc);
    Serial.print(", Extended: ");
    Serial.println(is_extended ? "Yes" : "No");
    
    // Extended IDs need to fit in 29 bits (0x1FFFFFFF max)
    if (is_extended && (id > 0x1FFFFFFF)) {
        Serial.print("ERROR: Extended ID 0x");
        Serial.print(id, HEX);
        Serial.println(" exceeds 29-bit maximum");
        return false;
    }
    
    // Create and initialize a frame manually
    CanFrame frame;
    frame.id = id;
    frame.dlc = dlc > 8 ? 8 : dlc; // Ensure DLC is not more than 8
    frame.is_extended = is_extended;
    
    // Copy data if provided
    if (data_bytes != nullptr && frame.dlc > 0) {
        memcpy(frame.data, data_bytes, frame.dlc);
    } else if (frame.dlc > 0) {
        // Zero out data if data_bytes is null but dlc > 0
        std::memset(frame.data, 0, frame.dlc);
    } else {
        frame.dlc = 0; // Ensure DLC is 0 if no data pointer or original dlc was 0
    }

    print_frame_details("SEND RAW FRAME:", frame); // Use existing helper
    return can_.sendFrame(frame);
}

// Implementation of the new jogMotor method
bool RS03Motor::jogMotor(float velocity_rad_s, bool useLogFormat) {
    Serial.print("RS03Motor::jogMotor called with velocity=");
    Serial.print(velocity_rad_s);
    Serial.print(", using ");
    Serial.println(useLogFormat ? "log format" : "MIT format");

    uint8_t commandType = 0x01; // MIT control mode command (Type 1)
    uint32_t jogId;
    
    if (useLogFormat) {
        // Use similar ID format as seen in logs - this is what works!
        jogId = ((static_cast<uint32_t>(commandType) << 24) | 
                (static_cast<uint32_t>(0x07EB) << 8) | 
                static_cast<uint32_t>(motor_id_)) & 0x1FFFFFFF;
    } else {
        // Use proper Type 1 format with torque in data field 2
        uint16_t packed_torque = packFloatToUint16(0.0f, T_MIN, T_MAX); // No additional torque
        jogId = ((static_cast<uint32_t>(commandType) << 24) | 
                (static_cast<uint32_t>(packed_torque) << 8) | 
                static_cast<uint32_t>(motor_id_)) & 0x1FFFFFFF;
    }
    
    uint8_t data[8] = {0};
    
    if (useLogFormat) {
        // Using exact format that worked in logs for Test 1
        data[0] = 0x05;  // Position MSB from log
        data[1] = 0x70;  // Position LSB from log
        data[2] = 0x00;  // Zero values from log
        data[3] = 0x00;  // Zero values from log
        data[4] = 0x07;  // Kp MSB from log
        
        // For direction (data[5]), 0x01 was used for forward, 0x00 for stop, use 0xFF for backward
        if (abs(velocity_rad_s) < 0.01f) {
            // Stop command
            data[5] = 0x00;  // Direction: stop
            data[6] = 0x7F;  // Velocity MSB for stop
            data[7] = 0xFF;  // Velocity LSB for stop
        } else if (velocity_rad_s > 0.0f) {
            // Forward command (using original log values that worked)
            data[5] = 0x01;  // Direction: forward
            data[6] = 0x86;  // Velocity MSB for forward
            data[7] = 0x65;  // Velocity LSB for forward
        } else {
            // Reverse command (modified from forward)
            data[5] = 0xFF;  // Direction: backward (using 0xFF as the negative indicator)
            data[6] = 0x86;  // Use same velocity magnitude as forward
            data[7] = 0x65;  // Use same velocity magnitude as forward
        }
    } else {
        // Using MIT mode format - this wasn't working, but keeping for reference
        uint16_t packed_pos = packFloatToUint16(0.0f, P_MIN, P_MAX);
        uint16_t packed_vel = packFloatToUint16(velocity_rad_s, V_MIN, V_MAX);
        uint16_t packed_kp = packFloatToUint16(5.0f, KP_MIN, KP_MAX);  // Small Kp for basic jogging
        uint16_t packed_kd = packFloatToUint16(0.1f, KD_MIN, KD_MAX);  // Small Kd for stability
        
        // Position (Big Endian according to manual)
        data[0] = (packed_pos >> 8) & 0xFF;  // MSB
        data[1] = packed_pos & 0xFF;         // LSB
        
        // Velocity (Big Endian according to manual)
        data[2] = (packed_vel >> 8) & 0xFF;  // MSB
        data[3] = packed_vel & 0xFF;         // LSB
        
        // Kp (Big Endian according to manual)
        data[4] = (packed_kp >> 8) & 0xFF;   // MSB
        data[5] = packed_kp & 0xFF;          // LSB
        
        // Kd (Big Endian according to manual)
        data[6] = (packed_kd >> 8) & 0xFF;   // MSB
        data[7] = packed_kd & 0xFF;          // LSB
    }
    
    // Print data for debugging
    Serial.print("JOG motor command - ID: 0x");
    Serial.print(jogId, HEX);
    Serial.print(", Data: ");
    for (int i = 0; i < 8; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Create and send the CAN frame
    CanFrame frame;
    frame.id = jogId;
    frame.dlc = 8;
    frame.is_extended = true;
    memcpy(frame.data, data, 8);
    
    print_frame_details("RS03Motor::jogMotor SEND:", frame);
    return can_.sendFrame(frame);
}

// Implementation of the new error handling functions
std::string RS03Motor::getErrorText() const {
    std::string errorText = "";
    
    if (last_feedback_.error_flags == 0) {
        return "No errors";
    }
    
    if (last_feedback_.error_flags & ERROR_UNCALIBRATED) {
        errorText += "Uncalibrated, ";
    }
    if (last_feedback_.error_flags & ERROR_GRIDLOCK) {
        errorText += "Gridlock overload, ";
    }
    if (last_feedback_.error_flags & ERROR_MAGNETIC) {
        errorText += "Magnetic coding fault, ";
    }
    if (last_feedback_.error_flags & ERROR_OVERTEMP) {
        errorText += "Overtemperature, ";
    }
    if (last_feedback_.error_flags & ERROR_OVERCURRENT) {
        errorText += "Overcurrent, ";
    }
    if (last_feedback_.error_flags & ERROR_UNDERVOLTAGE) {
        errorText += "Undervoltage, ";
    }
    
    // Remove trailing comma and space if any
    if (errorText.length() > 2) {
        errorText = errorText.substr(0, errorText.length() - 2);
    }
    
    return errorText;
}

bool RS03Motor::hasErrors() const {
    return last_feedback_.error_flags != 0;
}

// Implementation of setVelocityWithLimits
bool RS03Motor::setVelocityWithLimits(float velocity, float current_limit, float acceleration) {
    bool ok = true;
   // Serial.println("Setting current limit...");
    ok = ok && setParameterFloat(INDEX_LIMIT_CUR, current_limit);
   // Serial.println("Setting acceleration...");
    ok = ok && setParameterFloat(0x7022, acceleration); // acc_rad for velocity mode
   // Serial.println("Setting velocity...");
    ok = ok && setParameterFloat(INDEX_SPD_REF, velocity);
    return ok;
}

// Private helper methods
// ... existing code ... 