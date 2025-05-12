#include "rs03_motor.h"
#include <cmath>   // For std::isnan, std::isinf, round. Not for clamp.
#include <cstring> // For memcpy
#include <algorithm> // For std::min and std::max, if available and used instead of manual clamp

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
    frame.dlc = 0; // Type 3 has no data bytes according to manual example C code
    return sendFrame(frame);
}

bool RS03Motor::disable() {
    // Communication Type 4: Motor Stops Running (Reset/Disable)
    // Data field byte[0] = 0 (clear fault flag = false)
    CanFrame frame = createFrame(4, master_id_, motor_id_);
    frame.dlc = 8; // Manual C code example implies DLC 8, even if only byte 0 is used
    std::memset(frame.data, 0, sizeof(frame.data)); // Clear data field
    return sendFrame(frame);
}

bool RS03Motor::resetFaults() {
    // Communication Type 4: Motor Stops Running
    // Data field byte[0] = 1: The fault is cleared.
    CanFrame frame = createFrame(4, master_id_, motor_id_);
    frame.dlc = 8; // Manual C code example implies DLC 8
    std::memset(frame.data, 0, sizeof(frame.data));
    frame.data[0] = 1; // Set clear fault flag
    return sendFrame(frame);
}


// --- Mode Setting ---

bool RS03Motor::setModeVelocity() {
    return setParameterUint8(INDEX_RUN_MODE, MODE_VELOCITY);
}

bool RS03Motor::setModePositionCSP(float speed_limit, float current_limit) {
    bool success = true;
    // 1. Set current limit for position mode
    success &= setParameterFloat(INDEX_LIMIT_CUR, current_limit);
    // 2. Set speed limit for position mode (CSP)
    success &= setParameterFloat(INDEX_LIMIT_SPD, speed_limit);
    // 3. Set run mode to Position CSP
    success &= setParameterUint8(INDEX_RUN_MODE, MODE_POS_CSP);
    return success;
}

// --- Commands (depend on current mode) ---

bool RS03Motor::setVelocity(float speed_rad_s) {
    // Uses Communication Type 18 (Single Parameter Write)
    // Index: 0x700A (spd_ref)
    return setParameterFloat(INDEX_SPD_REF, speed_rad_s);
}

bool RS03Motor::setPosition(float position_rad) {
    // Uses Communication Type 18 (Single Parameter Write)
    // Index: 0x7016 (loc_ref)
    return setParameterFloat(INDEX_LOC_REF, position_rad);
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
            last_feedback_.error_flags = (data_area_2 >> 0) & 0b111111; // Extract bits 16-21 (relative to start of data_area_2 at bit 8) -> bits 0-5 of data_area_2
            uint8_t mode_status_bits = (data_area_2 >> 6) & 0b11; // Extract bits 22-23 -> bits 6-7 of data_area_2
            // Manual is ambiguous about mapping mode_status_bits to the run modes (0-5).
            // We'll tentatively map 2 (Motor mode [Run]) to the last known sent mode,
            // but this is unreliable without reading the run_mode parameter.
            // If it's not 2, maybe mark mode as unknown/reset?
            // For now, we just store the raw bits. A better approach would be to periodically read 0x7005.
            // last_feedback_.mode = mode_status_bits; // Store raw status bits for now

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

// Private helper methods
// ... existing code ... 