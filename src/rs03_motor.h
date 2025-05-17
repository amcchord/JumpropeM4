#ifndef RS03_MOTOR_H
#define RS03_MOTOR_H

#include <cstdint>
#include <vector>
#include <string>
#include <Arduino.h> // Include Arduino.h for the String type

// Constants from the manual
constexpr float P_MIN = -12.57f; // rad
constexpr float P_MAX = 12.57f; // rad (-4pi to 4pi)
constexpr float V_MIN = -20.0f; // rad/s
constexpr float V_MAX = 20.0f; // rad/s
constexpr float KP_MIN = 0.0f;
constexpr float KP_MAX = 5000.0f;
constexpr float KD_MIN = 0.0f;
constexpr float KD_MAX = 100.0f;
constexpr float T_MIN = -60.0f; // Nm
constexpr float T_MAX = 60.0f; // Nm
constexpr float I_MIN = -43.0f; // A
constexpr float I_MAX = 43.0f; // A

// Error flag bits
constexpr uint16_t ERROR_UNCALIBRATED = 0x20;    // bit 21: uncalibrated
constexpr uint16_t ERROR_GRIDLOCK = 0x10;        // bit 20: Gridlock overload fault
constexpr uint16_t ERROR_MAGNETIC = 0x08;        // bit 19: magnetic coding fault
constexpr uint16_t ERROR_OVERTEMP = 0x04;        // bit 18: overtemperature
constexpr uint16_t ERROR_OVERCURRENT = 0x02;     // bit 17: overcurrent
constexpr uint16_t ERROR_UNDERVOLTAGE = 0x01;    // bit 16: undervoltage fault

// Parameter indices (subset)
constexpr uint16_t INDEX_RUN_MODE = 0x7005;
constexpr uint16_t INDEX_IQ_REF = 0x7006;        // Current mode Iq command
constexpr uint16_t INDEX_SPD_REF = 0x700A;       // Velocity mode speed command
constexpr uint16_t INDEX_LIMIT_TORQUE = 0x700B; // Torque limit
constexpr uint16_t INDEX_LOC_REF = 0x7016;       // Position Mode Angle instruction
constexpr uint16_t INDEX_LIMIT_SPD = 0x7017;    // Location mode (CSP) speed limit
constexpr uint16_t INDEX_LIMIT_CUR = 0x7018;    // Velocity/position mode Current limitation
constexpr uint16_t INDEX_MECH_POS = 0x7019;      // Mechanical Angle (Read Only)
constexpr uint16_t INDEX_IQF = 0x701A;           // Iq Filtered (Read Only)
constexpr uint16_t INDEX_MECH_VEL = 0x701B;      // Velocity (Read Only)
constexpr uint16_t INDEX_VBUS = 0x701C;          // Bus voltage (Read Only)
constexpr uint16_t INDEX_VEL_MAX_PP = 0x7024;    // Location mode (PP) speed
constexpr uint16_t INDEX_ACC_SET_PP = 0x7025;    // Location mode (PP) acceleration

// Run modes
constexpr uint8_t MODE_MIT = 0;          // MIT Mode (Operation Control Mode in manual?)
constexpr uint8_t MODE_POS_PP = 1;       // Position Mode (PP)
constexpr uint8_t MODE_VELOCITY = 2;     // Velocity Mode
constexpr uint8_t MODE_CURRENT = 3;      // Current Mode
constexpr uint8_t MODE_POS_CSP = 5;      // Position Mode (CSP)

// Basic CAN Frame structure (Adapt to your specific CAN library)
struct CanFrame {
    uint32_t id = 0;      // 29-bit Extended ID
    uint8_t dlc = 8;     // Data Length Code (usually 8 for this motor)
    uint8_t data[8] = {0};
    bool is_extended = true;
};

// Abstract CAN Interface (User needs to implement this based on their hardware/library)
class CanInterface {
public:
    virtual ~CanInterface() = default;
    virtual bool sendFrame(const CanFrame& frame) = 0;
    // Optional: Add a receive method if needed for synchronous feedback
    virtual bool receiveFrame(CanFrame& frame, uint32_t timeout_ms) = 0;
};

// Class to control the RS03 motor
class RS03Motor {
public:
    struct Feedback {
        float position = 0.0f;    // rad (-4pi to 4pi)
        float velocity = 0.0f;    // rad/s (-20 to 20)
        float torque = 0.0f;      // Nm (-60 to 60)
        float temperature = 0.0f; // Celsius
        uint8_t motor_id = 0;
        uint8_t mode = 0;
        uint16_t error_flags = 0; // Extracted from type 2 feedback ID bits 16-21
    };

    RS03Motor(CanInterface& can_interface, uint8_t motor_id, uint8_t master_id = 0xFD);

    // Basic Control
    bool enable();
    bool disable();
    bool resetFaults(); // Uses Type 4 with data[0] = 1
    bool setActiveReporting(bool enable_reporting); // Uses Type 24
    bool setMechanicalZero(); // Uses Type 6 to set the current position as zero

    // Mode Setting
    bool setModeVelocity();
    bool setModePositionCSP(float speed_limit = 2.0f, float current_limit = 23.0f);
    bool setModePositionPP(float speed = 10.0f, float acceleration = 10.0f, float current_limit = 23.0f);
    bool setModeMit(); // Set Run Mode to 0 (MIT/Operation Control Mode)
    // bool setModeCurrent(float torque_limit = 17.0f); // TODO

    // Commands (depend on current mode)
    bool setVelocity(float speed_rad_s);                       // Requires Velocity Mode
    bool setPosition(float position_rad);                      // Requires Position Mode (CSP or PP)
    // bool setCurrent(float current_A);                          // Requires Current Mode // TODO
    bool setMitCommand(float position_rad, float velocity_rad_s, float kp, float kd, float torque_nm); // Uses Type 1

    // Parameter Access (using Type 17 Read / Type 18 Write)
    // Note: Writing parameters often requires saving (Type 22) to persist after power cycle.
    // These writes are generally temporary (lost on power cycle) unless saved.
    bool setParameterFloat(uint16_t index, float value);
    bool setParameterUint8(uint16_t index, uint8_t value);
    bool setParameterUint16(uint16_t index, uint16_t value);
    bool setParameterUint32(uint16_t index, uint32_t value);
    // bool readParameter(uint16_t index, float& value); // TODO: Requires receive handling
    // bool readParameter(uint16_t index, uint8_t& value); // TODO: Requires receive handling
    // bool saveParameters(); // Uses Type 22 // TODO

    // Feedback Handling (Call this regularly in your loop to process incoming frames)
    // Needs implementation in the user's application to call this with received frames
    bool processFeedback(const CanFrame& frame);
    Feedback getLastFeedback() const;

    void setMotorId(uint8_t new_id);
    bool sendRawFrame(uint32_t id, uint8_t dlc, const uint8_t* data_bytes, bool is_extended = true);

    // Data conversion utilities made public
    float uintToFloat(int x_int, float x_min, float x_max, int bits);
    int floatToUint(float x, float x_min, float x_max, int bits);

    // Make packing utility public for use in main if needed
    uint16_t packFloatToUint16(float x, float x_min, float x_max);

    // Add a specific method for jogging the motor at a given velocity
    bool jogMotor(float velocity_rad_s, bool useLogFormat = false);
    
    // Get text descriptions of current error flags
    std::string getErrorText() const;
    bool hasErrors() const;

    // Add a method to set velocity, current limit, and acceleration at once
    bool setVelocityWithLimits(float velocity, float current_limit, float acceleration);

private:
    CanInterface& can_;
    uint8_t motor_id_;
    uint8_t master_id_;
    Feedback last_feedback_;

    // Helper functions
    CanFrame createFrame(uint8_t type, uint16_t data_field_2 = 0, uint8_t dest_id = 0xFF);
    bool sendFrame(const CanFrame& frame);
};

#endif // RS03_MOTOR_H 