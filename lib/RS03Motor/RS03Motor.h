#ifndef RS03_MOTOR_H
#define RS03_MOTOR_H

#include <cstdint>
#include <string>

// ----- CAN Frame Structure -----
struct CanFrame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    bool is_extended;
};

// ----- CAN Interface Abstract Base Class -----
class CanInterface {
public:
    virtual ~CanInterface() = default;
    virtual bool sendFrame(const CanFrame& frame) = 0;
};

// ----- RS03 Motor Constants -----
// UPDATED: From manual firmware 0.0.2.7+ - Type 2 Change Description
const float P_MIN = -12.57f;  // rad (was -12.5f in older firmware)
const float P_MAX = 12.57f;   // rad (was 12.5f in older firmware)
const float V_MIN = -20.0f;   // rad/s
const float V_MAX = 20.0f;    // rad/s
const float T_MIN = -60.0f;   // Nm
const float T_MAX = 60.0f;    // Nm
const float KP_MIN = 0.0f;
const float KP_MAX = 500.0f;
const float KD_MIN = 0.0f;
const float KD_MAX = 5.0f;

// ----- Parameter Indices -----
const uint16_t INDEX_RUN_MODE = 0x7005;
const uint16_t INDEX_SPD_REF = 0x700A;
const uint16_t INDEX_LOC_REF = 0x7016;
const uint16_t INDEX_LIMIT_CUR = 0x7018;  // FIXED: Was 0x7022, should be 0x7018
const uint16_t INDEX_LIMIT_SPD = 0x7017;
const uint16_t INDEX_VEL_MAX_PP = 0x7024; // FIXED: Was 0x7081, should be 0x7024
const uint16_t INDEX_ACC_SET_PP = 0x7025; // FIXED: Was 0x7083, should be 0x7025
const uint16_t INDEX_ZERO_STA = 0x7029;   // Zero flag bit: 0=0-2π, 1=-π-π

// ----- Mode Constants -----
// FIXED: Based on manual parameter 0x7005 run_mode values
const uint8_t MODE_MIT = 0;       // Operation mode
const uint8_t MODE_POS_PP = 1;    // Position mode (PP) - FIXED: was 3
const uint8_t MODE_VELOCITY = 2;  // Velocity mode  
const uint8_t MODE_POS_CSP = 5;   // Position mode (CSP) - FIXED: was 1

// ----- Error Flag Constants -----
const uint8_t ERROR_UNCALIBRATED = 0x20;  // bit 5
const uint8_t ERROR_GRIDLOCK = 0x10;      // bit 4
const uint8_t ERROR_MAGNETIC = 0x08;      // bit 3
const uint8_t ERROR_OVERTEMP = 0x04;      // bit 2
const uint8_t ERROR_OVERCURRENT = 0x02;   // bit 1
const uint8_t ERROR_UNDERVOLTAGE = 0x01;  // bit 0

// ----- RS03Motor Class -----
class RS03Motor {
public:
    // ----- Feedback Structure -----
    struct Feedback {
        uint8_t motor_id = 0;
        float position = 0.0f;      // rad
        float velocity = 0.0f;      // rad/s
        float torque = 0.0f;        // Nm
        float temperature = 0.0f;   // °C
        uint8_t error_flags = 0;    // Error flags from CAN feedback
        uint8_t mode = 0;           // Mode status from CAN feedback
    };

private:
    CanInterface& can_;
    uint8_t motor_id_;
    uint8_t master_id_;
    Feedback last_feedback_;

public:
    // ----- Constructor -----
    RS03Motor(CanInterface& can_interface, uint8_t motor_id, uint8_t master_id);

    // ----- Getters -----
    uint8_t getMotorId() const { return motor_id_; }
    uint8_t getMasterId() const { return master_id_; }

    // ----- Basic Control -----
    bool enable();
    bool disable();
    bool resetFaults();
    bool setMechanicalZero();
    bool setZeroFlag(uint8_t flag); // Set zero flag: 0=0-2π, 1=-π-π

    // ----- Mode Setting -----
    bool setModeVelocity();
    bool setModePositionCSP(float speed_limit, float current_limit);
    bool setModePositionPP(float speed, float acceleration, float current_limit);
    bool setModeMit();

    // ----- Commands (depend on current mode) -----
    bool setVelocity(float speed_rad_s);
    bool setPosition(float position_rad);
    bool setMitCommand(float position_rad, float velocity_rad_s, float kp, float kd, float torque_nm);

    // ----- Parameter Access -----
    bool setParameterFloat(uint16_t index, float value);
    bool setParameterUint8(uint16_t index, uint8_t value);
    bool setParameterUint16(uint16_t index, uint16_t value);
    bool setParameterUint32(uint16_t index, uint32_t value);

    // ----- Feedback Handling -----
    bool processFeedback(const CanFrame& frame);
    Feedback getLastFeedback() const;

    // ----- Utility -----
    void setMotorId(uint8_t new_id);
    bool setActiveReporting(bool enable_reporting);

    // ----- Helper Functions -----
    static int floatToUint(float x, float x_min, float x_max, int bits);
    static float uintToFloat(int x_int, float x_min, float x_max, int bits);
    static uint16_t packFloatToUint16(float x, float x_min, float x_max);

    // ----- CAN Frame Handling -----
    CanFrame createFrame(uint8_t type, uint16_t data_field_2, uint8_t dest_id);
    bool sendFrame(const CanFrame& frame);

    // ----- Debug and Testing -----
    bool sendRawFrame(uint32_t id, uint8_t dlc, const uint8_t* data_bytes, bool is_extended = true);
    bool jogMotor(float velocity_rad_s, bool useLogFormat = true);

    // ----- Error Handling -----
    std::string getErrorText() const;
    bool hasErrors() const;

    // ----- Enhanced Control Methods -----
    bool setVelocityWithLimits(float velocity, float current_limit, float acceleration);
};

#endif // RS03_MOTOR_H 