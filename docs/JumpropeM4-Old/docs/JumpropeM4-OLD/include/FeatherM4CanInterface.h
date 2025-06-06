#ifndef FEATHER_M4_CAN_INTERFACE_H
#define FEATHER_M4_CAN_INTERFACE_H

#include <Arduino.h>
#include <CANSAME5x.h>
#include "rs03_motor.h" // For CanInterface base class

// Forward declarations
void log(int level, const String& message);

// Concrete CAN Interface implementation for Feather M4 CAN
class FeatherM4CanInterface : public CanInterface {
public:
    FeatherM4CanInterface();
    
    bool sendFrame(const CanFrame& frame) override;
    bool receiveFrame(CanFrame& frame, uint32_t timeout_ms) override;
    
private:
    CANSAME5x& canBus;
};

#endif // FEATHER_M4_CAN_INTERFACE_H 