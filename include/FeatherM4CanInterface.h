#ifndef FEATHER_M4_CAN_INTERFACE_H
#define FEATHER_M4_CAN_INTERFACE_H

#include <Arduino.h>
#include "RS03Motor.h" // For CanInterface base class

// Forward declarations
class CANSAME5x;
void log(int level, const String& message);

// Concrete CAN Interface implementation for Feather M4 CAN
class FeatherM4CanInterface : public CanInterface {
public:
    FeatherM4CanInterface();
    
    bool sendFrame(const CanFrame& frame) override;
    
private:
    CANSAME5x& canBus;
};

#endif // FEATHER_M4_CAN_INTERFACE_H 