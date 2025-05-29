#ifndef RC_INPUT_MANAGER_H
#define RC_INPUT_MANAGER_H

#include <Arduino.h>
#include "SystemConfig.h"
#include "SpektrumSatelliteReader.h"

class RCInputManager {
private:
    SpektrumSatelliteReader& spektrumReader;
    
public:
    RCInputManager(SpektrumSatelliteReader& reader);
    
    // Initialization
    bool begin();
    void update();
    
    // Signal status
    bool isReceiving() const;
    String getAllChannelValues() const;
    
    // Value mapping functions
    float mapPulseToVelocity(int pulseWidth) const;
    float mapPulseToPosition(int pulseWidth) const;
    float mapPulseToCurrentLimit(int pulseWidth) const;
    
    // Mode and control detection
    bool shouldBeInPositionMode() const;
    bool shouldZeroPosition() const;
    MotorSelection getMotorSelection() const;
    
    // Get raw channel values
    int getVelocityChannel() const;
    int getPositionChannel() const;
    int getCurrentChannel() const;
    int getModeChannel() const;
    int getSelectChannel() const;
    int getZeroChannel() const;
    
    // Bind mode handling
    void runBindModeDisplay();
};

#endif // RC_INPUT_MANAGER_H 