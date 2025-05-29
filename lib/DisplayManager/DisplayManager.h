#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "SystemConfig.h"

class DisplayManager {
private:
    Adafruit_SSD1306& display;
    unsigned long lastUpdateTime;
    
public:
    DisplayManager(Adafruit_SSD1306& disp);
    
    // Initialization
    bool begin();
    
    // Update methods
    void update(bool inPositionMode, MotorSelection currentMotorSelection, 
                const MotorRawFeedback& motor1_feedback, 
                const MotorRawFeedback& motor2_feedback);
    
    // Status displays
    void showInitializing();
    void showReady();
    void showError(const String& errorMessage);
    void showNoSignal();
    
    // Force immediate update
    void forceUpdate();
    
private:
    void updateMotorDisplay(bool inPositionMode, MotorSelection currentMotorSelection,
                           const MotorRawFeedback& motor1_feedback, 
                           const MotorRawFeedback& motor2_feedback);
};

#endif // DISPLAY_MANAGER_H 