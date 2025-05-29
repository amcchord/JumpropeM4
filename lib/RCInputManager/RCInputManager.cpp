#include "RCInputManager.h"
#include "Logger.h"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>

extern Adafruit_NeoPixel pixels;
extern Adafruit_SSD1306 display;

RCInputManager::RCInputManager(SpektrumSatelliteReader& reader) : spektrumReader(reader) {
}

bool RCInputManager::begin() {
    if (spektrumReader.begin()) {
        Logger::info("Spektrum Satellite Reader initialized on Serial1");
        return true;
    } else {
        Logger::error("Failed to initialize Spektrum Satellite Reader!");
        return false;
    }
}

void RCInputManager::update() {
    spektrumReader.update();
}

bool RCInputManager::isReceiving() const {
    return spektrumReader.isReceiving();
}

String RCInputManager::getAllChannelValues() const {
    String channelValues = "All Channels: ";
    for (int i = 0; i < SPEKTRUM_CHANNELS; i++) {
        channelValues += "Ch" + String(i + 1) + "=" + String(spektrumReader.getChannel(i)) + "us";
        if (i < SPEKTRUM_CHANNELS - 1) {
            channelValues += ", ";
        }
    }
    return channelValues;
}

float RCInputManager::mapPulseToVelocity(int pulseWidth) const {
    // Dead zone around center (1500us)
    if (pulseWidth >= (MID_PULSE - DEAD_ZONE) && pulseWidth <= (MID_PULSE + DEAD_ZONE)) {
        return 0.0f;
    }
    
    // Map pulse width to velocity
    if (pulseWidth < MID_PULSE) {
        // Map 1000-1450us to -MAX_VELOCITY-0 rad/s
        return map(pulseWidth, MIN_PULSE, MID_PULSE - DEAD_ZONE, -MAX_VELOCITY, 0) * 1.0f;
    } else {
        // Map 1550-2000us to 0-MAX_VELOCITY rad/s
        return map(pulseWidth, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, MAX_VELOCITY) * 1.0f;
    }
}

float RCInputManager::mapPulseToPosition(int pulseWidth) const {
    // Linear mapping from pulse width to position
    return map(pulseWidth, MIN_PULSE, MAX_PULSE, MIN_POSITION * 1000, MAX_POSITION * 1000) * 0.001f;
}

float RCInputManager::mapPulseToCurrentLimit(int pulseWidth) const {
    // Map pulse width from 1000-2000us to 0-MOTOR_CURRENT_LIMIT
    const float MIN_CURRENT = 0.0f; // Minimum current limit even at 1000us
    
    if (pulseWidth <= MIN_PULSE) {
        return MIN_CURRENT;
    } else if (pulseWidth >= MAX_PULSE) {
        return MOTOR_CURRENT_LIMIT;
    } else {
        return MIN_CURRENT + (MOTOR_CURRENT_LIMIT - MIN_CURRENT) * 
               (pulseWidth - MIN_PULSE) / (MAX_PULSE - MIN_PULSE);
    }
}

bool RCInputManager::shouldBeInPositionMode() const {
    int modeChannelValue = spektrumReader.getChannel(MODE_CHANNEL);
    return modeChannelValue > MODE_THRESHOLD;
}

bool RCInputManager::shouldZeroPosition() const {
    int zeroChannelValue = spektrumReader.getChannel(ZERO_CHANNEL);
    return zeroChannelValue > ZERO_THRESHOLD;
}

MotorSelection RCInputManager::getMotorSelection() const {
    int selectChannelValue = spektrumReader.getChannel(SELECT_CHANNEL);
    
    if (selectChannelValue > SELECT_HIGH) {
        return MOTOR_1_ONLY;
    } else if (selectChannelValue < SELECT_LOW) {
        return MOTOR_2_ONLY;
    } else {
        return BOTH_MOTORS;
    }
}

int RCInputManager::getVelocityChannel() const {
    return spektrumReader.getChannel(SPEKTRUM_CHANNEL);
}

int RCInputManager::getPositionChannel() const {
    return spektrumReader.getChannel(POS_CHANNEL);
}

int RCInputManager::getCurrentChannel() const {
    return spektrumReader.getChannel(CURRENT_CHANNEL);
}

int RCInputManager::getModeChannel() const {
    return spektrumReader.getChannel(MODE_CHANNEL);
}

int RCInputManager::getSelectChannel() const {
    return spektrumReader.getChannel(SELECT_CHANNEL);
}

int RCInputManager::getZeroChannel() const {
    return spektrumReader.getChannel(ZERO_CHANNEL);
}

void RCInputManager::runBindModeDisplay() {
    Logger::info("Entering bind mode display loop...");
    
    // Initialize display for bind mode
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("BIND MODE");
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println("DSMX 22ms");
    display.println("");
    display.println("1. Put TX in bind mode");
    display.println("2. Power cycle when");
    display.println("   LED goes solid");
    display.display();
    
    // Set LED to pulsing blue to indicate bind mode
    pixels.setPixelColor(0, pixels.Color(0, 0, 50));
    pixels.show();
    
    Logger::info("Spektrum satellite receiver is now in DSMX 22ms bind mode");
    Logger::info("The receiver LED should be flashing rapidly");
    Logger::info("");
    Logger::info("BINDING INSTRUCTIONS:");
    Logger::info("1. Put your transmitter into DSMX bind mode");
    Logger::info("2. Wait for the receiver LED to go solid");
    Logger::info("3. Power cycle both transmitter and receiver");
    Logger::info("4. Remove the bind mode jumper and restart");
    Logger::info("");
    Logger::info("System will remain in bind mode until power cycle...");
    
    // Stay in bind mode forever with pulsing LED
    unsigned long lastPulse = 0;
    bool ledState = false;
    
    while (true) {
        // Pulse the LED every 500ms to indicate bind mode is active
        if (millis() - lastPulse >= 500) {
            ledState = !ledState;
            if (ledState) {
                pixels.setPixelColor(0, pixels.Color(0, 0, 50)); // Blue
            } else {
                pixels.setPixelColor(0, pixels.Color(0, 0, 10)); // Dim blue
            }
            pixels.show();
            lastPulse = millis();
        }
        
        // Update display with elapsed time
        static unsigned long lastDisplayUpdate = 0;
        if (millis() - lastDisplayUpdate >= 1000) {
            unsigned long elapsed = millis() / 1000;
            
            display.clearDisplay();
            display.setTextSize(2);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("BIND MODE");
            display.setTextSize(1);
            display.setCursor(0, 20);
            display.println("DSMX 22ms");
            display.println("");
            display.println("Elapsed: " + String(elapsed) + "s");
            display.println("");
            display.println("Waiting for bind...");
            display.println("Power cycle when done");
            display.display();
            
            lastDisplayUpdate = millis();
        }
        
        delay(10);
    }
} 