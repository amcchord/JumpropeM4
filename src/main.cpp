#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Library includes
#include "SystemConfig.h"
#include "Logger.h"
#include "RS03Motor.h"
#include "SpektrumSatelliteReader.h"
#include "RCInputManager.h"
#include "DisplayManager.h"
#include "MotorController.h"
#include "FeatherM4CanInterface.h"
#include <CANSAME5x.h>

// ----- Global Objects -----
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
CANSAME5x CAN;
SpektrumSatelliteReader spektrumReader(SPEKTRUM_SERIAL, SPEKTRUM_CHANNELS, MIN_PULSE, MAX_PULSE, MID_PULSE);

// Hardware interfaces
FeatherM4CanInterface canBus;
RS03Motor motor1(canBus, MOTOR_ID_1, MASTER_ID);
RS03Motor motor2(canBus, MOTOR_ID_2, MASTER_ID);

// High-level managers
RCInputManager rcInput(spektrumReader);
DisplayManager displayMgr(display);
MotorController motorController(motor1, motor2, rcInput, pixels);

// ----- Bind Mode Check -----
bool checkBindMode() {
    pinMode(BIND_MODE_PIN, INPUT_PULLUP);
    delay(1);
    
    if (digitalRead(BIND_MODE_PIN) == LOW && BIND_ENABLED) {
        // Send bind pulses immediately
        pinMode(SPEKTRUM_RX_PIN, OUTPUT);
        digitalWrite(SPEKTRUM_RX_PIN, LOW);
        delay(200);
        digitalWrite(SPEKTRUM_RX_PIN, HIGH);
        delay(100);
        
        Logger::init();
        Logger::info("=== SPEKTRUM BIND MODE ACTIVATED ===");
        Logger::info("Bind pulses sent immediately at power-up");
        
        pixels.begin();
        pixels.setBrightness(50);
        pixels.clear();
        
        if (displayMgr.begin()) {
            rcInput.runBindModeDisplay(); // This function never returns
        }
        
        return true;
    }
    
    return false;
}

// ----- Hardware Initialization -----
bool initializeHardware() {
    // Initialize logger
    Logger::init();
    Logger::info("RS03 Dual Motor Control with RC Input - Starting");
    
    // Initialize display
    if (!displayMgr.begin()) {
        return false;
    }
    
    // Initialize NeoPixel
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(50, 0, 50)); // Purple while initializing
    pixels.show();
    Logger::info("NeoPixel initialized");
    
    // Initialize CAN bus
    pinMode(PIN_CAN_STANDBY, OUTPUT);
    digitalWrite(PIN_CAN_STANDBY, false);
    pinMode(PIN_CAN_BOOSTEN, OUTPUT);
    digitalWrite(PIN_CAN_BOOSTEN, true);
    
    if (!CAN.begin(1000000)) {
        Logger::error("Starting CAN failed!");
        pixels.setPixelColor(0, pixels.Color(50, 0, 0));
        pixels.show();
        return false;
    }
    Logger::info("CAN initialized at 1 Mbit/s");
    
    // Initialize RC input
    if (!rcInput.begin()) {
        pixels.setPixelColor(0, pixels.Color(50, 0, 50));
        pixels.show();
        return false;
    }
    
    return true;
}

// ----- Setup -----
void setup() {
    // CRITICAL: Check for bind mode FIRST
    if (checkBindMode()) {
        return; // Never reached if in bind mode
    }
    
    // Initialize hardware
    if (!initializeHardware()) {
        Logger::error("Hardware initialization failed!");
        while(1) {
            delay(1000);
        }
    }
    
    // Initialize motor controller
    if (!motorController.begin()) {
        Logger::error("Motor controller initialization failed!");
        displayMgr.showError("Motor init failed");
        while(1) {
            delay(1000);
        }
    }
    
    // System ready
    pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green indicates ready
    pixels.show();
    Logger::info("Setup complete. Ready for Spektrum control.");
    
    // Log channel assignments
    Logger::info("Channel 1 (index 0): 1000us=-20rad/s, 1500us=stop, 2000us=20rad/s");
    Logger::info("Channel 2 (index 1): 1000us=1A, 2000us=" + String(MOTOR_CURRENT_LIMIT) + "A current limit");
    Logger::info("Channel 3 (index 2): >1800us=position mode, <1800us=velocity mode");
    Logger::info("Channel 4 (index 3): 1000us=-4rad, 2000us=+4rad (in position mode)");
    Logger::info("Channel 5 (index 4): >1800us=motor 1, <1300us=motor 2, 1300-1800us=both");
    Logger::info("Channel 6 (index 5): >1800us=set current position as zero");
    
    displayMgr.showReady();
    delay(50);
}

// ----- Main Loop -----
void loop() {
    static bool firstLoop = true;
    if (firstLoop) {
        Logger::info("Main loop started");
        firstLoop = false;
    }
    
    // Update motor controller (handles all motor logic)
    motorController.update();
    
    // Update display
    if (motorController.areMotorsInitialized()) {
        displayMgr.update(
            motorController.isInPositionMode(),
            motorController.getCurrentMotorSelection(),
            motorController.getMotor1Feedback(),
            motorController.getMotor2Feedback()
        );
    } else {
        displayMgr.showError("Motors not initialized!");
    }
    
    // Small delay to prevent tight looping
    delay(5);
} 