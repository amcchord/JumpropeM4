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
SpektrumSatelliteReader spektrumReader(SPEKTRUM_SERIAL, 7, MIN_PULSE, MAX_PULSE, MID_PULSE);

// ----- Setup -----
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // Wait for serial connection
    }
    
    // Initialize logger
    Logger::init();
    Logger::info("Simple RC Channel Reader - Starting");
    
    // Initialize OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Logger::error("SSD1306 allocation failed");
        Serial.println("OLED Display initialization failed!");
        while(1) {
            delay(1000);
        }
    }
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("RC Channel Reader");
    display.println("Initializing...");
    display.display();
    
    // Initialize NeoPixel
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(50, 50, 0)); // Yellow while initializing
    pixels.show();
    Logger::info("NeoPixel initialized");
    
    // Initialize RC input
    if (!spektrumReader.begin()) {
        Logger::error("Spektrum reader initialization failed!");
        Serial.println("RC Receiver initialization failed!");
        pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red for error
        pixels.show();
        
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("RC Init Failed!");
        display.display();
        
        while(1) {
            delay(1000);
        }
    }
    
    // System ready
    pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green indicates ready
    pixels.show();
    Logger::info("Setup complete. Ready to read RC channels.");
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("RC Reader Ready");
    display.display();
    delay(1000);
    
    Serial.println("RC Channel Reader Ready");
    Serial.println("Reading first 7 channels...");
    Serial.println("Ch1  Ch2  Ch3  Ch4  Ch5  Ch6  Ch7");
}

// ----- Main Loop -----
void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastSerialPrint = 0;
    static unsigned long lastStatsLog = 0;
    
    // Update RC receiver
    spektrumReader.update();
    
    // Check if we're receiving signal
    if (!spektrumReader.isReceiving()) {
        // No signal - show error
        pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red
        pixels.show();
        
        // Update display every 200ms when no signal
        if (millis() - lastUpdate >= 200) {
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("RC Channel Reader");
            display.println("");
            display.println("NO SIGNAL");
            display.println("Check RC connection");
            display.display();
            lastUpdate = millis();
        }
        
        // Print to serial every 1000ms when no signal
        if (millis() - lastSerialPrint >= 1000) {
            Serial.println("NO RC SIGNAL");
            lastSerialPrint = millis();
        }
        
        delay(50);
        return;
    }
    
    // We have signal - show green LED
    pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green
    pixels.show();
    
    // Read all channels (up to 12)
    int numChannelsToShow = min(12, spektrumReader.getChannelCount());
    int channels[12];
    for (int i = 0; i < numChannelsToShow; i++) {
        channels[i] = spektrumReader.getChannel(i);
    }
    
    // Get decoded special channels
    int mode = spektrumReader.getModeFromChannel4();
    auto switches = spektrumReader.getSwitchStatesFromChannel6();
    
    // Update OLED display every 150ms
    if (millis() - lastUpdate >= 150) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print("RC Channels (");
        display.print(numChannelsToShow);
        display.println(")");
        
        // Show first 3 channels
        for (int i = 0; i < min(3, numChannelsToShow); i++) {
            display.print("Ch");
            display.print(i + 1);
            display.print(":");
            display.println(channels[i]);
        }
        
        // Show MODE instead of channel 4
        if (numChannelsToShow > 3) {
            display.print("MODE:");
            if (mode > 0) {
                display.println(mode);
            } else {
                display.println("?");
            }
        }
        
        // Show channel 5
        if (numChannelsToShow > 4) {
            display.print("Ch5:");
            display.println(channels[4]);
        }
        
        // Show switch states instead of channel 6
        if (numChannelsToShow > 5) {
            display.print("SW A:");
            display.print(switches.switchA ? "ON " : "OFF");
            display.print(" B:");
            display.println(switches.switchB ? "ON" : "OFF");
        }
        
        // Show channel 7 if available
        if (numChannelsToShow > 6) {
            display.print("Ch7:");
            display.println(channels[6]);
        }
        
        display.display();
        lastUpdate = millis();
    }
    
    // Print to serial console every 100ms
    if (millis() - lastSerialPrint >= 100) {
        // Print channels 1-3
        for (int i = 0; i < min(3, numChannelsToShow); i++) {
            Serial.print(channels[i]);
            Serial.print("  ");
        }
        
        // Print MODE instead of channel 4
        if (numChannelsToShow > 3) {
            Serial.print("MODE:");
            if (mode > 0) {
                Serial.print(mode);
            } else {
                Serial.print("?");
            }
            Serial.print("  ");
        }
        
        // Print channel 5
        if (numChannelsToShow > 4) {
            Serial.print(channels[4]);
            Serial.print("  ");
        }
        
        // Print switch states instead of channel 6
        if (numChannelsToShow > 5) {
            Serial.print("A:");
            Serial.print(switches.switchA ? "ON" : "OFF");
            Serial.print(",B:");
            Serial.print(switches.switchB ? "ON" : "OFF");
            Serial.print("  ");
        }
        
        // Print remaining channels
        for (int i = 6; i < numChannelsToShow; i++) {
            Serial.print(channels[i]);
            if (i < numChannelsToShow - 1) {
                Serial.print("  ");
            }
        }
        
        Serial.println();
        lastSerialPrint = millis();
    }
    
    // // Print frame statistics every 5 seconds for debugging
    // if (millis() - lastStatsLog >= 5000) {
    //     auto stats = spektrumReader.getFrameStats();
    //     Serial.println();
    //     Serial.println("=== FRAME STATISTICS ===");
    //     Serial.print("Total Frames: "); Serial.println(stats.totalFrames);
    //     Serial.print("Valid Frames: "); Serial.println(stats.validFrames);
    //     Serial.print("Format Samples: "); Serial.println(stats.formatDetectSamples);
    //     Serial.print("Format: "); Serial.println(stats.is11Bit ? "11-bit" : "10-bit");
    //     Serial.print("Detected Channels: "); Serial.println(stats.detectedChannels);
    //     Serial.print("Last Frame: "); Serial.print(millis() - stats.lastFrameTime); Serial.println("ms ago");
        
    //     // Calculate frame rate
    //     if (stats.totalFrames > 0 && stats.lastFrameTime > 0) {
    //         float frameRate = 1000.0 * stats.validFrames / stats.lastFrameTime;
    //         Serial.print("Frame Rate: "); Serial.print(frameRate, 1); Serial.println(" Hz");
    //     }
        
    //     Serial.println("========================");
    //     Serial.println();
    //     lastStatsLog = millis();
    // }
    
    // Small delay to prevent tight looping
    delay(10);
} 