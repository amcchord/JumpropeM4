#include "DisplayManager.h"
#include "Logger.h"

DisplayManager::DisplayManager(Adafruit_SSD1306& disp) : display(disp), lastUpdateTime(0) {
}

bool DisplayManager::begin() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Logger::error("SSD1306 OLED display initialization failed");
        return false;
    } else {
        Logger::info("SSD1306 OLED display initialized");
        showInitializing();
        return true;
    }
}

void DisplayManager::update(bool inPositionMode, MotorSelection currentMotorSelection, 
                           const MotorRawFeedback& motor1_feedback, 
                           const MotorRawFeedback& motor2_feedback) {
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= DISPLAY_UPDATE_RATE_MS) {
        updateMotorDisplay(inPositionMode, currentMotorSelection, motor1_feedback, motor2_feedback);
        lastUpdateTime = currentTime;
    }
}

void DisplayManager::showInitializing() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("RS03 Dual Motor");
    display.println("Control System");
    display.println("");
    display.println("Initializing...");
    display.display();
}

void DisplayManager::showReady() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("System Ready!");
    display.println("");
    display.println("Waiting for");
    display.println("Spektrum control input");
    display.display();
}

void DisplayManager::showError(const String& errorMessage) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ERROR:");
    display.println(errorMessage);
    display.display();
}

void DisplayManager::showNoSignal() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("NO SIGNAL");
    display.println("");
    display.println("Check Spektrum");
    display.println("transmitter");
    display.display();
}

void DisplayManager::forceUpdate() {
    lastUpdateTime = 0; // Force next update
}

void DisplayManager::updateMotorDisplay(bool inPositionMode, MotorSelection currentMotorSelection,
                                       const MotorRawFeedback& motor1_feedback, 
                                       const MotorRawFeedback& motor2_feedback) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    
    // Display mode and motor selection at the top
    display.println(inPositionMode ? "MODE: POSITION" : "MODE: VELOCITY");
    
    String motorSelectionText;
    switch (currentMotorSelection) {
        case MOTOR_1_ONLY: motorSelectionText = "MOTOR: M1 ONLY"; break;
        case MOTOR_2_ONLY: motorSelectionText = "MOTOR: M2 ONLY"; break;
        case BOTH_MOTORS: motorSelectionText = "MOTORS: M1 & M2"; break;
    }
    display.println(motorSelectionText);
    
    // Draw a line separator below the header
    display.drawLine(0, 16, 127, 16, SSD1306_WHITE);
    
    // Setup column headers
    display.setCursor(0, 18);
    display.print("MOTOR 1");
    display.setCursor(75, 18);
    display.print("MOTOR 2");
    
    // Draw a vertical separator between columns
    display.drawLine(64, 16, 64, 63, SSD1306_WHITE);
    
    // Set up data labels
    display.setCursor(0, 28);
    display.print("Pos:");
    display.setCursor(75, 28);
    display.print("Pos:");
    
    display.setCursor(0, 38);
    display.print("Vel:");
    display.setCursor(75, 38);
    display.print("Vel:");
    
    display.setCursor(0, 48);
    display.print("Cur:");
    display.setCursor(75, 48);
    display.print("Cur:");
    
    // Display Motor 1 data values (right-aligned in left column)
    if (motor1_feedback.received) {
        // Position
        String posStr = String(motor1_feedback.position, 2);
        int posWidth = posStr.length() * 6; // approx 6 pixels per character
        display.setCursor(59 - posWidth, 28);
        display.print(posStr);
        
        // Velocity
        String velStr = String(motor1_feedback.velocity, 2);
        int velWidth = velStr.length() * 6;
        display.setCursor(59 - velWidth, 38);
        display.print(velStr);
        
        // Current
        String curStr = String(motor1_feedback.current, 2);
        int curWidth = curStr.length() * 6;
        display.setCursor(59 - curWidth, 48);
        display.print(curStr);
    } else {
        display.setCursor(30, 38);
        display.print("NO DATA");
    }
    
    // Display Motor 2 data values (right-aligned in right column)
    if (motor2_feedback.received) {
        // Position
        String posStr = String(motor2_feedback.position, 2);
        int posWidth = posStr.length() * 6;
        display.setCursor(124 - posWidth, 28);
        display.print(posStr);
        
        // Velocity
        String velStr = String(motor2_feedback.velocity, 2);
        int velWidth = velStr.length() * 6;
        display.setCursor(124 - velWidth, 38);
        display.print(velStr);
        
        // Current
        String curStr = String(motor2_feedback.current, 2);
        int curWidth = curStr.length() * 6;
        display.setCursor(124 - curWidth, 48);
        display.print(curStr);
    } else {
        display.setCursor(95, 38);
        display.print("NO DATA");
    }
    
    display.display();
} 