#include "FeatherM4CanInterface.h"

// External CAN object defined in main.cpp
extern CANSAME5x CAN;

// Debug levels (must match main.cpp)
#define LOG_ERROR   1
#define LOG_INFO    2
#define LOG_DEBUG   3
#define LOG_VERBOSE 4

FeatherM4CanInterface::FeatherM4CanInterface() : canBus(CAN) {
}

bool FeatherM4CanInterface::sendFrame(const CanFrame& frame) {
    log(LOG_DEBUG, "Sending CAN frame ID=0x" + String(frame.id, HEX) + 
        ", extended=" + String(frame.is_extended ? "true" : "false") + 
        ", DLC=" + String(frame.dlc));
    
    // Check ID range for extended IDs (29-bit max)
    if (frame.is_extended && frame.id > 0x1FFFFFFF) {
        log(LOG_ERROR, "ERROR: Extended ID 0x" + String(frame.id, HEX) + " exceeds 29-bit maximum");
        return false;
    }
    
    // Use appropriate method based on ID type
    bool beginSuccess;
    if (frame.is_extended) {
        uint32_t id29bit = frame.id & 0x1FFFFFFF;
        log(LOG_VERBOSE, "Starting extended packet with ID 0x" + String(id29bit, HEX));
        beginSuccess = canBus.beginExtendedPacket(id29bit);
    } else {
        uint32_t id11bit = frame.id & 0x7FF;
        log(LOG_VERBOSE, "Starting standard packet with ID 0x" + String(id11bit, HEX));
        beginSuccess = canBus.beginPacket(id11bit);
    }
    
    if (!beginSuccess) {
        log(LOG_ERROR, "Failed to begin CAN packet");
        return false;
    }
    
    // Write the data if there is any
    if (frame.dlc > 0) {
        String dataStr = "Data bytes: ";
        for (int i = 0; i < frame.dlc; i++) {
            if (frame.data[i] < 0x10) dataStr += "0";
            dataStr += String(frame.data[i], HEX) + " ";
        }
        log(LOG_VERBOSE, dataStr);
        canBus.write(frame.data, frame.dlc);
    }
    
    // End the packet and transmit
    if (!canBus.endPacket()) {
        log(LOG_ERROR, "Failed to send CAN packet");
        return false;
    }
    
    log(LOG_VERBOSE, "CAN frame sent successfully");
    return true;
}

bool FeatherM4CanInterface::receiveFrame(CanFrame& frame, uint32_t timeout_ms) {
    unsigned long startTime = millis();
    
    while ((millis() - startTime) < timeout_ms || timeout_ms == 0) {
        int packetSize = canBus.parsePacket();
        if (packetSize) {
            frame.id = canBus.packetId();
            frame.is_extended = canBus.packetExtended();
            frame.dlc = packetSize;
            
            int i = 0;
            while (canBus.available() && i < 8) {
                frame.data[i++] = canBus.read();
            }
            
            if (frame.dlc < 8) {
                memset(frame.data + frame.dlc, 0, 8 - frame.dlc);
            }
            
            return true;
        }
        
        if (timeout_ms != 0) {
            delay(1);
        }
    }
    
    return false;
} 