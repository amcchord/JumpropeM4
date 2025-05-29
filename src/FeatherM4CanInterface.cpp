#include "FeatherM4CanInterface.h"
#include "Logger.h"
#include <CANSAME5x.h>

// External CAN object defined in main.cpp
extern CANSAME5x CAN;

FeatherM4CanInterface::FeatherM4CanInterface() : canBus(CAN) {
}

bool FeatherM4CanInterface::sendFrame(const CanFrame& frame) {
    Logger::debug("Sending CAN frame ID=0x" + String(frame.id, HEX) + 
        ", extended=" + String(frame.is_extended ? "true" : "false") + 
        ", DLC=" + String(frame.dlc));
    
    // Check ID range for extended IDs (29-bit max)
    if (frame.is_extended && frame.id > 0x1FFFFFFF) {
        Logger::error("ERROR: Extended ID 0x" + String(frame.id, HEX) + " exceeds 29-bit maximum");
        return false;
    }
    
    // Use appropriate method based on ID type
    bool beginSuccess;
    if (frame.is_extended) {
        uint32_t id29bit = frame.id & 0x1FFFFFFF;
        Logger::verbose("Starting extended packet with ID 0x" + String(id29bit, HEX));
        beginSuccess = canBus.beginExtendedPacket(id29bit);
    } else {
        uint32_t id11bit = frame.id & 0x7FF;
        Logger::verbose("Starting standard packet with ID 0x" + String(id11bit, HEX));
        beginSuccess = canBus.beginPacket(id11bit);
    }
    
    if (!beginSuccess) {
        Logger::error("Failed to begin CAN packet");
        return false;
    }
    
    // Write the data if there is any
    if (frame.dlc > 0) {
        String dataStr = "Data bytes: ";
        for (int i = 0; i < frame.dlc; i++) {
            if (frame.data[i] < 0x10) dataStr += "0";
            dataStr += String(frame.data[i], HEX) + " ";
        }
        Logger::verbose(dataStr);
        canBus.write(frame.data, frame.dlc);
    }
    
    // End the packet and transmit
    if (!canBus.endPacket()) {
        Logger::error("Failed to send CAN packet");
        return false;
    }
    
    Logger::verbose("CAN frame sent successfully");
    return true;
} 