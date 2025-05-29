#include "Logger.h"

void Logger::init() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
}

void Logger::log(int level, const String& message) {
    if (level <= DEBUG_LEVEL) {
        String prefix;
        switch (level) {
            case LOG_ERROR:   prefix = "[ERROR] "; break;
            case LOG_WARNING: prefix = "[WARNING] "; break;
            case LOG_INFO:    prefix = "[INFO] "; break;
            case LOG_DEBUG:   prefix = "[DEBUG] "; break;
            case LOG_VERBOSE: prefix = "[VERBOSE] "; break;
            default:          prefix = "[LOG] "; break;
        }
        Serial.println(prefix + message);
        if (level == LOG_ERROR) {
            Serial.flush(); // Only flush for errors
        }
    }
}

void Logger::error(const String& message) {
    log(LOG_ERROR, message);
}

void Logger::info(const String& message) {
    log(LOG_INFO, message);
}

void Logger::debug(const String& message) {
    log(LOG_DEBUG, message);
}

void Logger::verbose(const String& message) {
    log(LOG_VERBOSE, message);
}

void Logger::warning(const String& message) {
    log(LOG_WARNING, message);
} 