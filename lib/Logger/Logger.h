#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "SystemConfig.h"

class Logger {
public:
    static void init();
    static void log(int level, const String& message);
    static void error(const String& message);
    static void info(const String& message);
    static void debug(const String& message);
    static void verbose(const String& message);
    static void warning(const String& message);
};

#endif // LOGGER_H 