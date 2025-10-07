#include "logger.hh"
#include <Arduino.h> // for Serial

class ArduinoLogger : public Logger {
public:
    ArduinoLogger(Level level = Level::INFO) : Logger(level) {}
    void log(Level level, const char* message) override {
        const char* level_str = "";
        if (log_level_ > level) {
            return; // Skip messages below the current log level
        }
        switch (level) {
            case Level::DEBUG: level_str = "DEBUG"; break;
            case Level::INFO:  level_str = "INFO";  break;
            case Level::WARN:  level_str = "WARN";  break;
            case Level::ERROR: level_str = "ERROR"; break;
        }
        Serial.print(level_str);
        Serial.print(": ");
        Serial.println(message);
    }
};