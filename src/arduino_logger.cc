#include "arduino_logger.hh"
#include <Arduino.h>  // for Serial

ArduinoLogger::ArduinoLogger(LogLevel level) : level_(level) {}

void ArduinoLogger::log(LogLevel level, const char* message) {
    if (level < level_) {
        return;
    }
    Serial.print(log_level_name(level));
    Serial.print(": ");
    Serial.println(message);
}
