#pragma once
#include <Arduino.h>  // for Serial
#include "config.hh"
#include "logger.hh"

/// @brief Board glue: the "Logger" shape (see logger.hh) over Serial.
///
/// Default-constructible so lib/ code can own one; the threshold comes straight
/// from config::log_level.
class ArduinoLogger {
public:
    explicit ArduinoLogger(LogLevel level = config::log_level) : level_(level) {}

    void log(LogLevel level, const char* message) {
        if (level < level_) {
            return;
        }
        Serial.print(log_level_name(level));
        Serial.print(": ");
        Serial.println(message);
    }

private:
    LogLevel level_;
};

static_assert(is_logger<ArduinoLogger>::value, "ArduinoLogger must satisfy the Logger shape");
