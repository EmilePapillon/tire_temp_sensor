#pragma once
#include "config.hh"
#include "logger.hh"

/// @file arduino_logger.hh
/// @brief Board glue: the Logger shape (see logger.hh) over Serial.

/// @brief Serial logger with a level threshold.
///
/// Default-constructible so lib/ code can own one; the threshold comes straight
/// from config::log_level.
class ArduinoLogger {
public:
    /// @brief Create a logger.
    /// @param level Lowest level that is printed; lower levels are dropped.
    explicit ArduinoLogger(LogLevel level = config::log_level);

    /// @brief Print "LEVEL: message" on Serial if @p level meets the threshold.
    /// @param level Severity of the message.
    /// @param message NUL-terminated text.
    void log(LogLevel level, const char* message);

private:
    LogLevel level_;  ///< Threshold below which messages are dropped.
};

static_assert(is_logger<ArduinoLogger>::value, "ArduinoLogger must satisfy the Logger shape");
