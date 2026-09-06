// Inline definitions for logger.hh. Included by the header; do not include directly.
#pragma once

inline const char* log_level_name(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO";
        case LogLevel::WARN:  return "WARN";
        case LogLevel::ERROR: return "ERROR";
    }
    return "?";
}

inline void NullLogger::log(LogLevel, const char*) {}
