#pragma once
#include <cstdint>
#include <type_traits>
#include <utility>

// Logging shape shared by lib/ code.
//
// Portable: no Arduino dependency. Concrete loggers (e.g. ArduinoLogger in
// include/) implement the shape structurally; nothing inherits from anything.
//
// "Logger" shape:
//     void log(LogLevel level, const char* message);
//
// Compile-time dispatch: consumers take the logger as a template parameter and
// static_assert(is_logger<T>::value) so a mismatch is reported at the
// instantiation site rather than deep inside a template body.

enum class LogLevel : uint8_t {
    DEBUG,
    INFO,
    WARN,
    ERROR,
};

inline const char* log_level_name(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO";
        case LogLevel::WARN:  return "WARN";
        case LogLevel::ERROR: return "ERROR";
    }
    return "?";
}

template <typename T, typename = void>
struct is_logger : std::false_type {};

template <typename T>
struct is_logger<T, std::void_t<
    decltype(std::declval<T&>().log(std::declval<LogLevel>(), std::declval<const char*>()))
>> : std::true_type {};

/// @brief Logger that discards everything. Default for lib/ consumers.
class NullLogger {
public:
    void log(LogLevel, const char*) {}
};

static_assert(is_logger<NullLogger>::value, "NullLogger must satisfy the Logger shape");
