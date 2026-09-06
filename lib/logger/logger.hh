#pragma once
#include <cstdint>
#include <type_traits>
#include <utility>

/// @file logger.hh
/// @brief Logging shape shared by lib/ code.
///
/// Portable: no Arduino dependency. Concrete loggers (e.g. ArduinoLogger in
/// include/) implement the shape structurally; nothing inherits from anything.
///
/// "Logger" shape:
/// @code
///     void log(LogLevel level, const char* message);
/// @endcode
///
/// Compile-time dispatch: consumers take the logger as a template parameter and
/// `static_assert(is_logger<T>::value)` so a mismatch is reported at the
/// instantiation site rather than deep inside a template body.

/// @brief Severity of a log message, lowest first.
enum class LogLevel : uint8_t {
    DEBUG,  ///< Per-frame chatter; off by default.
    INFO,   ///< Lifecycle milestones (boot, init, connection).
    WARN,   ///< Recoverable problems the firmware worked around.
    ERROR,  ///< Failures that stop a feature or the whole board.
};

/// @brief Human-readable name of a log level, for prefixing messages.
/// @param level The level to name.
/// @return A static, upper-case string such as "INFO"; never null.
const char* log_level_name(LogLevel level);

/// @brief Trait: does @p T implement the Logger shape?
/// @tparam T Candidate logger type.
template <typename T, typename = void>
struct is_logger : std::false_type {};

/// @brief Specialisation selected when `T::log(LogLevel, const char*)` is callable.
/// @tparam T Candidate logger type.
template <typename T>
struct is_logger<T, std::void_t<
    decltype(std::declval<T&>().log(std::declval<LogLevel>(), std::declval<const char*>()))
>> : std::true_type {};

/// @brief Logger that discards everything. Default for lib/ consumers.
class NullLogger {
public:
    /// @brief Discard a message.
    /// @param level Ignored.
    /// @param message Ignored.
    void log(LogLevel level, const char* message);
};

static_assert(is_logger<NullLogger>::value, "NullLogger must satisfy the Logger shape");

#include "logger.inl"
