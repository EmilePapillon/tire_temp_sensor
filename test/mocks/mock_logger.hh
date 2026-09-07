#pragma once
#include <cstddef>
#include <string>
#include <vector>
#include "logger.hh"

/// @file mock_logger.hh
/// @brief Recording Logger double for tests that assert on what a driver reported.

/// @brief Records every logged message.
///
/// Consumers own their logger and default-construct it, so the log is static:
/// a test clears it, runs the code under test, then inspects it.
class MockLogger {
public:
    /// @brief One recorded log() call.
    struct Record {
        LogLevel level;       ///< Severity the caller used.
        std::string message;  ///< Message text.
    };

    /// @brief Every message logged since the last clear(), in order.
    static inline std::vector<Record> messages{};

    /// @brief Record a message.
    /// @param level Severity.
    /// @param message NUL-terminated text.
    void log(LogLevel level, const char* message);

    /// @brief Drop every recorded message.
    static void clear();

    /// @brief Whether a message of @p level containing @p substring was recorded.
    /// @param level Severity to match.
    /// @param substring Text to look for anywhere in the message.
    /// @return True if such a message was recorded.
    static bool logged(LogLevel level, const char* substring);
};

static_assert(is_logger<MockLogger>::value, "MockLogger must satisfy the Logger shape");

#include "mocks/mock_logger.inl"
