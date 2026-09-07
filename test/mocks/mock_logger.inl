// Inline definitions for mock_logger.hh. Included by the header; do not include directly.
#pragma once

inline void MockLogger::log(LogLevel level, const char* message) {
    messages.push_back({level, message == nullptr ? std::string() : std::string(message)});
}

inline void MockLogger::clear() { messages.clear(); }

inline bool MockLogger::logged(LogLevel level, const char* substring) {
    for (const auto& record : messages) {
        if (record.level == level && record.message.find(substring) != std::string::npos) {
            return true;
        }
    }
    return false;
}
