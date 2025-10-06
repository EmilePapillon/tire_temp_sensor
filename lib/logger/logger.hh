#pragma once

class Logger {
public:

    enum class Level {
        DEBUG,
        INFO,
        WARN,
        ERROR
    };  

    virtual void log(Level level, const char* message) = 0;
};