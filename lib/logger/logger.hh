#pragma once

class Logger {
public:
    enum class Level {
        DEBUG,
        INFO,
        WARN,
        ERROR
    };

    Logger(Level level = Level::INFO) : log_level_(level) {}
    virtual ~Logger() = default;
    
    virtual void log(Level level, const char* message) = 0;

  protected:
    Level log_level_;
};