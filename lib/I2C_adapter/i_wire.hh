// Abstract class to represent the wire library

#pragma once
#include <cstdint>
#include <cstddef>

class IWire {
public:
    virtual ~IWire() = default; 
    virtual void begin() = 0;
    virtual void setClock(uint32_t freq) = 0;
    virtual int endTransmission(bool stop = true) = 0;
    virtual void beginTransmission(uint8_t address) = 0;
    virtual uint8_t requestFrom(uint8_t address, std::size_t quantity) = 0;
    virtual std::size_t write(uint8_t data) = 0;
    virtual std::size_t write(const uint8_t* data, std::size_t quantity) = 0;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;
    virtual void delayMicroseconds(int us) = 0;
};