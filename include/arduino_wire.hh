#pragma once
#include <cstddef>
#include <cstdint>
#include "i_wire.hh"

/// @brief Board glue: the "Wire" shape (see i_wire.hh) over Arduino's global TwoWire.
class ArduinoWire {
public:
    void begin();
    void set_clock(uint32_t freq_hz);
    void begin_transmission(uint8_t address);
    int end_transmission(bool stop);
    std::size_t request_from(uint8_t address, std::size_t quantity);
    std::size_t write(uint8_t data);
    std::size_t write(const uint8_t* data, std::size_t quantity);
    int available();
    int read();
    void delay_microseconds(uint32_t us);
};

static_assert(is_wire<ArduinoWire>::value, "ArduinoWire must satisfy the Wire shape");
