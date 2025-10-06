#pragma once

#include <cstdint>
#include <vector>
#include "i_wire.hh"

/// @brief Abstract interface for I2C communication
/// 
/// @note Uses an IWire instance for low-level operations
/// @note This provides the ability to interface with the Arduino Wire library or a mock of it for testing
class I2CAdapter {
public:
    I2CAdapter(IWire& wire) : wire_(wire) {}
     ~I2CAdapter() = default;

    // Read `length` 16-bit words starting from `start_register`
     bool read(uint8_t device_address,
                      uint16_t start_register,
                      std::size_t length,
                      uint16_t* buffer);

    // Write a 16-bit word to a register
     bool write(uint8_t device_address,
                       uint16_t reg,
                       uint16_t value);

    // Initialize the I2C bus
     bool init(int freq);

    // Set I2C bus frequency in kHz
     void set_frequency(int freq);

protected:
    IWire& wire_;
};
