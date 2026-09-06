// Inline definitions for mock_i2c_adapter.hh. Included by the header; do not include directly.
#pragma once

inline I2cStatus MockI2CAdapter::init(uint32_t freq_khz) {
    initialised = true;
    init_freq_khz = freq_khz;
    return I2cStatus::Success;
}

inline I2cStatus MockI2CAdapter::read(uint8_t address, uint16_t reg, std::size_t length, uint16_t* buffer) {
    reads.push_back({address, reg, length});
    if (read_error != I2cStatus::Success) {
        return read_error;
    }
    for (std::size_t i = 0; i < length; i++) {
        const auto it = registers.find(static_cast<uint16_t>(reg + i));
        buffer[i] = (it == registers.end()) ? 0 : it->second;
    }
    return I2cStatus::Success;
}

inline I2cStatus MockI2CAdapter::write(uint8_t address, uint16_t reg, uint16_t value) {
    writes.push_back({address, reg, value});
    if (write_error != I2cStatus::Success) {
        return write_error;
    }
    registers[reg] = value;
    if (on_write) {
        on_write(reg, value);
    }
    return write_verdict;
}

inline void MockI2CAdapter::load_eeprom(const std::array<uint16_t, mlx90641::eeprom_size>& decoded) {
    for (std::size_t i = 0; i < decoded.size(); i++) {
        const uint16_t word = (i < 16) ? decoded[i] : mlx90641::hamming_encode(decoded[i]);
        registers[static_cast<uint16_t>(mlx90641::eeprom_start_address + i)] = word;
    }
}

inline bool MockI2CAdapter::was_read(uint16_t reg) const {
    for (const auto& r : reads) {
        if (r.reg == reg) {
            return true;
        }
    }
    return false;
}

inline bool MockI2CAdapter::was_written(uint16_t reg, uint16_t value) const {
    for (const auto& w : writes) {
        if (w.reg == reg && w.value == value) {
            return true;
        }
    }
    return false;
}
