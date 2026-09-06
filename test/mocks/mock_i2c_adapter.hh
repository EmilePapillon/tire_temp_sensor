#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <vector>
#include "fixtures/mlx90641_eeprom_fixture.hh"
#include "i2c_adapter.hh"

// Register-file "I2CAdapter" double for MLX90641Sensor tests.
//
// Reads serve words from `registers` (0 when absent); writes store into it and
// fire `on_write`, which tests use to emulate device-side behaviour such as
// the status register clearing its data-ready bit.
class MockI2CAdapter {
public:
    struct ReadRecord {
        uint8_t address;
        uint16_t reg;
        std::size_t length;
    };
    struct WriteRecord {
        uint8_t address;
        uint16_t reg;
        uint16_t value;
    };

    // --- scripting -----------------------------------------------------------
    std::map<uint16_t, uint16_t> registers;
    int read_error = 0;
    int write_error = 0;
    std::function<void(uint16_t reg, uint16_t value)> on_write;

    // --- recording -----------------------------------------------------------
    bool initialised = false;
    uint32_t init_freq_khz = 0;
    std::vector<ReadRecord> reads;
    std::vector<WriteRecord> writes;

    int init(uint32_t freq_khz) {
        initialised = true;
        init_freq_khz = freq_khz;
        return 0;
    }

    int read(uint8_t address, uint16_t reg, std::size_t length, uint16_t* buffer) {
        reads.push_back({address, reg, length});
        if (read_error != 0) {
            return read_error;
        }
        for (std::size_t i = 0; i < length; i++) {
            const auto it = registers.find(static_cast<uint16_t>(reg + i));
            buffer[i] = (it == registers.end()) ? 0 : it->second;
        }
        return 0;
    }

    int write(uint8_t address, uint16_t reg, uint16_t value) {
        writes.push_back({address, reg, value});
        if (write_error != 0) {
            return write_error;
        }
        registers[reg] = value;
        if (on_write) {
            on_write(reg, value);
        }
        return 0;
    }

    /// @brief Serve a decoded EEPROM image as the raw (Hamming-encoded) words the sensor stores.
    void load_eeprom(const std::array<uint16_t, mlx90641::eeprom_size>& decoded) {
        for (std::size_t i = 0; i < decoded.size(); i++) {
            const uint16_t word = (i < 16) ? decoded[i] : mlx90641::hamming_encode(decoded[i]);
            registers[static_cast<uint16_t>(mlx90641::eeprom_start_address + i)] = word;
        }
    }

    bool was_read(uint16_t reg) const {
        for (const auto& r : reads) {
            if (r.reg == reg) {
                return true;
            }
        }
        return false;
    }

    bool was_written(uint16_t reg, uint16_t value) const {
        for (const auto& w : writes) {
            if (w.reg == reg && w.value == value) {
                return true;
            }
        }
        return false;
    }
};

static_assert(is_i2c_adapter<MockI2CAdapter>::value, "MockI2CAdapter must satisfy the I2CAdapter shape");
