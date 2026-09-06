#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <vector>
#include "fixtures/mlx90641_eeprom_fixture.hh"
#include "i2c_adapter.hh"

/// @file mock_i2c_adapter.hh
/// @brief Register-file I2CAdapter double for MLX90641Sensor tests.

/// @brief Serves and records 16-bit register reads and writes.
///
/// Reads serve words from `registers` (0 when absent); writes store into it and
/// fire `on_write`, which tests use to emulate device-side behaviour such as
/// the status register clearing its data-ready bit.
class MockI2CAdapter {
public:
    /// @brief One recorded read() call.
    struct ReadRecord {
        uint8_t address;     ///< Device address.
        uint16_t reg;        ///< First register.
        std::size_t length;  ///< Words requested.
    };
    /// @brief One recorded write() call.
    struct WriteRecord {
        uint8_t address;  ///< Device address.
        uint16_t reg;     ///< Register.
        uint16_t value;   ///< Word written.
    };

    // --- scripting -----------------------------------------------------------
    std::map<uint16_t, uint16_t> registers;               ///< Register file; missing = 0.
    I2cStatus read_error = I2cStatus::Success;            ///< Returned by every read() when not Success.
    I2cStatus write_error = I2cStatus::Success;           ///< Returned instead of applying a write when not Success.
    I2cStatus write_verdict = I2cStatus::Success;         ///< Returned after applying a write (e.g. VerifyMismatch).
    std::function<void(uint16_t reg, uint16_t value)> on_write;  ///< Called after each applied write.

    // --- recording -----------------------------------------------------------
    bool initialised = false;         ///< init() was called.
    uint32_t init_freq_khz = 0;       ///< Frequency passed to init().
    std::vector<ReadRecord> reads;    ///< Every read(), in order.
    std::vector<WriteRecord> writes;  ///< Every write(), in order.

    /// @brief Record the bus start.
    /// @param freq_khz Requested bus frequency.
    /// @return Success.
    I2cStatus init(uint32_t freq_khz);

    /// @brief Serve @p length words from the register file.
    /// @param address Device address (recorded only).
    /// @param reg First register.
    /// @param length Words to read.
    /// @param buffer Destination.
    /// @return read_error if set, else Success.
    I2cStatus read(uint8_t address, uint16_t reg, std::size_t length, uint16_t* buffer);

    /// @brief Store a word and fire on_write.
    /// @param address Device address (recorded only).
    /// @param reg Register.
    /// @param value Word to store.
    /// @return write_error if set (nothing stored), else write_verdict.
    I2cStatus write(uint8_t address, uint16_t reg, uint16_t value);

    /// @brief Serve a decoded EEPROM image as the raw (Hamming-encoded) words the sensor stores.
    /// @param decoded 832 words, 11-bit payloads as the parser fixture holds them.
    void load_eeprom(const std::array<uint16_t, mlx90641::eeprom_size>& decoded);

    /// @brief Whether any read() started at @p reg.
    /// @param reg Register to look for.
    /// @return True if found.
    bool was_read(uint16_t reg) const;

    /// @brief Whether any write() stored @p value at @p reg.
    /// @param reg Register to look for.
    /// @param value Word to look for.
    /// @return True if found.
    bool was_written(uint16_t reg, uint16_t value) const;
};

static_assert(is_i2c_adapter<MockI2CAdapter>::value, "MockI2CAdapter must satisfy the I2CAdapter shape");

#include "mocks/mock_i2c_adapter.inl"
