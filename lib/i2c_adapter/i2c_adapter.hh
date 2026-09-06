#pragma once
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include "i_wire.hh"

// Register-level I2C protocol used by the MLX90641: 16-bit register addresses,
// 16-bit big-endian words, reads split into 32-byte chunks.
//
// Portable: talks to the bus through a WireT that satisfies the "Wire" shape
// (see i_wire.hh), so the byte assembly and error handling here are unit
// tested on the host against MockWire.
//

/// @brief Outcome of a bus operation. Success is 0, as the coding guidelines require.
enum class I2cStatus : uint8_t {
    Success = 0,
    Nack,            // the device did not acknowledge its address or a data byte
    BusError,        // transmit buffer overflow or other bus-level failure
    NoData,          // the device returned no bytes on a read
    VerifyMismatch,  // a write's read-back differed from the written value
};

inline const char* i2c_status_name(I2cStatus status) {
    switch (status) {
        case I2cStatus::Success:        return "success";
        case I2cStatus::Nack:           return "nack";
        case I2cStatus::BusError:       return "bus error";
        case I2cStatus::NoData:         return "no data";
        case I2cStatus::VerifyMismatch: return "verify mismatch";
    }
    return "?";
}

// "I2CAdapter" shape, as consumed by MLX90641Sensor:
//     I2cStatus init(uint32_t freq_khz);
//     I2cStatus read(uint8_t device_address, uint16_t start_register, std::size_t length, uint16_t* buffer);
//     I2cStatus write(uint8_t device_address, uint16_t reg, uint16_t value);
namespace detail {
template <typename Expr>
using returns_i2c_status = std::enable_if_t<std::is_same<Expr, I2cStatus>::value>;
}

template <typename T, typename = void>
struct is_i2c_adapter : std::false_type {};

template <typename T>
struct is_i2c_adapter<T, std::void_t<
    detail::returns_i2c_status<decltype(std::declval<T&>().init(std::declval<uint32_t>()))>,
    detail::returns_i2c_status<decltype(std::declval<T&>().read(
        std::declval<uint8_t>(), std::declval<uint16_t>(), std::declval<std::size_t>(), std::declval<uint16_t*>()))>,
    detail::returns_i2c_status<decltype(std::declval<T&>().write(
        std::declval<uint8_t>(), std::declval<uint16_t>(), std::declval<uint16_t>()))>
>> : std::true_type {};

template <typename WireT>
class I2CAdapter {
    static_assert(is_wire<WireT>::value, "WireT must implement the Wire shape (see i_wire.hh)");

public:
    static constexpr std::size_t max_chunk_bytes = 32;

    explicit I2CAdapter(WireT& wire) : wire_(wire) {}

    /// @brief Start the bus at `freq_khz` kHz.
    I2cStatus init(uint32_t freq_khz) {
        wire_.begin();
        set_frequency(freq_khz);
        wire_.delay_microseconds(1000);
        return I2cStatus::Success;
    }

    void set_frequency(uint32_t freq_khz) {
        wire_.set_clock(1000u * freq_khz);
    }

    /// @brief Read `length` 16-bit words starting at `start_register` into `buffer`.
    I2cStatus read(uint8_t device_address, uint16_t start_register, std::size_t length, uint16_t* buffer) {
        const std::size_t total_bytes = 2 * length;
        const std::size_t remainder = total_bytes % max_chunk_bytes;
        std::size_t chunks = total_bytes / max_chunk_bytes;
        if (remainder != 0) {
            chunks++;
        }

        uint16_t* out = buffer;
        for (std::size_t chunk = 0; chunk < chunks; chunk++) {
            const uint16_t address = static_cast<uint16_t>(start_register + (chunk * max_chunk_bytes) / 2);

            wire_.end_transmission(true);
            wire_.delay_microseconds(5);
            wire_.begin_transmission(device_address);
            wire_.write(static_cast<uint8_t>(address >> 8));
            wire_.write(static_cast<uint8_t>(address & 0x00FF));

            const int status = wire_.end_transmission(false);  // Arduino TwoWire status code
            if (status == 2 || status == 3) {
                return I2cStatus::Nack;  // address / data NACK
            }
            if (status != 0) {
                return I2cStatus::BusError;  // 1 = tx overflow, 4 = other, 5 = timeout
            }

            std::size_t num_bytes = max_chunk_bytes;
            if (chunk == chunks - 1 && remainder != 0) {
                num_bytes = remainder;
            }
            num_bytes = wire_.request_from(device_address, num_bytes);
            if (num_bytes == 0) {
                return I2cStatus::NoData;
            }

            for (std::size_t i = 0; i < num_bytes / 2; i++) {
                if (wire_.available()) {
                    const uint16_t high = static_cast<uint16_t>(wire_.read() << 8);
                    const uint16_t low = static_cast<uint16_t>(wire_.read());
                    *out++ = static_cast<uint16_t>(high | low);
                }
            }
        }
        return I2cStatus::Success;
    }

    /// @brief Write a 16-bit word to `reg` and read it back to verify.
    ///
    /// Read-back failures come back as the read's own status; a successful
    /// read that does not match returns VerifyMismatch. Callers writing to
    /// self-clearing registers (e.g. the MLX90641 status register) must expect
    /// and tolerate VerifyMismatch.
    I2cStatus write(uint8_t device_address, uint16_t reg, uint16_t value) {
        const uint8_t cmd[4] = {
            static_cast<uint8_t>(reg >> 8),
            static_cast<uint8_t>(reg & 0x00FF),
            static_cast<uint8_t>(value >> 8),
            static_cast<uint8_t>(value & 0x00FF),
        };

        wire_.end_transmission(true);
        wire_.begin_transmission(device_address);
        wire_.delay_microseconds(5);
        wire_.write(cmd, sizeof(cmd));
        wire_.end_transmission(true);

        uint16_t read_back = 0;
        const I2cStatus read_status = read(device_address, reg, 1, &read_back);
        if (read_status != I2cStatus::Success) {
            return read_status;
        }
        if (read_back != value) {
            return I2cStatus::VerifyMismatch;
        }
        return I2cStatus::Success;
    }

private:
    WireT& wire_;
};
