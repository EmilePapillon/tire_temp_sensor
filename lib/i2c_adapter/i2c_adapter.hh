#pragma once
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include "i_wire.hh"

/// @file i2c_adapter.hh
/// @brief Register-level I2C protocol used by the MLX90641.
///
/// 16-bit register addresses, 16-bit big-endian words, reads split into
/// 32-byte chunks. Portable: talks to the bus through a WireT that satisfies
/// the Wire shape (see i_wire.hh), so the byte assembly and error handling are
/// unit tested on the host against MockWire.

/// @brief Outcome of a bus operation. Success is 0, as the coding guidelines require.
enum class I2cStatus : uint8_t {
    Success = 0,     ///< The operation completed.
    Nack,            ///< The device did not acknowledge its address or a data byte.
    BusError,        ///< Transmit buffer overflow, timeout or other bus-level failure.
    NoData,          ///< The device returned no bytes on a read.
    VerifyMismatch,  ///< A write's read-back differed from the written value.
};

/// @brief Human-readable name of an I2C status, for log messages.
/// @param status The status to name.
/// @return A static lower-case string such as "nack"; never null.
const char* i2c_status_name(I2cStatus status);

/// @brief Helpers for the I2CAdapter shape trait.
namespace detail {
/// @brief SFINAE guard that only resolves when @p Expr is exactly I2cStatus.
/// @tparam Expr The decltype of a candidate member call.
template <typename Expr>
using returns_i2c_status = std::enable_if_t<std::is_same<Expr, I2cStatus>::value>;
}  // namespace detail

/// @brief Trait: does @p T implement the I2CAdapter shape consumed by MLX90641Sensor?
///
/// Required members:
/// @code
///     I2cStatus init(uint32_t freq_khz);
///     I2cStatus read(uint8_t device_address, uint16_t start_register, std::size_t length, uint16_t* buffer);
///     I2cStatus write(uint8_t device_address, uint16_t reg, uint16_t value);
/// @endcode
/// @tparam T Candidate adapter type.
template <typename T, typename = void>
struct is_i2c_adapter : std::false_type {};

/// @brief Specialisation selected when all three members exist and return I2cStatus.
/// @tparam T Candidate adapter type.
template <typename T>
struct is_i2c_adapter<T, std::void_t<
    detail::returns_i2c_status<decltype(std::declval<T&>().init(std::declval<uint32_t>()))>,
    detail::returns_i2c_status<decltype(std::declval<T&>().read(
        std::declval<uint8_t>(), std::declval<uint16_t>(), std::declval<std::size_t>(), std::declval<uint16_t*>()))>,
    detail::returns_i2c_status<decltype(std::declval<T&>().write(
        std::declval<uint8_t>(), std::declval<uint16_t>(), std::declval<uint16_t>()))>
>> : std::true_type {};

/// @brief 16-bit register access over a Wire-shaped bus.
/// @tparam WireT A type satisfying is_wire; injected by reference so tests can script it.
template <typename WireT>
class I2CAdapter {
    static_assert(is_wire<WireT>::value, "WireT must implement the Wire shape (see i_wire.hh)");

public:
    /// @brief Largest single read the MLX90641 serves; longer reads are chunked to this.
    static constexpr std::size_t max_chunk_bytes = 32;

    /// @brief Bind to a bus. The bus is not started until init().
    /// @param wire The bus implementation; must outlive this adapter.
    explicit I2CAdapter(WireT& wire);

    /// @brief Start the bus and set its clock.
    /// @param freq_khz Bus frequency in kHz (e.g. 400).
    /// @return Always I2cStatus::Success; kept as a status for shape uniformity.
    I2cStatus init(uint32_t freq_khz);

    /// @brief Change the bus clock on an already started bus.
    /// @param freq_khz Bus frequency in kHz.
    void set_frequency(uint32_t freq_khz);

    /// @brief Read consecutive 16-bit words.
    /// @param device_address 7-bit I2C address of the device.
    /// @param start_register First register to read.
    /// @param length Number of 16-bit words to read.
    /// @param buffer Destination for @p length words, assembled big-endian to host order.
    /// @return Success, or the first failure encountered (Nack, BusError, NoData).
    I2cStatus read(uint8_t device_address, uint16_t start_register, std::size_t length, uint16_t* buffer);

    /// @brief Write one 16-bit word and read it back to verify.
    ///
    /// Read-back failures come back as the read's own status; a successful read
    /// that does not match returns VerifyMismatch. Callers writing to
    /// self-clearing registers (e.g. the MLX90641 status register) must expect
    /// and tolerate VerifyMismatch.
    /// @param device_address 7-bit I2C address of the device.
    /// @param reg Register to write.
    /// @param value Word to write.
    /// @return Success, VerifyMismatch, or the read-back's failure status.
    I2cStatus write(uint8_t device_address, uint16_t reg, uint16_t value);

private:
    WireT& wire_;  ///< The injected bus.
};

#include "i2c_adapter.inl"
