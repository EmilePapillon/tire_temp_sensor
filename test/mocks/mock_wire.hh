#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <vector>
#include "i_wire.hh"

/// @file mock_wire.hh
/// @brief Scripted Wire double for I2CAdapter tests.

/// @brief Emulates an I2C device with 16-bit registers holding big-endian words.
///
/// - a 2-byte transmission selects the register to read from next,
/// - a 4-byte transmission writes a word to a register,
/// - request_from() serves consecutive words from the selected register.
///
/// Knobs (end_transmission_status, fail_request, short_response_bytes, write_mask) inject the
/// failure modes the adapter is expected to translate into error codes.
class MockWire {
public:
    // --- scripting -----------------------------------------------------------
    std::map<uint16_t, uint16_t> registers;  ///< Register file served to reads; missing = 0.
    int end_transmission_status = 0;         ///< Returned after a register-select (2-byte) transmission.
    int write_end_transmission_status = 0;   ///< Returned after a register write (4-byte) transmission.
    bool fail_request = false;               ///< Make request_from() return 0 bytes.
    std::size_t short_response_bytes = 0;     ///< Return fewer bytes than requested when nonzero.
    uint16_t write_mask = 0xFFFF;            ///< Bits the "device" actually stores on write.

    // --- recording -----------------------------------------------------------
    bool begun = false;                              ///< begin() was called.
    uint32_t clock_hz = 0;                           ///< Last set_clock() value.
    uint32_t delay_us_total = 0;                     ///< Sum of delay_microseconds() calls.
    std::vector<std::vector<uint8_t>> transactions;  ///< Bytes of each begin/end_transmission pair.
    std::vector<std::size_t> request_sizes;          ///< Quantity of each request_from().

    /// @brief Record that the bus was started.
    void begin();
    /// @brief Record the clock.
    /// @param freq_hz Frequency in Hz.
    void set_clock(uint32_t freq_hz);
    /// @brief Open a transaction and start collecting written bytes.
    /// @param address 7-bit device address.
    void begin_transmission(uint8_t address);
    /// @brief Close the transaction: select a register (2 bytes) or store a word (4 bytes).
    /// @param stop Ignored.
    /// @return end_transmission_status for a register select, 0 otherwise.
    int end_transmission(bool stop);
    /// @brief Queue @p quantity bytes of consecutive words starting at the selected register.
    /// @param address Ignored.
    /// @param quantity Bytes requested.
    /// @return @p quantity, fewer bytes when short_response_bytes is set, or 0 when fail_request is set.
    std::size_t request_from(uint8_t address, std::size_t quantity);
    /// @brief Append one byte to the open transaction.
    /// @param data The byte.
    /// @return 1.
    std::size_t write(uint8_t data);
    /// @brief Append bytes to the open transaction.
    /// @param data Source bytes.
    /// @param quantity Number of bytes.
    /// @return @p quantity.
    std::size_t write(const uint8_t* data, std::size_t quantity);
    /// @brief Bytes queued for reading.
    /// @return Count.
    int available();
    /// @brief Pop one queued byte.
    /// @return The byte, or -1 if none.
    int read();
    /// @brief Accumulate the requested delay.
    /// @param us Microseconds.
    void delay_microseconds(uint32_t us);

private:
    bool in_transaction_ = false;     ///< Between begin_transmission() and end_transmission().
    uint8_t last_address_ = 0;        ///< Address of the open transaction.
    uint16_t selected_register_ = 0;  ///< Register the next request_from() reads from.
    std::vector<uint8_t> tx_;         ///< Bytes written in the open transaction.
    std::deque<uint8_t> rx_;          ///< Bytes queued for read().
};

static_assert(is_wire<MockWire>::value, "MockWire must satisfy the Wire shape");

#include "mocks/mock_wire.inl"
