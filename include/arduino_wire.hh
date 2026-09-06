#pragma once
#include <cstddef>
#include <cstdint>
#include "i_wire.hh"

/// @file arduino_wire.hh
/// @brief Board glue: the Wire shape (see i_wire.hh) over Arduino's global TwoWire.

/// @brief Thin forwarding wrapper around the Arduino `Wire` object.
///
/// Named ArduinoWire rather than Wire so it cannot be confused with, or shadow,
/// the framework's global instance of the same name.
class ArduinoWire {
public:
    /// @brief What begin() found on the bus before starting it.
    enum class BusRecovery : uint8_t {
        NotNeeded,  ///< SDA was high: the bus was idle.
        Recovered,  ///< SDA was held low; clocking SCL freed it.
        Failed,     ///< SDA still low after nine clocks; expect the sensor to fail init.
    };

    /// @brief Free a slave holding SDA low, then start the bus (TwoWire::begin).
    ///
    /// A reset mid-transfer (reflash, watchdog) leaves the sensor waiting for
    /// clocks with SDA pinned low; TWI cannot start on a busy bus and only a
    /// power cycle would clear it. Clocking SCL up to nine times lets the slave
    /// finish its byte, then a STOP condition releases the bus.
    void begin();

    /// @brief Outcome of the recovery check. Valid after begin().
    BusRecovery recovery() const;

    /// @brief Set the bus clock.
    /// @param freq_hz Frequency in Hz.
    void set_clock(uint32_t freq_hz);

    /// @brief Start queueing bytes for a device.
    /// @param address 7-bit I2C address.
    void begin_transmission(uint8_t address);

    /// @brief Send the queued bytes.
    /// @param stop Whether to release the bus with a STOP condition.
    /// @return Arduino status: 0 ok, 1 tx overflow, 2 address NACK, 3 data NACK, 4 other, 5 timeout.
    int end_transmission(bool stop);

    /// @brief Read bytes from a device into the receive buffer.
    /// @param address 7-bit I2C address.
    /// @param quantity Bytes requested.
    /// @return Bytes actually received.
    std::size_t request_from(uint8_t address, std::size_t quantity);

    /// @brief Queue one byte for transmission.
    /// @param data The byte.
    /// @return Bytes queued (1).
    std::size_t write(uint8_t data);

    /// @brief Queue several bytes for transmission.
    /// @param data Source bytes.
    /// @param quantity Number of bytes.
    /// @return Bytes queued.
    std::size_t write(const uint8_t* data, std::size_t quantity);

    /// @brief Bytes waiting in the receive buffer.
    /// @return Count, 0 if none.
    int available();

    /// @brief Pop one byte from the receive buffer.
    /// @return The byte, or -1 if none.
    int read();

    /// @brief Busy-wait.
    /// @param us Microseconds.
    void delay_microseconds(uint32_t us);

private:
    BusRecovery recovery_ = BusRecovery::NotNeeded;  ///< Set by begin().
};

static_assert(is_wire<ArduinoWire>::value, "ArduinoWire must satisfy the Wire shape");
