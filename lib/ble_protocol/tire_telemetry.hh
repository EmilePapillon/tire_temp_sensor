#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

/// @file tire_telemetry.hh
/// @brief Protocol-agnostic data handed from the application to whichever BLE
/// tire protocol is active. Nothing here knows about packet layouts or UUIDs.

/// @brief Which wheel a sensor board is mounted on.
enum class WheelCorner : uint8_t {
    FL = 0,  ///< front left
    FR = 1,  ///< front right
    RL = 2,  ///< rear left
    RR = 3,  ///< rear right
};

/// @brief What a protocol needs to name and place this board.
struct DeviceIdentity {
    std::array<uint8_t, 6> mac_address;  ///< As reported by the radio: little-endian, [0] is the LSB.
    WheelCorner corner;                  ///< Wheel position, from config.hh.
};

/// @brief One sample of everything the sensor publishes.
struct TireTelemetry {
    /// @brief Number of temperature columns across the tread.
    static constexpr std::size_t num_columns = 16;

    std::array<float, num_columns> column_temps_c;  ///< Per-column average, degrees Celsius, [0] = leftmost.
    uint16_t battery_mv;                            ///< Battery voltage in millivolts.
    uint8_t battery_pct;                            ///< Battery state of charge, 0..100.
    int16_t distance_mm;                            ///< Suspension travel in mm; 0 if no distance sensor fitted.
};
