#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

// Protocol-agnostic data handed from the application to whichever BLE tire
// protocol is active. Nothing here knows about packet layouts or UUIDs.

enum class WheelCorner : uint8_t {
    FL = 0,  // front left
    FR = 1,  // front right
    RL = 2,  // rear left
    RR = 3,  // rear right
};

struct DeviceIdentity {
    std::array<uint8_t, 6> mac_address;  // as reported by the radio (little-endian, [0] is the LSB)
    WheelCorner corner;
};

struct TireTelemetry {
    static constexpr std::size_t num_columns = 16;

    std::array<float, num_columns> column_temps_c;  // per-column average, degrees Celsius
    uint16_t battery_mv;
    uint8_t battery_pct;
    int16_t distance_mm;  // 0 if no distance sensor fitted
};
