// Inline definitions for battery_lipo.hh. Included by the header; do not include directly.
#pragma once

inline uint8_t battery_lipo_percent(uint16_t mvolts) {
    if (mvolts >= 4200) {
        return 100;
    }
    if (mvolts > 4100) {
        return static_cast<uint8_t>(90 + (mvolts - 4100) * 10 / 100);
    }
    if (mvolts > 4000) {
        return static_cast<uint8_t>(80 + (mvolts - 4000) * 10 / 100);
    }
    if (mvolts > 3900) {
        return static_cast<uint8_t>(70 + (mvolts - 3900) * 10 / 100);
    }
    if (mvolts > 3800) {
        return static_cast<uint8_t>(50 + (mvolts - 3800) * 20 / 100);
    }
    if (mvolts > 3700) {
        return static_cast<uint8_t>(30 + (mvolts - 3700) * 20 / 100);
    }
    if (mvolts > 3600) {
        return static_cast<uint8_t>(20 + (mvolts - 3600) * 10 / 100);
    }
    if (mvolts > 3500) {
        return static_cast<uint8_t>(10 + (mvolts - 3500) * 10 / 100);
    }
    if (mvolts > 3400) {
        return static_cast<uint8_t>(2 + (mvolts - 3400) * 8 / 100);
    }
    if (mvolts > 3300) {
        return static_cast<uint8_t>(1 + (mvolts - 3300) * 1 / 100);
    }
    return 1;
}
