#pragma once
#include <Arduino.h>
#include <cstdint>

// LiPo battery monitoring for the Adafruit Feather nRF52832.
//
// VBAT is brought to pin A7 through an on-board 2M / 0.806M divider, so the ADC
// sees roughly VBAT * 0.288. The ADC is configured here for 12-bit resolution
// and the internal 3.0 V reference; the divider compensation factor (1.403) is
// Adafruit's documented value for this board revision.
//
// Header-only board glue, in the same spirit as arduino_logger.hh.

constexpr float battery_mv_per_lsb = 3000.0f / 4096.0f;  // 3.0 V reference, 12-bit ADC
constexpr float battery_divider_comp = 1.403f;           // Feather nRF52832 2M / 0.806M divider

// Configure the ADC for battery reads. Call once from setup().
inline void battery_begin() {
    analogReadResolution(12);
    analogReference(AR_INTERNAL_3_0);
    delay(1);
    (void)analogRead(A7);  // first sample after switching reference is unreliable
}

// Instantaneous battery voltage in millivolts.
inline uint16_t battery_read_millivolts() {
    uint32_t raw = analogRead(A7);
    return static_cast<uint16_t>(raw * battery_mv_per_lsb * battery_divider_comp);
}

// Map a LiPo terminal voltage (mV) to an approximate state of charge (0-100 %).
// Piecewise-linear fit of a typical single-cell LiPo discharge curve; matches the
// curve used by upstream RejsaRubberTrac so RaceChrono shows familiar numbers.
inline uint8_t battery_lipo_percent(uint16_t mvolts) {
    if (mvolts >= 4200) {
        return 100;
    }
    if (mvolts > 4100) {
        return 90 + (mvolts - 4100) * 10 / 100;
    }
    if (mvolts > 4000) {
        return 80 + (mvolts - 4000) * 10 / 100;
    }
    if (mvolts > 3900) {
        return 70 + (mvolts - 3900) * 10 / 100;
    }
    if (mvolts > 3800) {
        return 50 + (mvolts - 3800) * 20 / 100;
    }
    if (mvolts > 3700) {
        return 30 + (mvolts - 3700) * 20 / 100;
    }
    if (mvolts > 3600) {
        return 20 + (mvolts - 3600) * 10 / 100;
    }
    if (mvolts > 3500) {
        return 10 + (mvolts - 3500) * 10 / 100;
    }
    if (mvolts > 3400) {
        return 2 + (mvolts - 3400) * 8 / 100;
    }
    if (mvolts > 3300) {
        return 1 + (mvolts - 3300) * 1 / 100;
    }
    return 1;
}
