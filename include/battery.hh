#pragma once
#include <Arduino.h>
#include <cstdint>
#include "battery_lipo.hh"  // battery_lipo_percent(), pure math in lib/battery

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
    const uint32_t raw = analogRead(A7);
    return static_cast<uint16_t>(raw * battery_mv_per_lsb * battery_divider_comp);
}
