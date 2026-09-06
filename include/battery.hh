#pragma once
#include <cstdint>
#include "battery_lipo.hh"  // battery_lipo_percent(), pure math in lib/battery

/// @file battery.hh
/// @brief LiPo battery monitoring for the Adafruit Feather nRF52832.
///
/// VBAT is brought to pin A7 through an on-board 2M / 0.806M divider, so the ADC
/// sees roughly VBAT * 0.288. The ADC is configured for 12-bit resolution and
/// the internal 3.0 V reference; the divider compensation factor (1.403) is
/// Adafruit's documented value for this board revision.

constexpr float battery_mv_per_lsb = 3000.0f / 4096.0f;  ///< 3.0 V reference over a 12-bit ADC.
constexpr float battery_divider_comp = 1.403f;           ///< Feather nRF52832 2M / 0.806M divider.

/// @brief Configure the ADC for battery reads. Call once from setup().
void battery_begin();

/// @brief Sample the battery voltage.
/// @return Instantaneous terminal voltage in millivolts.
uint16_t battery_read_millivolts();
