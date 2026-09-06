#pragma once
#include <cstdint>

/// @file battery_lipo.hh
/// @brief Pure battery math, no board dependency. The ADC-facing half lives in include/battery.hh.

/// @brief Map a single-cell LiPo terminal voltage to an approximate state of charge.
///
/// Piecewise-linear fit of a typical LiPo discharge curve; matches the curve
/// used by upstream RejsaRubberTrac so RaceChrono shows familiar numbers.
/// @param mvolts Terminal voltage in millivolts.
/// @return Charge in percent, 1..100. Never 0: a reading exists, so the cell is not empty.
uint8_t battery_lipo_percent(uint16_t mvolts);

#include "battery_lipo.inl"
