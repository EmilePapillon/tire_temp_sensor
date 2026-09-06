#pragma once
#include <cstdint>

/// @file watchdog.hh
/// @brief nRF52 hardware watchdog.
///
/// Once started it cannot be stopped; if it is not fed within the timeout the
/// SoftDevice-independent WDT resets the chip. Keeps running while the CPU
/// sleeps, pauses while halted by a debugger. Clocked from the 32.768 kHz
/// LFCLK, which the SoftDevice keeps running.

/// @brief Arm the watchdog. Idempotent: a second call while running does nothing.
/// @param timeout_s Seconds without watchdog_feed() before the chip resets.
void watchdog_begin(uint32_t timeout_s);

/// @brief Restart the countdown. Call at least once per @c timeout_s.
void watchdog_feed();
