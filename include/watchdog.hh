#pragma once
#include <cstdint>
#include <nrf_wdt.h>

// nRF52 hardware watchdog. Once started it cannot be stopped; if it is not fed
// within the timeout the SoftDevice-independent WDT resets the chip. Keeps
// running while the CPU sleeps, pauses while halted by a debugger.
//
// Header-only board glue. The reload register is clocked from the 32.768 kHz
// LFCLK, which the SoftDevice keeps running.

inline void watchdog_begin(uint32_t timeout_s) {
    if (nrf_wdt_started(NRF_WDT)) {
        return;
    }
    nrf_wdt_behaviour_set(NRF_WDT, NRF_WDT_BEHAVIOUR_RUN_SLEEP);
    nrf_wdt_reload_value_set(NRF_WDT, timeout_s * 32768u - 1u);
    nrf_wdt_reload_request_enable(NRF_WDT, NRF_WDT_RR0);
    nrf_wdt_task_trigger(NRF_WDT, NRF_WDT_TASK_START);
}

inline void watchdog_feed() {
    nrf_wdt_reload_request_set(NRF_WDT, NRF_WDT_RR0);
}
