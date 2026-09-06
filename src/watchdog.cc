#include "watchdog.hh"
#include <nrf_wdt.h>

void watchdog_begin(uint32_t timeout_s) {
    if (nrf_wdt_started(NRF_WDT)) {
        return;
    }
    nrf_wdt_behaviour_set(NRF_WDT, NRF_WDT_BEHAVIOUR_RUN_SLEEP);
    nrf_wdt_reload_value_set(NRF_WDT, timeout_s * 32768u - 1u);
    nrf_wdt_reload_request_enable(NRF_WDT, NRF_WDT_RR0);
    nrf_wdt_task_trigger(NRF_WDT, NRF_WDT_TASK_START);
}

void watchdog_feed() {
    nrf_wdt_reload_request_set(NRF_WDT, NRF_WDT_RR0);
}
