#include "battery.hh"
#include <Arduino.h>

void battery_begin() {
    analogReadResolution(12);
    analogReference(AR_INTERNAL_3_0);
    delay(1);
    (void)analogRead(A7);  // first sample after switching reference is unreliable
}

uint16_t battery_read_millivolts() {
    const uint32_t raw = analogRead(A7);
    return static_cast<uint16_t>(raw * battery_mv_per_lsb * battery_divider_comp);
}
