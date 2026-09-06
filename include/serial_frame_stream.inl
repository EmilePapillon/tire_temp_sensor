// Template definitions for serial_frame_stream.hh. Included by the header; do not include directly.
#pragma once
#include <Arduino.h>

template <std::size_t N>
void serial_stream_frame(const std::array<float, N>& frame) {
    Serial.write(serial_frame_magic, sizeof(serial_frame_magic));
    Serial.write(reinterpret_cast<const uint8_t*>(frame.data()), N * sizeof(float));
}
