#pragma once
#include <Arduino.h>
#include <array>
#include <cstddef>
#include <cstdint>

// Raw thermal frame stream over the USB serial port, consumed by
// scripts/vizualisation/serial_viz.py.
//
// Text logs share the same port, so every frame is prefixed with a 4-byte
// magic the reader synchronises on. The two leading bytes are outside the
// printable ASCII range and can never occur inside a log line.
//
//   magic  : AA 55 'T' 'T'
//   payload: N little-endian float32, row-major (12 rows x 16 columns)

constexpr uint8_t serial_frame_magic[4] = {0xAA, 0x55, 'T', 'T'};

template <std::size_t N>
inline void serial_stream_frame(const std::array<float, N>& frame) {
    Serial.write(serial_frame_magic, sizeof(serial_frame_magic));
    Serial.write(reinterpret_cast<const uint8_t*>(frame.data()), N * sizeof(float));
}
