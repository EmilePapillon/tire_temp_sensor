#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

/// @file serial_frame_stream.hh
/// @brief Raw thermal frame stream over the USB serial port, consumed by
/// scripts/visualization/serial_viz.py.
///
/// Text logs share the same port, so every frame is prefixed with a 4-byte
/// magic the reader synchronises on. The two leading bytes are outside the
/// printable ASCII range and can never occur inside a log line.
///
/// Wire format:
/// @code
///   magic  : AA 55 'T' 'T'
///   payload: N little-endian float32, row-major (12 rows x 16 columns)
/// @endcode

/// @brief Bytes that start every frame on the wire.
constexpr uint8_t serial_frame_magic[4] = {0xAA, 0x55, 'T', 'T'};

/// @brief Write the magic followed by the frame's raw float32 bytes to Serial.
/// @tparam N Number of floats in the frame.
/// @param frame Per-pixel values, row-major.
template <std::size_t N>
void serial_stream_frame(const std::array<float, N>& frame);

#include "serial_frame_stream.inl"
