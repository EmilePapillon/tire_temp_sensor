#pragma once
#include <cstdint>

// MLX90641 configuration types. Kept apart from the driver so config.hh can
// describe a sensor without pulling in the driver template.

namespace mlx90641 {

// Control register 1 (0x800D) fields, per the MLX90641 datasheet / Melexis
// reference driver. Values are the raw register field encodings.
enum class Resolution : uint8_t {
    Bits16 = 0x00,
    Bits17 = 0x01,
    Bits18 = 0x02,
    Bits19 = 0x03,
};

enum class RefreshRate : uint8_t {
    Hz0_5 = 0x00,
    Hz1 = 0x01,
    Hz2 = 0x02,
    Hz4 = 0x03,
    Hz8 = 0x04,
    Hz16 = 0x05,
    Hz32 = 0x06,
    Hz64 = 0x07,
};

/// @brief Everything init() needs. No defaults: the caller (config.hh) owns every value.
struct Mlx90641Config {
    uint32_t i2c_freq_khz;
    Resolution resolution;
    RefreshRate refresh_rate;
    /// Status-register reads to wait for a new frame before read_frame() gives
    /// up with Status::DataReadyTimeout. Each poll is one 1-word I2C read
    /// (~100 us at 400 kHz); size it to cover the configured frame period.
    uint32_t data_ready_max_polls;
};

}  // namespace mlx90641
