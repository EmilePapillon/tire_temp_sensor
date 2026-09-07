#pragma once
#include <cstdint>

/// @file mlx90641_config.hh
/// @brief MLX90641 configuration types. Kept apart from the driver so config.hh
/// can describe a sensor without pulling in the driver template.

/// @brief MLX90641 thermal sensor driver and calibration.
namespace mlx90641 {

/// @brief ADC resolution field of control register 1 (0x800D), per the datasheet.
enum class Resolution : uint8_t {
    Bits16 = 0x00,  ///< 16-bit ADC
    Bits17 = 0x01,  ///< 17-bit ADC
    Bits18 = 0x02,  ///< 18-bit ADC
    Bits19 = 0x03,  ///< 19-bit ADC
};

/// @brief Refresh-rate field of control register 1 (0x800D), per the datasheet.
enum class RefreshRate : uint8_t {
    Hz0_5 = 0x00,  ///< 0.5 Hz
    Hz1 = 0x01,    ///< 1 Hz
    Hz2 = 0x02,    ///< 2 Hz
    Hz4 = 0x03,    ///< 4 Hz
    Hz8 = 0x04,    ///< 8 Hz
    Hz16 = 0x05,   ///< 16 Hz
    Hz32 = 0x06,   ///< 32 Hz
    Hz64 = 0x07,   ///< 64 Hz
};

/// @brief Nominal frame period of a refresh-rate setting.
///
/// The rate doubles with every enum step from 0.5 Hz, so the period halves from 2 s.
/// @param rate The refresh-rate field value.
/// @return Period in microseconds; exact for every setting.
constexpr uint32_t frame_period_us(RefreshRate rate);

/// @brief Everything MLX90641Sensor::init() needs. No defaults: the caller (config.hh) owns every value.
struct Mlx90641Config {
    uint32_t i2c_freq_khz;      ///< I2C bus frequency in kHz.
    Resolution resolution;      ///< ADC resolution to program.
    RefreshRate refresh_rate;   ///< Frame rate to program.
    /// Status-register reads to wait for a new frame before read_frame() gives
    /// up with Status::DataReadyTimeout. Each poll is one 1-word I2C read whose
    /// cost depends on the bus speed and the host's I2C stack (about 270 us on a
    /// Feather nRF52832 at 400 kHz); size it from frame_period_us(refresh_rate)
    /// so a healthy frame never times out.
    uint32_t data_ready_max_polls;
};

}  // namespace mlx90641

#include "mlx90641_config.inl"
