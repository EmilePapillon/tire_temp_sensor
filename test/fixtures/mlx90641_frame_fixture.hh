#pragma once
#include <cstddef>
#include <cstdint>
#include <map>

/// @file mlx90641_frame_fixture.hh
/// @brief Synthetic MLX90641 RAM frame for driver tests, plus the temperatures
/// the driver is expected to compute from it together with the EEPROM fixture.
///
/// Pixel values were chosen to land in the 25-45 C band for a 3.3 V, 25 C
/// ambient sensor; the expected outputs were captured from the original
/// double-precision port of the Melexis reference math and are the regression
/// baseline for the single-precision implementation.

namespace mlx90641 {

constexpr uint16_t frame_ptat_art = 19947;                     ///< frame[192]: yields Ta ~ 25 C.
constexpr uint16_t frame_cp = static_cast<uint16_t>(-470);     ///< frame[200]: compensation pixel.
constexpr uint16_t frame_gain = 7680;                          ///< frame[202]: yields gain ~ 1.0.
constexpr uint16_t frame_ptat = 1600;                          ///< frame[224]: PTAT.
constexpr uint16_t frame_vdd = static_cast<uint16_t>(-25024);  ///< frame[234]: 3.3 V at 19-bit resolution.

/// @brief Raw value of one synthetic pixel: a gentle gradient across columns and rows.
/// @param pixel Row-major pixel index, 0..191.
/// @return The 16-bit word the sensor would hold.
uint16_t frame_pixel_raw(std::size_t pixel);

/// @brief Populate a register file with the frame as the sensor lays it out in RAM.
///
/// Sub-page 0 pixel banks start at 0x0400, sub-page 1 banks at 0x0420, each
/// bank holding 32 pixels 0x40 apart. The 48-word aux block at 0x0580 maps to
/// frame[192..239].
/// @param registers Register file to fill (e.g. MockI2CAdapter::registers).
/// @param sub_page 0 or 1; selects which bank addresses are populated.
void load_frame(std::map<uint16_t, uint16_t>& registers, uint8_t sub_page);

/// @brief Ambient temperature the sub-page 0 frame must yield after init() at 19-bit resolution.
constexpr float expected_frame_ambient_c = 25.00010f;

/// @brief One reference pixel temperature.
struct ExpectedPixel {
    std::size_t index;  ///< Row-major pixel index.
    float temp_c;       ///< Degrees Celsius from the double-precision reference.
};

/// @brief Reference temperatures for a sample of pixels of the sub-page 0 frame.
constexpr ExpectedPixel expected_frame_pixels[] = {
    {0, 37.12713f},   {1, 35.38161f},   {15, 56.50808f},  {16, 33.35082f},
    {95, 42.48254f},  {100, 32.77352f}, {150, 34.53619f}, {191, 64.15893f},
};
constexpr float expected_frame_min_c = 31.8287f;  ///< Coldest pixel of the reference frame.
constexpr float expected_frame_max_c = 64.1589f;  ///< Hottest pixel of the reference frame.

}  // namespace mlx90641

#include "fixtures/mlx90641_frame_fixture.inl"
