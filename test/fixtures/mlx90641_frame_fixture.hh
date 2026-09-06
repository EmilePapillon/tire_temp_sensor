#pragma once
#include <cstddef>
#include <cstdint>
#include <map>

// Synthetic MLX90641 RAM frame for driver tests, plus the temperatures the
// driver is expected to compute from it together with the EEPROM fixture.
//
// Pixel values were chosen to land in the 25-45 C band for a 3.3 V, 25 C
// ambient sensor; the expected outputs were captured from the original
// double-precision port of the Melexis reference math and are the regression
// baseline for the single-precision implementation.
namespace mlx90641 {

constexpr uint16_t frame_ptat_art = 19947;                         // frame[192] -> Ta ~ 25 C
constexpr uint16_t frame_cp = static_cast<uint16_t>(-470);         // frame[200] compensation pixel
constexpr uint16_t frame_gain = 7680;                              // frame[202] -> gain ~ 1.0
constexpr uint16_t frame_ptat = 1600;                              // frame[224]
constexpr uint16_t frame_vdd = static_cast<uint16_t>(-25024);      // frame[234] -> 3.3 V at 19-bit resolution

inline uint16_t frame_pixel_raw(std::size_t pixel) {
    const int column = static_cast<int>(pixel % 16);
    const int row = static_cast<int>(pixel / 16);
    return static_cast<uint16_t>(-150 + column * 8 + row * 3);
}

/// @brief Populate a register file with the frame as the sensor lays it out in RAM.
///
/// Sub-page 0 pixel banks start at 0x0400, sub-page 1 banks at 0x0420, each
/// bank holding 32 pixels 0x40 apart. The 48-word aux block at 0x0580 maps to
/// frame[192..239].
inline void load_frame(std::map<uint16_t, uint16_t>& registers, uint8_t sub_page) {
    const uint16_t bank_base = (sub_page == 0) ? 0x0400 : 0x0420;
    for (std::size_t pixel = 0; pixel < 192; pixel++) {
        const uint16_t reg = static_cast<uint16_t>(bank_base + (pixel / 32) * 0x40 + (pixel % 32));
        registers[reg] = frame_pixel_raw(pixel);
    }
    for (uint16_t k = 0; k < 48; k++) {
        registers[static_cast<uint16_t>(0x0580 + k)] = 0;
    }
    registers[0x0580 + (192 - 192)] = frame_ptat_art;
    registers[0x0580 + (200 - 192)] = frame_cp;
    registers[0x0580 + (202 - 192)] = frame_gain;
    registers[0x0580 + (224 - 192)] = frame_ptat;
    registers[0x0580 + (234 - 192)] = frame_vdd;
}

// Expected results for the sub-page 0 frame after init() with 19-bit
// resolution, captured from the original double-precision port.
constexpr float expected_frame_ambient_c = 25.00010f;

struct ExpectedPixel {
    std::size_t index;
    float temp_c;
};

constexpr ExpectedPixel expected_frame_pixels[] = {
    {0, 37.12713f},   {1, 35.38161f},   {15, 56.50808f},  {16, 33.35082f},
    {95, 42.48254f},  {100, 32.77352f}, {150, 34.53619f}, {191, 64.15893f},
};
constexpr float expected_frame_min_c = 31.8287f;
constexpr float expected_frame_max_c = 64.1589f;

}  // namespace mlx90641
