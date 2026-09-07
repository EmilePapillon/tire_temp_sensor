#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

/// @file mlx90641_params.hh
/// @brief Calibration parameter set decoded from the MLX90641 EEPROM.

namespace mlx90641 {

/// @brief Broken pixels the driver tolerates.
///
/// The datasheet allows a part to ship with one dead pixel;
/// MLX90641Sensor::bad_pixels_correction() interpolates it from its neighbours.
constexpr std::size_t max_broken_pixels = 1;

/// @brief Slots in the broken-pixel list: one per tolerated pixel, plus an overflow slot.
constexpr std::size_t broken_pixel_slots = max_broken_pixels + 1;

/// @brief Value marking an unused broken-pixel slot. No pixel index can reach it.
constexpr std::uint16_t no_broken_pixel = 0xFFFFu;

/// @brief All calibration parameters the temperature math needs, in the Melexis reference naming.
struct ParamsMLX90641 {
    std::int16_t kVdd;                                     ///< Supply-voltage coefficient, LSB/V.
    std::int16_t vdd25;                                    ///< VDD pixel reading at 25 C, LSB.
    float KvPTAT;                                          ///< PTAT supply-voltage coefficient.
    float KtPTAT;                                          ///< PTAT temperature coefficient.
    std::uint16_t vPTAT25;                                 ///< PTAT reading at 25 C, LSB.
    float alphaPTAT;                                       ///< PTAT proportionality factor.
    std::int16_t gainEE;                                   ///< Gain calibration, LSB.
    float tgc;                                             ///< Temperature gradient coefficient.
    float cpKv;                                            ///< Compensation-pixel supply-voltage coefficient.
    float cpKta;                                           ///< Compensation-pixel ambient coefficient.
    std::uint8_t resolutionEE;                             ///< ADC resolution the calibration was done at, 0..3.
    std::uint8_t calibrationModeEE;                        ///< Calibration mode flag (unused by the math).
    float KsTa;                                            ///< Ambient sensitivity coefficient.
    std::array<float, 8> ksTo;                             ///< Per-range object-temperature sensitivity slope.
    std::array<std::int16_t, 8> ct;                        ///< Corner temperatures of the ranges, C.
    std::array<float, 192> alpha;                          ///< Per-pixel sensitivity.
    std::array<std::array<std::int16_t, 192>, 2> offset;   ///< Per-pixel offset for sub-page 0 and 1.
    std::array<float, 192> kta;                            ///< Per-pixel ambient coefficient.
    std::array<float, 192> kv;                             ///< Per-pixel supply-voltage coefficient.
    float cpAlpha;                                         ///< Compensation-pixel sensitivity.
    std::int16_t cpOffset;                                 ///< Compensation-pixel offset, LSB.
    float emissivityEE;                                    ///< Default emissivity, 0..1.
    std::array<std::uint16_t, broken_pixel_slots> brokenPixels;  ///< Indices of broken pixels; no_broken_pixel = empty slot.
};

}  // namespace mlx90641
