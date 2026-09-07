#pragma once
#include <array>
#include <cstdint>

/// @file mlx90641_params.hh
/// @brief Calibration parameter set decoded from the MLX90641 EEPROM.

namespace mlx90641 {

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
    std::uint16_t deviatingPixel;                          ///< Index of the first EEPROM-flagged deviating pixel; 0xFFFF = none.
};

}  // namespace mlx90641
