// Inline definitions for mlx90641_eeprom_parser.hh. Included by the header; do not include directly.
#pragma once

namespace mlx90641 {

inline float scale_by_division(int32_t raw_value, uint8_t scale_exp) {
    // 1ULL << scale_exp is undefined once scale_exp reaches 64, and the six-bit
    // alpha-scale field can decode that high on a corrupt EEPROM image. Such a
    // divisor is meaningless; treat it as effectively infinite.
    if (scale_exp >= 64u) {
        return 0.0f;
    }
    return static_cast<float>(raw_value) / static_cast<float>(1ULL << scale_exp);
}

inline std::int16_t scale_by_multiplication(int16_t raw_value, uint8_t scale_exp) {
    // 1 << scale_exp is undefined for scale_exp >= 31 (it is a plain int). A
    // shift that large only comes from a corrupt scale field; saturate to zero
    // rather than execute it. Promote to int32_t so the multiply cannot overflow.
    if (scale_exp >= 31u) {
        return 0;
    }
    return static_cast<int16_t>(static_cast<int32_t>(raw_value) * (1 << scale_exp));
}

inline MLX90641EEpromParser::MLX90641EEpromParser(const std::array<uint16_t, eeprom_size>& eeprom_data)
    : eeprom_data_(eeprom_data) {}

}  // namespace mlx90641
