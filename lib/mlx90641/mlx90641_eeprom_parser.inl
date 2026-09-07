// Inline definitions for mlx90641_eeprom_parser.hh. Included by the header; do not include directly.
#pragma once

namespace mlx90641 {

inline float scale_by_division(int32_t raw_value, uint8_t scale_exp) {
    // Scale exponents come out of the EEPROM, and the six-bit alpha-scale field
    // carries a +20 bias, so a corrupt image can ask for a shift of up to 83 -
    // past the width of the shifted type, which is undefined. Such a divisor is
    // far beyond anything a float can represent usefully, so it saturates to 0.
    if (scale_exp >= max_shift_exp) {
        return 0.0f;
    }
    return static_cast<float>(raw_value) / static_cast<float>(1ULL << scale_exp);
}

inline std::int16_t scale_by_multiplication(int16_t raw_value, uint8_t scale_exp) {
    // Same corrupt-image hazard as above. Only the low 16 bits of the product
    // survive the return cast and a shift of 16 or more clears every one of
    // them, so saturating there is exact and keeps the shift in range.
    if (scale_exp >= 16u) {
        return 0;
    }
    // Widen before multiplying so the product cannot overflow the operand type.
    return static_cast<int16_t>(static_cast<int64_t>(raw_value) * (INT64_C(1) << scale_exp));
}

inline MLX90641EEpromParser::MLX90641EEpromParser(const std::array<uint16_t, eeprom_size>& eeprom_data)
    : eeprom_data_(eeprom_data) {}

}  // namespace mlx90641
