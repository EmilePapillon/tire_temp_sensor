// Inline definitions for mlx90641_eeprom_parser.hh. Included by the header; do not include directly.
#pragma once

namespace mlx90641 {

inline float scale_by_division(int32_t raw_value, uint8_t scale_exp) {
    // A corrupt image can ask for a shift past the width of the shifted type,
    // which is undefined; such a divisor saturates the float anyway.
    if (scale_exp >= max_shift_exp) {
        return 0.0f;
    }
    return static_cast<float>(raw_value) / static_cast<float>(1ULL << scale_exp);
}

inline std::int16_t scale_by_multiplication(int16_t raw_value, uint8_t scale_exp) {
    // Same hazard. A shift of 16 clears every bit the return cast keeps,
    // so saturating there is exact.
    if (scale_exp >= 16u) {
        return 0;
    }
    // Widen before multiplying so the product cannot overflow the operand type.
    return static_cast<int16_t>(static_cast<int64_t>(raw_value) * (INT64_C(1) << scale_exp));
}

inline MLX90641EEpromParser::MLX90641EEpromParser(const std::array<uint16_t, eeprom_size>& eeprom_data)
    : eeprom_data_(eeprom_data) {}

}  // namespace mlx90641
