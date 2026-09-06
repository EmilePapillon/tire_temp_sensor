// Inline definitions for mlx90641_eeprom_parser.hh. Included by the header; do not include directly.
#pragma once

namespace mlx90641 {

inline float scale_by_division(int32_t raw_value, uint8_t scale_exp) {
    return static_cast<float>(raw_value) / static_cast<float>(1ULL << scale_exp);
}

inline std::int16_t scale_by_multiplication(int16_t raw_value, uint8_t scale_exp) {
    // Promote to int32_t before multiplying so the intermediate cannot overflow.
    return static_cast<int16_t>(static_cast<int32_t>(raw_value) * (1 << scale_exp));
}

inline MLX90641EEpromParser::MLX90641EEpromParser(const std::array<uint16_t, eeprom_size>& eeprom_data)
    : eeprom_data_(eeprom_data) {}

}  // namespace mlx90641
