#pragma once

#include <array>
#include <cstdint>
#include "mlx90641_params.hh"

namespace mlx90641 {
// Test EEPROM data for MLX90641 sensor
extern const std::array<uint16_t, 832> test_eeprom_data;

// Expected parameters extracted from the test EEPROM data
extern const ParamsMLX90641 expected_params;

} // namespace mlx90641