#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include "mlx90641_eeprom_parser.hh"
#include "mlx90641_params.hh"

/// @file mlx90641_eeprom_fixture.hh
/// @brief EEPROM image captured from a real MLX90641 and the parameters expected from it.
///
/// The image is already Hamming-decoded (each word masked to 11 bits, as
/// MLX90641Sensor::hamming_decode() leaves it). Header-only on purpose: test/
/// is on every test binary's include path, but only test_*/ folders are
/// compiled, so shared data has to travel by include.

namespace mlx90641 {

/// @brief Re-add the five Hamming check bits the sensor stores in bits 11-15 of every EEPROM word from address 16 on.
///
/// Lets a mock bus serve raw words that MLX90641Sensor::hamming_decode() accepts without error.
/// @param data11 An 11-bit payload.
/// @return The 16-bit word with a zero Hamming syndrome.
uint16_t hamming_encode(uint16_t data11);

/// @brief Copy of the fixture image with the given pixels blanked out, as a part with dead pixels ships.
///
/// A pixel counts as broken when its offset, alpha and KTA words are all zero,
/// which is what MLX90641EEpromParser::get_broken_pixels() looks for.
/// @param pixels Row-major pixel indices to blank, 0..191.
/// @return The image with those pixels zeroed; every other word is untouched.
std::array<uint16_t, eeprom_size> eeprom_with_broken_pixels(std::initializer_list<std::size_t> pixels);

#include "fixtures/mlx90641_eeprom_fixture.inc"

}  // namespace mlx90641

#include "fixtures/mlx90641_eeprom_fixture.inl"
