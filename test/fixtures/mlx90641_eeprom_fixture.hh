#pragma once

#include <array>
#include <cstdint>
#include "mlx90641_params.hh"

// EEPROM fixture captured from a real MLX90641, already Hamming-decoded (each
// word masked to 11 bits, as MLX90641Sensor::hamming_decode() leaves it), plus
// the calibration parameters the parser is expected to extract from it.
//
// Header-only on purpose: test/ is on every test binary's include path, but
// only test_*/ folders are compiled, so shared data has to travel by include.
namespace mlx90641 {

// Re-add the five Hamming check bits the sensor stores in bits 11-15 of every
// EEPROM word from address 16 onwards, so a mock bus can serve raw words that
// MLX90641Sensor::hamming_decode() accepts without error.
inline uint16_t hamming_encode(uint16_t data11) {
    int d[16] = {};
    for (int i = 0; i < 11; i++) {
        d[i] = (data11 >> i) & 1;
    }
    d[11] = d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[6] ^ d[8] ^ d[10];
    d[12] = d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[9] ^ d[10];
    d[13] = d[1] ^ d[2] ^ d[3] ^ d[7] ^ d[8] ^ d[9] ^ d[10];
    d[14] = d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[10];
    int overall = 0;
    for (int i = 0; i < 15; i++) {
        overall ^= d[i];
    }
    d[15] = overall;
    uint16_t word = 0;
    for (int i = 0; i < 16; i++) {
        word |= static_cast<uint16_t>(d[i] << i);
    }
    return word;
}

#include "fixtures/mlx90641_eeprom_fixture.inc"

}  // namespace mlx90641
