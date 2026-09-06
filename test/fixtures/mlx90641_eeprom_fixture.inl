// Inline definitions for mlx90641_eeprom_fixture.hh. Included by the header; do not include directly.
#pragma once

namespace mlx90641 {

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

}  // namespace mlx90641
