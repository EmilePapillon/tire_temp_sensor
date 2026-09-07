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

inline std::array<uint16_t, eeprom_size> eeprom_with_broken_pixels(std::initializer_list<std::size_t> pixels) {
    std::array<uint16_t, eeprom_size> data = test_eeprom_data;
    for (const std::size_t pixel : pixels) {
        for (const uint16_t base : {EepromAddr::offset_even, EepromAddr::alpha_pixel, EepromAddr::kta_pixel,
                                    EepromAddr::offset_odd}) {
            data[base - eeprom_start_address + pixel] = 0;
        }
    }
    return data;
}

}  // namespace mlx90641
