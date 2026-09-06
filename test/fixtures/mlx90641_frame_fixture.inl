// Inline definitions for mlx90641_frame_fixture.hh. Included by the header; do not include directly.
#pragma once

namespace mlx90641 {

inline uint16_t frame_pixel_raw(std::size_t pixel) {
    const int column = static_cast<int>(pixel % 16);
    const int row = static_cast<int>(pixel / 16);
    return static_cast<uint16_t>(-150 + column * 8 + row * 3);
}

inline void load_frame(std::map<uint16_t, uint16_t>& registers, uint8_t sub_page) {
    const uint16_t bank_base = (sub_page == 0) ? 0x0400 : 0x0420;
    for (std::size_t pixel = 0; pixel < 192; pixel++) {
        const uint16_t reg = static_cast<uint16_t>(bank_base + (pixel / 32) * 0x40 + (pixel % 32));
        registers[reg] = frame_pixel_raw(pixel);
    }
    for (uint16_t k = 0; k < 48; k++) {
        registers[static_cast<uint16_t>(0x0580 + k)] = 0;
    }
    registers[0x0580 + (192 - 192)] = frame_ptat_art;
    registers[0x0580 + (200 - 192)] = frame_cp;
    registers[0x0580 + (202 - 192)] = frame_gain;
    registers[0x0580 + (224 - 192)] = frame_ptat;
    registers[0x0580 + (234 - 192)] = frame_vdd;
}

}  // namespace mlx90641
