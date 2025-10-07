#pragma once
#include <cstdint>

namespace mlx90641 {

/// @brief Class containing all MLX90641 EEPROM addresses as constexpr static values
class EepromAddr {
public:
    // Offset parameters
    static constexpr uint16_t scale_offset = 0x2410;
    static constexpr uint16_t offset_ref0 = 0x2411;
    static constexpr uint16_t offset_ref1 = 0x2412;
    
    // KTA parameters
    static constexpr uint16_t kta_avg = 0x2415;
    static constexpr uint16_t kta_scale = 0x2416;
    
    // KV parameters
    static constexpr uint16_t kv_avg = 0x2417;
    static constexpr uint16_t kv_scale = 0x2418;
    
    // Alpha scaling parameters
    static constexpr uint16_t alpha_scale0 = 0x2419;
    static constexpr uint16_t alpha_scale1 = 0x241A;
    static constexpr uint16_t alpha_scale2 = 0x241B;
    
    // Alpha max row parameters
    static constexpr uint16_t alpha_max_row0 = 0x241C;
    static constexpr uint16_t alpha_max_row1 = 0x241D;
    static constexpr uint16_t alpha_max_row2 = 0x241E;
    static constexpr uint16_t alpha_max_row3 = 0x241F;
    static constexpr uint16_t alpha_max_row4 = 0x2420;
    static constexpr uint16_t alpha_max_row5 = 0x2421;
    
    // Temperature coefficient parameters
    static constexpr uint16_t ks_ta = 0x2422;
    static constexpr uint16_t emissivity = 0x2423;
    static constexpr uint16_t gain_ee0 = 0x2424;
    static constexpr uint16_t gain_ee1 = 0x2425;
    static constexpr uint16_t vdd25 = 0x2426;
    static constexpr uint16_t kvdd = 0x2427;
    static constexpr uint16_t vptat25_0 = 0x2428;
    static constexpr uint16_t vptat25_1 = 0x2429;
    static constexpr uint16_t kt_ptat = 0x242A;
    static constexpr uint16_t kv_ptat = 0x242B;
    static constexpr uint16_t alpha_ptat = 0x242C;
    
    // Compensation pixel parameters
    static constexpr uint16_t cp_alpha = 0x242D;
    static constexpr uint16_t cp_alpha_scale = 0x242E;
    static constexpr uint16_t cp_offset0 = 0x242F;
    static constexpr uint16_t cp_offset1 = 0x2430;
    static constexpr uint16_t cp_kta = 0x2431;
    static constexpr uint16_t cp_kv = 0x2432;
    
    // Additional parameters
    static constexpr uint16_t tgc = 0x2433;
    static constexpr uint16_t resolution = 0x2433;  // Same address as tgc, different bits
    static constexpr uint16_t ks_to_scale = 0x2434;
    static constexpr uint16_t ks_to0 = 0x2435;
    static constexpr uint16_t ks_to1 = 0x2436;
    static constexpr uint16_t ks_to2 = 0x2437;
    static constexpr uint16_t ks_to3 = 0x2438;
    static constexpr uint16_t ks_to4 = 0x2439;
    static constexpr uint16_t ct0 = 0x243A;
    static constexpr uint16_t ks_to5 = 0x243B;
    static constexpr uint16_t ct1 = 0x243C;
    static constexpr uint16_t ks_to6 = 0x243D;
    static constexpr uint16_t ct2 = 0x243E;
    static constexpr uint16_t ks_to7 = 0x243F;
    
    // Pixel data base addresses
    static constexpr uint16_t offset_even = 0x2440;      // Even subpage offset base (0x2440 + pixel_index)
    static constexpr uint16_t alpha_pixel = 0x2500;      // Alpha pixel data base (0x2500 + pixel_index)
    static constexpr uint16_t kta_pixel = 0x25C0;        // KTA pixel data base (0x25C0 + pixel_index)
    static constexpr uint16_t kv_pixel = 0x25C0;         // KV pixel data base (same as kta_pixel, different bits)
    static constexpr uint16_t offset_odd = 0x2680;       // Odd subpage offset base (0x2680 + pixel_index)
};

} // namespace mlx90641_eeprom
