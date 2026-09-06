#pragma once
#include <cstdint>

/// @file mlx90641_eeprom_addr.hh
/// @brief MLX90641 EEPROM address map, datasheet numbering.

namespace mlx90641 {

/// @brief Absolute EEPROM addresses of every calibration word, as named in the datasheet.
class EepromAddr {
public:
    // Offset parameters
    static constexpr uint16_t scale_offset = 0x2410;  ///< Offset and KTA/KV scale exponents.
    static constexpr uint16_t offset_ref0 = 0x2411;  ///< Offset reference, upper word.
    static constexpr uint16_t offset_ref1 = 0x2412;  ///< Offset reference, lower word.

    // KTA parameters
    static constexpr uint16_t kta_avg = 0x2415;  ///< Average KTA.
    static constexpr uint16_t kta_scale = 0x2416;  ///< KTA scale exponents.

    // KV parameters
    static constexpr uint16_t kv_avg = 0x2417;  ///< Average KV.
    static constexpr uint16_t kv_scale = 0x2418;  ///< KV scale exponents.

    // Alpha scaling parameters
    static constexpr uint16_t alpha_scale0 = 0x2419;  ///< Alpha scale, word 0.
    static constexpr uint16_t alpha_scale1 = 0x241A;  ///< Alpha scale, word 1.
    static constexpr uint16_t alpha_scale2 = 0x241B;  ///< Alpha scale, word 2.

    // Alpha max row parameters
    static constexpr uint16_t alpha_max_row0 = 0x241C;  ///< Per-row alpha maximum, rows 0-1.
    static constexpr uint16_t alpha_max_row1 = 0x241D;  ///< Per-row alpha maximum, rows 2-3.
    static constexpr uint16_t alpha_max_row2 = 0x241E;  ///< Per-row alpha maximum, rows 4-5.
    static constexpr uint16_t alpha_max_row3 = 0x241F;  ///< Per-row alpha maximum, rows 6-7.
    static constexpr uint16_t alpha_max_row4 = 0x2420;  ///< Per-row alpha maximum, rows 8-9.
    static constexpr uint16_t alpha_max_row5 = 0x2421;  ///< Per-row alpha maximum, rows 10-11.

    // Temperature coefficient parameters
    static constexpr uint16_t ks_ta = 0x2422;  ///< KS_TA ambient sensitivity.
    static constexpr uint16_t emissivity = 0x2423;  ///< Default emissivity.
    static constexpr uint16_t gain_ee0 = 0x2424;  ///< Gain, upper word.
    static constexpr uint16_t gain_ee1 = 0x2425;  ///< Gain, lower word.
    static constexpr uint16_t vdd25 = 0x2426;  ///< VDD at 25 C.
    static constexpr uint16_t kvdd = 0x2427;  ///< KVdd coefficient.
    static constexpr uint16_t vptat25_0 = 0x2428;  ///< VPTAT25, upper word.
    static constexpr uint16_t vptat25_1 = 0x2429;  ///< VPTAT25, lower word.
    static constexpr uint16_t kt_ptat = 0x242A;  ///< KT_PTAT coefficient.
    static constexpr uint16_t kv_ptat = 0x242B;  ///< KV_PTAT coefficient.
    static constexpr uint16_t alpha_ptat = 0x242C;  ///< ALPHA_PTAT factor.

    // Compensation pixel parameters
    static constexpr uint16_t cp_alpha = 0x242D;  ///< Compensation-pixel alpha.
    static constexpr uint16_t cp_alpha_scale = 0x242E;  ///< Compensation-pixel alpha scale.
    static constexpr uint16_t cp_offset0 = 0x242F;  ///< Compensation-pixel offset, upper word.
    static constexpr uint16_t cp_offset1 = 0x2430;  ///< Compensation-pixel offset, lower word.
    static constexpr uint16_t cp_kta = 0x2431;  ///< Compensation-pixel KTA.
    static constexpr uint16_t cp_kv = 0x2432;  ///< Compensation-pixel KV.

    // Additional parameters
    static constexpr uint16_t tgc = 0x2433;  ///< TGC (low bits).
    static constexpr uint16_t resolution = 0x2433;  ///< Calibration resolution (shares the word with tgc).
    static constexpr uint16_t ks_to_scale = 0x2434;  ///< KS_TO scale exponent.
    static constexpr uint16_t ks_to0 = 0x2435;  ///< KS_TO range 0.
    static constexpr uint16_t ks_to1 = 0x2436;  ///< KS_TO range 1.
    static constexpr uint16_t ks_to2 = 0x2437;  ///< KS_TO range 2.
    static constexpr uint16_t ks_to3 = 0x2438;  ///< KS_TO range 3.
    static constexpr uint16_t ks_to4 = 0x2439;  ///< KS_TO range 4.
    static constexpr uint16_t ct0 = 0x243A;  ///< Corner temperature CT0.
    static constexpr uint16_t ks_to5 = 0x243B;  ///< KS_TO range 5.
    static constexpr uint16_t ct1 = 0x243C;  ///< Corner temperature CT1.
    static constexpr uint16_t ks_to6 = 0x243D;  ///< KS_TO range 6.
    static constexpr uint16_t ct2 = 0x243E;  ///< Corner temperature CT2.
    static constexpr uint16_t ks_to7 = 0x243F;  ///< KS_TO range 7.

    // Pixel data base addresses
    static constexpr uint16_t offset_even = 0x2440;  ///< Sub-page 0 per-pixel offsets, base + pixel index.
    static constexpr uint16_t alpha_pixel = 0x2500;  ///< Per-pixel alpha, base + pixel index.
    static constexpr uint16_t kta_pixel = 0x25C0;  ///< Per-pixel KTA, base + pixel index.
    static constexpr uint16_t kv_pixel = 0x25C0;  ///< Per-pixel KV (shares the words with kta_pixel).
    static constexpr uint16_t offset_odd = 0x2680;  ///< Sub-page 1 per-pixel offsets, base + pixel index.
};

}  // namespace mlx90641
