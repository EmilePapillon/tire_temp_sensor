#include "mlx90641_eeprom_parser.hh"
#include <stdexcept>
#include <algorithm>

namespace mlx90641 {

bool MLX90641EEpromParser::extract_all(ParamsMLX90641& params) const
{
    params.kVdd = get_kvdd();
    params.vdd25 = get_vdd25();
    params.KvPTAT = get_kv_ptat();
    params.KtPTAT = get_kt_ptat();
    params.vPTAT25 = get_vptat25();
    params.alphaPTAT = get_alpha_ptat();
    params.gainEE = get_gain_ee();
    params.tgc = get_tgc();
    params.emissivityEE = get_emissivity_ee();
    params.resolutionEE = get_resolution_ee();
    params.KsTa = get_ks_ta();
    params.ksTo = get_ks_to();
    params.alpha = get_alpha();
    params.offset = get_offset();
    params.kta = get_kta();
    params.kv = get_kv();
    params.cpAlpha = get_cp_alpha();
    params.cpOffset = get_cp_offset();
    params.ct = get_ct();
    params.cpKv = get_cp_kv();
    params.cpKta = get_cp_kta();
    params.brokenPixels = get_broken_pixels();
    if (params.brokenPixels[0] != 0xFFFF  || params.brokenPixels[1] != 0xFFFF) {
        return false; // too many broken pixels
    }
    return true;
}

int16_t MLX90641EEpromParser::get_kvdd() const
{
    // kvdd = ee_data[39] >> 5`
    constexpr SingleEepromWord kvdd{EepromAddr::kvdd, 0, 11, 5, true};
    const auto kvdd_raw = extract_param(kvdd);
    return scale_by_multiplication(kvdd_raw, kvdd.scale_exp);
}

int16_t MLX90641EEpromParser::get_vdd25() const
{
    // vdd25 = ee_data[43] >> 5
    constexpr SingleEepromWord vdd25{EepromAddr::vdd25, 0, 11, 5, true};
    const auto vdd25_raw = extract_param(vdd25);
    return scale_by_multiplication(vdd25_raw, vdd25.scale_exp);
}

float MLX90641EEpromParser::get_kv_ptat() const
{
    // kv_ptat = ee_data[43] / 4096.0
    constexpr SingleEepromWord kv_ptat{EepromAddr::kv_ptat, 0, 11, 12, true};
    const auto kv_ptat_raw = extract_param(kv_ptat);
    return scale_by_division(kv_ptat_raw, kv_ptat.scale_exp);
}

float MLX90641EEpromParser::get_kt_ptat() const
{
    // kt_ptat = ee_data[42] / 8
    constexpr SingleEepromWord kt_ptat{EepromAddr::kt_ptat, 0, 11, 3, true};
    const auto kt_ptat_raw = extract_param(kt_ptat);
    return scale_by_division(kt_ptat_raw, kt_ptat.scale_exp);
}

std::uint16_t MLX90641EEpromParser::get_vptat25() const
{
    // vPTAT25 = 32 * ee_data[40] + ee_data[41]
    constexpr DualEepromWord vptat25_words = {{
        EepromWord{EepromAddr::vptat25_0, 0, 11}, 
        EepromWord{EepromAddr::vptat25_1, 0, 5}  
    }, 
    0, false};
    return extract_param_array(vptat25_words);
}

float MLX90641EEpromParser::get_alpha_ptat() const
{
    // alphaPTAT = ee_data[44] / 128.0
    // note: datasheet p.20 mentions scaling factor 2^11 (fixed scale 11) , but 
    // the calculations shown on page 22 shows a scaling factor of 2^7.
    constexpr SingleEepromWord alpha_ptat{EepromAddr::alpha_ptat, 0, 11, 7, false};
    const auto alpha_ptat_raw = extract_param(alpha_ptat);
    return scale_by_division(alpha_ptat_raw, alpha_ptat.scale_exp);
}

std::int16_t MLX90641EEpromParser::get_gain_ee() const
{
    // gainEE = 32 * ee_data[36] + ee_data[37]
    constexpr DualEepromWord gain_ee{{
        EepromWord{EepromAddr::gain_ee0, 0, 11},
        EepromWord{EepromAddr::gain_ee1, 0, 5}}, 
        0, false
    };
    return extract_param_array(gain_ee);
}

float MLX90641EEpromParser::get_tgc() const
{
    // tgc = (ee_data[50] & 0x01FF) / 64.0
    constexpr SingleEepromWord tgc{EepromAddr::tgc, 0, 9, 6, true};
    const auto tgc_raw = extract_param(tgc);
    return scale_by_division(tgc_raw, tgc.scale_exp);
}

float MLX90641EEpromParser::get_emissivity_ee() const
{
    // emissivity = ee_data[35] / 512.0
    constexpr SingleEepromWord emissivity{EepromAddr::emissivity, 0, 11, 9, false};
    const auto emissivity_raw = extract_param(emissivity);
    return scale_by_division(emissivity_raw, emissivity.scale_exp);
}

uint8_t MLX90641EEpromParser::get_resolution_ee() const
{
    // resolutionEE = (ee_data[51] & 0x0600) >> 9
    constexpr SingleEepromWord resolution{EepromAddr::resolution, 9, 2, 0, false};
    return static_cast<uint8_t>(extract_param(resolution));
}

float MLX90641EEpromParser::get_ks_ta() const
{
    // ksTa = ee_data[34] / 32768.0
    constexpr SingleEepromWord ks_ta{EepromAddr::ks_ta, 0, 11, 15, true};
    const auto ks_ta_raw = extract_param(ks_ta);
    return scale_by_division(ks_ta_raw, ks_ta.scale_exp);
}

std::array<float, 8> MLX90641EEpromParser::get_ks_to() const
{
    constexpr SingleEepromWord ks_to_scale{EepromAddr::ks_to_scale, 0, 11, 0, false};
    std::array<float, 8> ks_to_values;
    // the scaling factor is ee_data[52], we need to extract it first
    const uint8_t ks_to_scale_value = static_cast<uint8_t>(extract_param(ks_to_scale));
    const std::array<SingleEepromWord, 8> ks_to_words = {{
        SingleEepromWord{EepromAddr::ks_to0, 0, 11, ks_to_scale_value, true}, 
        SingleEepromWord{EepromAddr::ks_to1, 0, 11, ks_to_scale_value, true}, 
        SingleEepromWord{EepromAddr::ks_to2, 0, 11, ks_to_scale_value, true}, 
        SingleEepromWord{EepromAddr::ks_to3, 0, 11, ks_to_scale_value, true}, 
        SingleEepromWord{EepromAddr::ks_to4, 0, 11, ks_to_scale_value, true}, 
        SingleEepromWord{EepromAddr::ks_to5, 0, 11, ks_to_scale_value, true}, 
        SingleEepromWord{EepromAddr::ks_to6, 0, 11, ks_to_scale_value, true}, 
        SingleEepromWord{EepromAddr::ks_to7, 0, 11, ks_to_scale_value, true}  
    }};
    for (uint8_t i = 0; i < ks_to_words.size(); ++i) {
        ks_to_values[i] = scale_by_division(extract_param(ks_to_words[i]), ks_to_words[i].scale_exp);
    }
    return ks_to_values;
}

std::array<float, 192> MLX90641EEpromParser::get_alpha() const
{
    // note: the datasheet shows that the values for alpha_scale_row altern from 
    // a bit-width of 5 and 6, but in the melexis library we see that
    // they are all treated as 5-bit values. This is more intuitive and is
    // what we will use here.
    constexpr std::array<SingleEepromWord, 6> scale_row_alpha = {
    SingleEepromWord{EepromAddr::alpha_scale0, 5, 5, 0, false},   // (eeData[25] >> 5) + 20
    SingleEepromWord{EepromAddr::alpha_scale0, 0, 5, 0, false},   // (eeData[25] & 0x001F) + 20
    SingleEepromWord{EepromAddr::alpha_scale1, 5, 5, 0, false},   // (eeData[26] >> 5) + 20
    SingleEepromWord{EepromAddr::alpha_scale1, 0, 5, 0, false},   // (eeData[26] & 0x001F) + 20
    SingleEepromWord{EepromAddr::alpha_scale2, 5, 5, 0, false},   // (eeData[27] >> 5) + 20
    SingleEepromWord{EepromAddr::alpha_scale2, 0, 5, 0, false}    // (eeData[27] & 0x001F) + 20
};

    std::array<float, 192> alpha;   
    std::array<float, 6> row_max_alpha_norm;
    std::array<std::uint8_t, 6> scale_row_alpha_values;

    // Extract scaling factors for each row
    for (uint8_t i = 0; i < row_max_alpha_norm.size(); ++i) {
        scale_row_alpha_values[i] = static_cast<uint8_t>(extract_param(scale_row_alpha[i])) + 20;
    }

    const std::array<SingleEepromWord, 6> alpha_max_row = {
        SingleEepromWord{EepromAddr::alpha_max_row0, 0, 11, scale_row_alpha_values[0], false}, 
        SingleEepromWord{EepromAddr::alpha_max_row1, 0, 11, scale_row_alpha_values[1], false}, 
        SingleEepromWord{EepromAddr::alpha_max_row2, 0, 11, scale_row_alpha_values[2], false}, 
        SingleEepromWord{EepromAddr::alpha_max_row3, 0, 11, scale_row_alpha_values[3], false}, 
        SingleEepromWord{EepromAddr::alpha_max_row4, 0, 11, scale_row_alpha_values[4], false}, 
        SingleEepromWord{EepromAddr::alpha_max_row5, 0, 11, scale_row_alpha_values[5], false}  
    };

    // Calculate normalized maximum alpha values for each row
    for (uint8_t i = 0; i < row_max_alpha_norm.size(); ++i) {
        row_max_alpha_norm[i] = static_cast<uint16_t>(extract_param(alpha_max_row[i]));
        row_max_alpha_norm[i] = scale_by_division(row_max_alpha_norm[i],  alpha_max_row[i].scale_exp);
        row_max_alpha_norm[i] = row_max_alpha_norm[i] / 2047.0f; // Why 2047? Couldn't find in datasheet
    }

    // Calculate alpha for each pixel
    for (uint8_t i = 0; i < 6; ++i) {
        for (uint8_t j = 0; j < 32; ++j) {
            const uint16_t p = 32 * i + j;
            alpha[p] = static_cast<float>(eeprom_data_[256 + p]) * row_max_alpha_norm[i];
        }
    }

    return alpha;
}

std::array<std::int16_t, 8> MLX90641EEpromParser::get_ct() const
{
    constexpr std::array<SingleEepromWord, 3> ct_words = {
        SingleEepromWord{EepromAddr::ct0, 0, 11, 0, false}, 
        SingleEepromWord{EepromAddr::ct1, 0, 11, 0, false}, 
        SingleEepromWord{EepromAddr::ct2, 0, 11, 0, false}  
    };
    
    return {
        -40, -20, 0, 80, 120, 
        static_cast<int16_t>(extract_param(ct_words[0])),
        static_cast<int16_t>(extract_param(ct_words[1])), 
        static_cast<int16_t>(extract_param(ct_words[2]))
    };
}

std::array<float, 192> MLX90641EEpromParser::get_kta() const
{
    constexpr SingleEepromWord kta_avg{EepromAddr::kta_avg, 0, 11, 0, true};
    constexpr std::array<SingleEepromWord, 2> kta_scale_words = {
        SingleEepromWord{EepromAddr::kta_scale, 5, 5, 0, false}, 
        SingleEepromWord{EepromAddr::kta_scale, 0, 5, 0, false}  
    };
    std::array<float, 192> kta;
    
    // Extract KTA average
    int16_t kta_avg_value = static_cast<int16_t>(extract_param(kta_avg));
    
    // Extract scale factors
    uint8_t kta_scale1_value = static_cast<uint8_t>(extract_param(kta_scale_words[0U]));
    uint8_t kta_scale2_value = static_cast<uint8_t>(extract_param(kta_scale_words[1U]));

    // Extract KTA for each pixel
    for (uint16_t i = 0U; i < kta.size(); ++i) {
        const std::uint16_t address = EepromAddr::kta_pixel + i;
        const SingleEepromWord word = {address, 5, 6, kta_scale2_value, true};
        const auto temp_kta = extract_param(word);
        // Keeping original code from Melexis library because using the 
        // scaling functions caused issues with truncation and overflow.
        kta[i] = temp_kta * static_cast<float>(1ULL << kta_scale2_value);
        kta[i] = kta[i] + kta_avg_value;
        kta[i] = kta[i] / static_cast<float>(1ULL << kta_scale1_value);
    }
    
    return kta;
}

std::array<float, 192> MLX90641EEpromParser::get_kv() const
{
    constexpr SingleEepromWord kv_avg{EepromAddr::kv_avg, 0, 11, 0, true};
    constexpr std::array<SingleEepromWord, 2> kv_scale_words = {
        SingleEepromWord{EepromAddr::kv_scale, 5, 5, 0, false}, 
        SingleEepromWord{EepromAddr::kv_scale, 0, 5, 0, false}  
    };
    std::array<float, 192> kv;

    // Extract KV average
    int16_t kv_avg_value = static_cast<int16_t>(extract_param(kv_avg));
    
    // Extract scale factors
    uint8_t kv_scale1_value = static_cast<uint8_t>(extract_param(kv_scale_words[0U]));
    uint8_t kv_scale2_value = static_cast<uint8_t>(extract_param(kv_scale_words[1U]));

    // Extract KV for each pixel
    for (uint16_t i = 0U; i < kv.size(); ++i) {
        // Extract tempKv from eeData[448 + i] & 0x001F
        const std::uint16_t address = EepromAddr::kv_pixel + i;
        const SingleEepromWord word = {address, 0, 5, kv_scale2_value, true};
        const auto temp_kv = extract_param(word);
        // Keeping original code from Melexis library because using the 
        // scaling functions caused issues with truncation and overflow.
        kv[i] = static_cast<float>(temp_kv) * static_cast<float>(1ULL << kv_scale2_value);
        kv[i] = kv[i] + static_cast<float>(kv_avg_value);
        kv[i] = kv[i] / static_cast<float>(1ULL << kv_scale1_value);
    }
    
    return kv;
}

float MLX90641EEpromParser::get_cp_kta() const
{
    constexpr SingleEepromWord cp_kta{EepromAddr::cp_kta, 0, 6, 0, true};
    constexpr SingleEepromWord cp_kta_scale{EepromAddr::cp_kta, 6, 5, 0, false};
    // Extract cpKta value and scale
    int16_t cp_kta_value = static_cast<int16_t>(extract_param(cp_kta));
    uint8_t cp_kta_scale_value = static_cast<uint8_t>(extract_param(cp_kta_scale));
    return scale_by_division(cp_kta_value, cp_kta_scale_value);
}

float MLX90641EEpromParser::get_cp_kv() const
{
    constexpr SingleEepromWord cp_kv{EepromAddr::cp_kv, 0, 6, 0, true};
    constexpr SingleEepromWord cp_kv_scale{EepromAddr::cp_kv, 6, 5, 0, false};
    // Extract cpKv value and scale
    int16_t cp_kv_value = static_cast<int16_t>(extract_param(cp_kv));
    uint8_t cp_kv_scale_value = static_cast<uint8_t>(extract_param(cp_kv_scale));

    return scale_by_division(cp_kv_value, cp_kv_scale_value);
}

float MLX90641EEpromParser::get_cp_alpha() const
{
    constexpr SingleEepromWord cp_alpha{EepromAddr::cp_alpha, 0, 11, 0, false};
    constexpr SingleEepromWord cp_alpha_scale{EepromAddr::cp_alpha_scale, 0, 11, 0, false};
    // Extract alphaCP value and scale
    int16_t cp_alpha_value = extract_param(cp_alpha);
    uint8_t cp_alpha_scale_value = static_cast<uint8_t>(extract_param(cp_alpha_scale));

    return scale_by_division(cp_alpha_value, cp_alpha_scale_value);
}

int16_t MLX90641EEpromParser::get_cp_offset() const
{
    constexpr DualEepromWord cp_offset_words = {{
        EepromWord{EepromAddr::cp_offset0, 0, 11},
        EepromWord{EepromAddr::cp_offset1, 0, 5}}, 
        0, true
    };
    return static_cast<int16_t>(extract_param_array(cp_offset_words));
}

std::array<std::array<std::int16_t, 192>, 2> MLX90641EEpromParser::get_offset() const
{
    constexpr SingleEepromWord scale_offset{EepromAddr::scale_offset, 5, 6, 0, false};
    constexpr DualEepromWord offset_ref_words = {{
        EepromWord{EepromAddr::offset_ref0, 0, 11}, 
        EepromWord{EepromAddr::offset_ref1, 0, 5} 
    }, 0, true};

    const auto scale_offset_value = static_cast<uint8_t>(extract_param(scale_offset));
    const auto offset_ref_value = static_cast<int16_t>(extract_param_array(offset_ref_words));

    std::array<std::array<std::int16_t, 192>, 2> offset;

    for (uint16_t i = 0; i < offset[0].size(); ++i) {
        // Even subpage offset
        std::uint16_t address1 = EepromAddr::offset_even + i;
        const SingleEepromWord word = {address1, 0, 11, 0, true};
        offset[0][i] = static_cast<std::int16_t>(extract_param(word));
        offset[0][i] = scale_by_multiplication(offset[0][i], scale_offset_value);
        offset[0][i] = offset[0][i] + offset_ref_value;
        // Odd subpage offset
        std::uint16_t address2 = EepromAddr::offset_odd + i;
        const SingleEepromWord word2 = {address2, 0, 11, 0, true};
        offset[1][i] = static_cast<std::int16_t>(extract_param(word2));
        offset[1][i] = scale_by_multiplication(offset[1][i], scale_offset_value);
        offset[1][i] = offset[1][i] + offset_ref_value;
    }
    
    return offset;
}

std::array<std::uint16_t, 2> MLX90641EEpromParser::get_broken_pixels() const
{
    constexpr std::size_t pixel_count = 192u; // total number of pixels
    std::array<std::uint16_t, 2> broken_pixels;
    std::fill(broken_pixels.begin(), broken_pixels.end(), 0xFFFF);

    std::size_t num_broken_pixels = 0u;
    for (std::size_t i = 0u; i < pixel_count && num_broken_pixels < 3; ++i) 
    {
        const uint16_t address1 = EepromAddr::offset_even + i;
        const SingleEepromWord word1 = {address1, 0, 11, 0, false};
        const uint16_t address2 = EepromAddr::alpha_pixel + i;
        const SingleEepromWord word2 = {address2, 0, 11, 0, false};
        const uint16_t address3 = EepromAddr::kta_pixel + i;
        const SingleEepromWord word3 = {address3, 0, 11, 0, false};
        const uint16_t address4 = EepromAddr::offset_odd + i;
        const SingleEepromWord word4 = {address4, 0, 11, 0, false};
        if (extract_param(word1) == 0 && extract_param(word2) == 0 &&
            extract_param(word3) == 0 && extract_param(word4) == 0) 
        {
        broken_pixels[num_broken_pixels] = i;
        num_broken_pixels++; 
        }
    }
    
    return broken_pixels;
}

uint32_t MLX90641EEpromParser::extract_raw_field(const std::array<uint16_t, eeprom_size>& eeprom_data,
                                          const EepromWord& w)
{
    if (w.address < eeprom_start_address)
    {
        throw std::out_of_range("EEPROM address below start address");
    }
    const uint16_t index = w.address - eeprom_start_address;
    if (index >= eeprom_size)
    {
        throw std::out_of_range("EEPROM index out of range");
    }
    // check for bit_width of 32 to avoid undefined behavior
    const uint32_t mask = w.bit_width >= 32 ? 0xFFFFFFFFu : ((1ul << w.bit_width) - 1ul);
    return (eeprom_data[index] >> w.start_bit) & mask;
}

int32_t MLX90641EEpromParser::apply_sign_extension(uint32_t value, uint8_t bit_width)
{
    const uint32_t sign_bit = 1u << (bit_width - 1);
    if (value & sign_bit) {
        value -= (1u << bit_width);
    }
    return static_cast<int32_t>(value);
}

int32_t MLX90641EEpromParser::extract_param(const SingleEepromWord& word) const
{
    const uint32_t raw = extract_raw_field(eeprom_data_, word.word);
    return word.is_signed ? apply_sign_extension(raw, word.word.bit_width)
                        : static_cast<int32_t>(raw);
}

int32_t MLX90641EEpromParser::extract_param_array(const DualEepromWord& words) const
{
    const auto& upper = words.words[0];
    const auto& lower = words.words[1];
    const uint8_t total_bit_width = upper.bit_width + lower.bit_width;

    if (total_bit_width > 32) {
        throw std::runtime_error("Combined bit width exceeds 32 bits");
    }

    const uint32_t upper_val = extract_raw_field(eeprom_data_, upper);
    const uint32_t lower_val = extract_raw_field(eeprom_data_, lower);
    const uint32_t combined = (upper_val << lower.bit_width) | lower_val;

    return words.is_signed ? apply_sign_extension(combined, total_bit_width)
                        : static_cast<int32_t>(combined);
}

} // namespace mlx90641_eeprom
