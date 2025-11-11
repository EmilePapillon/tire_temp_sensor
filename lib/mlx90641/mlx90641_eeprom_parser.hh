#pragma once
#include <array>
#include <cstdint>
#include "mlx90641_eeprom_addr.hh"
#include "mlx90641_params.hh"

namespace mlx90641 {

constexpr std::size_t eeprom_size = 832;
constexpr std::size_t eeprom_start_address = 0x2400;

/// @brief Structure to define an EEPROM word to extract a parameter.
/// address: the EEPROM address in hexadecimal format (from datasheet).
/// bit_width: the number of bits used for the parameter.
/// start_bit: the starting bit position for the parameter.
struct EepromWord {
    uint16_t address;
    uint8_t start_bit;
    uint8_t bit_width;
};

/// @brief Structure to define a single EEPROM word with scaling information.
/// scale_exp: the exponent for scaling (i.e., divide by 2^scale_exp or multiply by 2^scale_exp)
/// is_signed: whether the parameter is signed
/// Most parameters are contained within a single 16-bit word.
struct SingleEepromWord {
    EepromWord word;
    uint8_t scale_exp;
    bool is_signed; 
};

/// @brief Structure to define two EEPROM words to extract a parameter.
/// Some parameters span two consecutive 16-bit words in the EEPROM data.
/// For these, we combine the two words into a single integer value.
/// scale_exp: the exponent for scaling (i.e., divide by 2^scale_exp or multiply by 2^scale_exp)
/// is_signed: whether the combined parameter is signed
struct DualEepromWord {
    std::array<EepromWord, 2> words; 
    uint8_t scale_exp;
    bool is_signed;
};

// Values with a decimal point are scaled by division
// the denominator is 2^scale_exp. 
inline float scale_by_division(int32_t raw_value, uint8_t scale_exp) {
    return static_cast<float>(raw_value) / static_cast<float>(1ULL << scale_exp);
}

// Signed integral values are scaled by bit shifting to the right
// Note: Unsigned integers are used raw, without scaling. 
inline std::int16_t scale_by_multiplication(int16_t raw_value, uint8_t scale_exp) {
    // Handle potential overflow by promoting to int32_t before multiplication
    return static_cast<int16_t>(static_cast<int32_t>(raw_value) * (1 << scale_exp));
}

/// @brief Class to extract and hold MLX90641 EEPROM parameters
class MLX90641EEpromParser {
public: 
    MLX90641EEpromParser(const std::array<uint16_t, eeprom_size>& eeprom_data)
        : eeprom_data_(eeprom_data) {}

    /// @brief Extracts all parameters and fills the provided ParamsMLX90641 structure.
    bool extract_all(ParamsMLX90641& params) const;

    /// @brief Returns the KVdd calibration coefficient (units: LSB/V).
    ///  
    /// KVdd is a temperature coefficient used to compensate for supply-voltage dependence.
    /// It is stored in EEPROM, extracted as a signed 11-bit value, and scaled by 2⁵  
    /// (i.e. *32).  
    /// If KVdd > 1023, it is converted to negative via two’s complement (KVdd - 2048).  
    int16_t get_kvdd() const;

    /// @brief Returns VDD25, the reference supply voltage value (units: LSB).
    ///  
    /// VDD25 is the sensor’s supply voltage (Vdd) measured at 25 °C, used in internal compensation.  
    /// Like KVdd, it is stored as a signed 11-bit value and scaled by 2⁵.  
    /// If VDD25 > 1023, then VDD25 -= 2048.  
    int16_t get_vdd25() const;

    /// @brief Returns KV_PTAT, the proportional-to-absolute-temperature voltage coefficient.
    ///  
    /// KV_PTAT is derived from the PTAT (proportional-to-absolute-temperature) sensor  
    /// inside the device. Stored in EEPROM, it is read, sign-extended, and scaled (division by 2¹²).  
    /// It is used in Ta calculation to relate PTAT voltage changes to temperature.  
    float get_kv_ptat() const;

    /// @brief Returns KT_PTAT, the temperature coefficient for the PTAT sensor.
    ///  
    /// KT_PTAT is a gain factor (scaling) applied to the PTAT voltage to compute ambient temperature (Ta).  
    /// It is read from EEPROM, sign-extended, and scaled via division by 2³.  
    float get_kt_ptat() const;

    /// @brief Returns VPTAT25, the PTAT voltage at 25 °C (unsigned).
    ///  
    /// VPTAT25 is stored across two EEPROM words: 11 bits from a “upper” word and 5 bits from a “lower” word.  
    /// The combined unsigned value is computed as `upper << 5 | lower`.  
    std::uint16_t get_vptat25() const;

    /// @brief Returns ALPHA_PTAT, the proportionality factor that maps PTAT voltage to temperature offset.
    ///  
    /// ALPHA_PTAT is used in object temperature calculations to relate PTAT to the target pixel’s temperature.  
    /// It is stored in EEPROM, read as an unsigned 11-bit value, and scaled by 2⁷.  
    float get_alpha_ptat() const;

    /// @brief Returns GAIN_EE, the device gain calibration coefficient (unsigned).
    ///  
    /// GAIN_EE is stored across two EEPROM words (11+5 bits). It is the amplifier gain setting used internally  
    /// for pixel measurements and corrections.  
    std::int16_t get_gain_ee() const;

    /// @brief Returns TGC, the Temperature Gradient Coefficient (signed).
    ///  
    /// TGC is used to correct for temperature gradient effects across the sensor.  
    /// It is read from EEPROM as a signed 9-bit field and scaled (division by 2⁶).  
    float get_tgc() const;

    /// @brief Returns Emissivity calibration (unitless float).
    ///  
    /// Emissivity_EE is stored as an 11-bit unsigned value in EEPROM and scaled by division by 2⁹.  
    /// Represents the default emissivity of the measured object (e.g. 0.95) for compensation.  
    float get_emissivity_ee() const;

    /// @brief Returns the EEPROM‐stored ADC resolution setting (0–3).
    ///  
    /// This indicates the resolution (number of ADC bits) that the device was calibrated for (e.g. 16, 17, 18, 19 bits).  
    uint8_t get_resolution_ee() const;

    /// @brief Returns KS_TA, the ambient temperature sensitivity coefficient.
    ///  
    /// KS_TA is a temperature coefficient used to compensate for ambient temperature effects on pixel readings.  
    /// Stored as a signed 11-bit value, scaled by 2¹⁵ in the datasheet.  
    float get_ks_ta() const;

    /// @brief Returns KS_TO for the 8 temperature ranges (float array).
    ///  
    /// KS_TO is a per-range slope factor that models how sensor sensitivity changes with object temperature.  
    /// The EEPROM stores scale factors and 8 per-range sensitivity coefficients (signed 11-bit).  
    std::array<float, 8> get_ks_to() const;

    /// @brief Returns α (alpha) for each pixel (192 entries).
    ///  
    /// Alpha is the sensitivity coefficient for each pixel (how many LSB per Kelvin).  
    /// It is stored per pixel in EEPROM and adjusted by row-scaling, normalization, etc.  
    std::array<float, 192> get_alpha() const;

    /// @brief Returns CT (corner temperature) calibration values (8 entries).
    ///  
    /// The CT array holds fixed corner temperatures used in piecewise linear models for pixel correction.  
    std::array<std::int16_t, 8> get_ct() const;

    /// @brief Returns KTA coefficients for each pixel (192 entries).
    ///  
    /// KTA is the temperature coefficient per pixel (how much the offset changes per ambient °C).  
    /// Stored in EEPROM (signed), scaled by two stage scales (KTA_scale1 & scale2).  
    std::array<float, 192> get_kta() const;

    /// @brief Returns KV coefficients for each pixel (192 entries).
    ///  
    /// KV is the temperature coefficient per pixel (how much the sensitivity changes per ambient °C).  
    /// Stored (signed) per pixel in EEPROM and adjusted via scaling factors.  
    std::array<float, 192> get_kv() const;

    /// @brief Returns CP_KTA, the compensation KTA coefficient.
    float get_cp_kta() const;

    /// @brief Returns CP_KV, the compensation KV coefficient.
    float get_cp_kv() const;

    /// @brief Returns CP_ALPHA, the compensation alpha coefficient.
    float get_cp_alpha() const;

    /// @brief Returns CP_OFFSET, the compensation pixel offset (signed).
    int16_t get_cp_offset() const;

    /// @brief Returns pixel offset array for both subpages (2 × 192).
    ///  
    /// Provides the per-pixel offset (baseline reading) for subpage 0 and 1 corrections.  
    std::array<std::array<std::int16_t, 192>, 2> get_offset() const;

    /// @brief Returns indices of broken pixels (max 2) that should be masked/ignored.
    ///  
    /// Pixels are considered “broken” if all EEPROM offset values at their positions (across subpages) are zero.  
    std::array<std::uint16_t, 2> get_broken_pixels() const; 


private: 
    /// @brief Utility function to extract a raw bitfield from a single EEPROM word.
    /// 
    /// This masks and shifts the EEPROM word according to the bit width and start bit.
    /// No sign handling or scaling is performed here.
    /// 
    /// @param eeprom_data The EEPROM data array.
    /// @param w The EEPROM word descriptor containing index, bit width, and start bit.
    /// @return The extracted raw (unsigned) value.
    static uint32_t extract_raw_field(const std::array<uint16_t, eeprom_size>& eeprom_data,
                                     const EepromWord& w);
    /// @brief Utility function to apply sign extension to a bitfield value.
    /// 
    /// This interprets the most significant bit as a sign bit and performs
    /// two’s complement sign extension if the value is negative.
    /// 
    /// @param value The raw (unsigned) value.
    /// @param bit_width The width of the signed field in bits.
    /// @return The properly sign-extended value as a signed integer.
    static int32_t apply_sign_extension(uint32_t value, uint8_t bit_width);
    
    /// @brief Extracts a parameter value from a single EEPROM word.
    /// 
    /// This function reads a single EEPROM word, applies a mask and right shift, 
    /// and returns the resulting value. If the field is signed, the most significant 
    /// bit is interpreted as the sign and the result is sign-extended.
    /// 
    /// @param word A SingleEepromWord defining the EEPROM index, bit layout, scaling, and signedness.
    /// @return The extracted parameter value as a signed 32-bit integer.
    int32_t extract_param(const SingleEepromWord& word) const;
    
    /// @brief Extracts a parameter value spanning two EEPROM words.
    /// 
    /// This function reconstructs a value that spans two 16-bit EEPROM words by 
    /// combining the masked and aligned bits from each segment. The upper word 
    /// contributes the high bits, and the lower contributes the low bits.
    /// 
    /// Example (from MLX90641 datasheet):
    ///   vPTAT25 = 32 * ee_data[40] + ee_data[41]
    ///   → upper = { index=40, bit_width=11 }
    ///   → lower = { index=41, bit_width=5 }
    ///   → result = (ee_data[40] << 5) | ee_data[41]
    /// 
    /// @param words A DualEepromWord defining the indices, bit widths, scaling, and signedness.
    /// @return The reconstructed signed 32-bit value.
    /// 
    /// @note The combined bit width must not exceed 32 bits.
    int32_t extract_param_array(const DualEepromWord& words) const;

    std::array<uint16_t, eeprom_size> eeprom_data_;
};

} // namespace mlx90641_eeprom