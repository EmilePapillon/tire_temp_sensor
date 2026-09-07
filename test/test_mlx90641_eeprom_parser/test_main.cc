#include <unity.h>
#include <array>
#include <cmath>
#include "mlx90641_eeprom_parser.hh"
#include "mlx90641_params.hh"
#include "fixtures/mlx90641_eeprom_fixture.hh"

using namespace mlx90641;

constexpr float float_epsilon = 0.0001;

// Global EEPROM object for all tests
MLX90641EEpromParser* eeprom = nullptr;

void setUp(void) {
    eeprom = new MLX90641EEpromParser(test_eeprom_data);
}

void tearDown(void) {
    delete eeprom;
    eeprom = nullptr;
}

void test_kv_ptat() {
    const auto kv_ptat = eeprom->get_kv_ptat();
    TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.KvPTAT, kv_ptat);
}

void test_kt_ptat() {
    const auto kt_ptat = eeprom->get_kt_ptat();
    TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.KtPTAT, kt_ptat);
}

void test_alpha_ptat() {
    const auto alpha_ptat = eeprom->get_alpha_ptat();
    TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.alphaPTAT, alpha_ptat);
}

void test_tgc() {
    const auto tgc = eeprom->get_tgc();
    TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.tgc, tgc);
}

void test_ks_ta() {
    const auto ks_ta = eeprom->get_ks_ta();
    TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.KsTa, ks_ta);
}

void test_ks_to() {
    const auto ks_to = eeprom->get_ks_to();
    for (size_t i = 0; i < ks_to.size(); ++i) {
        TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.ksTo[i], ks_to[i]);
    }
}

void test_alpha() {
    const auto alpha = eeprom->get_alpha();
    for (size_t i = 0; i < alpha.size(); ++i) {
        TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.alpha[i], alpha[i]);
    }
}

void test_alpha_scale_preserves_sixth_bit() {
    auto data = test_eeprom_data;
    constexpr std::size_t alpha_scale0_index = EepromAddr::alpha_scale0 - eeprom_start_address;
    data[alpha_scale0_index] = static_cast<uint16_t>((data[alpha_scale0_index] & 0x001F) | (32U << 5));

    MLX90641EEpromParser parser(data);
    const auto alpha = parser.get_alpha();
    const auto row_max = static_cast<float>(data[EepromAddr::alpha_max_row0 - eeprom_start_address] & 0x07FF);
    const auto pixel_alpha = static_cast<float>(data[EepromAddr::alpha_pixel - eeprom_start_address] & 0x07FF);
    const auto expected = pixel_alpha * (row_max / static_cast<float>(1ULL << 52U)) / 2047.0f;

    // Relative tolerance: with the sixth bit set the scale exponent is 52, so expected is
    // ~2e-13 and an absolute 1e-4 window would pass for almost any alpha[0].
    TEST_ASSERT_FLOAT_WITHIN(expected * 1e-4f, expected, alpha[0]);
}

void test_kta() {
    const auto kta = eeprom->get_kta();
    for (size_t i = 0; i < kta.size(); ++i) {
        TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.kta[i], kta[i]);
    }
}

void test_kv() {
    const auto kv = eeprom->get_kv();
    for (size_t i = 0; i < kv.size(); ++i) {
        TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.kv[i], kv[i]);
    }
}

void test_cp_kv() {
    const auto cp_kv = eeprom->get_cp_kv();
    TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.cpKv, cp_kv);
}

void test_cp_kta() {
    const auto cp_kta = eeprom->get_cp_kta();
    TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.cpKta, cp_kta);
}

void test_cp_alpha() {
    const auto cp_alpha = eeprom->get_cp_alpha();
    TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.cpAlpha, cp_alpha);
}

void test_cp_offset() {
    const auto cp_offset = eeprom->get_cp_offset();
    TEST_ASSERT_EQUAL(expected_params.cpOffset, cp_offset);
}

void test_kvdd() {
    const auto kvdd = eeprom->get_kvdd();
    TEST_ASSERT_EQUAL(expected_params.kVdd, kvdd);
}

void test_vdd25() {
    const auto vdd25 = eeprom->get_vdd25();
    TEST_ASSERT_EQUAL(expected_params.vdd25, vdd25);
}

void test_vptat25() {
    const auto vptat25 = eeprom->get_vptat25();
    TEST_ASSERT_EQUAL(expected_params.vPTAT25, vptat25);
}

void test_gain_ee() {
    const auto gain_ee = eeprom->get_gain_ee();
    TEST_ASSERT_EQUAL(expected_params.gainEE, gain_ee);
}

void test_emissivity_ee() {
    const auto emissivity_ee = eeprom->get_emissivity_ee();
    TEST_ASSERT_FLOAT_WITHIN(float_epsilon, expected_params.emissivityEE, emissivity_ee);
}

void test_resolution_ee() {
    const auto resolution_ee = eeprom->get_resolution_ee();
    TEST_ASSERT_EQUAL(expected_params.resolutionEE, resolution_ee);
}

void test_ct() {
    const auto ct = eeprom->get_ct();
    for (size_t i = 0; i < ct.size(); ++i) {
        TEST_ASSERT_EQUAL(expected_params.ct[i], ct[i]);
    }
}

void test_offset() {
    const auto offset = eeprom->get_offset();
    for (size_t i = 0; i < offset[0].size(); ++i) {
        TEST_ASSERT_EQUAL(expected_params.offset[0][i], offset[0][i]);
        TEST_ASSERT_EQUAL(expected_params.offset[1][i], offset[1][i]);
    }
}

void test_broken_pixels() {
    const auto broken_pixels = eeprom->get_broken_pixels();
    for (size_t i = 0; i < broken_pixels.size(); ++i) {
        TEST_ASSERT_EQUAL(expected_params.brokenPixels[i], broken_pixels[i]);
    }
}

void test_broken_pixels_lists_a_dead_pixel() {
    MLX90641EEpromParser parser(eeprom_with_broken_pixels({100}));
    const auto broken_pixels = parser.get_broken_pixels();
    TEST_ASSERT_EQUAL_UINT16(100, broken_pixels[0]);
    TEST_ASSERT_EQUAL_UINT16(no_broken_pixel, broken_pixels[1]);
}

void test_extract_all_accepts_one_broken_pixel() {
    // The datasheet allows a part to ship with one; the driver corrects it.
    ParamsMLX90641 params{};
    TEST_ASSERT_TRUE(MLX90641EEpromParser(eeprom_with_broken_pixels({100})).extract_all(params));
    TEST_ASSERT_EQUAL_UINT16(100, params.brokenPixels[0]);
    TEST_ASSERT_EQUAL_UINT16(no_broken_pixel, params.brokenPixels[1]);
}

void test_extract_all_rejects_more_broken_pixels_than_can_be_corrected() {
    // More broken pixels than slots: the scan must stop at the last slot rather
    // than run past the end of the list, and the image must be rejected.
    ParamsMLX90641 params{};
    TEST_ASSERT_FALSE(MLX90641EEpromParser(eeprom_with_broken_pixels({10, 20, 30, 40, 50})).extract_all(params));
    TEST_ASSERT_EQUAL_UINT16(10, params.brokenPixels[0]);
    TEST_ASSERT_EQUAL_UINT16(20, params.brokenPixels[1]);
}

void test_scale_helpers_saturate_on_an_oversized_exponent() {
    // A corrupt image can decode a scale exponent past the width of the shifted
    // type; the helpers must saturate rather than shift out of range.
    TEST_ASSERT_EQUAL_FLOAT(0.0f, scale_by_division(1234, 200));
    TEST_ASSERT_EQUAL_INT16(0, scale_by_multiplication(1234, 200));
    // Exponents the EEPROM can legitimately carry are unaffected.
    TEST_ASSERT_FLOAT_WITHIN(float_epsilon, 0.5f, scale_by_division(1024, 11));
    TEST_ASSERT_EQUAL_INT16(4936, scale_by_multiplication(1234, 2));
}

void test_oversized_alpha_scale_yields_finite_alphas() {
    // The six-bit alpha-scale field is biased by 20, so its maximum decodes to a
    // shift of 83 - past the 64 bits scale_by_division shifts.
    auto data = test_eeprom_data;
    constexpr std::size_t alpha_scale0_index = EepromAddr::alpha_scale0 - eeprom_start_address;
    data[alpha_scale0_index] = static_cast<uint16_t>((data[alpha_scale0_index] & 0x001F) | (63U << 5));

    MLX90641EEpromParser parser(data);
    const auto alpha = parser.get_alpha();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, alpha[0]);
    for (const float value : alpha) {
        TEST_ASSERT_TRUE(std::isfinite(value));
    }
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_kv_ptat);
    RUN_TEST(test_kt_ptat);
    RUN_TEST(test_alpha_ptat);
    RUN_TEST(test_tgc);
    RUN_TEST(test_ks_ta);
    RUN_TEST(test_ks_to);
    RUN_TEST(test_alpha);
    RUN_TEST(test_alpha_scale_preserves_sixth_bit);
    RUN_TEST(test_kta);
    RUN_TEST(test_kv);
    RUN_TEST(test_cp_kv);
    RUN_TEST(test_cp_kta);
    RUN_TEST(test_cp_alpha);
    RUN_TEST(test_cp_offset);
    RUN_TEST(test_kvdd);
    RUN_TEST(test_vdd25);
    RUN_TEST(test_vptat25);
    RUN_TEST(test_gain_ee);
    RUN_TEST(test_emissivity_ee);
    RUN_TEST(test_resolution_ee);
    RUN_TEST(test_ct);
    RUN_TEST(test_offset);
    RUN_TEST(test_broken_pixels);
    RUN_TEST(test_broken_pixels_lists_a_dead_pixel);
    RUN_TEST(test_extract_all_accepts_one_broken_pixel);
    RUN_TEST(test_extract_all_rejects_more_broken_pixels_than_can_be_corrected);
    RUN_TEST(test_scale_helpers_saturate_on_an_oversized_exponent);
    RUN_TEST(test_oversized_alpha_scale_yields_finite_alphas);
    return UNITY_END();
}