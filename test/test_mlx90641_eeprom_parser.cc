#include <unity.h>
#include <array>
#include "mlx90641_eeprom_parser.hh"
#include "mlx90641_params.hh"
#include "test_data_mlx90641_eeprom.hh"

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

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_kv_ptat);
    RUN_TEST(test_kt_ptat);
    RUN_TEST(test_alpha_ptat);
    RUN_TEST(test_tgc);
    RUN_TEST(test_ks_ta);
    RUN_TEST(test_ks_to);
    RUN_TEST(test_alpha);
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
    return UNITY_END();
}