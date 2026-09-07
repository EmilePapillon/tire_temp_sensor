#include <unity.h>
#include <array>
#include <cmath>
#include <cstdint>
#include "fixtures/mlx90641_eeprom_fixture.hh"
#include "fixtures/mlx90641_frame_fixture.hh"
#include "mlx90641_driver.hh"
#include "mocks/mock_i2c_adapter.hh"

using namespace mlx90641;
using Sensor = MLX90641Sensor<MockI2CAdapter>;

constexpr uint8_t sensor_addr = 0x33;
constexpr uint16_t status_reg = 0x8000;
constexpr uint16_t control_reg = 0x800D;
constexpr uint16_t control_reg_initial = 0x0901;  // resolution 2, refresh 2, plus unrelated bits
constexpr uint16_t status_data_ready = 0x0008;
constexpr uint16_t status_clear_ready = 0x0030;

const Mlx90641Config test_config{400, Resolution::Bits19, RefreshRate::Hz32};

MockI2CAdapter* bus = nullptr;
Sensor* sensor = nullptr;

void setUp(void) {
    bus = new MockI2CAdapter();
    bus->load_eeprom(test_eeprom_data);
    bus->registers[control_reg] = control_reg_initial;
    bus->registers[status_reg] = status_data_ready;  // sub-page 0, new data
    sensor = new Sensor(*bus, sensor_addr);
}

void tearDown(void) {
    delete sensor;
    delete bus;
    sensor = nullptr;
    bus = nullptr;
}

void assert_status(Status expected, Status actual) {
    TEST_ASSERT_EQUAL_STRING(status_name(expected), status_name(actual));
}

uint16_t resolution_bits(uint16_t control) { return (control >> 10) & 0x03; }
uint16_t refresh_bits(uint16_t control) { return (control >> 7) & 0x07; }

// Emulate the sensor: acknowledging the status register clears "new data" and
// leaves the sub-page bit in place.
void arm_status_register(uint8_t sub_page) {
    bus->registers[status_reg] = status_data_ready | sub_page;
    bus->on_write = [sub_page](uint16_t reg, uint16_t value) {
        if (reg == status_reg && value == status_clear_ready) {
            bus->registers[status_reg] = sub_page;
        }
    };
}

void prime_frame(uint8_t sub_page) {
    assert_status(Status::Success, sensor->init(test_config));
    arm_status_register(sub_page);
    load_frame(bus->registers, sub_page);
    bus->reads.clear();
    bus->writes.clear();
}

// ---------------------------------------------------------------- init()

void test_init_succeeds_with_valid_eeprom() {
    assert_status(Status::Success, sensor->init(test_config));
    TEST_ASSERT_TRUE(bus->initialised);
}

void test_init_applies_config_to_bus_and_control_register() {
    assert_status(Status::Success, sensor->init(test_config));

    TEST_ASSERT_EQUAL_UINT32(400u, bus->init_freq_khz);
    const uint16_t control = bus->registers[control_reg];
    TEST_ASSERT_EQUAL_HEX16(0x03, resolution_bits(control));
    TEST_ASSERT_EQUAL_HEX16(0x06, refresh_bits(control));
    // Bits outside the resolution / refresh-rate fields are preserved.
    TEST_ASSERT_EQUAL_HEX16(control_reg_initial & ~0x0F80, control & ~0x0F80);
}

void test_init_has_no_hidden_defaults() {
    const Mlx90641Config other{100, Resolution::Bits16, RefreshRate::Hz1};
    assert_status(Status::Success, sensor->init(other));

    TEST_ASSERT_EQUAL_UINT32(100u, bus->init_freq_khz);
    const uint16_t control = bus->registers[control_reg];
    TEST_ASSERT_EQUAL_HEX16(0x00, resolution_bits(control));
    TEST_ASSERT_EQUAL_HEX16(0x01, refresh_bits(control));
}

void test_init_reports_bus_error() {
    bus->read_error = I2cStatus::Nack;
    assert_status(Status::I2cNack, sensor->init(test_config));
    TEST_ASSERT_EQUAL_size_t(0, bus->writes.size());
}

void test_init_rejects_a_device_that_is_not_an_mlx90641() {
    bus->registers[eeprom_start_address + 10] &= static_cast<uint16_t>(~0x0040);  // device-select bit
    assert_status(Status::NotAnMlx90641, sensor->init(test_config));
}

void test_init_rejects_uncorrectable_eeprom_corruption() {
    bus->registers[eeprom_start_address + 100] ^= 0x0003;  // two flipped bits
    assert_status(Status::EepromCorrupt, sensor->init(test_config));
}

void test_init_rejects_a_sensor_with_a_deviating_pixel() {
    // Zero every per-pixel calibration word for pixel 5: the EEPROM's way of
    // flagging a dead pixel. extract_all() must fail closed rather than correct it.
    for (const uint16_t base : {EepromAddr::offset_even, EepromAddr::alpha_pixel,
                                EepromAddr::kta_pixel, EepromAddr::offset_odd}) {
        bus->registers[base + 5] = 0;
    }
    assert_status(Status::CalibrationExtractionFailed, sensor->init(test_config));
}

void test_init_tolerates_a_correctable_eeprom_bit_flip() {
    bus->registers[eeprom_start_address + 100] ^= 0x0001;  // single flipped bit, Hamming-correctable
    assert_status(Status::Success, sensor->init(test_config));
}

// ---------------------------------------------------------------- read_frame()

void test_read_frame_follows_the_status_register_handshake() {
    prime_frame(0);

    assert_status(Status::Success, sensor->read_frame());

    TEST_ASSERT_TRUE(bus->was_written(status_reg, status_clear_ready));
    // Sub-page 0 pixel banks, then the aux data block, then control register 1.
    for (uint16_t bank : {0x0400, 0x0440, 0x0480, 0x04C0, 0x0500, 0x0540}) {
        TEST_ASSERT_TRUE_MESSAGE(bus->was_read(bank), "pixel bank not read");
    }
    TEST_ASSERT_TRUE(bus->was_read(0x0580));
    TEST_ASSERT_TRUE(bus->was_read(control_reg));
    // Sub-page 1 banks must not be touched for a sub-page 0 frame.
    TEST_ASSERT_FALSE(bus->was_read(0x0420));
}

void test_read_frame_accepts_sub_page_1() {
    prime_frame(1);

    assert_status(Status::Success, sensor->read_frame());

    for (uint16_t bank : {0x0420, 0x0460, 0x04A0, 0x04E0, 0x0520, 0x0560}) {
        TEST_ASSERT_TRUE_MESSAGE(bus->was_read(bank), "sub-page 1 pixel bank not read");
    }
    TEST_ASSERT_FALSE(bus->was_read(0x0400));
}

void test_read_frame_tolerates_status_register_readback_mismatch() {
    // The real status register self-clears, so the adapter's write verification
    // reports a mismatch. That must not abort the frame.
    prime_frame(0);
    bus->write_verdict = I2cStatus::VerifyMismatch;

    assert_status(Status::Success, sensor->read_frame());
    TEST_ASSERT_TRUE(bus->was_written(status_reg, status_clear_ready));
}

void test_read_frame_times_out_when_no_frame_ever_arrives() {
    assert_status(Status::Success, sensor->init(test_config));
    bus->registers[status_reg] = 0;  // new-data bit never sets
    bus->reads.clear();

    assert_status(Status::DataReadyTimeout, sensor->read_frame());
    TEST_ASSERT_EQUAL_size_t(Sensor::new_data_poll_limit, bus->reads.size());
}

void test_read_frame_fails_when_new_data_flag_never_clears() {
    assert_status(Status::Success, sensor->init(test_config));
    // A device that keeps re-asserting "new data" no matter how often it is acknowledged.
    bus->on_write = [](uint16_t reg, uint16_t) {
        if (reg == status_reg) {
            bus->registers[status_reg] = status_data_ready;
        }
    };
    assert_status(Status::FrameSyncFailed, sensor->read_frame());
}

void test_read_frame_reports_bus_error() {
    assert_status(Status::Success, sensor->init(test_config));
    bus->read_error = I2cStatus::Nack;
    assert_status(Status::I2cNack, sensor->read_frame());
}

// ---------------------------------------------------------------- calculate_temps()

void test_calculate_temps_reproduces_reference_frame() {
    prime_frame(0);
    assert_status(Status::Success, sensor->read_frame());

    sensor->calculate_temps();
    const auto temps = sensor->get_temps();

    TEST_ASSERT_FLOAT_WITHIN(0.001f, expected_frame_ambient_c, sensor->get_ambient());
    for (const auto& expected : expected_frame_pixels) {
        TEST_ASSERT_FLOAT_WITHIN(0.01f, expected.temp_c, temps[expected.index]);
    }
    for (float t : temps) {
        TEST_ASSERT_TRUE(std::isfinite(t));
        TEST_ASSERT_TRUE(t >= expected_frame_min_c - 0.01f);
        TEST_ASSERT_TRUE(t <= expected_frame_max_c + 0.01f);
    }
}

void test_calculate_temps_on_sub_page_1_agrees_with_sub_page_0() {
    prime_frame(0);
    assert_status(Status::Success, sensor->read_frame());
    sensor->calculate_temps();
    const auto temps_sub_page_0 = sensor->get_temps();

    prime_frame(1);
    assert_status(Status::Success, sensor->read_frame());
    sensor->calculate_temps();
    const auto temps_sub_page_1 = sensor->get_temps();

    // Same raw pixels, second offset calibration set: expect sub-degree agreement.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, expected_frame_ambient_c, sensor->get_ambient());
    for (std::size_t i = 0; i < num_pixels; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.5f, temps_sub_page_0[i], temps_sub_page_1[i]);
    }
}

void test_calculate_temps_accepts_deployment_emissivity() {
    prime_frame(0);
    assert_status(Status::Success, sensor->read_frame());
    sensor->calculate_temps();
    const auto blackbody_temps = sensor->get_temps();

    sensor->calculate_temps(0.90f);
    const auto tire_temps = sensor->get_temps();

    // The correction must materially respond to the deployment setting.
    TEST_ASSERT_TRUE(std::fabs(tire_temps[191] - blackbody_temps[191]) > 0.01f);
    for (float temperature : tire_temps) {
        TEST_ASSERT_TRUE(std::isfinite(temperature));
    }
}

void test_calculate_temps_holds_last_value_on_a_corrupt_frame() {
    prime_frame(0);
    assert_status(Status::Success, sensor->read_frame());
    sensor->calculate_temps();
    const auto good = sensor->get_temps();

    // Zero the gain word: gain = gainEE / 0 -> inf, which poisons every pixel.
    prime_frame(0);
    bus->registers[static_cast<uint16_t>(0x0580 + (202 - 192))] = 0;
    assert_status(Status::Success, sensor->read_frame());
    sensor->calculate_temps();
    const auto after = sensor->get_temps();

    // Each pixel keeps its previous good value; nothing NaN/inf reaches the average.
    for (std::size_t i = 0; i < num_pixels; i++) {
        TEST_ASSERT_TRUE(std::isfinite(after[i]));
        TEST_ASSERT_EQUAL_FLOAT(good[i], after[i]);
    }
    for (float avg : column_averages(after)) {
        TEST_ASSERT_TRUE(std::isfinite(avg));
    }
}

// ---------------------------------------------------------------- helpers

void test_column_averages_average_each_column_over_all_rows() {
    std::array<float, num_pixels> temps{};
    for (std::size_t row = 0; row < sensor_rows; row++) {
        for (std::size_t col = 0; col < sensor_columns; col++) {
            temps[row * sensor_columns + col] = static_cast<float>(col + row * 100);
        }
    }

    const auto averages = column_averages(temps);

    // Mean of row*100 over rows 0..11 is 550.
    for (std::size_t col = 0; col < sensor_columns; col++) {
        TEST_ASSERT_FLOAT_WITHIN(0.001f, static_cast<float>(col) + 550.0f, averages[col]);
    }
}

void test_hamming_encode_round_trips_through_the_fixture() {
    // Every decoded fixture word, re-encoded, must carry a zero syndrome, or
    // the mock bus would not be a faithful sensor.
    for (std::size_t i = 16; i < test_eeprom_data.size(); i++) {
        const uint16_t word = hamming_encode(test_eeprom_data[i]);
        TEST_ASSERT_EQUAL_HEX16(test_eeprom_data[i], word & 0x07FF);
    }
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_init_succeeds_with_valid_eeprom);
    RUN_TEST(test_init_applies_config_to_bus_and_control_register);
    RUN_TEST(test_init_has_no_hidden_defaults);
    RUN_TEST(test_init_reports_bus_error);
    RUN_TEST(test_init_rejects_a_device_that_is_not_an_mlx90641);
    RUN_TEST(test_init_rejects_uncorrectable_eeprom_corruption);
    RUN_TEST(test_init_rejects_a_sensor_with_a_deviating_pixel);
    RUN_TEST(test_init_tolerates_a_correctable_eeprom_bit_flip);
    RUN_TEST(test_read_frame_follows_the_status_register_handshake);
    RUN_TEST(test_read_frame_accepts_sub_page_1);
    RUN_TEST(test_read_frame_tolerates_status_register_readback_mismatch);
    RUN_TEST(test_read_frame_times_out_when_no_frame_ever_arrives);
    RUN_TEST(test_read_frame_fails_when_new_data_flag_never_clears);
    RUN_TEST(test_read_frame_reports_bus_error);
    RUN_TEST(test_calculate_temps_reproduces_reference_frame);
    RUN_TEST(test_calculate_temps_on_sub_page_1_agrees_with_sub_page_0);
    RUN_TEST(test_calculate_temps_accepts_deployment_emissivity);
    RUN_TEST(test_calculate_temps_holds_last_value_on_a_corrupt_frame);
    RUN_TEST(test_column_averages_average_each_column_over_all_rows);
    RUN_TEST(test_hamming_encode_round_trips_through_the_fixture);
    return UNITY_END();
}
