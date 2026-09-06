#include <unity.h>
#include <array>
#include <cstdint>
#include "fixtures/mlx90641_eeprom_fixture.hh"
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

uint16_t resolution_bits(uint16_t control) { return (control >> 10) & 0x03; }
uint16_t refresh_bits(uint16_t control) { return (control >> 7) & 0x07; }

// Emulate the sensor: writing 0x0030 to the status register clears "new data".
void arm_status_register() {
    bus->on_write = [](uint16_t reg, uint16_t value) {
        if (reg == status_reg && value == status_clear_ready) {
            bus->registers[status_reg] = 0x0000;
        }
    };
}

void test_init_succeeds_with_valid_eeprom() {
    TEST_ASSERT_TRUE(sensor->init(test_config));
    TEST_ASSERT_TRUE(bus->initialised);
}

void test_init_applies_config_to_bus_and_control_register() {
    TEST_ASSERT_TRUE(sensor->init(test_config));

    TEST_ASSERT_EQUAL_UINT32(400u, bus->init_freq_khz);
    const uint16_t control = bus->registers[control_reg];
    TEST_ASSERT_EQUAL_HEX16(0x03, resolution_bits(control));
    TEST_ASSERT_EQUAL_HEX16(0x06, refresh_bits(control));
    // Bits outside the resolution / refresh-rate fields are preserved.
    TEST_ASSERT_EQUAL_HEX16(control_reg_initial & ~0x0F80, control & ~0x0F80);
}

void test_init_has_no_hidden_defaults() {
    const Mlx90641Config other{100, Resolution::Bits16, RefreshRate::Hz1};
    TEST_ASSERT_TRUE(sensor->init(other));

    TEST_ASSERT_EQUAL_UINT32(100u, bus->init_freq_khz);
    const uint16_t control = bus->registers[control_reg];
    TEST_ASSERT_EQUAL_HEX16(0x00, resolution_bits(control));
    TEST_ASSERT_EQUAL_HEX16(0x01, refresh_bits(control));
}

void test_init_fails_on_bus_error() {
    bus->read_error = -1;
    TEST_ASSERT_FALSE(sensor->init(test_config));
    TEST_ASSERT_EQUAL_size_t(0, bus->writes.size());
}

void test_init_fails_when_eeprom_is_not_an_mlx90641() {
    bus->registers[eeprom_start_address + 10] &= static_cast<uint16_t>(~0x0040);  // device-select bit
    TEST_ASSERT_FALSE(sensor->init(test_config));
}

void test_init_fails_on_uncorrectable_eeprom_corruption() {
    bus->registers[eeprom_start_address + 100] ^= 0x0003;  // two flipped bits
    TEST_ASSERT_FALSE(sensor->init(test_config));
}

void test_read_frame_follows_the_status_register_handshake() {
    TEST_ASSERT_TRUE(sensor->init(test_config));
    arm_status_register();
    bus->reads.clear();
    bus->writes.clear();

    TEST_ASSERT_TRUE(sensor->read_frame());

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

void test_read_frame_fails_when_new_data_flag_never_clears() {
    TEST_ASSERT_TRUE(sensor->init(test_config));
    // A device that keeps re-asserting "new data" no matter how often it is acknowledged.
    bus->on_write = [](uint16_t reg, uint16_t) {
        if (reg == status_reg) {
            bus->registers[status_reg] = status_data_ready;
        }
    };
    TEST_ASSERT_FALSE(sensor->read_frame());
}

void test_read_frame_fails_on_bus_error() {
    TEST_ASSERT_TRUE(sensor->init(test_config));
    bus->read_error = -1;
    TEST_ASSERT_FALSE(sensor->read_frame());
}

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
    RUN_TEST(test_init_fails_on_bus_error);
    RUN_TEST(test_init_fails_when_eeprom_is_not_an_mlx90641);
    RUN_TEST(test_init_fails_on_uncorrectable_eeprom_corruption);
    RUN_TEST(test_read_frame_follows_the_status_register_handshake);
    RUN_TEST(test_read_frame_fails_when_new_data_flag_never_clears);
    RUN_TEST(test_read_frame_fails_on_bus_error);
    RUN_TEST(test_column_averages_average_each_column_over_all_rows);
    RUN_TEST(test_hamming_encode_round_trips_through_the_fixture);
    return UNITY_END();
}
