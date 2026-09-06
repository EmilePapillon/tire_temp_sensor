#include <unity.h>
#include <cstdint>
#include <vector>
#include "i2c_adapter.hh"
#include "mocks/mock_wire.hh"

constexpr uint8_t device_addr = 0x33;

MockWire* wire = nullptr;
I2CAdapter<MockWire>* adapter = nullptr;

void setUp(void) {
    wire = new MockWire();
    adapter = new I2CAdapter<MockWire>(*wire);
}

void tearDown(void) {
    delete adapter;
    delete wire;
    adapter = nullptr;
    wire = nullptr;
}

void assert_status(I2cStatus expected, I2cStatus actual) {
    TEST_ASSERT_EQUAL_STRING(i2c_status_name(expected), i2c_status_name(actual));
}

void test_init_starts_bus_at_requested_frequency() {
    assert_status(I2cStatus::Success, adapter->init(400));
    TEST_ASSERT_TRUE(wire->begun);
    TEST_ASSERT_EQUAL_UINT32(400000u, wire->clock_hz);
}

void test_read_single_word_selects_register_and_assembles_big_endian() {
    wire->registers[0x800D] = 0x1234;
    uint16_t word = 0;

    assert_status(I2cStatus::Success, adapter->read(device_addr, 0x800D, 1, &word));

    TEST_ASSERT_EQUAL_HEX16(0x1234, word);
    TEST_ASSERT_EQUAL_size_t(1, wire->transactions.size());
    const std::vector<uint8_t> expected_select = {0x80, 0x0D};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_select.data(), wire->transactions[0].data(), 2);
    TEST_ASSERT_EQUAL_size_t(1, wire->request_sizes.size());
    TEST_ASSERT_EQUAL_size_t(2, wire->request_sizes[0]);
}

void test_read_splits_long_reads_into_32_byte_chunks() {
    constexpr std::size_t words = 40;  // 80 bytes -> 32 + 32 + 16
    for (uint16_t i = 0; i < words; i++) {
        wire->registers[static_cast<uint16_t>(0x2400 + i)] = static_cast<uint16_t>(0x0A00 + i);
    }
    uint16_t buffer[words] = {};

    assert_status(I2cStatus::Success, adapter->read(device_addr, 0x2400, words, buffer));

    TEST_ASSERT_EQUAL_size_t(3, wire->request_sizes.size());
    TEST_ASSERT_EQUAL_size_t(32, wire->request_sizes[0]);
    TEST_ASSERT_EQUAL_size_t(32, wire->request_sizes[1]);
    TEST_ASSERT_EQUAL_size_t(16, wire->request_sizes[2]);

    // Each chunk re-selects the register it starts at.
    TEST_ASSERT_EQUAL_size_t(3, wire->transactions.size());
    const std::vector<uint8_t> select0 = {0x24, 0x00};
    const std::vector<uint8_t> select1 = {0x24, 0x10};
    const std::vector<uint8_t> select2 = {0x24, 0x20};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(select0.data(), wire->transactions[0].data(), 2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(select1.data(), wire->transactions[1].data(), 2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(select2.data(), wire->transactions[2].data(), 2);

    for (uint16_t i = 0; i < words; i++) {
        TEST_ASSERT_EQUAL_HEX16(0x0A00 + i, buffer[i]);
    }
}

void test_read_reports_nack() {
    uint16_t word = 0;
    wire->end_transmission_status = 2;  // address NACK
    assert_status(I2cStatus::Nack, adapter->read(device_addr, 0x800D, 1, &word));
    wire->end_transmission_status = 3;  // data NACK
    assert_status(I2cStatus::Nack, adapter->read(device_addr, 0x800D, 1, &word));
}

void test_read_reports_bus_error() {
    uint16_t word = 0;
    wire->end_transmission_status = 1;  // tx buffer overflow
    assert_status(I2cStatus::BusError, adapter->read(device_addr, 0x800D, 1, &word));
    wire->end_transmission_status = 4;  // other error
    assert_status(I2cStatus::BusError, adapter->read(device_addr, 0x800D, 1, &word));
    wire->end_transmission_status = 5;  // timeout
    assert_status(I2cStatus::BusError, adapter->read(device_addr, 0x800D, 1, &word));
}

void test_read_reports_no_data() {
    uint16_t word = 0;
    wire->fail_request = true;
    assert_status(I2cStatus::NoData, adapter->read(device_addr, 0x800D, 1, &word));
}

void test_write_sends_register_and_value_then_verifies() {
    assert_status(I2cStatus::Success, adapter->write(device_addr, 0x800D, 0xBEEF));

    TEST_ASSERT_EQUAL_HEX16(0xBEEF, wire->registers[0x800D]);
    const std::vector<uint8_t> expected = {0x80, 0x0D, 0xBE, 0xEF};
    TEST_ASSERT_EQUAL_size_t(4, wire->transactions[0].size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected.data(), wire->transactions[0].data(), 4);
    // Followed by a read-back of the same register.
    TEST_ASSERT_EQUAL_size_t(2, wire->transactions.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected.data(), wire->transactions[1].data(), 2);
}

void test_write_reports_readback_mismatch() {
    wire->write_mask = 0x00FF;  // device silently drops the high byte
    assert_status(I2cStatus::VerifyMismatch, adapter->write(device_addr, 0x800D, 0xBEEF));
}

void test_write_reports_readback_failure() {
    wire->fail_request = true;
    assert_status(I2cStatus::NoData, adapter->write(device_addr, 0x800D, 0xBEEF));
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_init_starts_bus_at_requested_frequency);
    RUN_TEST(test_read_single_word_selects_register_and_assembles_big_endian);
    RUN_TEST(test_read_splits_long_reads_into_32_byte_chunks);
    RUN_TEST(test_read_reports_nack);
    RUN_TEST(test_read_reports_bus_error);
    RUN_TEST(test_read_reports_no_data);
    RUN_TEST(test_write_sends_register_and_value_then_verifies);
    RUN_TEST(test_write_reports_readback_mismatch);
    RUN_TEST(test_write_reports_readback_failure);
    return UNITY_END();
}
