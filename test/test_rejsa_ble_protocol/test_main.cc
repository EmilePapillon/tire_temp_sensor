#include <unity.h>
#include <cstdint>
#include <cstring>
#include <vector>
#include "mocks/mock_ble_peripheral.hh"
#include "rejsa_ble_protocol.hh"
#include "tire_telemetry.hh"

using Protocol = RejsaBleProtocol<MockBlePeripheral>;

const ble::AdvertisingParams test_advertising{4, 160, 160, 30, 0, true};
const DeviceIdentity test_identity{{0xEF, 0xCD, 0xAB, 0x12, 0x34, 0x56}, WheelCorner::FL};

MockBlePeripheral* peripheral = nullptr;
Protocol* protocol = nullptr;

void setUp(void) {
    peripheral = new MockBlePeripheral();
    protocol = new Protocol(*peripheral, test_advertising);
}

void tearDown(void) {
    delete protocol;
    delete peripheral;
    protocol = nullptr;
    peripheral = nullptr;
}

int16_t le16(const std::vector<uint8_t>& bytes, std::size_t offset) {
    return static_cast<int16_t>(bytes[offset] | (bytes[offset + 1] << 8));
}

uint16_t le16u(const std::vector<uint8_t>& bytes, std::size_t offset) {
    return static_cast<uint16_t>(bytes[offset] | (bytes[offset + 1] << 8));
}

TireTelemetry sample_telemetry() {
    TireTelemetry t{};
    for (std::size_t i = 0; i < TireTelemetry::num_columns; i++) {
        t.column_temps_c[i] = 20.0f + static_cast<float>(i);  // 20, 21, ... 35
    }
    t.column_temps_c[0] = -5.5f;  // exercise a negative reading
    t.battery_mv = 3850;
    t.battery_pct = 55;
    t.distance_mm = 0;
    return t;
}

void test_begin_registers_rejsa_gatt_layout() {
    protocol->begin(test_identity);

    TEST_ASSERT_EQUAL_size_t(1, peripheral->services.size());
    TEST_ASSERT_EQUAL_HEX16(0x1ff7, peripheral->services[0]);

    TEST_ASSERT_EQUAL_size_t(3, peripheral->characteristics.size());
    for (std::size_t i = 0; i < 3; i++) {
        const auto& chr = peripheral->characteristics[i];
        TEST_ASSERT_EQUAL_HEX16(0x1ff7, chr.service);
        TEST_ASSERT_EQUAL_HEX16(i + 1, chr.uuid);
        TEST_ASSERT_TRUE(chr.props.notify);
        TEST_ASSERT_TRUE(chr.props.read);
        TEST_ASSERT_FALSE(chr.props.write);
        TEST_ASSERT_FALSE(chr.props.write_without_response);
        TEST_ASSERT_FALSE(chr.props.indicate);
        TEST_ASSERT_EQUAL_UINT16(20, chr.props.fixed_len);
    }
}

void test_begin_names_device_from_corner_and_mac() {
    protocol->begin(test_identity);
    TEST_ASSERT_EQUAL_STRING("RejsaRubberFLABCDEF", peripheral->device_name.c_str());
    TEST_ASSERT_EQUAL_STRING("RejsaRubberFLABCDEF", protocol->device_name());
}

void test_device_name_covers_every_corner() {
    char name[Protocol::device_name_len + 1];
    DeviceIdentity identity{{0x0A, 0x00, 0xF0, 0x00, 0x00, 0x00}, WheelCorner::FL};

    identity.corner = WheelCorner::FL;
    Protocol::build_device_name(identity, name);
    TEST_ASSERT_EQUAL_STRING("RejsaRubberFLF0000A", name);
    identity.corner = WheelCorner::FR;
    Protocol::build_device_name(identity, name);
    TEST_ASSERT_EQUAL_STRING("RejsaRubberFRF0000A", name);
    identity.corner = WheelCorner::RL;
    Protocol::build_device_name(identity, name);
    TEST_ASSERT_EQUAL_STRING("RejsaRubberRLF0000A", name);
    identity.corner = WheelCorner::RR;
    Protocol::build_device_name(identity, name);
    TEST_ASSERT_EQUAL_STRING("RejsaRubberRRF0000A", name);
}

void test_begin_starts_advertising_with_configured_params() {
    protocol->begin(test_identity);

    TEST_ASSERT_TRUE(peripheral->advertising);
    TEST_ASSERT_EQUAL_INT8(4, peripheral->advertising_params.tx_power_dbm);
    TEST_ASSERT_EQUAL_UINT16(160, peripheral->advertising_params.interval_fast);
    TEST_ASSERT_EQUAL_UINT16(160, peripheral->advertising_params.interval_slow);
    TEST_ASSERT_EQUAL_UINT16(30, peripheral->advertising_params.fast_timeout_s);
    TEST_ASSERT_EQUAL_UINT16(0, peripheral->advertising_params.timeout_s);
    TEST_ASSERT_TRUE(peripheral->advertising_params.restart_on_disconnect);
}

void test_is_ready_mirrors_connection_state() {
    protocol->begin(test_identity);
    TEST_ASSERT_FALSE(protocol->is_ready());
    peripheral->connected = true;
    TEST_ASSERT_TRUE(protocol->is_ready());
}

void test_publish_without_consumer_sends_nothing() {
    protocol->begin(test_identity);
    peripheral->connected = false;

    TEST_ASSERT_FALSE(protocol->publish(sample_telemetry()));
    TEST_ASSERT_EQUAL_size_t(0, peripheral->notifications.size());
}

void test_publish_frames_three_20_byte_packets() {
    protocol->begin(test_identity);
    peripheral->connected = true;

    TEST_ASSERT_TRUE(protocol->publish(sample_telemetry()));

    TEST_ASSERT_EQUAL_size_t(3, peripheral->notifications.size());
    const auto& one = peripheral->notifications[0];
    const auto& two = peripheral->notifications[1];
    const auto& thr = peripheral->notifications[2];
    TEST_ASSERT_EQUAL_HEX16(0x01, one.uuid);
    TEST_ASSERT_EQUAL_HEX16(0x02, two.uuid);
    TEST_ASSERT_EQUAL_HEX16(0x03, thr.uuid);
    TEST_ASSERT_EQUAL_size_t(20, one.data.size());
    TEST_ASSERT_EQUAL_size_t(20, two.data.size());
    TEST_ASSERT_EQUAL_size_t(20, thr.data.size());

    // DataPackOne: protocol, unused, distance, even columns (x10)
    TEST_ASSERT_EQUAL_HEX8(0x02, one.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, one.data[1]);
    TEST_ASSERT_EQUAL_INT16(0, le16(one.data, 2));
    TEST_ASSERT_EQUAL_INT16(-55, le16(one.data, 4));  // column 0 = -5.5 C
    for (std::size_t i = 1; i < 8; i++) {
        TEST_ASSERT_EQUAL_INT16((20 + 2 * i) * 10, le16(one.data, 4 + i * 2));
    }

    // DataPackTwo: protocol, charge, voltage, odd columns (x10)
    TEST_ASSERT_EQUAL_HEX8(0x02, two.data[0]);
    TEST_ASSERT_EQUAL_UINT8(55, two.data[1]);
    TEST_ASSERT_EQUAL_UINT16(3850, le16u(two.data, 2));
    for (std::size_t i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL_INT16((21 + 2 * i) * 10, le16(two.data, 4 + i * 2));
    }

    // DataPackThr: protocol, unused, distance, per-pair max (odd column wins here)
    TEST_ASSERT_EQUAL_HEX8(0x02, thr.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, thr.data[1]);
    TEST_ASSERT_EQUAL_INT16(0, le16(thr.data, 2));
    for (std::size_t i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL_INT16((21 + 2 * i) * 10, le16(thr.data, 4 + i * 2));
    }
}

void test_publish_pair_max_picks_the_hotter_column() {
    protocol->begin(test_identity);
    peripheral->connected = true;
    TireTelemetry t = sample_telemetry();
    t.column_temps_c[4] = 90.0f;  // even column hotter than its odd neighbour (25)

    TEST_ASSERT_TRUE(protocol->publish(t));

    const auto& thr = peripheral->notifications[2];
    TEST_ASSERT_EQUAL_INT16(900, le16(thr.data, 4 + 2 * 2));
}

void test_publish_reports_notify_failure() {
    protocol->begin(test_identity);
    peripheral->connected = true;
    peripheral->notify_result = false;

    TEST_ASSERT_FALSE(protocol->publish(sample_telemetry()));
}

void test_poll_forwards_to_peripheral() {
    protocol->poll();
    TEST_ASSERT_EQUAL_INT(1, peripheral->poll_count);
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_begin_registers_rejsa_gatt_layout);
    RUN_TEST(test_begin_names_device_from_corner_and_mac);
    RUN_TEST(test_device_name_covers_every_corner);
    RUN_TEST(test_begin_starts_advertising_with_configured_params);
    RUN_TEST(test_is_ready_mirrors_connection_state);
    RUN_TEST(test_publish_without_consumer_sends_nothing);
    RUN_TEST(test_publish_frames_three_20_byte_packets);
    RUN_TEST(test_publish_pair_max_picks_the_hotter_column);
    RUN_TEST(test_publish_reports_notify_failure);
    RUN_TEST(test_poll_forwards_to_peripheral);
    return UNITY_END();
}
