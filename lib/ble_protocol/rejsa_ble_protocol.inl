// Template definitions for rejsa_ble_protocol.hh. Included by the header; do not include directly.
#pragma once
#include <cstring>

template <typename PeripheralT>
RejsaBleProtocol<PeripheralT>::RejsaBleProtocol(PeripheralT& peripheral, const ble::AdvertisingParams& advertising)
    : peripheral_(peripheral), advertising_(advertising) {
    device_name_[0] = '\0';
}

template <typename PeripheralT>
bool RejsaBleProtocol<PeripheralT>::begin(const DeviceIdentity& identity) {
    build_device_name(identity, device_name_);
    peripheral_.set_device_name(device_name_);

    ble::CharacteristicProps props;
    props.read = true;
    props.notify = true;
    props.fixed_len = packet_size;

    if (!peripheral_.add_service(service_uuid)) {
        return false;
    }
    for (ble::Uuid16 uuid : {char_one_uuid, char_two_uuid, char_thr_uuid}) {
        if (!peripheral_.add_characteristic(uuid, props)) {
            return false;
        }
    }
    return peripheral_.start_advertising(advertising_);
}

template <typename PeripheralT>
bool RejsaBleProtocol<PeripheralT>::is_ready() {
    return peripheral_.is_connected();
}

template <typename PeripheralT>
bool RejsaBleProtocol<PeripheralT>::publish(const TireTelemetry& telemetry) {
    if (!is_ready()) {
        return false;
    }

    DataPackOne one{};
    DataPackTwo two{};
    DataPackThr thr{};
    one.protocol = protocol_version;
    two.protocol = protocol_version;
    thr.protocol = protocol_version;
    one.distance = telemetry.distance_mm;
    thr.distance = telemetry.distance_mm;
    two.charge = telemetry.battery_pct;
    two.voltage = telemetry.battery_mv;

    for (std::size_t i = 0; i < temps_per_packet; i++) {
        const int16_t even_col = to_tenths(telemetry.column_temps_c[i * 2]);
        const int16_t odd_col = to_tenths(telemetry.column_temps_c[i * 2 + 1]);
        one.temps[i] = even_col;
        two.temps[i] = odd_col;
        thr.temps[i] = (even_col > odd_col) ? even_col : odd_col;
    }

    bool ok = true;
    ok &= peripheral_.notify(char_one_uuid, reinterpret_cast<const uint8_t*>(&one), sizeof(one));
    ok &= peripheral_.notify(char_two_uuid, reinterpret_cast<const uint8_t*>(&two), sizeof(two));
    ok &= peripheral_.notify(char_thr_uuid, reinterpret_cast<const uint8_t*>(&thr), sizeof(thr));
    return ok;
}

template <typename PeripheralT>
void RejsaBleProtocol<PeripheralT>::poll() {
    peripheral_.poll();
}

template <typename PeripheralT>
const char* RejsaBleProtocol<PeripheralT>::device_name() const {
    return device_name_;
}

template <typename PeripheralT>
void RejsaBleProtocol<PeripheralT>::build_device_name(const DeviceIdentity& identity,
                                                      char (&out)[device_name_len + 1]) {
    const char* prefix = corner_prefix(identity.corner);
    std::memcpy(out, prefix, 13);
    // mac_address[2..0] rendered as 6 hex digits: [2] first, [0] last.
    for (std::size_t i = 0; i < 3; i++) {
        const uint8_t byte = identity.mac_address[2 - i];
        out[13 + i * 2] = hex_digit(byte >> 4);
        out[14 + i * 2] = hex_digit(byte & 0x0f);
    }
    out[device_name_len] = '\0';
}

template <typename PeripheralT>
int16_t RejsaBleProtocol<PeripheralT>::to_tenths(float celsius) {
    return static_cast<int16_t>(celsius * 10.0f);
}

template <typename PeripheralT>
const char* RejsaBleProtocol<PeripheralT>::corner_prefix(WheelCorner corner) {
    switch (corner) {
        case WheelCorner::FR: return "RejsaRubberFR";
        case WheelCorner::RL: return "RejsaRubberRL";
        case WheelCorner::RR: return "RejsaRubberRR";
        case WheelCorner::FL: break;
    }
    return "RejsaRubberFL";
}

template <typename PeripheralT>
char RejsaBleProtocol<PeripheralT>::hex_digit(uint8_t nibble) {
    return static_cast<char>((nibble < 0xA) ? ('0' + nibble) : ('A' + nibble - 10));
}
