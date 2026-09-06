// Inline definitions for mock_ble_peripheral.hh. Included by the header; do not include directly.
#pragma once

inline bool MockBlePeripheral::begin() {
    begun = true;
    return true;
}

inline void MockBlePeripheral::set_device_name(const char* name) { device_name = name; }

inline bool MockBlePeripheral::add_service(ble::Uuid16 uuid) {
    services.push_back(uuid);
    return add_service_result;
}

inline bool MockBlePeripheral::add_characteristic(ble::Uuid16 uuid, const ble::CharacteristicProps& props) {
    const ble::Uuid16 service = services.empty() ? 0 : services.back();
    characteristics.push_back({service, uuid, props});
    return add_characteristic_result;
}

inline bool MockBlePeripheral::notify(ble::Uuid16 characteristic, const uint8_t* data, std::size_t len) {
    notifications.push_back({characteristic, std::vector<uint8_t>(data, data + len)});
    return notify_result;
}

inline bool MockBlePeripheral::start_advertising(const ble::AdvertisingParams& params) {
    advertising = true;
    advertising_params = params;
    return start_advertising_result;
}

inline bool MockBlePeripheral::is_connected() { return connected; }

inline void MockBlePeripheral::poll() { poll_count++; }
