#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include "i_ble_peripheral.hh"

// Recording "BlePeripheral" double for BLE protocol tests. Everything the
// protocol asks for is stored so tests can assert on the resulting GATT layout,
// device name, advertising settings and notified bytes.
class MockBlePeripheral {
public:
    struct Characteristic {
        ble::Uuid16 service;
        ble::Uuid16 uuid;
        ble::CharacteristicProps props;
    };
    struct Notification {
        ble::Uuid16 uuid;
        std::vector<uint8_t> data;
    };

    // --- scripting -----------------------------------------------------------
    bool connected = false;
    bool notify_result = true;
    bool add_service_result = true;
    bool add_characteristic_result = true;
    bool start_advertising_result = true;

    // --- recording -----------------------------------------------------------
    bool begun = false;
    std::string device_name;
    std::vector<ble::Uuid16> services;
    std::vector<Characteristic> characteristics;
    std::vector<Notification> notifications;
    bool advertising = false;
    ble::AdvertisingParams advertising_params{};
    int poll_count = 0;

    bool begin() {
        begun = true;
        return true;
    }

    void set_device_name(const char* name) { device_name = name; }

    bool add_service(ble::Uuid16 uuid) {
        services.push_back(uuid);
        return add_service_result;
    }

    bool add_characteristic(ble::Uuid16 uuid, const ble::CharacteristicProps& props) {
        const ble::Uuid16 service = services.empty() ? 0 : services.back();
        characteristics.push_back({service, uuid, props});
        return add_characteristic_result;
    }

    bool notify(ble::Uuid16 characteristic, const uint8_t* data, std::size_t len) {
        notifications.push_back({characteristic, std::vector<uint8_t>(data, data + len)});
        return notify_result;
    }

    bool start_advertising(const ble::AdvertisingParams& params) {
        advertising = true;
        advertising_params = params;
        return start_advertising_result;
    }

    bool is_connected() { return connected; }

    void poll() { poll_count++; }
};

static_assert(ble::is_ble_peripheral<MockBlePeripheral>::value,
              "MockBlePeripheral must satisfy the BlePeripheral shape");
