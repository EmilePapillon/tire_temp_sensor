#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include "i_ble_peripheral.hh"

/// @file mock_ble_peripheral.hh
/// @brief Recording BlePeripheral double for BLE protocol tests.

/// @brief Stores everything a protocol asks for so tests can assert on the
/// resulting GATT layout, device name, advertising settings and notified bytes.
class MockBlePeripheral {
public:
    /// @brief One registered characteristic.
    struct Characteristic {
        ble::Uuid16 service;             ///< Service it was attached to.
        ble::Uuid16 uuid;                ///< Its own UUID.
        ble::CharacteristicProps props;  ///< Properties requested.
    };
    /// @brief One notify() call.
    struct Notification {
        ble::Uuid16 uuid;           ///< Characteristic notified.
        std::vector<uint8_t> data;  ///< Payload bytes.
    };

    // --- scripting -----------------------------------------------------------
    bool connected = false;                 ///< Value returned by is_connected().
    bool notify_result = true;              ///< Value returned by notify().
    bool add_service_result = true;         ///< Value returned by add_service().
    bool add_characteristic_result = true;  ///< Value returned by add_characteristic().
    bool start_advertising_result = true;   ///< Value returned by start_advertising().

    // --- recording -----------------------------------------------------------
    bool begun = false;                             ///< begin() was called.
    std::string device_name;                        ///< Last set_device_name().
    std::vector<ble::Uuid16> services;              ///< Services in registration order.
    std::vector<Characteristic> characteristics;    ///< Characteristics in registration order.
    std::vector<Notification> notifications;        ///< Every notify(), in order.
    bool advertising = false;                       ///< start_advertising() was called.
    ble::AdvertisingParams advertising_params{};    ///< Parameters of the last start_advertising().
    int poll_count = 0;                             ///< Number of poll() calls.

    /// @brief Record the radio start.
    /// @return True.
    bool begin();
    /// @brief Record the name.
    /// @param name NUL-terminated name.
    void set_device_name(const char* name);
    /// @brief Record a service.
    /// @param uuid Service UUID.
    /// @return add_service_result.
    bool add_service(ble::Uuid16 uuid);
    /// @brief Record a characteristic under the last service.
    /// @param uuid Characteristic UUID.
    /// @param props Properties requested.
    /// @return add_characteristic_result.
    bool add_characteristic(ble::Uuid16 uuid, const ble::CharacteristicProps& props);
    /// @brief Record a notification.
    /// @param characteristic Characteristic UUID.
    /// @param data Payload.
    /// @param len Payload length.
    /// @return notify_result.
    bool notify(ble::Uuid16 characteristic, const uint8_t* data, std::size_t len);
    /// @brief Record the advertising parameters.
    /// @param params Parameters.
    /// @return start_advertising_result.
    bool start_advertising(const ble::AdvertisingParams& params);
    /// @brief Scripted connection state.
    /// @return connected.
    bool is_connected();
    /// @brief Count the call.
    void poll();
};

static_assert(ble::is_ble_peripheral<MockBlePeripheral>::value,
              "MockBlePeripheral must satisfy the BlePeripheral shape");

#include "mocks/mock_ble_peripheral.inl"
