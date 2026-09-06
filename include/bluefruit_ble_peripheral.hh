#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <bluefruit.h>
#include "i_ble_peripheral.hh"

/// @brief Board glue: the "BlePeripheral" shape (see i_ble_peripheral.hh) over
/// Adafruit's Bluefruit stack. The only file in the repo that touches Bluefruit.
///
/// Services and characteristics are stored in fixed-capacity arrays because
/// Bluefruit objects register themselves with the SoftDevice on begin() and must
/// not move afterwards.
class BluefruitBlePeripheral {
public:
    static constexpr std::size_t max_services = 2;
    static constexpr std::size_t max_characteristics = 8;

    bool begin();
    void set_device_name(const char* name);
    /// @brief False if the service table is full or the SoftDevice rejected it.
    bool add_service(ble::Uuid16 uuid);
    /// @brief False if the characteristic table is full, no service was added, or the SoftDevice rejected it.
    bool add_characteristic(ble::Uuid16 uuid, const ble::CharacteristicProps& props);
    bool notify(ble::Uuid16 characteristic, const uint8_t* data, std::size_t len);
    bool start_advertising(const ble::AdvertisingParams& params);
    bool is_connected();
    void poll();

    /// @brief Radio MAC address, little-endian as reported by the SoftDevice. Valid after begin().
    std::array<uint8_t, 6> mac_address() const;

private:
    BLECharacteristic* find_characteristic(ble::Uuid16 uuid);

    BLEService services_[max_services];
    ble::Uuid16 characteristic_uuids_[max_characteristics] = {};
    BLECharacteristic characteristics_[max_characteristics];
    std::size_t service_count_ = 0;
    std::size_t characteristic_count_ = 0;
};

static_assert(ble::is_ble_peripheral<BluefruitBlePeripheral>::value,
              "BluefruitBlePeripheral must satisfy the BlePeripheral shape");
