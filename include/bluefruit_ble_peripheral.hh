#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <bluefruit.h>
#include "i_ble_peripheral.hh"

/// @file bluefruit_ble_peripheral.hh
/// @brief Board glue: the BlePeripheral shape (see i_ble_peripheral.hh) over Adafruit's Bluefruit stack.

/// @brief BLE peripheral over Bluefruit. The only class in the repo that touches Bluefruit.
///
/// Services and characteristics are stored in fixed-capacity arrays because
/// Bluefruit objects register themselves with the SoftDevice on begin() and must
/// not move afterwards.
class BluefruitBlePeripheral {
public:
    static constexpr std::size_t max_services = 2;         ///< Capacity of the service table.
    static constexpr std::size_t max_characteristics = 8;  ///< Capacity of the characteristic table.

    /// @brief Construct the radio glue.
    /// @param config Radio bring-up settings; see ble::PeripheralConfig. Defaults leave the
    /// SoftDevice defaults untouched. The composition root fills it (e.g. notify_burst from the
    /// active protocol's notifications_per_publish).
    explicit BluefruitBlePeripheral(const ble::PeripheralConfig& config = {});

    /// @brief Start the SoftDevice and the Bluefruit stack.
    /// @return False if the stack failed to start.
    bool begin();

    /// @brief Set the GAP device name used in advertisements.
    /// @param name NUL-terminated name; copied by the stack.
    void set_device_name(const char* name);

    /// @brief Register a primary service with the SoftDevice.
    /// @param uuid 16-bit service UUID.
    /// @return False if the service table is full or the SoftDevice rejected it.
    bool add_service(ble::Uuid16 uuid);

    /// @brief Register a characteristic under the most recently added service.
    /// @param uuid 16-bit characteristic UUID.
    /// @param props Properties and fixed length to apply.
    /// @return False if the table is full, no service was added, or the SoftDevice rejected it.
    bool add_characteristic(ble::Uuid16 uuid, const ble::CharacteristicProps& props);

    /// @brief Send a notification on a registered characteristic.
    /// @param characteristic UUID passed to add_characteristic().
    /// @param data Payload bytes.
    /// @param len Payload length.
    /// @return False if the UUID is unknown, nobody is subscribed, or the stack rejected it.
    bool notify(ble::Uuid16 characteristic, const uint8_t* data, std::size_t len);

    /// @brief Configure and start advertising flags, TX power, the device name and every added service.
    /// @param params Radio-level advertising settings.
    /// @return False if advertising could not be started.
    bool start_advertising(const ble::AdvertisingParams& params);

    /// @brief Whether at least one central is connected.
    /// @return True while connected.
    bool is_connected();

    /// @brief No-op: Bluefruit services the SoftDevice from its own FreeRTOS task.
    void poll();

    /// @brief Radio MAC address, little-endian as reported by the SoftDevice. Valid after begin().
    /// @return Six bytes, [0] least significant.
    std::array<uint8_t, 6> mac_address() const;

private:
    /// @brief Look up a registered characteristic.
    /// @param uuid UUID passed to add_characteristic().
    /// @return The characteristic, or nullptr if not registered.
    BLECharacteristic* find_characteristic(ble::Uuid16 uuid);

    ble::PeripheralConfig config_;                                 ///< Radio bring-up settings, applied in begin().
    BLEService services_[max_services];                            ///< Registered services, in order.
    ble::Uuid16 characteristic_uuids_[max_characteristics] = {};   ///< UUID of each registered characteristic.
    BLECharacteristic characteristics_[max_characteristics];       ///< Registered characteristics, in order.
    std::size_t service_count_ = 0;                                ///< Services registered so far.
    std::size_t characteristic_count_ = 0;                         ///< Characteristics registered so far.
};

static_assert(ble::is_ble_peripheral<BluefruitBlePeripheral>::value,
              "BluefruitBlePeripheral must satisfy the BlePeripheral shape");
