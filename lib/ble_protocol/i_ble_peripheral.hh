#pragma once
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

// "BlePeripheral" shape: what a BLE tire protocol needs from the radio.
//
// Portable: no Bluefruit dependency. Resolved at compile time
// (RejsaBleProtocol<PeripheralT>), never through a vtable. Two implementations:
//   - BluefruitBlePeripheral (include/bluefruit_ble_peripheral.hh), the only
//     file in the repo that touches Bluefruit.
//   - MockBlePeripheral (test/mocks/mock_ble_peripheral.hh), records calls.
//
// Required members:
//     bool begin();                                       // bring up the radio
//     void set_device_name(const char* name);
//     void add_service(Uuid16 uuid);
//     void add_characteristic(Uuid16 uuid, const CharacteristicProps& props);
//         // attaches to the most recently added service
//     bool notify(Uuid16 characteristic, const uint8_t* data, std::size_t len);
//     void start_advertising(const AdvertisingParams& params);
//         // advertises flags, TX power, the device name and every added service
//     bool is_connected();
//     void poll();                                        // service the radio; no-op if event driven

namespace ble {

using Uuid16 = uint16_t;

struct CharacteristicProps {
    bool read = false;
    bool notify = false;
    bool indicate = false;
    bool write = false;
    bool write_without_response = false;
    uint16_t fixed_len = 0;  // 0 = variable length
};

struct AdvertisingParams {
    int8_t tx_power_dbm;
    uint16_t interval_fast;   // in units of 0.625 ms
    uint16_t interval_slow;   // in units of 0.625 ms
    uint16_t fast_timeout_s;  // seconds of fast advertising before dropping to slow
    uint16_t timeout_s;       // total advertising timeout, 0 = advertise forever
    bool restart_on_disconnect;
};

template <typename T, typename = void>
struct is_ble_peripheral : std::false_type {};

template <typename T>
struct is_ble_peripheral<T, std::void_t<
    decltype(std::declval<T&>().begin()),
    decltype(std::declval<T&>().set_device_name(std::declval<const char*>())),
    decltype(std::declval<T&>().add_service(std::declval<Uuid16>())),
    decltype(std::declval<T&>().add_characteristic(std::declval<Uuid16>(),
                                                   std::declval<const CharacteristicProps&>())),
    decltype(std::declval<T&>().notify(std::declval<Uuid16>(), std::declval<const uint8_t*>(),
                                       std::declval<std::size_t>())),
    decltype(std::declval<T&>().start_advertising(std::declval<const AdvertisingParams&>())),
    decltype(std::declval<T&>().is_connected()),
    decltype(std::declval<T&>().poll())
>> : std::true_type {};

}  // namespace ble
