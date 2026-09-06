#pragma once
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

/// @file i_ble_peripheral.hh
/// @brief "BlePeripheral" shape: what a BLE tire protocol needs from the radio.
///
/// Portable: no Bluefruit dependency. Resolved at compile time
/// (`RejsaBleProtocol<PeripheralT>`), never through a vtable. Two implementations:
///   - BluefruitBlePeripheral (include/bluefruit_ble_peripheral.hh), the only
///     file in the repo that touches Bluefruit.
///   - MockBlePeripheral (test/mocks/mock_ble_peripheral.hh), records calls.
///
/// Required members (the bool-returning ones must return exactly bool, so a
/// failed registration can never be silently ignored):
/// @code
///     bool begin();                                       // bring up the radio
///     void set_device_name(const char* name);
///     bool add_service(Uuid16 uuid);
///     bool add_characteristic(Uuid16 uuid, const CharacteristicProps& props);
///         // attaches to the most recently added service
///     bool notify(Uuid16 characteristic, const uint8_t* data, std::size_t len);
///     bool start_advertising(const AdvertisingParams& params);
///         // advertises flags, TX power, the device name and every added service
///     bool is_connected();
///     void poll();                                        // service the radio; no-op if event driven
/// @endcode

/// @brief BLE peripheral abstraction shared by every wire protocol.
namespace ble {

/// @brief A 16-bit Bluetooth SIG-style UUID (e.g. 0x1ff7). All supported protocols use short UUIDs.
using Uuid16 = uint16_t;

/// @brief GATT characteristic properties, mapped by the peripheral onto its stack's flags.
struct CharacteristicProps {
    bool read = false;                    ///< Central may read the current value.
    bool notify = false;                  ///< Peripheral pushes updates without acknowledgement.
    bool indicate = false;                ///< Peripheral pushes updates with acknowledgement.
    bool write = false;                   ///< Central may write with response.
    bool write_without_response = false;  ///< Central may write without response.
    uint16_t fixed_len = 0;               ///< Fixed value length in bytes; 0 = variable length.
};

/// @brief Radio bring-up settings, passed to the concrete peripheral at construction.
struct PeripheralConfig {
    std::size_t notify_burst = 1;  ///< Back-to-back notify() calls per publish cycle; sizes the TX queue.
};

/// @brief Radio-level advertising settings, owned by config.hh.
struct AdvertisingParams {
    int8_t tx_power_dbm;         ///< Transmit power in dBm.
    uint16_t interval_fast;      ///< Fast advertising interval in units of 0.625 ms.
    uint16_t interval_slow;      ///< Slow advertising interval in units of 0.625 ms.
    uint16_t fast_timeout_s;     ///< Seconds of fast advertising before dropping to slow.
    uint16_t timeout_s;          ///< Total advertising timeout in seconds; 0 = advertise forever.
    bool restart_on_disconnect;  ///< Resume advertising when the central disconnects.
};

/// @brief Helpers for the BlePeripheral shape trait.
namespace detail {
/// @brief SFINAE guard that only resolves when @p Expr is exactly bool.
/// @tparam Expr The decltype of a candidate member call.
template <typename Expr>
using returns_bool = std::enable_if_t<std::is_same<Expr, bool>::value>;
}  // namespace detail

/// @brief Trait: does @p T implement the BlePeripheral shape?
/// @tparam T Candidate peripheral type.
template <typename T, typename = void>
struct is_ble_peripheral : std::false_type {};

/// @brief Specialisation selected when every required member exists with the required return type.
/// @tparam T Candidate peripheral type.
template <typename T>
struct is_ble_peripheral<T, std::void_t<
    detail::returns_bool<decltype(std::declval<T&>().begin())>,
    decltype(std::declval<T&>().set_device_name(std::declval<const char*>())),
    detail::returns_bool<decltype(std::declval<T&>().add_service(std::declval<Uuid16>()))>,
    detail::returns_bool<decltype(std::declval<T&>().add_characteristic(
        std::declval<Uuid16>(), std::declval<const CharacteristicProps&>()))>,
    detail::returns_bool<decltype(std::declval<T&>().notify(
        std::declval<Uuid16>(), std::declval<const uint8_t*>(), std::declval<std::size_t>()))>,
    detail::returns_bool<decltype(std::declval<T&>().start_advertising(std::declval<const AdvertisingParams&>()))>,
    detail::returns_bool<decltype(std::declval<T&>().is_connected())>,
    decltype(std::declval<T&>().poll())
>> : std::true_type {};

}  // namespace ble
