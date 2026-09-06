#pragma once
#include <cstddef>
#include <cstdint>
#include "i_ble_peripheral.hh"
#include "tire_telemetry.hh"

/// @file rejsa_ble_protocol.hh
/// @brief RejsaRubberTrac BLE protocol (natively supported by RaceChrono and Harry's LapTimer).
///
/// Service 0x1ff7 exposes three NOTIFY characteristics, each carrying a fixed
/// 20-byte packet:
///   - 0x01 DataPackOne : even-numbered temp spots + suspension distance
///   - 0x02 DataPackTwo : odd-numbered temp spots + battery state
///   - 0x03 DataPackThr : per-pair max of all 16 spots + suspension distance
///
/// All temps are degrees Celsius x 10. Fields are little-endian (native nRF52).
///
/// RaceChrono keys off the device name to detect the sensor and assign it to a
/// wheel: "RejsaRubber" + corner (FL/FR/RL/RR) + the last three MAC bytes as hex.
///
/// "BleTireProtocol" shape (what main.cpp drives, shared with future protocols):
/// @code
///     static constexpr std::size_t notifications_per_publish;  // radio TX-queue sizing
///     bool begin(const DeviceIdentity& identity);  // name + GATT + advertising
///     bool is_ready();                             // a consumer can receive data
///     bool publish(const TireTelemetry& telemetry);
///     void poll();                                 // per-loop housekeeping
/// @endcode

/// @brief RejsaRubberTrac framing over any BlePeripheral-shaped radio.
///
/// The peripheral must already be started (`PeripheralT::begin()`) before
/// begin(): that is where the MAC address in DeviceIdentity comes from.
/// @tparam PeripheralT A type satisfying ble::is_ble_peripheral; injected by reference so tests can inspect it.
template <typename PeripheralT>
class RejsaBleProtocol {
    static_assert(ble::is_ble_peripheral<PeripheralT>::value,
                  "PeripheralT must implement the BlePeripheral shape (see i_ble_peripheral.hh)");

public:
    static constexpr ble::Uuid16 service_uuid = 0x1ff7;   ///< GATT service RaceChrono scans for.
    static constexpr ble::Uuid16 char_one_uuid = 0x01;    ///< DataPackOne characteristic.
    static constexpr ble::Uuid16 char_two_uuid = 0x02;    ///< DataPackTwo characteristic.
    static constexpr ble::Uuid16 char_thr_uuid = 0x03;    ///< DataPackThr characteristic.
    static constexpr uint8_t protocol_version = 0x02;     ///< Value of every packet's first byte.
    static constexpr std::size_t packet_size = 20;        ///< Wire size of each packet in bytes.
    static constexpr std::size_t device_name_len = 19;    ///< "RejsaRubber" + "FL" + 6 hex digits.
    static constexpr std::size_t temps_per_packet = TireTelemetry::num_columns / 2;  ///< Temp slots per packet.
    static constexpr std::size_t notifications_per_publish = 3;  ///< publish() notifies 0x01, 0x02, 0x03 back to back.

    /// @brief Bind to a radio and remember how to advertise on it.
    /// @param peripheral The radio; must outlive this object.
    /// @param advertising Radio-level advertising settings, applied in begin().
    RejsaBleProtocol(PeripheralT& peripheral, const ble::AdvertisingParams& advertising);

    /// @brief Set the device name, register the GATT layout and start advertising.
    /// @param identity MAC address and wheel corner used to build the device name.
    /// @return False if any service or characteristic registration, or advertising start, failed.
    bool begin(const DeviceIdentity& identity);

    /// @brief Whether a consumer is connected and notifications can be delivered.
    /// @return True while a central is connected.
    bool is_ready();

    /// @brief Push one telemetry sample as three notifications.
    /// @param telemetry Column temperatures, battery state and distance to encode.
    /// @return False if no consumer is connected or any notification was rejected by the stack.
    bool publish(const TireTelemetry& telemetry);

    /// @brief Per-loop housekeeping; forwards to the peripheral. Notifications are fire-and-forget.
    void poll();

    /// @brief The advertised device name, valid after begin().
    /// @return NUL-terminated string owned by this object.
    const char* device_name() const;

    /// @brief Build "RejsaRubber" + corner + last three MAC bytes (most significant first).
    /// @param identity MAC address and corner to encode.
    /// @param out Receives the NUL-terminated name.
    static void build_device_name(const DeviceIdentity& identity, char (&out)[device_name_len + 1]);

private:
    /// @brief Characteristic 0x01: even columns and suspension distance.
    struct DataPackOne {
        uint8_t protocol;                 ///< protocol_version
        uint8_t unused;                   ///< Always 0.
        int16_t distance;                 ///< Suspension travel, mm.
        int16_t temps[temps_per_packet];  ///< Columns 0, 2, ... 14 in tenths of a degree C.
    } __attribute__((packed));

    /// @brief Characteristic 0x02: odd columns and battery state.
    struct DataPackTwo {
        uint8_t protocol;                 ///< protocol_version
        uint8_t charge;                   ///< Battery percent, 0..100.
        uint16_t voltage;                 ///< Battery millivolts.
        int16_t temps[temps_per_packet];  ///< Columns 1, 3, ... 15 in tenths of a degree C.
    } __attribute__((packed));

    /// @brief Characteristic 0x03: per-pair maximum, the 8-zone strip RaceChrono logs.
    struct DataPackThr {
        uint8_t protocol;                 ///< protocol_version
        uint8_t unused;                   ///< Always 0.
        int16_t distance;                 ///< Suspension travel, mm.
        int16_t temps[temps_per_packet];  ///< max(column 2i, column 2i+1) in tenths of a degree C.
    } __attribute__((packed));

    static_assert(sizeof(DataPackOne) == packet_size, "DataPackOne must be 20 bytes on the wire");
    static_assert(sizeof(DataPackTwo) == packet_size, "DataPackTwo must be 20 bytes on the wire");
    static_assert(sizeof(DataPackThr) == packet_size, "DataPackThr must be 20 bytes on the wire");

    /// @brief Convert a temperature to the wire unit.
    /// @param celsius Temperature in degrees Celsius.
    /// @return Tenths of a degree, truncated toward zero.
    static int16_t to_tenths(float celsius);

    /// @brief Device-name prefix for a wheel corner.
    /// @param corner The corner.
    /// @return Static 13-character string such as "RejsaRubberFL".
    static const char* corner_prefix(WheelCorner corner);

    /// @brief Upper-case hexadecimal digit for a nibble.
    /// @param nibble Value 0..15.
    /// @return '0'..'9' or 'A'..'F'.
    static char hex_digit(uint8_t nibble);

    PeripheralT& peripheral_;                 ///< The injected radio.
    ble::AdvertisingParams advertising_;      ///< Applied in begin().
    char device_name_[device_name_len + 1];  ///< Built in begin(); empty before.
};

#include "rejsa_ble_protocol.inl"
