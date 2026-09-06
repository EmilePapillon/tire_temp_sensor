#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "i_ble_peripheral.hh"
#include "tire_telemetry.hh"

// RejsaRubberTrac BLE protocol (natively supported by RaceChrono and Harry's
// LapTimer).
//
// Service 0x1ff7 exposes three NOTIFY characteristics, each carrying a fixed
// 20-byte packet:
//   0x01 -> DataPackOne : even-numbered temp spots + suspension distance
//   0x02 -> DataPackTwo : odd-numbered temp spots + battery state
//   0x03 -> DataPackThr : per-pair max of all 16 spots + suspension distance
// All temps are degrees Celsius x 10. Fields are little-endian (native nRF52).
//
// RaceChrono keys off the device name to detect the sensor and assign it to a
// wheel: "RejsaRubber" + corner (FL/FR/RL/RR) + the last three MAC bytes as hex.
//
// "BleTireProtocol" shape (what main.cpp drives, shared with future protocols):
//     void begin(const DeviceIdentity& identity);  // name + GATT + advertising
//     bool is_ready();                             // a consumer can receive data
//     bool publish(const TireTelemetry& telemetry);
//     void poll();                                 // per-loop housekeeping
//
// The peripheral must already be started (PeripheralT::begin()) before
// begin(): that is where the MAC address in DeviceIdentity comes from.

template <typename PeripheralT>
class RejsaBleProtocol {
    static_assert(ble::is_ble_peripheral<PeripheralT>::value,
                  "PeripheralT must implement the BlePeripheral shape (see i_ble_peripheral.hh)");

public:
    static constexpr ble::Uuid16 service_uuid = 0x1ff7;
    static constexpr ble::Uuid16 char_one_uuid = 0x01;
    static constexpr ble::Uuid16 char_two_uuid = 0x02;
    static constexpr ble::Uuid16 char_thr_uuid = 0x03;
    static constexpr uint8_t protocol_version = 0x02;
    static constexpr std::size_t packet_size = 20;
    static constexpr std::size_t device_name_len = 19;  // "RejsaRubber" + "FL" + 6 hex digits
    static constexpr std::size_t temps_per_packet = TireTelemetry::num_columns / 2;

    RejsaBleProtocol(PeripheralT& peripheral, const ble::AdvertisingParams& advertising)
        : peripheral_(peripheral), advertising_(advertising) {
        device_name_[0] = '\0';
    }

    void begin(const DeviceIdentity& identity) {
        build_device_name(identity, device_name_);
        peripheral_.set_device_name(device_name_);

        ble::CharacteristicProps props;
        props.read = true;
        props.notify = true;
        props.fixed_len = packet_size;

        peripheral_.add_service(service_uuid);
        peripheral_.add_characteristic(char_one_uuid, props);
        peripheral_.add_characteristic(char_two_uuid, props);
        peripheral_.add_characteristic(char_thr_uuid, props);

        peripheral_.start_advertising(advertising_);
    }

    bool is_ready() {
        return peripheral_.is_connected();
    }

    /// @brief Push one telemetry sample. Returns false if no consumer is connected.
    bool publish(const TireTelemetry& telemetry) {
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

    /// @brief Nothing to do: notifications are fire-and-forget.
    void poll() {
        peripheral_.poll();
    }

    const char* device_name() const {
        return device_name_;
    }

    /// @brief "RejsaRubber" + corner + last three MAC bytes (most significant first).
    static void build_device_name(const DeviceIdentity& identity, char (&out)[device_name_len + 1]) {
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

private:
    struct DataPackOne {
        uint8_t protocol;
        uint8_t unused;
        int16_t distance;
        int16_t temps[temps_per_packet];
    } __attribute__((packed));

    struct DataPackTwo {
        uint8_t protocol;
        uint8_t charge;
        uint16_t voltage;
        int16_t temps[temps_per_packet];
    } __attribute__((packed));

    struct DataPackThr {
        uint8_t protocol;
        uint8_t unused;
        int16_t distance;
        int16_t temps[temps_per_packet];
    } __attribute__((packed));

    static_assert(sizeof(DataPackOne) == packet_size, "DataPackOne must be 20 bytes on the wire");
    static_assert(sizeof(DataPackTwo) == packet_size, "DataPackTwo must be 20 bytes on the wire");
    static_assert(sizeof(DataPackThr) == packet_size, "DataPackThr must be 20 bytes on the wire");

    static int16_t to_tenths(float celsius) {
        return static_cast<int16_t>(celsius * 10.0f);
    }

    static const char* corner_prefix(WheelCorner corner) {
        switch (corner) {
            case WheelCorner::FR: return "RejsaRubberFR";
            case WheelCorner::RL: return "RejsaRubberRL";
            case WheelCorner::RR: return "RejsaRubberRR";
            case WheelCorner::FL: break;
        }
        return "RejsaRubberFL";
    }

    static char hex_digit(uint8_t nibble) {
        return static_cast<char>((nibble < 0xA) ? ('0' + nibble) : ('A' + nibble - 10));
    }

    PeripheralT& peripheral_;
    ble::AdvertisingParams advertising_;
    char device_name_[device_name_len + 1];
};
