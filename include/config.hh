#pragma once
#include <cstdint>
#include "i_ble_peripheral.hh"
#include "logger.hh"
#include "mlx90641_config.hh"
#include "tire_telemetry.hh"

// Every tunable a firmware builder adjusts per board / deployment lives here.
// Hardware invariants (sensor geometry, ADC divider ratios) stay with their module.
//
// Values are compile-time constants read directly wherever they are needed;
// nothing here is threaded through constructors.
//
// This header stays light on purpose (no Arduino / Bluefruit includes) so that
// board glue such as ArduinoLogger can read a constant without dragging in the
// BLE stack. The protocol alias below only needs forward declarations; main.cpp
// includes the full definitions.

template <typename PeripheralT>
class RejsaBleProtocol;
class BluefruitBlePeripheral;

namespace config {

// --- Wheel position ----------------------------------------------------------
// Each sensor board advertises which corner it is on so RaceChrono can place
// it. Set per board before flashing.
constexpr WheelCorner wheel_corner = WheelCorner::FL;

// --- Logging -----------------------------------------------------------------
constexpr LogLevel log_level = LogLevel::INFO;  // DEBUG for more verbosity
constexpr uint32_t serial_baud = 115200;
// Stream every raw 12x16 float frame over serial for scripts/vizualisation/serial_viz.py.
constexpr bool stream_frames_over_serial = true;

// --- MLX90641 ----------------------------------------------------------------
constexpr uint8_t mlx90641_i2c_addr = 0x33;
constexpr mlx90641::Mlx90641Config mlx90641_config{
    400,                           // I2C bus frequency, kHz
    mlx90641::Resolution::Bits19,  // ADC resolution
    mlx90641::RefreshRate::Hz32,   // frame rate
    50000,                         // status-register polls before a frame read gives up (~5 s at 400 kHz)
};
constexpr uint8_t frame_read_max_retries = 5;

// --- Battery -----------------------------------------------------------------
constexpr uint32_t battery_refresh_ms = 60000;  // how often to re-read the LiPo

// --- Startup / supervision ---------------------------------------------------
// Grace period before the radio starts, leaves time to attach a serial monitor.
constexpr uint32_t boot_delay_ms = 5000;
// Hardware watchdog: the board resets if loop() stalls for longer than this.
// Must exceed boot_delay_ms and one full frame read (see data_ready_max_polls).
constexpr uint32_t watchdog_timeout_s = 8;

// --- BLE radio ---------------------------------------------------------------
constexpr ble::AdvertisingParams ble_advertising{
    4,     // tx_power_dbm
    160,   // interval_fast, 0.625 ms units (100 ms)
    160,   // interval_slow, 0.625 ms units (100 ms)
    30,    // fast_timeout_s
    0,     // timeout_s, 0 = advertise forever
    true,  // restart_on_disconnect
};

// --- Active BLE wire protocol ------------------------------------------------
// Swap the protocol here; nothing else in the firmware changes.
using ActiveBleProtocol = RejsaBleProtocol<BluefruitBlePeripheral>;

}  // namespace config
