#pragma once
#include <cstdint>
#include "i_ble_peripheral.hh"
#include "logger.hh"
#include "mlx90641_config.hh"
#include "tire_telemetry.hh"

/// @file config.hh
/// @brief Every tunable a firmware builder adjusts per board / deployment.
///
/// Hardware invariants (sensor geometry, ADC divider ratios) stay with their
/// module. Values are compile-time constants read directly wherever they are
/// needed; nothing here is threaded through constructors.
///
/// This header stays light on purpose (no Arduino / Bluefruit includes) so that
/// board glue such as ArduinoLogger can read a constant without dragging in the
/// BLE stack. The protocol alias below only needs forward declarations; main.cpp
/// includes the full definitions.

template <typename PeripheralT>
class RejsaBleProtocol;
class BluefruitBlePeripheral;

/// @brief Per-board / per-deployment tunables.
namespace config {

// --- Wheel position ----------------------------------------------------------

/// Which corner this board is on. Advertised so RaceChrono can place it; set per board before flashing.
constexpr WheelCorner wheel_corner = WheelCorner::FL;

// --- Logging -----------------------------------------------------------------

/// Lowest log level printed on Serial. DEBUG for per-frame chatter.
constexpr LogLevel log_level = LogLevel::INFO;
/// Serial port baud rate.
constexpr uint32_t serial_baud = 115200;
/// Stream every raw 12x16 float frame over Serial for scripts/vizualisation/serial_viz.py.
/// A bench aid for calibration and physical positioning that bypasses BLE. The car build
/// only uses BLE, and the 772-byte blocking UART write would pace loop() at ~11 Hz, so off.
constexpr bool stream_frames_over_serial = false;

// --- MLX90641 ----------------------------------------------------------------

/// 7-bit I2C address of the sensor.
constexpr uint8_t mlx90641_i2c_addr = 0x33;
/// Sensor frame rate; loop() and the BLE publish rate follow it. Hz8 for the car: a tyre's
/// thermal time constant is seconds, and lower rates cut radio/I2C load and improve NETD.
/// Hz32/Hz64 (paired with a 7.5 ms connection interval in ble_peripheral below) to test
/// end-to-end responsiveness at full throttle.
constexpr mlx90641::RefreshRate mlx90641_refresh_rate = mlx90641::RefreshRate::Hz8;
/// Bus speed, resolution and frame rate programmed at init.
constexpr mlx90641::Mlx90641Config mlx90641_config{
    400,                           // I2C bus frequency, kHz
    mlx90641::Resolution::Bits19,  // ADC resolution
    mlx90641_refresh_rate,         // frame rate
};
/// Use the sensor's EEPROM emissivity unless a deployment-specific value is calibrated.
constexpr bool mlx90641_use_eeprom_emissivity = false;
/// Effective emissivity used when mlx90641_use_eeprom_emissivity is false.
constexpr float mlx90641_emissivity = 0.95f;
// calculate_to() divides by this value; a zero or out-of-range setting yields NaN/inf frames.
static_assert(mlx90641_emissivity > 0.0f && mlx90641_emissivity <= 1.0f,
              "mlx90641_emissivity must be in (0, 1]");

// --- Battery -----------------------------------------------------------------

/// How often the LiPo voltage is re-sampled.
constexpr uint32_t battery_refresh_ms = 60000;

// --- Startup / supervision ---------------------------------------------------

/// Grace period before the radio starts; leaves time to attach a serial monitor.
constexpr uint32_t boot_delay_ms = 5000;
/// Hardware watchdog timeout. loop() feeds it only after a frame has been read
/// and published, so this is also the longest the sensor may stop producing
/// frames before the board resets and re-runs setup(). Must exceed boot_delay_ms.
constexpr uint32_t watchdog_timeout_s = 8;

// --- BLE radio ---------------------------------------------------------------

/// Transmit power and advertising cadence.
constexpr ble::AdvertisingParams ble_advertising{
    4,     // tx_power_dbm
    160,   // interval_fast, 0.625 ms units (100 ms)
    160,   // interval_slow, 0.625 ms units (100 ms)
    30,    // fast_timeout_s
    0,     // timeout_s, 0 = advertise forever
    true,  // restart_on_disconnect
};

/// Connection-level radio settings. notify_burst is overlaid by main.cpp from the active
/// protocol. Interval 0/0 keeps the stack default (20-30 ms preferred, the central decides);
/// 6/12 asks for 7.5-15 ms for full-throttle tests, at a radio-power cost.
constexpr ble::PeripheralConfig ble_peripheral{
    1,  // notify_burst; overlaid from ActiveBleProtocol::notifications_per_publish in main.cpp
    0,  // conn_interval_min, 1.25 ms units; 0 = stack default
    0,  // conn_interval_max, 1.25 ms units; 0 = stack default
};

// --- Active BLE wire protocol ------------------------------------------------

/// The wire protocol this build speaks. Swap it here; nothing else in the firmware changes.
using ActiveBleProtocol = RejsaBleProtocol<BluefruitBlePeripheral>;

}  // namespace config
