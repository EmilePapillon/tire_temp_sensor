/// @file main.cpp
/// @brief Composition root: construct the concrete pieces, wire them together, run.
///
/// Protocol framing, naming and GATT setup live in lib/ble_protocol; board glue
/// in include/. Tunables live in config.hh.
#include <Arduino.h>
#include <cstdio>
#include "arduino_logger.hh"
#include "arduino_wire.hh"
#include "battery.hh"
#include "bluefruit_ble_peripheral.hh"
#include "config.hh"
#include "i2c_adapter.hh"
#include "mlx90641_driver.hh"
#include "rejsa_ble_protocol.hh"
#include "serial_frame_stream.hh"
#include "tire_telemetry.hh"
#include "watchdog.hh"

// Git revision stamped in by scripts/build_info.py (extra_scripts in platformio.ini).
#if __has_include("build_info.hh")
#include "build_info.hh"
#else
/// @brief Fallback revision string when the build stamp was not generated.
#define BUILD_VERSION "unknown"
#endif

namespace {

ArduinoWire wire;                                   ///< The I2C bus.
I2CAdapter<ArduinoWire> i2c_adapter(wire);          ///< Register-level access over the bus.
/// The thermal sensor, logging through Serial.
mlx90641::MLX90641Sensor<I2CAdapter<ArduinoWire>, ArduinoLogger> mlx_sensor(i2c_adapter, config::mlx90641_i2c_addr);
BluefruitBlePeripheral peripheral;                  ///< The BLE radio.
/// The wire protocol selected in config.hh, driving the radio.
config::ActiveBleProtocol ble_protocol(peripheral, config::ble_advertising);
ArduinoLogger logger;                               ///< Logger for main.cpp's own messages.
TireTelemetry telemetry{};                          ///< The sample being assembled for publish().

/// @brief Refresh the battery fields of `telemetry` from the ADC.
///
/// Reads once on the first call, then at most every config::battery_refresh_ms.
/// Unsigned subtraction keeps this correct across the 49-day millis() wraparound.
void refresh_battery() {
    static bool read_once = false;
    static uint32_t last_read_ms = 0;
    const uint32_t now = millis();
    if (read_once && (now - last_read_ms) < config::battery_refresh_ms) {
        return;
    }
    read_once = true;
    last_read_ms = now;
    telemetry.battery_mv = battery_read_millivolts();
    telemetry.battery_pct = battery_lipo_percent(telemetry.battery_mv);
}

/// @brief Read one frame, retrying up to config::frame_read_max_retries times.
/// @return True if a frame was acquired.
bool read_frame_with_retries() {
    for (uint8_t attempt = 1; attempt <= config::frame_read_max_retries; attempt++) {
        watchdog_feed();  // a frame read may legitimately take several seconds to time out
        const mlx90641::Status status = mlx_sensor.read_frame();
        if (status == mlx90641::Status::Success) {
            return true;
        }
        char msg[64];
        snprintf(msg, sizeof(msg), "Frame read failed (%s), retry %u/%u", mlx90641::status_name(status), attempt,
                 config::frame_read_max_retries);
        logger.log(LogLevel::DEBUG, msg);
        delay(1);
    }
    return false;
}

/// @brief Log the reason and stop feeding the watchdog: the board resets itself.
/// @param reason Message logged at ERROR level.
void halt(const char* reason) {
    logger.log(LogLevel::ERROR, reason);
    logger.log(LogLevel::ERROR, "Halting; the watchdog will reset the board.");
    while (true) {
        delay(1000);
    }
}

}  // namespace

/// @brief Arduino entry point: arm the watchdog, bring up sensor, battery and BLE.
void setup() {
    watchdog_begin(config::watchdog_timeout_s);

    Serial.begin(config::serial_baud);
    logger.log(LogLevel::INFO, "Firmware build " BUILD_VERSION);
    logger.log(LogLevel::INFO, "Starting setup...");

    logger.log(LogLevel::INFO, "Initializing MLX90641 sensor...");
    const mlx90641::Status sensor_status = mlx_sensor.init(config::mlx90641_config);
    switch (wire.recovery()) {
        case ArduinoWire::BusRecovery::Recovered:
            logger.log(LogLevel::WARN, "I2C bus was held low at boot; freed it by clocking SCL");
            break;
        case ArduinoWire::BusRecovery::Failed:
            logger.log(LogLevel::ERROR, "I2C bus still held low after recovery; power-cycle the sensor");
            break;
        case ArduinoWire::BusRecovery::NotNeeded:
            break;
    }
    if (sensor_status != mlx90641::Status::Success) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to initialize MLX90641: %s", mlx90641::status_name(sensor_status));
        halt(msg);
    }
    logger.log(LogLevel::INFO, "MLX90641 initialized successfully");

    battery_begin();
    refresh_battery();

    watchdog_feed();
    delay(config::boot_delay_ms);
    watchdog_feed();

    logger.log(LogLevel::INFO, "Starting Bluetooth...");
    if (!peripheral.begin()) {
        halt("Failed to start the BLE radio!");
    }
    const DeviceIdentity identity{peripheral.mac_address(), config::wheel_corner};
    {
        const auto& mac = identity.mac_address;
        char msg[64];
        snprintf(msg, sizeof(msg), "BLE MAC address %02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2],
                 mac[1], mac[0]);
        logger.log(LogLevel::INFO, msg);
    }

    if (!ble_protocol.begin(identity)) {
        halt("Failed to register the BLE GATT services!");
    }
    logger.log(LogLevel::INFO, ble_protocol.device_name());
    logger.log(LogLevel::INFO, "Setup complete - Running!");
}

/// @brief Arduino main loop: one frame in, one telemetry sample out.
void loop() {
    watchdog_feed();

    logger.log(LogLevel::DEBUG, "Attempting to read frame...");
    if (!read_frame_with_retries()) {
        logger.log(LogLevel::ERROR, "Missed frame, all retries failed. Skipping notification.");
        return;
    }

    mlx_sensor.calculate_temps();
    const auto temps = mlx_sensor.get_temps();

    if (config::stream_frames_over_serial) {
        serial_stream_frame(temps);
    }

    telemetry.column_temps_c = mlx90641::column_averages(temps);
    refresh_battery();

    ble_protocol.poll();
    ble_protocol.publish(telemetry);
}
