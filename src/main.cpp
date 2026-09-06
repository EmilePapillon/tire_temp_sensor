// Composition root: construct the concrete pieces, wire them together, run.
// Protocol framing, naming and GATT setup live in lib/ble_protocol; board glue
// in include/. Tunables live in config.hh.
#include <Arduino.h>
#include <cstdio>
#include "arduino_logger.hh"
#include "arduino_wire.hh"
#include "battery.hh"
#include "config.hh"
#include "i2c_adapter.hh"
#include "mlx90641_driver.hh"
#include "tire_telemetry.hh"

// Git revision stamped in by scripts/build_info.py (extra_scripts in platformio.ini).
#if __has_include("build_info.hh")
#include "build_info.hh"
#else
#define BUILD_VERSION "unknown"
#endif

namespace {

ArduinoWire wire;
I2CAdapter<ArduinoWire> i2c_adapter(wire);
mlx90641::MLX90641Sensor<I2CAdapter<ArduinoWire>, ArduinoLogger> mlx_sensor(i2c_adapter, config::mlx90641_i2c_addr);
BluefruitBlePeripheral peripheral;
config::ActiveBleProtocol ble_protocol(peripheral, config::ble_advertising);
ArduinoLogger logger;
TireTelemetry telemetry{};

// Refresh the battery fields of `telemetry` from the ADC. Reads once on the
// first call, then at most every config::battery_refresh_ms.
void refresh_battery() {
    static uint32_t next_read_ms = 0;
    const uint32_t now = millis();
    if (now < next_read_ms) {
        return;
    }
    next_read_ms = now + config::battery_refresh_ms;
    telemetry.battery_mv = battery_read_millivolts();
    telemetry.battery_pct = battery_lipo_percent(telemetry.battery_mv);
}

bool read_frame_with_retries() {
    for (uint8_t attempt = 1; attempt <= config::frame_read_max_retries; attempt++) {
        if (mlx_sensor.read_frame()) {
            return true;
        }
        char msg[48];
        snprintf(msg, sizeof(msg), "Frame read failed, retry %u/%u", attempt, config::frame_read_max_retries);
        logger.log(LogLevel::DEBUG, msg);
        delay(1);
    }
    return false;
}

void halt(const char* reason) {
    logger.log(LogLevel::ERROR, reason);
    while (true) {
        delay(1000);
    }
}

}  // namespace

void setup() {
    Serial.begin(config::serial_baud);
    logger.log(LogLevel::INFO, "Firmware build " BUILD_VERSION);
    logger.log(LogLevel::INFO, "Starting setup...");

    logger.log(LogLevel::INFO, "Initializing MLX90641 sensor...");
    if (!mlx_sensor.init(config::mlx90641_config)) {
        halt("Failed to initialize MLX90641!");
    }
    logger.log(LogLevel::INFO, "MLX90641 initialized successfully");

    battery_begin();
    refresh_battery();

    delay(config::boot_delay_ms);

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

    ble_protocol.begin(identity);
    logger.log(LogLevel::INFO, ble_protocol.device_name());
    logger.log(LogLevel::INFO, "Setup complete - Running!");
}

void loop() {
    logger.log(LogLevel::DEBUG, "Attempting to read frame...");
    if (!read_frame_with_retries()) {
        logger.log(LogLevel::ERROR, "Missed frame, all retries failed. Skipping notification.");
        return;
    }

    mlx_sensor.calculate_temps();
    const auto temps = mlx_sensor.get_temps();

    if (config::stream_frames_over_serial) {
        Serial.write(reinterpret_cast<const uint8_t*>(temps.data()), temps.size() * sizeof(float));
    }

    telemetry.column_temps_c = mlx90641::column_averages(temps);
    refresh_battery();

    ble_protocol.poll();
    ble_protocol.publish(telemetry);
}
