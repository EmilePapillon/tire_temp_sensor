#include <Arduino.h>
#include "arduino_wire.hh"
#include "mlx90641_driver.hh"
#include "BLE_gatt.h"
#include <bluefruit.h>
#include "data_pack.hh"
#include "arduino_logger.hh"
#include <cstdio>

constexpr uint8_t mlx90641_i2c_addr = 0x33; // MLX90641 I2C address
constexpr size_t ee_data_size = 832u;
constexpr size_t frame_data_size = 834u;
constexpr size_t num_pixels = 192u;  // 16x12

constexpr float temp_scaling = 1.00f; // Default = 1.00
constexpr int temp_offset = 0;       // Default = 0 (in tenths of degrees Celsius)

uint8_t macaddr[6];
uint16_t eeData[ee_data_size];
uint16_t frameData[frame_data_size];
float tempData[num_pixels];
char rowBuf[512];
Wire wire; 
I2CAdapter i2c_adapter(wire);
ArduinoLogger logger(Logger::Level::INFO); // Change to DEBUG for more verbosity
mlx90641::MLX90641Sensor mlx_sensor(i2c_adapter, mlx90641_i2c_addr, &logger);
DataPack datapack;


void setup() {
    Serial.begin(115200);
    logger.log(Logger::Level::INFO, "Starting setup...");

    logger.log(Logger::Level::INFO, "Initializing MLX90641 sensor...");
    bool result = mlx_sensor.init();
    if (!result) {
        logger.log(Logger::Level::ERROR, "Failed to initialize MLX90641!");
        while (1) delay(1000);
    }
    logger.log(Logger::Level::INFO, "MLX90641 initialized successfully");

    delay(5000);
    // START UP BLUETOOTH
    logger.log(Logger::Level::INFO, "Starting Bluetooth...");
    Bluefruit.begin();
    Bluefruit.getAddr(macaddr);
    char macMsg[64];
    snprintf(macMsg, sizeof(macMsg), "Starting bluetooth with MAC address %02X:%02X:%02X:%02X:%02X:%02X",
             macaddr[5], macaddr[4], macaddr[3], macaddr[2], macaddr[1], macaddr[0]);
    logger.log(Logger::Level::INFO, macMsg);
    Bluefruit.setName("MLX90641");
    logger.log(Logger::Level::INFO, "Bluetooth initialized");

    // RUN BLUETOOTH GATT
    logger.log(Logger::Level::INFO, "Setting up GATT services...");
    setupMainService();
    startAdvertising();
    logger.log(Logger::Level::INFO, "Setup complete - Running!");
}



void sendColumnAveragesBLE(float* avgColumns16) {
    if (!Bluefruit.connected()) return;

    for (uint8_t packetId = 0; packetId < 2; packetId++) {
        datapack.protocol = 1;
        datapack.packet_id = packetId;
        datapack.reserved = 0;

        // Fill 8 temps for this half
        for (uint8_t i = 0; i < 8; i++) {
            uint8_t col = i + packetId * 8;
            datapack.temps[i] = static_cast<int16_t>(avgColumns16[col] * 10.0f);
        }

        GATTone.notify((uint8_t*)&datapack, sizeof(datapack));
        delay(5); // small delay to avoid BLE congestion
    }
}

void loop() {
    logger.log(Logger::Level::DEBUG, "Starting new loop iteration");

    const int maxRetries = 5;
    int retries = 0;
    bool frameSuccess = false;

    logger.log(Logger::Level::DEBUG, "Attempting to read frame...");
    while (!frameSuccess && retries < maxRetries) {
        frameSuccess = mlx_sensor.read_frame();
        if (!frameSuccess) {
            retries++;
            char msg[48];
            snprintf(msg, sizeof(msg), "Frame read failed, retry %d/%d", retries, maxRetries);
            logger.log(Logger::Level::DEBUG, msg);
            delay(1); // short delay before retry
        }
    }

    // If still failed after max retries, skip this iteration entirely
    if (!frameSuccess) {
        logger.log(Logger::Level::ERROR, "Missed frame, all retries failed. Skipping notification.");
        return;
    }

    logger.log(Logger::Level::DEBUG, "Frame read successful, calculating temperatures...");
    mlx_sensor.calculate_temps();
    logger.log(Logger::Level::DEBUG, "Temperature calculation complete");

    auto tempData = mlx_sensor.get_temps();
    {
        char msg[160];
        int offset = snprintf(msg, sizeof(msg), "Retrieved temperature array: ");
        for (size_t i = 0; i < 10 && offset < (int)sizeof(msg); i++) {
            offset += snprintf(msg + offset, sizeof(msg) - offset, "%.2f, ", tempData[i]);
        }
        logger.log(Logger::Level::DEBUG, msg);
    }

    Serial.write((uint8_t*)tempData.data(), tempData.size() * sizeof(float));

    logger.log(Logger::Level::DEBUG, "Calculating column averages...");
    // Row 0, pixels [0 .. 15]
    float colAvg[16];
    for (int col = 0; col < 16; col++) {
        float sum = 0.0f;
        for (int row = 0; row < 12; row++) {
            sum += tempData[row * 16 + col];  // row-major order
        }
        colAvg[col] = sum / 12.0f;  // average of this column
    }

    logger.log(Logger::Level::DEBUG, "Sending BLE data...");
    sendColumnAveragesBLE(colAvg);
    logger.log(Logger::Level::DEBUG, "Loop iteration complete");
}
