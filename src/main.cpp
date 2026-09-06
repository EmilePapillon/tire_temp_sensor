#include <Arduino.h>
#include "arduino_wire.hh"
#include "mlx90641_driver.hh"
#include "BLE_gatt.h"
#include <bluefruit.h>
#include "data_pack.hh"
#include "arduino_logger.hh"
#include "battery.hh"
#include <cstdio>
#include <cstring>

// --- RaceChrono / RejsaRubberTrac wheel position -----------------------------
// Each sensor board must advertise which corner it is on so RaceChrono can
// place it. Set this per board before flashing:
//   0 = FL (front left)   1 = FR (front right)
//   2 = RL (rear left)    3 = RR (rear right)
constexpr uint8_t wheel_pos = 0;

constexpr uint8_t mlx90641_i2c_addr = 0x33; // MLX90641 I2C address
constexpr size_t ee_data_size = 832u;
constexpr size_t frame_data_size = 834u;
constexpr size_t num_pixels = 192u;  // 16x12

constexpr float temp_scaling = 1.00f; // Default = 1.00
constexpr int temp_offset = 0;       // Default = 0 (in tenths of degrees Celsius)

constexpr uint32_t battery_refresh_ms = 60000u; // how often to re-read the LiPo

uint8_t macaddr[6];
char bleName[20];
uint16_t eeData[ee_data_size];
uint16_t frameData[frame_data_size];
float tempData[num_pixels];
char rowBuf[512];
Wire wire;
I2CAdapter i2c_adapter(wire);
ArduinoLogger logger(Logger::Level::INFO); // Change to DEBUG for more verbosity
mlx90641::MLX90641Sensor mlx_sensor(i2c_adapter, mlx90641_i2c_addr, &logger);
DataPackOne datapack_one;
DataPackTwo datapack_two;
DataPackThr datapack_thr;


// Refresh datapack_two's battery fields from the ADC. Reads once on the first
// call, then at most every battery_refresh_ms.
void refreshBattery() {
    static uint32_t next_read_ms = 0;
    uint32_t now = millis();
    if (now < next_read_ms) {
        return;
    }
    next_read_ms = now + battery_refresh_ms;
    datapack_two.voltage = battery_read_millivolts();
    datapack_two.charge = battery_lipo_percent(datapack_two.voltage);
}


// Build the RejsaRubberTrac BLE name: "RejsaRubber" + corner (FL/FR/RL/RR) +
// the last three MAC bytes as hex. RaceChrono keys off this name to detect the
// device and assign it to a wheel.
void buildBLEname() {
    const char* prefix;
    switch (wheel_pos) {
        case 1:  prefix = "RejsaRubberFR"; break;
        case 2:  prefix = "RejsaRubberRL"; break;
        case 3:  prefix = "RejsaRubberRR"; break;
        default: prefix = "RejsaRubberFL"; break;
    }
    strncpy(bleName, prefix, 13);
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t hi = macaddr[i] >> 4;
        uint8_t lo = macaddr[i] & 0x0f;
        bleName[17 - i * 2] = (hi < 0xA) ? (hi + '0') : (hi + 'A' - 10);
        bleName[18 - i * 2] = (lo < 0xA) ? (lo + '0') : (lo + 'A' - 10);
    }
    bleName[19] = '\0';
}


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

    battery_begin();

    delay(5000);
    // START UP BLUETOOTH
    logger.log(Logger::Level::INFO, "Starting Bluetooth...");
    Bluefruit.begin();
    Bluefruit.getAddr(macaddr);
    char macMsg[64];
    snprintf(macMsg, sizeof(macMsg), "Starting bluetooth with MAC address %02X:%02X:%02X:%02X:%02X:%02X",
             macaddr[5], macaddr[4], macaddr[3], macaddr[2], macaddr[1], macaddr[0]);
    logger.log(Logger::Level::INFO, macMsg);
    buildBLEname();
    Bluefruit.setName(bleName);
    logger.log(Logger::Level::INFO, bleName);
    logger.log(Logger::Level::INFO, "Bluetooth initialized");

    // Fixed fields for the RejsaRubberTrac packets. Distance stays zero (no
    // distance sensor on this build); battery is filled in by refreshBattery().
    datapack_one.protocol = rejsa_protocol;
    datapack_one.unused = 0;
    datapack_one.distance = 0;
    datapack_two.protocol = rejsa_protocol;
    datapack_thr.protocol = rejsa_protocol;
    datapack_thr.unused = 0;
    datapack_thr.distance = 0;
    refreshBattery();

    // RUN BLUETOOTH GATT
    logger.log(Logger::Level::INFO, "Setting up GATT services...");
    setupMainService();
    startAdvertising();
    logger.log(Logger::Level::INFO, "Setup complete - Running!");
}



void sendColumnAveragesBLE(float* avgColumns16) {
    if (!Bluefruit.connected()) return;

    // RejsaRubberTrac layout: even columns -> characteristic 0x01, odd columns
    // -> 0x02, per-pair max -> 0x03. 16 columns collapse to 8 values each.
    for (uint8_t i = 0; i < 8; i++) {
        int16_t even_col = static_cast<int16_t>(avgColumns16[i * 2] * 10.0f);
        int16_t odd_col = static_cast<int16_t>(avgColumns16[i * 2 + 1] * 10.0f);
        datapack_one.temps[i] = even_col;
        datapack_two.temps[i] = odd_col;
        datapack_thr.temps[i] = (even_col > odd_col) ? even_col : odd_col;
    }

    GATTone.notify((uint8_t*)&datapack_one, sizeof(datapack_one));
    GATTtwo.notify((uint8_t*)&datapack_two, sizeof(datapack_two));
    GATTthr.notify((uint8_t*)&datapack_thr, sizeof(datapack_thr));
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

    refreshBattery();

    logger.log(Logger::Level::DEBUG, "Sending BLE data...");
    sendColumnAveragesBLE(colAvg);
    logger.log(Logger::Level::DEBUG, "Loop iteration complete");
}
