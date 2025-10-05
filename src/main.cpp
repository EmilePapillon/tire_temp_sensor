#include <Arduino.h>
#include "arduino_wire.hh"
#include "mlx90641_driver.hh"
#include "BLE_gatt.h"
#include <bluefruit.h>
#include "data_pack.hh"

// Replace #define with constexpr
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
mlx90641::MLX90641Sensor mlx_sensor(i2c_adapter);
DataPack datapack;

void setup() {
    Serial.begin(115200);
    bool result = mlx_sensor.init();
    if (!result) {
        Serial.println("Failed to initialize MLX90641!");
        while (1) delay(1000);
    }
    Serial.println("MLX90641 ready.");

    delay(5000);
    // START UP BLUETOOTH
    Serial.print("Starting bluetooth with MAC address ");
    Bluefruit.begin();
    Bluefruit.getAddr(macaddr);
    Serial.printBufferReverse(macaddr, 6, ':');
    Serial.println();
    Bluefruit.setName("MLX90641");

    // RUN BLUETOOTH GATT
    setupMainService();
    startAdvertising(); 
    Serial.println("Running!");
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
    const int maxRetries = 5;   
    int retries = 0;
    bool frameSuccess = false;

    while (!frameSuccess && retries < maxRetries) {
        frameSuccess = mlx_sensor.read_frame();
        if (!frameSuccess) {
            retries++;
            delay(1); // short delay before retry
        }
    }

    // If still failed after max retries, skip this iteration entirely
    if (!frameSuccess) {
        Serial.println("Missed frame, all retries failed. Skipping notification.");
        return;
    }

    mlx_sensor.calculate_temps();
    auto tempData = mlx_sensor.get_temps();

    Serial.write((uint8_t*)tempData.data(), tempData.size() * sizeof(float));
    
    // Row 0, pixels [0 .. 15]
    float colAvg[16];
    for (int col = 0; col < 16; col++) {
        float sum = 0.0f;
        for (int row = 0; row < 12; row++) {
            sum += tempData[row * 16 + col];  // row-major order
        }
        colAvg[col] = sum / 12.0f;  // average of this column
    }    

    sendColumnAveragesBLE(colAvg);
    
}
