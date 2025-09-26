#include <Arduino.h>
#include <Wire.h>
#include "MLX90641_API.h"
#include "BLE_gatt.h"

#include <bluefruit.h>

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
paramsMLX90641 mlxParams;

struct DataPack {
  uint8_t  protocol{};         // version of protocol used
  uint8_t  unused_one{};
  int16_t  unused_two{};       
  int16_t  temps[6]{};         // all even numbered temp spots (degrees Celsius x 10)
} __attribute__((packed)) ;

DataPack datapackOne;
DataPack datapackTwo;
DataPack datapackThr;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    delay(1000);
    Serial.println("const int div = 32; DONT FOGET TO CHANGE THIS IN THE DRIVER");
    Serial.println("Initializing MLX90641...");

    // Read EEPROM data
    if (MLX90641_DumpEE(mlx90641_i2c_addr, eeData) != 0) {
        Serial.println("Failed to read EEPROM data!");
        while (1);
    }

    // Extract calibration parameters
    if (MLX90641_ExtractParameters(eeData, &mlxParams) != 0) {
        Serial.println("Failed to extract calibration parameters!");
        while (1);
    }

    // Optional: Set resolution and refresh rate
    MLX90641_SetResolution(mlx90641_i2c_addr, 0x03);     // 17-bit resolution
    MLX90641_SetRefreshRate(mlx90641_i2c_addr, 0x06);     // 16Hz refresh

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

void printStatus(void) {

  for (uint8_t i=0; i<6; i++) {
    Serial.print(datapackOne.temps[i]);
    Serial.print("\t");
    Serial.print(datapackTwo.temps[i]);
    Serial.print("\t");
    Serial.print(datapackThr.temps[i]);
    Serial.print("\t");
  }

  Serial.println();
}

void loop() {
    const int maxRetries = 5;   
    int retries = 0;
    int status;

    do {
        status = MLX90641_GetFrameData(mlx90641_i2c_addr, frameData);
        if (status != 0) {
            retries++;
            // Serial.printf("Missed frame %d\n", retries);
            delay(1); // short delay before retry
        }
    } while (status != 0 && retries < maxRetries);

    // If still failed after max retries, skip this iteration entirely
    if (status != 0) {
        Serial.println("Missed frame, all retries failed. Skipping notification.");
        return;
    }
    // delay(20);
    // float vdd = MLX90641_GetVdd(frameData, &mlxParams);
    float ta = MLX90641_GetTa(frameData, &mlxParams);

    // For ambient reflection compensation, estimate reflected temperature
    float tr = ta - 8.0f;

    MLX90641_CalculateTo(frameData, &mlxParams, 0.95f, tr, tempData);
    Serial.write((uint8_t*)tempData, sizeof(tempData));
    
    
}
