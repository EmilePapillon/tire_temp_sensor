#include <Arduino.h>
#include "arduino_wire.hh"
#include "mlx90641_driver.hh"
#include "BLE_gatt.h"
#include <bluefruit.h>
#include "data_pack.hh"

constexpr uint8_t mlx90641_i2c_addr = 0x33; // MLX90641 I2C address

uint8_t macaddr[6]; 
Wire wire; 
I2CAdapter i2c_adapter(wire);
MLX90641Sensor mlx_sensor(i2c_adapter);

BLEDataPacks data;

void setup() {
    Serial.begin(115200);
    wire.begin();

    delay(1000);
    Serial.println("Initializing MLX90641...");

    if (!mlx_sensor.init()) {
        Serial.println("Failed to initialize MLX90641!");
        while (1) { 
            delay(1000); 
        }
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

void printStatus(void) {

  for (uint8_t i=0; i<6; i++) {
    Serial.print(data.one.temps[i]);
    Serial.print("\t");
    Serial.print(data.two.temps[i]);
    Serial.print("\t");
    Serial.print(data.three.temps[i]);
    Serial.print("\t");
  }

  Serial.println();
}

void loop() {
    if (!mlx_sensor.read_frame()) {
        Serial.println("Failed to get frame data");
        return;
    }

    mlx_sensor.calculate_temps();
    auto tempData = mlx_sensor.get_temps();

    for (uint8_t row = 0u; row < 6u; row++) {
        float avg1 = 0.0f;
        float avg2 = 0.0f;
        for (uint8_t col = 0u; col < 16u; col++) {
            avg1 += tempData[(row * 2u) * 16u + col];
            avg2 += tempData[(row * 2u + 1u) * 16u + col];
        }
        data.one.temps[row] = static_cast<int16_t>(avg1 * 10.0f / 16.0f);
        data.two.temps[row] = static_cast<int16_t>(avg2 * 10.0f / 16.0f);
        data.three.temps[row] = static_cast<int16_t>(max(data.one.temps[row], data.two.temps[row]));
    }
    printStatus();
    if (Bluefruit.connected()) {
        GATTone.notify(&data.one, sizeof(data.one));
        GATTtwo.notify(&data.two, sizeof(data.two));
        GATTthr.notify(&data.three, sizeof(data.three));
    }
    delay(1000u);
}
