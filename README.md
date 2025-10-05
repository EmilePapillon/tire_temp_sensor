# Tire Temperature Sensor BLE Firmware

This project is firmware for an Adafruit Feather nRF52832 board that reads tire surface temperatures using an MLX90641 IR sensor and broadcasts the data via Bluetooth Low Energy (BLE). The firmware averages sensor readings, packages them, and sends them using custom BLE GATT services.

## Quick Start

1. **Install [PlatformIO](https://platformio.org/).**
2. **Connect your Adafruit Feather nRF52832 via USB.**
3. **Build and upload the firmware:**
   ```sh
   pio run --target upload
   ```

## Contributing

To contribute to this repository:

1. Open a pull request (PR) targeting the `main` branch.
2. Request a review from a project maintainer.
3. Ensure your changes have unit test coverage.
4. Verify that your changes do not break the build.
5. Make sure all unit tests are passing.