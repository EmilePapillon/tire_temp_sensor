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
5. Make sure all unit tests are passing.'

## Coding guidelines

### Error reporting from functions

- If a function uses output parameters (passed by reference) and the return value is not used for the computed value, use the return value to return an enum error/status code. The enum value corresponding to 0 is ALWAYS success.
- If a function uses its return value to communicate the computed result, signal errors by throwing exceptions.
- Output parameter definition: a parameter passed by reference (or pointer) that the function writes into to communicate its outcome or data.

Rationale: Prefer enum status codes over plain ints because named values are clearer and type-safe. Avoid booleans since they only convey success/failure and cannot propagate the reason for an error. Enums keep 0 = Success and still communicate the cause.

Examples:

```cpp
// Enum return (uses output parameter), 0 means success
enum class ReadStatus {
  Success = 0,
  EepromNotReady,
  InvalidArgs,
  IoError,
};

ReadStatus read_eeprom(uint8_t* buffer, size_t size) {
  if (!buffer || size == 0) return ReadStatus::InvalidArgs;
  // ...perform read...
  return ReadStatus::Success;
}

void example_call_enum() {
  uint8_t buf[64];
  ReadStatus rc = read_eeprom(buf, sizeof(buf));
  if (rc != ReadStatus::Success) {
    // handle error
  }
}
```

```cpp
// Exception-based (return value is the computed result)
int read_temperature_celsius() {
  // ...read sensor...
  if (/* sensor error */) throw std::runtime_error("sensor read failed");
  return /* computed temperature */;
}

void example_call_exception() {
  try {
    int t = read_temperature_celsius();
    // use t
  } catch (const std::exception& e) {
    // handle error
  }
}
```

Note: An output parameter example signature:
```cpp
void read_eeprom(int& value);               // single value by reference
void read_eeprom(uint8_t* buffer, size_t);  // buffer via pointer + size
```

### Naming conventions

- Variables and functions use snake_case (e.g., tire_temp, read_sensor_data).
- Classes use CamelCase (e.g., TemperatureReader).
- Maximum line length is 120 characters.
- Always use braces for control-flow blocks (if/else/for/while/do). Single-line bodies without braces are not permitted.

Rationale: Consistent naming improves readability and searchability; a 120-column limit fits common editors and code review tools; mandatory braces remove ambiguity and prevent bugs introduced by later edits.

Examples:
```cpp
// Disallowed: missing braces and single-line body
if (ready) do_work();
for (int i = 0; i < 5; i++) do_work();

// Allowed: braces always used; snake_case for variables/functions; CamelCase for classes
class TemperatureReader { /* ... */ };

void do_work() {
  for (int sample_idx = 0; sample_idx < 5; sample_idx++) {
    if (is_ready()) {
      process_sample();
    }
  }
}
```

