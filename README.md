# Tire Temperature Sensor BLE Firmware

This project is firmware for an Adafruit Feather nRF52832 board that reads tire surface temperatures using an MLX90641 IR sensor and broadcasts the data via Bluetooth Low Energy (BLE). The firmware averages each of the sensor's 16 columns and publishes them over the [RejsaRubberTrac](https://github.com/MagnusThome/RejsaRubberTrac) BLE protocol, which RaceChrono and Harry's LapTimer decode natively. The wire protocol is a swappable, compile-time-selected piece so other protocols (e.g. RaceChrono DIY) can be added alongside it.

## Quick Start

1. **Install [PlatformIO](https://platformio.org/).**
2. **Connect your Adafruit Feather nRF52832 via USB.**
3. **Build and upload the firmware:**
   ```sh
   pio run -e adafruit_feather_nrf52832 --target upload
   ```

## Build stamping

Every firmware build logs the git revision it was built from on boot (`Firmware build v1.2-3-g1387909-dirty`). [`scripts/build_info.py`](scripts/build_info.py) runs before each build and regenerates `include/build_info.hh` from the working tree, so nothing has to be updated by hand; `-dirty` means uncommitted changes were flashed. The header is git-ignored.

## Runtime supervision

The nRF52 hardware watchdog is armed first thing in `setup()` and fed once per `loop()` iteration (and per frame-read attempt, since a read can take seconds to time out). Anything that stalls longer than `config::watchdog_timeout_s`, including a fatal init error, ends in a reset rather than a hung board. The boot log's `Firmware build` line tells you which revision came back up.

## Configuration

Every per-board / per-deployment tunable lives in [`include/config.hh`](include/config.hh): wheel corner, log level, MLX90641 I²C address / bus speed / resolution / refresh rate, battery refresh interval, BLE TX power and advertising intervals, and the active BLE protocol (`config::ActiveBleProtocol`). Modules carry no hidden defaults of their own.

## Project layout

The litmus test for where a file lives: **does it need `Arduino.h` / `Bluefruit.h` / `Wire.h` to compile?**

- **No → `lib/<name>/`.** Portable C++, standard library only. Depends on other `lib/` code by template parameter, never on a concrete Arduino type. This is what the `native` environment builds and `test/` exercises on the host.
- **Yes → `include/` (declarations) + `src/` (definitions).** Board-coupled glue, only exercised by flashing the board.

`src/main.cpp` is the composition root and the one place allowed to know about both sides: it constructs the `include/`-side types and hands them to the `lib/`-side templates.

```
lib/
  logger/         LogLevel, the "Logger" shape, NullLogger
  i2c_adapter/    the "Wire" shape, I2CAdapter<WireT> (register-level I²C)
  mlx90641/       MLX90641Sensor<I2CAdapterT, LoggerT>, EEPROM parser, Mlx90641Config + Status
  ble_protocol/   the "BlePeripheral" shape, TireTelemetry/DeviceIdentity, RejsaBleProtocol<PeripheralT>
  battery/        LiPo voltage -> percent curve
include/          config.hh, ArduinoWire, ArduinoLogger, BluefruitBlePeripheral, battery ADC, watchdog, serial frame stream
src/              main.cpp (composition root) + the include/ definitions
test/             host unit tests: test_*/ per suite, mocks/ and fixtures/ shared by include
scripts/vizualisation/   live dashboards over serial and BLE (see its README)
```

Dispatch is compile-time everywhere: interfaces are documented "shapes" enforced by a `static_assert` on a small SFINAE trait (`is_wire<T>`, `is_logger<T>`, `is_ble_peripheral<T>`), so a mock only has to implement the same member functions, no inheritance. Collaborators are injected by reference only where a test needs to script or inspect them (the Wire and the BLE peripheral); the logger is owned internally.

## Running the tests

```sh
pio test -e native                                                     # C++ unit tests on the host
python -m unittest discover -s scripts/vizualisation -p "test_*.py"    # visualization tooling
```

## Contributing

To contribute to this repository:

1. Open a pull request (PR) targeting the `main` branch.
2. Request a review from a project maintainer.
3. Ensure your changes have unit test coverage.
4. Verify that your changes do not break the build.
5. Make sure all unit tests are passing.

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

