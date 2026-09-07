# Tire Temperature Sensor BLE Firmware

Firmware for an [Adafruit Feather nRF52832](https://www.adafruit.com/product/3406) that reads tire surface temperatures with a [Melexis MLX90641](https://www.melexis.com/en/product/MLX90641/) 16x12 thermal array and publishes them over Bluetooth Low Energy. It speaks the [RejsaRubberTrac](https://github.com/MagnusThome/RejsaRubberTrac) protocol, which RaceChrono and Harry's LapTimer decode natively, and is structured so another wire protocol (e.g. RaceChrono DIY) can be added without touching anything else.

## Contents

- [Hardware](#hardware)
- [Quick start](#quick-start)
- [Configuration](#configuration)
- [What the firmware does](#what-the-firmware-does)
- [Project layout](#project-layout)
- [Architecture rules](#architecture-rules)
- [Testing and documentation checks](#testing-and-documentation-checks)
- [Visualization tooling](#visualization-tooling)
- [Contributing](#contributing)
- [Coding guidelines](#coding-guidelines)

## Hardware

| Part | Connection |
|---|---|
| Adafruit Feather nRF52832 | USB for flashing and the serial log |
| MLX90641 thermal array | I²C, address `0x33`, 400 kHz |
| Single-cell LiPo | Feather JST connector; voltage sensed on `A7` through the on-board divider |

One board per wheel. Each board is flashed with its corner set in `config.hh` so RaceChrono can place it.

## Quick start

1. Install [PlatformIO](https://platformio.org/).
2. Set the wheel corner (and anything else you need) in [`include/config.hh`](include/config.hh).
3. Connect the Feather over USB and run:

```sh
pio run -e adafruit_feather_nrf52832 --target upload   # build and flash
pio device monitor                                     # 115200 baud serial log
```

The first log line is `Firmware build <git describe>`, e.g. `v1.2-3-g1387909-dirty`; `-dirty` means uncommitted changes were flashed. The value comes from the git-ignored `include/build_info.hh`, regenerated before every build by `scripts/build_info.py`. That script is a plain CLI with no PlatformIO dependency (`python scripts/build_info.py -o <path>`); PlatformIO merely invokes it through `extra_scripts`, so another build system can call it the same way.

## Configuration

Every per-board / per-deployment tunable lives in [`include/config.hh`](include/config.hh). Modules carry no hidden defaults of their own.

| Setting | Default | Notes |
|---|---|---|
| `wheel_corner` | `FL` | Advertised in the device name; set per board |
| `log_level` | `INFO` | `DEBUG` prints per-frame chatter |
| `stream_frames_over_serial` | `false` | Raw frames for `serial_viz.py` (bench aid for calibration and positioning); its blocking UART write paces `loop()` at ~11 Hz, so off for the car |
| `mlx90641_i2c_addr` | `0x33` | |
| `mlx90641_refresh_rate` | `Hz8` | Sensor frame rate; `loop()` and the BLE publish rate follow it. `Hz32`/`Hz64` for full-throttle tests |
| `mlx90641_config` | 400 kHz, 19-bit, `mlx90641_refresh_rate` | Bus speed, ADC resolution, frame rate |
| `mlx90641_use_eeprom_emissivity` | `false` | `true` uses the sensor-stored emissivity; `false` uses the `mlx90641_emissivity` override below |
| `mlx90641_emissivity` | `0.95` | Effective tire-surface emissivity when the EEPROM value is overridden; calibrate this for the tire/finish |
| `battery_refresh_ms` | 60 000 | How often the LiPo is sampled |
| `boot_delay_ms` | 5 000 | Grace period to attach a monitor before the radio starts |
| `watchdog_timeout_s` | 8 | Longest the sensor may stop producing frames before the board resets; must exceed the boot delay |
| `ble_advertising` | +4 dBm, 100 ms | TX power, advertising intervals and timeouts |
| `ble_peripheral` | interval 0/0 | Connection-level radio settings. `notify_burst` is overlaid from the active protocol; connection interval in 1.25 ms units, 0 = stack default, `6`/`12` (7.5–15 ms) for full-throttle tests |
| `ActiveBleProtocol` | `RejsaBleProtocol<BluefruitBlePeripheral>` | The wire protocol this build speaks |

## What the firmware does

**Boot.** `setup()` arms the hardware watchdog, initialises the MLX90641 (EEPROM dump, Hamming check, calibration extraction, resolution and refresh rate), samples the battery, waits `boot_delay_ms`, starts the radio, and registers the BLE protocol. Any fatal error is logged and the board deliberately lets the watchdog reset it.

**Loop.** Each iteration reads one frame, computes per-pixel temperatures, averages the 16 columns, refreshes the battery reading when due, and publishes the sample. A transient frame-read failure just skips the iteration; if frames stop for `watchdog_timeout_s` the board resets and re-runs boot. Both MLX90641 sub-pages are accepted, so the BLE update rate equals the sensor's refresh rate.

**BLE protocol (RejsaRubberTrac).** Service `0x1ff7` with three 20-byte NOTIFY characteristics, all temperatures in tenths of a degree, little-endian:

| Characteristic | Contents |
|---|---|
| `0x01` | protocol byte, unused, distance (mm), even columns 0, 2, … 14 |
| `0x02` | protocol byte, battery %, battery mV, odd columns 1, 3, … 15 |
| `0x03` | protocol byte, unused, distance (mm), max of each column pair (the 8 zones RaceChrono logs) |

The device name is `RejsaRubber` + corner + the last three MAC bytes in hex, e.g. `RejsaRubberFLABCDEF`. Full details in [`lib/ble_protocol/rejsa_ble_protocol.hh`](lib/ble_protocol/rejsa_ble_protocol.hh).

**Serial frame stream.** When enabled, every frame is also written to the USB serial port as the 4-byte magic `AA 55 54 54` followed by 192 little-endian `float32` values in row-major order. Text logs share the port; the magic is how `scripts/visualization/serial_viz.py` finds frame boundaries.

**Supervision.** The nRF52 watchdog is fed only after a frame has been read and published. A wedged sensor, a stuck bus or a fatal init error all end in a reset rather than a hung board; the `Firmware build` log line tells you which revision came back up. Before the bus starts, `ArduinoWire::begin()` checks for a slave holding SDA low (left over from a reset mid-transfer) and frees it by clocking SCL; the outcome is logged at boot.

## Project layout

```
include/config.hh          every tunable, and the active BLE protocol alias
lib/                       portable C++, built and tested on the host
  logger/                  LogLevel, the Logger shape, NullLogger
  i2c_adapter/             the Wire shape, I2CAdapter<WireT>, I2cStatus
  mlx90641/                MLX90641Sensor<I2CAdapterT, LoggerT>, EEPROM parser, Mlx90641Config, Status; see its README
  ble_protocol/            the BlePeripheral shape, TireTelemetry, RejsaBleProtocol<PeripheralT>
  battery/                 LiPo voltage-to-percent curve
include/ + src/            board glue: ArduinoWire, ArduinoLogger, BluefruitBlePeripheral,
                           battery ADC, watchdog, serial frame stream, and main.cpp
test/                      host unit tests, see test/README.md
scripts/build_info.py      git revision stamp; standalone CLI, hooked into PlatformIO
scripts/visualization/     live dashboards over serial and BLE, see its README
docs/Doxyfile              API documentation build and coverage check
```

## Architecture rules

**Where a file lives.** Does it need `Arduino.h`, `Bluefruit.h` or `Wire.h` to compile? No: it goes in `lib/<name>/` and depends on other code only through template parameters. Yes: declarations in `include/`, definitions in `src/`. [`src/main.cpp`](src/main.cpp) is the composition root and the one place allowed to know about both sides.

**Declarations in headers, definitions elsewhere.** A `.hh` holds declarations and documentation only. Non-template definitions go in a `.cc`; template and inline definitions go in a sibling `.inl` that the header includes as its last line. Readers get the interface without the implementation; the compiler still sees everything it needs.

**Compile-time dispatch, no vtables.** Interfaces are documented "shapes" (`Wire`, `Logger`, `I2CAdapter`, `BlePeripheral`) enforced by a small `std::void_t` trait plus a `static_assert` at the point of use, so a mismatch is reported at the instantiation site. A mock only has to implement the same member functions. Nothing is chosen at run time; the protocol is selected by a type alias in `config.hh`.

**Inject only where a test needs control.** The bus and the BLE peripheral are injected by reference because tests script and inspect them. The logger is owned and default-constructed because nothing asserts on log output.

**Status codes are enums.** Functions that report an outcome return `I2cStatus`, `mlx90641::Status` or `bool`, never a bare `int`. Zero is always success.

## Testing and documentation checks

```sh
pio test -e native                                                    # C++ unit tests on the host
python -m unittest discover -s scripts -p "test_*.py"                 # build stamp script
python -m unittest discover -s scripts/visualization -p "test_*.py"   # visualization tooling
doxygen docs/Doxyfile                                                 # API docs -> docs/api/html; fails on any undocumented item
```

CI (`.github/workflows/build.yaml`) runs the firmware build and all of these on every push and pull request. The Doxygen run treats warnings as errors, so every function, parameter, template parameter and return value must be documented.

The API reference is published to GitHub Pages from `main` by `.github/workflows/docs.yaml` on every push, so the published docs always match `main`: https://emilepapillon.github.io/tire_temp_sensor/ (one-time setup: repository Settings > Pages > Source = "GitHub Actions").

## Visualization tooling

`scripts/visualization/` (see its own README) contains `serial_viz.py` (full 12x16 heatmap from the serial frame stream) and `ble.py` (what a RaceChrono-style consumer sees, auto-detecting the protocol from the advertisement). The BLE decoders mirror the firmware's protocol split and have their own unit tests.

## Contributing

1. Open a pull request targeting `main`.
2. Request a review from a project maintainer.
3. Cover your change with unit tests and document new functions.
4. Make sure the build, the tests and the Doxygen check pass.

## Coding guidelines

### Error reporting from functions

- If a function uses output parameters (passed by reference) and the return value is not used for the computed value, use the return value to return an enum error/status code. The enum value corresponding to 0 is ALWAYS success.
- If a function uses its return value to communicate the computed result, signal errors by throwing exceptions.
- Output parameter definition: a parameter passed by reference (or pointer) that the function writes into to communicate its outcome or data.

Rationale: Prefer enum status codes over plain ints because named values are clearer and type-safe. Avoid booleans since they only convey success/failure and cannot propagate the reason for an error. Enums keep 0 = Success and still communicate the cause.

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
```

```cpp
// Exception-based (return value is the computed result)
int read_temperature_celsius() {
  // ...read sensor...
  if (/* sensor error */) throw std::runtime_error("sensor read failed");
  return /* computed temperature */;
}
```

### Headers and documentation

- Headers (`.hh`) contain declarations and Doxygen comments only. Template and inline definitions go in a sibling `.inl` included at the end of the header; other definitions go in a `.cc`.
- Every function declaration has a `@brief`, one `@param` per parameter, one `@tparam` per template parameter, and a `@return` unless it returns `void`. Data members and enumerators get a trailing `///<`.
- `doxygen docs/Doxyfile` must pass with no warnings.

### Naming conventions

- Variables and functions use snake_case (e.g., tire_temp, read_sensor_data).
- Classes use CamelCase (e.g., TemperatureReader).
- Maximum line length is 120 characters.
- Always use braces for control-flow blocks (if/else/for/while/do). Single-line bodies without braces are not permitted.

Rationale: Consistent naming improves readability and searchability; a 120-column limit fits common editors and code review tools; mandatory braces remove ambiguity and prevent bugs introduced by later edits.

```cpp
// Disallowed: missing braces and single-line body
if (ready) do_work();

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
