# MLX90641 driver

A portable driver for the Melexis MLX90641 16×12 thermal array: EEPROM
calibration extraction, frame acquisition, and object-temperature computation.
Ported from the Melexis reference API, restructured to the project's
compile-time-dispatch and status-code conventions.

Nothing here includes `Arduino.h` or `Wire.h`. The bus and the logger are
template parameters, so the whole driver is exercised on the host against test
doubles, under `test/test_mlx90641_sensor/` and `test/test_mlx90641_eeprom_parser/`.

## Files

| File | Contents |
|---|---|
| `mlx90641_driver.hh` / `.inl` | `MLX90641Sensor<I2CAdapterT, LoggerT>`, the `Status` enum, `column_averages()` |
| `mlx90641_config.hh` | `Mlx90641Config`, and the `Resolution` / `RefreshRate` register enums |
| `mlx90641_params.hh` | `ParamsMLX90641`: the decoded calibration set, Melexis reference naming |
| `mlx90641_eeprom_parser.hh` / `.cc` / `.inl` | `MLX90641EEpromParser`: EEPROM image → `ParamsMLX90641` |
| `mlx90641_eeprom_addr.hh` | `EepromAddr`: the fixed EEPROM field address table |

## Shape dependencies

`MLX90641Sensor<I2CAdapterT, LoggerT>` resolves both parameters at compile time:

- **`I2CAdapterT`** must satisfy `is_i2c_adapter` (see `i2c_adapter.hh`): `init`,
  `read`, `write`, each returning `I2cStatus`. Injected **by reference** so a
  test can script and inspect the bus. Register addresses are 16-bit, words are
  16-bit big-endian, reads are chunked to 32 bytes by the adapter.
- **`LoggerT`** must satisfy `is_logger` (see `logger.hh`). **Owned and
  default-constructed** — nothing asserts on log output; `NullLogger` by default.

## Lifecycle

```
MLX90641Sensor sensor(i2c_adapter, 0x33);
sensor.init(config);          // once
loop:
    sensor.read_frame();      // one sub-page + its ambient temperature
    sensor.calculate_temps(); // -> per-pixel object temperatures
    auto t = sensor.get_temps();      // std::array<float, 192>, row-major, degC
    auto a = sensor.get_ambient();    // die temperature, degC
```

`column_averages(t)` collapses the 12×16 grid to 16 column means (`[0]` =
leftmost), which is what the firmware publishes over BLE.

## `init(const Mlx90641Config&)`

Runs in order, stopping at the first fatal failure:

1. `i2c_.init(i2c_freq_khz)` — bus failure is fatal.
2. **EEPROM dump** — read all 832 words, then Hamming-decode words 16–831 in
   place (the check bits are stripped, leaving 11-bit payloads):
   - clean → continue;
   - single-bit errors corrected → `WARN`, continue;
   - any multi-bit error → `EepromCorrupt` (fatal).
3. **Identify + parse** (`extract_parameters` → `MLX90641EEpromParser`):
   - device-select bit (`ee_data_[10] & 0x0040`) clear → `NotAnMlx90641` (fatal);
   - any pixel with all four per-pixel calibration words zero is a factory-flagged
     deviating pixel → `CalibrationExtractionFailed` (fatal). The driver does
     **not** interpolate deviating pixels; it logs the offending index at
     `ERROR` and refuses to run. A healthy part has none.
4. `set_resolution` / `set_refresh_rate` — a bus failure here is logged at
   `WARN` only; the sensor keeps its power-on defaults and `init` still succeeds.

`Mlx90641Config` carries only `i2c_freq_khz`, `resolution`, `refresh_rate`.
There is no timeout/poll-count field — see the recovery contract below.

## `read_frame()`

Implements the status-register handshake:

1. **Poll** `status_register` until the new-data bit is set. This wait is
   **unbounded** — see the recovery contract.
2. Note the sub-page bit, then acknowledge (write `0x0030`, which self-clears, so
   a write-back mismatch is expected and ignored) and read all 6 pixel banks plus
   the 48-word auxiliary block. Re-read the status register; if new-data is set
   again the frame rolled over mid-read, so retry. After
   `frame_sync_max_attempts` (5) unsuccessful rounds → `FrameSyncFailed`.
3. Read control register 1, stash it and the sub-page at `frame_data_[240]` /
   `[241]`, and set `ambient_` from the PTAT pixels.

**Sub-pages.** Both are complete 12×16 frames; they differ only in which of the
two per-pixel offset calibration sets applies. `read_frame()` takes whichever the
sensor offers and `calculate_temps()` picks the matching offsets, so the output
rate equals the sensor's refresh rate.

Any bus error returns immediately as the corresponding `I2c*` status.

## `calculate_temps()`

Three overloads, all writing `temps_` (read back with `get_temps()`):

| Call | Target emissivity | Reflected temperature |
|---|---|---|
| `calculate_temps()` | EEPROM value (`emissivityEE`) | frame's own die ambient |
| `calculate_temps(e)` | `e` | frame's own die ambient |
| `calculate_temps(e, tr)` | `e` | `tr` (measure separately if the sensor die is not representative of the background) |

The math is the Melexis `CalculateTo` reference in single precision (the
Cortex-M4F has no double FPU).

**Bad-value handling.** A corrupt frame or calibration can make an individual
pixel come out non-finite (e.g. a zero gain word) or wildly out of range. Such a
pixel is **not** written — `temps_` keeps that pixel's previous value — so one
glitch cannot poison `column_averages()` or the publish. The accepted range is
the sensor's −40…300 °C spec limit. Before the first successful frame the held
value is 0.

## Recovery contract

The driver never resets the board and has no internal frame timeout. The
composition root (`src/main.cpp`) feeds the hardware watchdog **only after a
frame has been read and published**, so:

- a transient `read_frame()` failure → the caller skips that loop iteration;
- frames stopping entirely → no watchdog feed → the board resets and re-runs
  `init()` (which also re-runs I²C bus recovery).

This is why the new-data poll in `get_frame_data()` is an unbounded `while`.

## `Status`

`Success` is 0. `I2cNack` / `I2cBusError` / `I2cNoData` / `I2cVerifyMismatch`
wrap `I2cStatus`. `NotAnMlx90641`, `EepromCorrupt`,
`CalibrationExtractionFailed` are fatal `init()` outcomes; `FrameSyncFailed` is a
`read_frame()` outcome. `status_name()` gives a static lower-case string for
logs.

## Geometry and units

16 columns across the tread, 12 rows along it, 192 pixels row-major
(`index = row * 16 + column`). All temperatures are degrees Celsius as `float`.
