# Host unit tests

Run with `pio test -e native`. Everything under `lib/` is portable C++ and is exercised here against test doubles; nothing in `include/` or `src/` is built.

## Layout

```
test/
  test_<suite>/test_main.cc   one Unity binary per folder
  mocks/                      MockWire, MockI2CAdapter, MockBlePeripheral (header-only)
  fixtures/                   EEPROM image + expected parameters, synthetic RAM frame + expected temperatures
```

PlatformIO compiles only `test_*/` folders (plus any `.cc` directly in `test/`, of which there are none), but puts `test/` on every binary's include path. That is why the doubles and fixtures are header-only and included as `"mocks/..."` / `"fixtures/..."`.

## Suites

| Suite | Covers |
|---|---|
| `test_i2c_adapter` | `I2CAdapter<MockWire>`: word assembly, 32-byte chunking, every `I2cStatus` |
| `test_mlx90641_eeprom_parser` | Every calibration parameter against a captured EEPROM image |
| `test_mlx90641_sensor` | `init()` config application and failure paths, the frame handshake on both sub-pages, timeouts, and the temperature math against a reference frame |
| `test_rejsa_ble_protocol` | GATT layout, device naming, advertising, packet framing, failure propagation |
| `test_battery_lipo` | The LiPo voltage-to-percent curve |

## Adding a test

Create `test/test_<name>/test_main.cc` with Unity's `setUp`/`tearDown`/`main`. Reuse the mocks; if a mock needs a new knob, add it as a public field with a `///<` comment. The doubles are covered by the Doxygen check too.

## Regenerating the frame fixture

`fixtures/mlx90641_frame_fixture.hh` holds temperatures captured from the double-precision port of the Melexis math at commit `54eb4ce`. If the synthetic frame or the EEPROM image changes, recapture by running that revision's driver on the new inputs; do not copy numbers from the current single-precision implementation, or the regression test would only prove it agrees with itself.
