# Visualization tooling

Two live dashboards for the sensor, one over USB serial and one over BLE.

## Setup

```bash
python -m venv venv && source venv/bin/activate
pip install -r requirements.txt
```

## serial_viz.py

Shows the full 12x16 thermal image and a column-averaged strip from the firmware's raw serial frame stream (`config::stream_frames_over_serial`).

Each frame on the wire is the 4-byte magic `AA 55 54 54` followed by 192 little-endian `float32` values in row-major order (see `include/serial_frame_stream.hh`). Text logs share the port; the script synchronises on the magic and drops any frame whose values fall outside a sanity range.

```bash
python serial_viz.py -p /dev/cu.usbserial-XXXXXXXX     # -b to change the baud rate
```

## ble.py

Shows what a RaceChrono-style consumer sees: the reconstructed 16-column strip, the 8-zone pair-max strip, a battery gauge and a connection banner (`● CONNECTED — streaming`, `● CONNECTED — no data` after 2 s of silence, `○ DISCONNECTED`). The window repaints on its own timer, so it stays closable if the device drops out; close it to exit.

The wire protocol is auto-detected from the advertisement: each protocol is required by its own consumer app to advertise a distinct service UUID (RejsaRubberTrac `0x1ff7`, RaceChrono DIY `0x1ff8`). Pass `--protocol` to insist on one and fail if the device doesn't match.

```bash
python ble.py                      # auto-detect
python ble.py --protocol rejsa     # insist on RejsaRubberTrac
python ble.py --scan-timeout 20    # scan longer
```

The code mirrors the firmware's split:

| File | Role |
|---|---|
| `tire_protocols.py` | One class per wire protocol (`matches` / `characteristic_uuids` / `decode`), the shared `TireState`, and the `KNOWN_PROTOCOLS` registry. Pure Python, no BLE stack. Add new protocols here. |
| `dashboard.py` | Protocol-agnostic rendering of a `TireState`. |
| `ble.py` | Scan / connect / subscribe / disconnect lifecycle and the entry point. |

## Tests

The decoders need neither hardware nor a BLE stack:

```bash
python -m unittest discover -p 'test_*.py'
```
