# Thermal Sensor Visualization

Two standalone scripts for visualizing MLX90641 thermal data in real time, one reading over serial (USB) and one reading over BLE.

## Setup

1. Activate the virtual environment:
   ```bash
   source venv/bin/activate
   ```

2. Install dependencies (if not already installed):
   ```bash
   pip install -r requirements.txt
   ```

## serial_viz.py

Reads raw 12x16 float32 frames from a serial connection and displays two live heatmaps side by side: the full 12x16 thermal image and a column-averaged strip.

The firmware shares the port with its text logs, so each frame is prefixed with the 4-byte magic `AA 55 54 54` (`include/serial_frame_stream.hh`) followed by 192 little-endian float32 values in row-major order. The script synchronises on the magic and drops any frame whose values fall outside a sanity range.

Pass your device's serial port with `-p`/`--port` (e.g. `/dev/cu.usbserial-XXXXXXXX` on macOS, `COM3` on Windows). If omitted, it falls back to a default port defined in the script.

```bash
python serial_viz.py -p /dev/cu.usbserial-XXXXXXXX
```

## ble.py

Scans for a sensor speaking any known BLE tire protocol, connects with the matching decoder and renders a live dashboard. The protocol is auto-detected from the advertisement (each protocol is required by its own consumer app to advertise a distinct service UUID); pass `--protocol rejsa` to insist on one and fail if the device doesn't match.

The code is split the same way as the firmware:

- `tire_protocols.py` — one class per wire protocol (`matches` / `characteristic_uuids` / `decode`), a shared `TireState`, and the `KNOWN_PROTOCOLS` registry. Pure Python, no BLE stack; add new protocols here.
- `dashboard.py` — protocol-agnostic rendering of a `TireState`.
- `ble.py` — scan / connect / subscribe / disconnect lifecycle and the entry point.

Currently the only protocol is RejsaRubberTrac (service `0x1ff7`, three 20-byte NOTIFY characteristics):

- `0x01` — even columns (0, 2, … 14) + suspension distance
- `0x02` — odd columns (1, 3, … 15) + battery charge/voltage
- `0x03` — per-pair max, the 8-zone strip RaceChrono actually logs

The dashboard reconstructs the 16-column strip from `0x01` + `0x02`, shows the 8-zone `0x03` strip below it, a battery gauge (percent / millivolts) on the right, and a connection-status banner:

- `● CONNECTED — streaming` (green): linked and receiving packets
- `● CONNECTED — no data` (amber): linked but no packet for 2 s
- `○ DISCONNECTED` (red): device powered off or out of range

The plot is repainted on its own timer, so if the device drops out the window keeps updating the status and stays closable. Close the window to exit.

```bash
python ble.py                      # auto-detect
python ble.py --protocol rejsa     # insist on RejsaRubberTrac
```

## Tests

The protocol decoders have unit tests that need no hardware or BLE stack:

```bash
python -m unittest discover -p 'test_*.py'
```
