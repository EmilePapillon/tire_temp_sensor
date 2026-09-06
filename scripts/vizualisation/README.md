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

Pass your device's serial port with `-p`/`--port` (e.g. `/dev/cu.usbserial-XXXXXXXX` on macOS, `COM3` on Windows). If omitted, it falls back to a default port defined in the script.

```bash
python serial_viz.py -p /dev/cu.usbserial-XXXXXXXX
```

## ble.py

Connects over BLE to a device advertising as `RejsaRubber*` and subscribes to the three RejsaRubberTrac NOTIFY characteristics on service `0x1ff7`:

- `0x01` — even columns (0, 2, … 14) + suspension distance
- `0x02` — odd columns (1, 3, … 15) + battery charge/voltage
- `0x03` — per-pair max, the 8-zone strip RaceChrono actually logs

It reconstructs the 16-column strip from `0x01` + `0x02`, shows the 8-zone `0x03` strip below it, a battery gauge (percent / millivolts) on the right, and a connection-status banner:

- `● CONNECTED — streaming` (green): linked and receiving packets
- `● CONNECTED — no data` (amber): linked but no packet for 2 s
- `○ DISCONNECTED` (red): device powered off or out of range

The plot is repainted on its own timer, so if the device drops out the window keeps updating the status and stays closable. Close the window to exit.

```bash
python ble.py
```
