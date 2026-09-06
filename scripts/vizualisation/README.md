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

Connects over BLE to a device advertising as `MLX90641`, subscribes to notifications carrying 8-column temperature chunks (2 packets per full frame), and displays a live 16-column thermal strip.

```bash
python ble.py
```
