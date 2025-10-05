import asyncio
from bleak import BleakScanner, BleakClient
import struct
import matplotlib.pyplot as plt
import numpy as np
import sys

# BLE device and characteristic
DEVICE_NAME = "MLX90641"
CHAR_UUID = "00000001-0000-1000-8000-00805f9b34fb"

FULL_COLUMNS = 16
full_buffer = [0.0] * FULL_COLUMNS  # 16-column averaged temperatures

# Global flag for window status
window_closed = asyncio.Event()

def update_plot():
    """Update live heatmap plot."""
    plt.clf()
    plt.imshow([full_buffer], cmap="inferno", aspect="auto")
    plt.colorbar(label="°C")
    plt.title("16-column Thermal Strip")
    plt.yticks([])
    plt.xticks(range(FULL_COLUMNS))
    plt.pause(0.01)

def notification_handler(_, data):
    """Handle incoming BLE notifications."""
    global full_buffer

    if len(data) != 19:
        print(f"Unexpected packet length: {len(data)}")
        return

    protocol, packet_id, reserved, *temps = struct.unpack("<BBB8h", data)
    temps = [t / 10.0 for t in temps]

    start_idx = packet_id * 8
    for i, t in enumerate(temps):
        full_buffer[start_idx + i] = t

    update_plot()

def on_close(event):
    """Matplotlib window close callback."""
    print("Window closed — exiting...")
    window_closed.set()  # trigger async shutdown

async def main():
    global window_closed

    print("Scanning for BLE device...")
    devices = await BleakScanner.discover()
    device = next((d for d in devices if d.name == DEVICE_NAME), None)
    if not device:
        print(f"Device '{DEVICE_NAME}' not found")
        return

    async with BleakClient(device) as client:
        print("Connected to", DEVICE_NAME)
        await client.start_notify(CHAR_UUID, notification_handler)

        # Setup live plotting
        plt.ion()
        fig = plt.figure(figsize=(8, 2))
        fig.canvas.mpl_connect("close_event", on_close)

        # Wait until window is closed
        await window_closed.wait()

        print("Stopping BLE notifications and closing...")
        await client.stop_notify(CHAR_UUID)
        plt.close(fig)
        sys.exit(0)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Interrupted — exiting...")
