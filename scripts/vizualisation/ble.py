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
VMIN, VMAX = 20, 40  # fixed color scale, degrees Celsius

# Global flag for window status
window_closed = asyncio.Event()

# Plot objects, created once in main() and updated in place thereafter.
fig = None
im = None

def update_plot():
    """Update live heatmap plot in place (no new figures, no window churn)."""
    if window_closed.is_set() or not plt.fignum_exists(fig.number):
        return
    im.set_data([full_buffer])
    fig.canvas.draw_idle()
    fig.canvas.flush_events()

def notification_handler(_, data):
    """Handle incoming BLE notifications."""
    global full_buffer

    if window_closed.is_set():
        return

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
    global window_closed, fig, im

    print("Scanning for BLE device...")
    devices = await BleakScanner.discover()
    device = next((d for d in devices if d.name == DEVICE_NAME), None)
    if not device:
        print(f"Device '{DEVICE_NAME}' not found")
        return

    async with BleakClient(device) as client:
        print("Connected to", DEVICE_NAME)
        await client.start_notify(CHAR_UUID, notification_handler)

        # Setup live plotting (created once; notification_handler only mutates it)
        plt.ion()
        fig = plt.figure(figsize=(8, 2))
        fig.canvas.mpl_connect("close_event", on_close)
        ax = fig.add_subplot()
        im = ax.imshow([full_buffer], cmap="inferno", aspect="auto", vmin=VMIN, vmax=VMAX)
        fig.colorbar(im, ax=ax, label="°C")
        ax.set_title("16-column Thermal Strip")
        ax.set_yticks([])
        ax.set_xticks(range(FULL_COLUMNS))

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
