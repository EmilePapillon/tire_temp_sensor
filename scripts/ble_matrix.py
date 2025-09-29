import asyncio
from bleak import BleakScanner, BleakClient
import struct
import matplotlib.pyplot as plt
import numpy as np

# BLE device and characteristic
DEVICE_NAME = "MLX90641"
CHAR_UUID = "00000001-0000-1000-8000-00805f9b34fb"

FULL_COLUMNS = 16
full_buffer = [0.0] * FULL_COLUMNS  # 16-column averaged temperatures

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

    # Unpack DataPack: protocol(1), packet_id(1), reserved(1), temps[8] (16 bytes)
    protocol, packet_id, reserved, t0, t1, t2, t3, t4, t5, t6, t7 = struct.unpack("<BBB8h", data)
    temps = [t0, t1, t2, t3, t4, t5, t6, t7]

    # Map temps into the correct half of the 16-column buffer
    start_idx = packet_id * 8
    for i, t in enumerate(temps):
        full_buffer[start_idx + i] = t / 10.0  # convert from tenths of °C

    update_plot()

async def main():
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
        plt.figure(figsize=(8, 2))
        while True:
            await asyncio.sleep(1)  # keep loop alive for notifications

asyncio.run(main())
