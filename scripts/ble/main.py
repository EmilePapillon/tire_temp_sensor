import asyncio
from bleak import BleakScanner, BleakClient
import struct
import matplotlib.pyplot as plt
import numpy as np

# BLE device name to search for
DEVICE_NAME = "MLX90641"

# UUIDs of the two BLE characteristics providing thermal data
CHAR_UUIDS = [
    "00000001-0000-1000-8000-00805f9b34fb",  # First data packet
    "00000002-0000-1000-8000-00805f9b34fb",  # Second data packet
]

# Expected BLE payload length (bytes)
PACKET_SIZE_BYTES = 20  

# Number of temperature readings per packet
TEMPS_PER_PACKET = 6  

# Total number of temperature readings (from both packets)
TOTAL_TEMPS = TEMPS_PER_PACKET * 2  

# Struct formats for unpacking BLE data packets
PACKET_ONE_FORMAT = "<BBh6hxxxx"   # First data packet
PACKET_TWO_FORMAT = "<BBH6hxxxx"   # Second data packet

# Buffers for even and odd indexed temperature samples
temps_from_packet_one = [0] * TEMPS_PER_PACKET  # Even-indexed readings
temps_from_packet_two = [0] * TEMPS_PER_PACKET  # Odd-indexed readings


def update_plot():
    """
    Combine readings from both data packets, scale to Celsius, 
    and update the live heatmap plot.
    """
    combined_readings = [None] * TOTAL_TEMPS

    # Interleave even (packet one) and odd (packet two) readings
    for i in range(TEMPS_PER_PACKET):
        combined_readings[i * 2] = temps_from_packet_one[i]
        combined_readings[i * 2 + 1] = temps_from_packet_two[i]

    # Convert raw values (tenths of °C) to Celsius
    temps_celsius = np.array(combined_readings) / 10.0

    plt.clf()  # Clear previous plot

    # Normalize color scale
    vmin = np.min(temps_celsius)
    vmax = np.max(temps_celsius)
    if vmax == vmin:  # Prevent division by zero
        vmax += 1  

    # Display as a horizontal strip
    plt.imshow(
        [temps_celsius],
        cmap="jet",
        aspect="auto",
        vmin=vmin,
        vmax=vmax
    )

    # Add color scale and labels
    plt.colorbar(label="°C")
    plt.title(f"Thermal Strip — {vmin:.1f}°C to {vmax:.1f}°C")
    plt.yticks([])  # Hide y-axis ticks
    plt.xticks(range(TOTAL_TEMPS), [f"{i}" for i in range(TOTAL_TEMPS)])
    plt.pause(0.01)


def decode_packet_one(data):
    """
    Decode the first BLE packet and update the even-indexed temperatures.
    """
    global temps_from_packet_one
    if len(data) != PACKET_SIZE_BYTES:
        print(f"[PACKET ONE] Unexpected data length: {len(data)}")
        return
    _, _, _, *temps = struct.unpack(PACKET_ONE_FORMAT, data)
    temps_from_packet_one = temps
    update_plot()


def decode_packet_two(data):
    """
    Decode the second BLE packet and update the odd-indexed temperatures.
    """
    global temps_from_packet_two
    if len(data) != PACKET_SIZE_BYTES:
        print(f"[PACKET TWO] Unexpected data length: {len(data)}")
        return
    _, _, _, *temps = struct.unpack(PACKET_TWO_FORMAT, data)
    temps_from_packet_two = temps
    update_plot()


async def main():
    """
    Scan for the BLE device, connect, subscribe to notifications,
    and continuously update the plot with incoming thermal data.
    """
    print("Scanning for BLE device...")
    devices = await BleakScanner.discover()
    device = next((d for d in devices if d.name == DEVICE_NAME), None)
    if not device:
        print(f"Device named '{DEVICE_NAME}' not found")
        return

    async with BleakClient(device) as client:
        print("Connected!")

        # Subscribe to notifications from both characteristics
        await client.start_notify(CHAR_UUIDS[0], lambda _, d: decode_packet_one(d))
        await client.start_notify(CHAR_UUIDS[1], lambda _, d: decode_packet_two(d))

        print("Receiving and visualizing data...")
        plt.ion()
        plt.figure(figsize=(8, 2))  # Set up plotting window
        while True:
            await asyncio.sleep(1)  # Keep loop alive for async events


# Run the main event loop
asyncio.run(main())
