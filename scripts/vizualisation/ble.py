"""Live BLE dashboard for a tire temperature sensor.

Scans for any device speaking a known tire protocol (see tire_protocols.py),
connects with the matching decoder, and renders the shared dashboard. Pass
``--protocol`` to insist on one protocol; by default the device's own
advertisement decides.
"""
from __future__ import annotations

import argparse
import asyncio

import matplotlib.pyplot as plt
from bleak import BleakClient, BleakScanner

from dashboard import Dashboard, render_loop
from tire_protocols import TireBleProtocol, TireState, identify_protocol, protocol_names

SCAN_TIMEOUT = 10.0


# ---------------------------------------------------------------------------
# BLE lifecycle

async def discover(expected: str | None, timeout: float):
    print("Scanning for BLE device...")
    found = await BleakScanner.discover(timeout=timeout, return_adv=True)
    for dev, adv in found.values():
        protocol = identify_protocol(dev, adv, expected)
        if protocol is not None:
            print(f"Found {adv.local_name or dev.name} ({dev.address}, rssi {adv.rssi}) "
                  f"speaking '{protocol.name}'")
            return dev, protocol
    wanted = f"'{expected}'" if expected else f"any of {protocol_names()}"
    print(f"No device speaking {wanted} found")
    return None, None


async def stream(device, protocol: TireBleProtocol, state: TireState, window_closed: asyncio.Event):
    """Connect, subscribe, and hold the link until the device drops or the window closes."""

    def on_disconnect(_client):
        state.connected = False
        print("Device disconnected — close the window to exit.")

    def make_handler(uuid: str):
        def handle(_sender, data: bytearray):
            try:
                protocol.decode(uuid, bytes(data), state)
            except (ValueError, KeyError) as exc:
                print(exc)
        return handle

    client = BleakClient(device, disconnected_callback=on_disconnect)
    try:
        await client.connect()
        state.connected = True
        print("Connected to", device.address)
        for uuid in protocol.characteristic_uuids():
            await client.start_notify(uuid, make_handler(uuid))
        while not window_closed.is_set() and client.is_connected:
            state.connected = client.is_connected
            await asyncio.sleep(0.25)
        state.connected = client.is_connected
    except Exception as exc:
        state.connected = False
        print(f"BLE error: {exc}")
    finally:
        try:
            await asyncio.wait_for(client.disconnect(), timeout=3.0)
        except Exception:
            pass


async def main(args):
    device, protocol = await discover(args.protocol, args.scan_timeout)
    if device is None:
        return

    state = TireState()
    window_closed = asyncio.Event()
    dash = Dashboard(state, protocol, window_closed)
    render = asyncio.create_task(render_loop(dash))
    try:
        await stream(device, protocol, state, window_closed)
        # Device is gone but the window is still open: keep it responsive so the
        # user can close it.
        while not window_closed.is_set():
            await asyncio.sleep(0.1)
    finally:
        window_closed.set()
        render.cancel()
        try:
            await render
        except asyncio.CancelledError:
            pass
        plt.close("all")


def parse_args():
    parser = argparse.ArgumentParser(description="Live dashboard for a BLE tire temperature sensor.")
    parser.add_argument("--protocol", choices=protocol_names(), default=None,
                        help="insist on this wire protocol instead of auto-detecting from the advertisement")
    parser.add_argument("--scan-timeout", type=float, default=SCAN_TIMEOUT,
                        help=f"seconds to scan for a device (default: {SCAN_TIMEOUT:g})")
    return parser.parse_args()


if __name__ == "__main__":
    try:
        asyncio.run(main(parse_args()))
    except KeyboardInterrupt:
        print("Interrupted — exiting...")
