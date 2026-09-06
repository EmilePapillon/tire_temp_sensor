import asyncio
import struct
import time

from bleak import BleakClient, BleakScanner
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# The firmware advertises as "RejsaRubber" + corner (FL/FR/RL/RR) + MAC suffix,
# on GATT service 0x1ff7. macOS caches the old GAP name across bonds, so match on
# the live advertisement (local_name / service UUID), not just BleakDevice.name.
DEVICE_NAME_PREFIX = "RejsaRubber"
SERVICE_UUID = "00001ff7-0000-1000-8000-00805f9b34fb"

# Three 20-byte little-endian NOTIFY characteristics (see include/data_pack.hh):
#   0x01 DataPackOne : protocol,B unused,B  distance,h  temps[8],8h  -> even columns
#   0x02 DataPackTwo : protocol,B charge,B  voltage,H   temps[8],8h  -> odd columns + battery
#   0x03 DataPackThr : protocol,B unused,B  distance,h  temps[8],8h  -> per-pair max
CHAR_ONE = "00000001-0000-1000-8000-00805f9b34fb"
CHAR_TWO = "00000002-0000-1000-8000-00805f9b34fb"
CHAR_THR = "00000003-0000-1000-8000-00805f9b34fb"
PACK_ONE = "<BBh8h"
PACK_TWO = "<BBH8h"
PACK_THR = "<BBh8h"

FULL_COLUMNS = 16
PAIR_ZONES = 8
TEMP_VMIN, TEMP_VMAX = 20, 40      # degrees Celsius color scale
STALE_AFTER = 2.0                  # seconds without a packet -> "no data"
RENDER_HZ = 15

# Shared state: written by the BLE notification callbacks, read by the render loop.
state = {
    "full": [0.0] * FULL_COLUMNS,
    "pair": [0.0] * PAIR_ZONES,
    "batt_pct": None,
    "batt_mv": None,
    "connected": False,
    "last_packet": 0.0,           # time.monotonic() of the last notification
}
window_closed = asyncio.Event()


# ---------------------------------------------------------------------------
# BLE notification handlers

def _unpack(data, fmt, label):
    if len(data) != 20:
        print(f"{label}: unexpected packet length {len(data)}")
        return None
    return struct.unpack(fmt, data)


def on_one(_, data):
    fields = _unpack(data, PACK_ONE, "char 0x01")
    if fields is None:
        return
    for i, raw in enumerate(fields[3:]):
        state["full"][i * 2] = raw / 10.0          # even columns 0, 2, ... 14
    state["last_packet"] = time.monotonic()


def on_two(_, data):
    fields = _unpack(data, PACK_TWO, "char 0x02")
    if fields is None:
        return
    state["batt_pct"] = fields[1]
    state["batt_mv"] = fields[2]
    for i, raw in enumerate(fields[3:]):
        state["full"][i * 2 + 1] = raw / 10.0      # odd columns 1, 3, ... 15
    state["last_packet"] = time.monotonic()


def on_thr(_, data):
    fields = _unpack(data, PACK_THR, "char 0x03")
    if fields is None:
        return
    for i, raw in enumerate(fields[3:]):
        state["pair"][i] = raw / 10.0
    state["last_packet"] = time.monotonic()


# ---------------------------------------------------------------------------
# Dashboard

def _batt_color(pct):
    if pct is None:
        return "0.6"
    if pct >= 50:
        return "#2e7d32"
    if pct >= 20:
        return "#f9a825"
    return "#c62828"


class Dashboard:
    """Two thermal strips + a battery gauge + a connection-status banner.

    The window is repainted from render_loop() on a fixed timer, so it stays
    responsive (and closable) even when BLE notifications stop.
    """

    def __init__(self):
        plt.ion()
        self.fig = plt.figure(figsize=(10, 4), constrained_layout=True)
        try:
            self.fig.canvas.manager.set_window_title("RejsaRubberTrac live")
        except Exception:
            pass
        gs = self.fig.add_gridspec(2, 2, width_ratios=[26, 1], height_ratios=[2, 1])
        self.ax_full = self.fig.add_subplot(gs[0, 0])
        self.ax_pair = self.fig.add_subplot(gs[1, 0])
        self.ax_batt = self.fig.add_subplot(gs[:, 1])

        self.im_full = self.ax_full.imshow(
            [state["full"]], cmap="inferno", aspect="auto", vmin=TEMP_VMIN, vmax=TEMP_VMAX
        )
        self.fig.colorbar(self.im_full, ax=self.ax_full, label="°C")
        self.ax_full.set_yticks([])
        self.ax_full.set_xticks(range(FULL_COLUMNS))
        self.ax_full.set_xlabel("16-column strip  (chars 0x01 + 0x02)")

        self.im_pair = self.ax_pair.imshow(
            [state["pair"]], cmap="inferno", aspect="auto", vmin=TEMP_VMIN, vmax=TEMP_VMAX
        )
        self.fig.colorbar(self.im_pair, ax=self.ax_pair, label="°C")
        self.ax_pair.set_yticks([])
        self.ax_pair.set_xticks(range(PAIR_ZONES))
        self.ax_pair.set_xlabel("8-zone pair-max  (char 0x03 — what RaceChrono logs)")

        self.batt_bar = self.ax_batt.bar([0], [0], width=1.0, color="0.6")[0]
        self.ax_batt.set_xlim(-0.6, 0.6)
        self.ax_batt.set_ylim(0, 100)
        self.ax_batt.set_xticks([])
        self.ax_batt.set_yticks([0, 25, 50, 75, 100])
        self.ax_batt.set_title("batt", fontsize=9)
        self.batt_text = self.ax_batt.text(
            0, 50, "—", ha="center", va="center", fontsize=8, rotation=90
        )

        self.status = self.fig.suptitle("connecting…", color="0.4", fontsize=12)
        self.fig.canvas.mpl_connect("close_event", lambda _e: window_closed.set())

    def alive(self):
        return plt.fignum_exists(self.fig.number)

    def refresh(self):
        now = time.monotonic()
        fresh = state["last_packet"] > 0.0 and (now - state["last_packet"] < STALE_AFTER)

        self.im_full.set_data([state["full"]])
        self.im_pair.set_data([state["pair"]])

        pct, mv = state["batt_pct"], state["batt_mv"]
        self.batt_bar.set_height(pct or 0)
        self.batt_bar.set_color(_batt_color(pct))
        if pct is None:
            self.batt_text.set_text("—")
            self.batt_text.set_y(50)
        else:
            self.batt_text.set_text(f"{pct}%  ·  {mv} mV")
            self.batt_text.set_y(min(max(pct, 8), 92))

        if not state["connected"]:
            self.status.set_text("○  DISCONNECTED — device off / out of range")
            self.status.set_color("#c62828")
        elif fresh:
            self.status.set_text("●  CONNECTED — streaming")
            self.status.set_color("#2e7d32")
        else:
            self.status.set_text("●  CONNECTED — no data")
            self.status.set_color("#f9a825")

        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception:
            window_closed.set()


async def render_loop(dash):
    period = 1.0 / RENDER_HZ
    while not window_closed.is_set():
        if not dash.alive():
            window_closed.set()
            break
        dash.refresh()
        await asyncio.sleep(period)


# ---------------------------------------------------------------------------
# BLE lifecycle

async def discover():
    print("Scanning for BLE device...")
    found = await BleakScanner.discover(timeout=10.0, return_adv=True)
    for dev, adv in found.values():
        names = [n for n in (adv.local_name, dev.name) if n]
        svcs = [s.lower() for s in (adv.service_uuids or [])]
        if any(n.startswith(DEVICE_NAME_PREFIX) for n in names) or SERVICE_UUID in svcs:
            print(f"Found {adv.local_name or dev.name} ({dev.address}, rssi {adv.rssi})")
            return dev
    print(f"No device advertising as '{DEVICE_NAME_PREFIX}*' or service {SERVICE_UUID} found")
    return None


async def stream(device):
    """Connect, subscribe, and hold the link until the device drops or the window closes."""

    def on_disconnect(_client):
        state["connected"] = False
        print("Device disconnected — close the window to exit.")

    client = BleakClient(device, disconnected_callback=on_disconnect)
    try:
        await client.connect()
        state["connected"] = True
        print("Connected to", device.address)
        for uuid, cb in ((CHAR_ONE, on_one), (CHAR_TWO, on_two), (CHAR_THR, on_thr)):
            await client.start_notify(uuid, cb)
        while not window_closed.is_set() and client.is_connected:
            state["connected"] = client.is_connected
            await asyncio.sleep(0.25)
        state["connected"] = client.is_connected
    except Exception as exc:
        state["connected"] = False
        print(f"BLE error: {exc}")
    finally:
        try:
            await asyncio.wait_for(client.disconnect(), timeout=3.0)
        except Exception:
            pass


async def main():
    device = await discover()
    if device is None:
        return

    dash = Dashboard()
    render = asyncio.create_task(render_loop(dash))
    try:
        await stream(device)
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


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Interrupted — exiting...")
