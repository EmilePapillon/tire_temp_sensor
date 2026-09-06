"""Protocol-agnostic live dashboard: two thermal strips, a battery gauge and a
connection-status banner, rendered from a ``TireState``."""
from __future__ import annotations

import asyncio
import time

import matplotlib.pyplot as plt

from tire_protocols import FULL_COLUMNS, PAIR_ZONES, TireBleProtocol, TireState

TEMP_VMIN, TEMP_VMAX = 20, 40  # degrees Celsius color scale
STALE_AFTER = 2.0              # seconds without a packet -> "no data"
RENDER_HZ = 15


def _batt_color(pct):
    if pct is None:
        return "0.6"
    if pct >= 50:
        return "#2e7d32"
    if pct >= 20:
        return "#f9a825"
    return "#c62828"


class Dashboard:
    """The window is repainted from render_loop() on a fixed timer, so it stays
    responsive (and closable) even when BLE notifications stop."""

    def __init__(self, state: TireState, protocol: TireBleProtocol, window_closed: asyncio.Event):
        self.state = state
        self.window_closed = window_closed

        plt.ion()
        self.fig = plt.figure(figsize=(10, 4), constrained_layout=True)
        try:
            self.fig.canvas.manager.set_window_title(f"{protocol.name} tire sensor live")
        except Exception:
            pass
        gs = self.fig.add_gridspec(2, 2, width_ratios=[26, 1], height_ratios=[2, 1])
        self.ax_full = self.fig.add_subplot(gs[0, 0])
        self.ax_pair = self.fig.add_subplot(gs[1, 0])
        self.ax_batt = self.fig.add_subplot(gs[:, 1])

        self.im_full = self.ax_full.imshow(
            [state.columns], cmap="inferno", aspect="auto", vmin=TEMP_VMIN, vmax=TEMP_VMAX
        )
        self.fig.colorbar(self.im_full, ax=self.ax_full, label="°C")
        self.ax_full.set_yticks([])
        self.ax_full.set_xticks(range(FULL_COLUMNS))
        self.ax_full.set_xlabel(protocol.columns_label)

        self.im_pair = self.ax_pair.imshow(
            [state.pair_zones], cmap="inferno", aspect="auto", vmin=TEMP_VMIN, vmax=TEMP_VMAX
        )
        self.fig.colorbar(self.im_pair, ax=self.ax_pair, label="°C")
        self.ax_pair.set_yticks([])
        self.ax_pair.set_xticks(range(PAIR_ZONES))
        self.ax_pair.set_xlabel(protocol.pair_zones_label)

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
        state = self.state
        now = time.monotonic()
        fresh = state.last_packet > 0.0 and (now - state.last_packet < STALE_AFTER)

        self.im_full.set_data([state.columns])
        self.im_pair.set_data([state.pair_zones])

        pct, mv = state.battery_pct, state.battery_mv
        self.batt_bar.set_height(pct or 0)
        self.batt_bar.set_color(_batt_color(pct))
        if pct is None:
            self.batt_text.set_text("—")
            self.batt_text.set_y(50)
        else:
            self.batt_text.set_text(f"{pct}%  ·  {mv} mV")
            self.batt_text.set_y(min(max(pct, 8), 92))

        if not state.connected:
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
            self.window_closed.set()


async def render_loop(dash: Dashboard):
    period = 1.0 / RENDER_HZ
    while not dash.window_closed.is_set():
        if not dash.alive():
            dash.window_closed.set()
            break
        dash.refresh()
        await asyncio.sleep(period)
