"""BLE wire protocols spoken by tire temperature sensors.

Each protocol knows how to recognise its device from an advertisement, which
GATT characteristics to subscribe to, and how to decode their packets into a
protocol-agnostic ``TireState``. Nothing here imports bleak or matplotlib, so
the decoders are unit-testable with plain bytes.

Protocols are matched at runtime: each real protocol is required by its own
consumer app to advertise a distinct service UUID (Rejsa 0x1ff7, RaceChrono
DIY 0x1ff8), so the device self-identifies for free. Register new protocols in
``KNOWN_PROTOCOLS``.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from typing import Any, Protocol

FULL_COLUMNS = 16
PAIR_ZONES = 8


@dataclass
class TireState:
    """What the dashboard renders; filled in by whichever protocol matched."""

    columns: list[float] = field(default_factory=lambda: [0.0] * FULL_COLUMNS)
    pair_zones: list[float] = field(default_factory=lambda: [0.0] * PAIR_ZONES)
    battery_pct: int | None = None
    battery_mv: int | None = None
    connected: bool = False
    last_packet: float = 0.0  # time.monotonic() of the last decoded packet
    revision: int = 0         # bumped on every decoded packet; renderers repaint only when it changes

    def touch(self) -> None:
        self.last_packet = time.monotonic()
        self.revision += 1


class TireBleProtocol(Protocol):
    """Structural interface: any object with these members is a protocol."""

    name: str
    columns_label: str
    pair_zones_label: str

    def matches(self, device: Any, adv_data: Any) -> bool: ...

    def characteristic_uuids(self) -> list[str]: ...

    def decode(self, uuid: str, data: bytes, state: TireState) -> None: ...


def _bluetooth_uuid(short: int) -> str:
    """Expand a 16-bit UUID to the 128-bit Bluetooth base form bleak reports."""
    return f"{short:08x}-0000-1000-8000-00805f9b34fb"


class RejsaTireProtocol:
    """RejsaRubberTrac: service 0x1ff7, three 20-byte NOTIFY characteristics.

    Little-endian packets (see lib/ble_protocol/rejsa_ble_protocol.hh):
      0x01 DataPackOne : protocol,B unused,B  distance,h  temps[8],8h  -> even columns
      0x02 DataPackTwo : protocol,B charge,B  voltage,H   temps[8],8h  -> odd columns + battery
      0x03 DataPackThr : protocol,B unused,B  distance,h  temps[8],8h  -> per-pair max
    Temperatures are degrees Celsius x 10.
    """

    name = "rejsa"
    columns_label = "16-column strip  (chars 0x01 + 0x02)"
    pair_zones_label = "8-zone pair-max  (char 0x03 — what RaceChrono logs)"

    SERVICE_UUID = _bluetooth_uuid(0x1FF7)
    NAME_PREFIX = "RejsaRubber"
    CHAR_ONE = _bluetooth_uuid(0x01)
    CHAR_TWO = _bluetooth_uuid(0x02)
    CHAR_THR = _bluetooth_uuid(0x03)
    PACKET_LEN = 20
    _PACK_ONE = struct.Struct("<BBh8h")
    _PACK_TWO = struct.Struct("<BBH8h")
    _PACK_THR = struct.Struct("<BBh8h")

    def matches(self, device: Any, adv_data: Any) -> bool:
        # macOS caches the old GAP name across bonds, so prefer the live
        # advertisement (local_name / service UUID) over device.name.
        names = [n for n in (getattr(adv_data, "local_name", None), getattr(device, "name", None)) if n]
        services = [s.lower() for s in (getattr(adv_data, "service_uuids", None) or [])]
        return self.SERVICE_UUID in services or any(n.startswith(self.NAME_PREFIX) for n in names)

    def characteristic_uuids(self) -> list[str]:
        return [self.CHAR_ONE, self.CHAR_TWO, self.CHAR_THR]

    def decode(self, uuid: str, data: bytes, state: TireState) -> None:
        if len(data) != self.PACKET_LEN:
            raise ValueError(f"{uuid}: unexpected packet length {len(data)}, want {self.PACKET_LEN}")
        uuid = uuid.lower()
        if uuid == self.CHAR_ONE:
            fields = self._PACK_ONE.unpack(data)
            for i, raw in enumerate(fields[3:]):
                state.columns[i * 2] = raw / 10.0  # even columns 0, 2, ... 14
        elif uuid == self.CHAR_TWO:
            fields = self._PACK_TWO.unpack(data)
            state.battery_pct = fields[1]
            state.battery_mv = fields[2]
            for i, raw in enumerate(fields[3:]):
                state.columns[i * 2 + 1] = raw / 10.0  # odd columns 1, 3, ... 15
        elif uuid == self.CHAR_THR:
            fields = self._PACK_THR.unpack(data)
            for i, raw in enumerate(fields[3:]):
                state.pair_zones[i] = raw / 10.0
        else:
            raise KeyError(f"{uuid}: not a {self.name} characteristic")
        state.touch()


KNOWN_PROTOCOLS: list[TireBleProtocol] = [RejsaTireProtocol()]  # append e.g. DiyTireProtocol() here


def protocol_names() -> list[str]:
    return [p.name for p in KNOWN_PROTOCOLS]


def protocol_by_name(name: str) -> TireBleProtocol:
    for protocol in KNOWN_PROTOCOLS:
        if protocol.name == name:
            return protocol
    raise KeyError(f"unknown protocol {name!r}; known: {protocol_names()}")


def identify_protocol(device: Any, adv_data: Any, expected: str | None = None) -> TireBleProtocol | None:
    """Return the protocol this advertisement belongs to, or None.

    With ``expected`` set, only that protocol is considered: a safety net to
    assert the device speaks what you think it speaks.
    """
    candidates = [protocol_by_name(expected)] if expected else KNOWN_PROTOCOLS
    return next((p for p in candidates if p.matches(device, adv_data)), None)
