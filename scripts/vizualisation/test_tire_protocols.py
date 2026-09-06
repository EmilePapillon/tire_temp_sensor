"""Unit tests for tire_protocols.py. Pure Python, no BLE stack needed:

    python -m unittest discover -s scripts/vizualisation -p 'test_*.py'
"""
import struct
import unittest
from types import SimpleNamespace

from tire_protocols import (
    KNOWN_PROTOCOLS,
    RejsaTireProtocol,
    TireState,
    identify_protocol,
    protocol_by_name,
)


def adv(local_name=None, service_uuids=None):
    return SimpleNamespace(local_name=local_name, service_uuids=service_uuids)


def dev(name=None):
    return SimpleNamespace(name=name, address="AA:BB:CC:DD:EE:FF")


class RejsaMatchingTest(unittest.TestCase):
    def setUp(self):
        self.proto = RejsaTireProtocol()

    def test_matches_on_service_uuid(self):
        self.assertTrue(self.proto.matches(dev(), adv(service_uuids=["00001FF7-0000-1000-8000-00805F9B34FB"])))

    def test_matches_on_advertised_name(self):
        self.assertTrue(self.proto.matches(dev(), adv(local_name="RejsaRubberFLABCDEF")))

    def test_matches_on_cached_device_name(self):
        self.assertTrue(self.proto.matches(dev(name="RejsaRubberRR000001"), adv()))

    def test_rejects_unrelated_device(self):
        battery_service = "0000180f-0000-1000-8000-00805f9b34fb"
        self.assertFalse(self.proto.matches(dev(name="Garmin"), adv(service_uuids=[battery_service])))

    def test_rejects_empty_advertisement(self):
        self.assertFalse(self.proto.matches(dev(), adv()))


class RejsaDecodeTest(unittest.TestCase):
    def setUp(self):
        self.proto = RejsaTireProtocol()
        self.state = TireState()

    def test_pack_one_fills_even_columns(self):
        temps = [200 + 20 * i for i in range(8)]  # 20.0, 22.0, ... in tenths
        self.proto.decode(self.proto.CHAR_ONE, struct.pack("<BBh8h", 2, 0, 0, *temps), self.state)
        self.assertEqual(self.state.columns[0::2], [20.0 + 2 * i for i in range(8)])
        self.assertEqual(self.state.columns[1::2], [0.0] * 8)
        self.assertGreater(self.state.last_packet, 0.0)

    def test_every_packet_bumps_the_revision(self):
        packet = struct.pack("<BBh8h", 2, 0, 0, *([250] * 8))
        self.assertEqual(self.state.revision, 0)
        self.proto.decode(self.proto.CHAR_ONE, packet, self.state)
        self.proto.decode(self.proto.CHAR_THR, packet, self.state)
        self.assertEqual(self.state.revision, 2)

    def test_pack_two_fills_odd_columns_and_battery(self):
        temps = [210 + 20 * i for i in range(8)]
        self.proto.decode(self.proto.CHAR_TWO, struct.pack("<BBH8h", 2, 55, 3850, *temps), self.state)
        self.assertEqual(self.state.columns[1::2], [21.0 + 2 * i for i in range(8)])
        self.assertEqual(self.state.battery_pct, 55)
        self.assertEqual(self.state.battery_mv, 3850)

    def test_pack_three_fills_pair_zones(self):
        temps = [-55, 300, 310, 320, 330, 340, 350, 360]
        self.proto.decode(self.proto.CHAR_THR, struct.pack("<BBh8h", 2, 0, 0, *temps), self.state)
        self.assertEqual(self.state.pair_zones, [-5.5, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0])

    def test_uuid_match_is_case_insensitive(self):
        packet = struct.pack("<BBh8h", 2, 0, 0, *([250] * 8))
        self.proto.decode(self.proto.CHAR_THR.upper(), packet, self.state)
        self.assertEqual(self.state.pair_zones, [25.0] * 8)

    def test_rejects_wrong_length(self):
        with self.assertRaises(ValueError):
            self.proto.decode(self.proto.CHAR_ONE, b"\x02\x00", self.state)

    def test_rejects_unknown_characteristic(self):
        with self.assertRaises(KeyError):
            self.proto.decode("00000009-0000-1000-8000-00805f9b34fb", bytes(20), self.state)

    def test_subscribes_to_all_three_characteristics(self):
        self.assertEqual(len(self.proto.characteristic_uuids()), 3)


class RegistryTest(unittest.TestCase):
    def test_identify_picks_rejsa_from_service_uuid(self):
        proto = identify_protocol(dev(), adv(service_uuids=[RejsaTireProtocol.SERVICE_UUID]))
        self.assertIsInstance(proto, RejsaTireProtocol)

    def test_identify_returns_none_for_unknown_device(self):
        self.assertIsNone(identify_protocol(dev(name="Garmin"), adv()))

    def test_expected_protocol_must_still_match(self):
        self.assertIsNone(identify_protocol(dev(name="Garmin"), adv(), expected="rejsa"))
        self.assertIsNotNone(identify_protocol(dev(name="RejsaRubberFL"), adv(), expected="rejsa"))

    def test_unknown_expected_name_is_an_error(self):
        with self.assertRaises(KeyError):
            protocol_by_name("nope")

    def test_every_registered_protocol_has_the_required_shape(self):
        for proto in KNOWN_PROTOCOLS:
            for member in ("name", "columns_label", "pair_zones_label", "matches", "characteristic_uuids", "decode"):
                self.assertTrue(hasattr(proto, member), f"{proto!r} lacks {member}")


if __name__ == "__main__":
    unittest.main()
