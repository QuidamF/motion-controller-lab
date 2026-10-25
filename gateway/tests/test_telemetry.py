import json
import math
import unittest

from _path import *  # noqa: F401,F403

from motion_gateway import protocol as P
from motion_gateway.protocol import Flags, Quaternion
from motion_gateway.telemetry import LatencyEstimator, Normalizer, RateMeter, SequenceTracker

OK = (Flags.REFERENCE_CAPTURED | Flags.EULER_FINITE | Flags.QUAT_VALID
      | Flags.MAG_DATA_READY | Flags.MAG_COVERAGE_VALID | Flags.MAG_CALIBRATED
      | Flags.YAW_MAG_VALID)


def pkt(seq, ts=0, mag=10.0, flags=OK):
    return P.decode_telemetry(P.encode_telemetry(
        sequence=seq, timestamp_ms=ts, roll_deg=1.0, pitch_deg=2.0, yaw_rel_deg=3.0,
        yaw_mag_rel_deg=mag, quaternion=Quaternion(1, 0, 0, 0), flags=flags))


class SequenceTests(unittest.TestCase):
    def test_consecutive(self):
        t = SequenceTracker()
        for s in range(10):
            r = t.update(s)
            self.assertEqual(r.lost, 0)
        self.assertEqual((t.received, t.lost), (10, 0))

    def test_gap(self):
        t = SequenceTracker()
        t.update(10)
        r = t.update(14)
        self.assertEqual(r.lost, 3)
        self.assertEqual(r.extended, 14)

    def test_wraparound(self):
        t = SequenceTracker()
        t.update(65534)
        self.assertEqual(t.update(65535).lost, 0)
        r = t.update(1)  # sequence 0 was lost across the wrap
        self.assertEqual((r.lost, r.extended), (1, 65537))
        self.assertEqual(t.lost, 1)

    def test_duplicate_and_reorder(self):
        t = SequenceTracker()
        t.update(100)
        t.update(101)
        self.assertTrue(t.update(101).duplicate)
        self.assertTrue(t.update(99).reordered)
        self.assertEqual(t.update(102).lost, 0)

    def test_device_restart(self):
        t = SequenceTracker()
        t.update(30000)
        r = t.update(0)
        self.assertTrue(r.reset)
        self.assertEqual(t.resets, 1)
        self.assertEqual(t.update(1).lost, 0)

    def test_new_link_does_not_count_loss(self):
        t = SequenceTracker()
        t.update(5)
        t.reset()
        self.assertEqual(t.update(500).lost, 0)


class RateLatencyTests(unittest.TestCase):
    def test_rate_50hz(self):
        m = RateMeter(window_s=2.0)
        for i in range(200):
            m.add(i * 0.02)
        self.assertAlmostEqual(m.rate(), 50.0, places=3)

    def test_rate_decays_when_silent(self):
        m = RateMeter(window_s=1.0)
        for i in range(50):
            m.add(i * 0.02)
        self.assertEqual(m.rate(now=10.0), 0.0)

    def test_extra_delay(self):
        e = LatencyEstimator()
        self.assertEqual(e.add(100.000, 0), 0.0)
        self.assertAlmostEqual(e.add(100.035, 20), 15.0, places=6)  # 15 ms late
        self.assertAlmostEqual(e.stats()["extra_delay_ms_max"], 15.0)


class NormalizerTests(unittest.TestCase):
    def test_message_schema(self):
        n = Normalizer(device_id="MCL-TEST")
        msg = n.telemetry(pkt(1, ts=1000), host_time_s=5.0)
        json.dumps(msg, allow_nan=False)  # must be strict JSON
        self.assertEqual(msg["type"], "telemetry")
        self.assertEqual(msg["schema_version"], 1)
        self.assertEqual(msg["device_id"], "MCL-TEST")
        self.assertEqual(set(msg["orientation"]),
                         {"roll_deg", "pitch_deg", "yaw_rel_deg", "yaw_mag_rel_deg"})
        self.assertEqual(msg["orientation"]["yaw_rel_deg"], 3.0)
        self.assertEqual(msg["orientation"]["yaw_mag_rel_deg"], 10.0)
        self.assertTrue(all(msg["validity"].values()))

    def test_invalid_heading_is_null(self):
        n = Normalizer()
        msg = n.telemetry(pkt(1, mag=None), 0.0)
        self.assertIsNone(msg["orientation"]["yaw_mag_rel_deg"])
        self.assertFalse(msg["validity"]["yaw_mag_valid"])
        self.assertIn('"yaw_mag_rel_deg": null', json.dumps(msg, allow_nan=False))
        self.assertEqual(msg["orientation"]["yaw_rel_deg"], 3.0)  # independent fields

    def test_drops_duplicates_and_counts(self):
        n = Normalizer()
        self.assertIsNotNone(n.telemetry(pkt(1), 0.0))
        self.assertIsNone(n.telemetry(pkt(1), 0.01))
        self.assertIsNotNone(n.telemetry(pkt(5), 0.02))
        s = n.stats(now=0.02)
        self.assertEqual((s["lost"], s["rejected"].get("duplicate")), (3, 1))
        json.dumps(s, allow_nan=False)

    def test_stats_with_status(self):
        n = Normalizer()
        n.last_status = P.decode_status(P.encode_status(phase=P.Phase.RUNNING, i2c_errors=4))
        s = n.stats()
        self.assertEqual(s["device"]["phase"], "running")
        self.assertEqual(s["device"]["i2c_errors"], 4)

    def test_no_nan_in_output(self):
        n = Normalizer()
        msg = n.telemetry(pkt(1, mag=float("nan")), 0.0)
        for v in msg["orientation"].values():
            self.assertTrue(v is None or math.isfinite(v))

    def test_notify_drops_and_sequence_loss_distinction(self):
        """Verify distinction between firmware-level notify drops and RF sequence loss."""
        n = Normalizer()
        # Receive packet 10, then packet 15 (4 packets lost over the air)
        n.telemetry(pkt(10), host_time_s=1.0)
        n.telemetry(pkt(15), host_time_s=1.1)

        # Status packet reports 2 stack-level notify drops and 1 loop overrun from ESP32
        n.last_status = P.decode_status(P.encode_status(
            phase=P.Phase.RUNNING, notify_drops=2, loop_overruns=1,
        ))

        stats = n.stats(now=1.1)
        # Sequence-level RF loss is tracked by the gateway sequence tracker
        self.assertEqual(stats["lost"], 4)
        # Firmware notify drops are reported in device diagnostics
        self.assertEqual(stats["device"]["notify_drops"], 2)
        self.assertEqual(stats["device"]["loop_overruns"], 1)

    def test_status_packet_drops_wrap(self):
        """Verify notify_drops u16 encoding and roundtrip at extreme values."""
        st = P.decode_status(P.encode_status(phase=P.Phase.RUNNING, notify_drops=65535))
        self.assertEqual(st.notify_drops, 65535)


if __name__ == "__main__":
    unittest.main()
