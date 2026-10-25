import math
import random
import struct
import unittest

from _path import *  # noqa: F401,F403  (adds src/ to sys.path)

from motion_gateway import protocol as P
from motion_gateway.protocol import Flags, Quaternion

ALL_OK = (Flags.REFERENCE_CAPTURED | Flags.EULER_FINITE | Flags.QUAT_VALID
          | Flags.MAG_DATA_READY | Flags.MAG_COVERAGE_VALID
          | Flags.MAG_CALIBRATED | Flags.YAW_MAG_VALID)


def quat_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(math.radians(roll) / 2), math.sin(math.radians(roll) / 2)
    cp, sp = math.cos(math.radians(pitch) / 2), math.sin(math.radians(pitch) / 2)
    cy, sy = math.cos(math.radians(yaw) / 2), math.sin(math.radians(yaw) / 2)
    return Quaternion(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                      cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def sample(**kw):
    base = dict(sequence=1234, timestamp_ms=45678, roll_deg=1.23, pitch_deg=-3.45,
                yaw_rel_deg=15.67, yaw_mag_rel_deg=14.81,
                quaternion=quat_from_euler(1.23, -3.45, 15.67), flags=ALL_OK)
    base.update(kw)
    return base


class GoldenVector(unittest.TestCase):
    """Pins the byte layout; the firmware encoder must produce the same bytes."""

    def test_layout(self):
        data = P.encode_telemetry(**sample(quaternion=Quaternion(1, 0, 0, 0)))
        self.assertEqual(len(data), 20)
        expected = bytes([0x01, 0x7F]) + struct.pack("<H", 1234) + struct.pack("<I", 45678) \
            + struct.pack("<hhhh", 123, -345, 1567, 1481) + struct.pack("<I", 0)
        self.assertEqual(data, expected)
        self.assertEqual(data.hex(), "017fd2046eb200007b00a7fe1f06c90500000000")

    def test_invalid_sentinel_bytes(self):
        data = P.encode_telemetry(**sample(yaw_mag_rel_deg=None))
        self.assertEqual(data[14:16], b"\x00\x80")
        self.assertFalse(data[1] & Flags.YAW_MAG_VALID)


class TelemetryRoundTrip(unittest.TestCase):
    def test_roundtrip(self):
        pkt = P.decode_telemetry(P.encode_telemetry(**sample()))
        self.assertEqual(pkt.sequence, 1234)
        self.assertEqual(pkt.timestamp_ms, 45678)
        self.assertAlmostEqual(pkt.roll_deg, 1.23)
        self.assertAlmostEqual(pkt.pitch_deg, -3.45)
        self.assertAlmostEqual(pkt.yaw_rel_deg, 15.67)
        self.assertAlmostEqual(pkt.yaw_mag_rel_deg, 14.81)
        self.assertEqual(pkt.flags, ALL_OK)
        self.assertEqual(pkt.warnings, ())

    def test_range_extremes(self):
        pkt = P.decode_telemetry(P.encode_telemetry(**sample(
            roll_deg=-180.0, pitch_deg=90.0, yaw_rel_deg=180.0, yaw_mag_rel_deg=-180.0)))
        self.assertEqual((pkt.roll_deg, pkt.pitch_deg, pkt.yaw_rel_deg, pkt.yaw_mag_rel_deg),
                         (-180.0, 90.0, 180.0, -180.0))

    def test_wrap_of_counters(self):
        pkt = P.decode_telemetry(P.encode_telemetry(**sample(sequence=65536 + 5,
                                                             timestamp_ms=2**32 + 7)))
        self.assertEqual((pkt.sequence, pkt.timestamp_ms), (5, 7))


class InvalidValues(unittest.TestCase):
    def test_nan_inf_never_valid(self):
        for bad in (float("nan"), float("inf"), -float("inf"), None):
            pkt = P.decode_telemetry(P.encode_telemetry(**sample(roll_deg=bad)))
            self.assertIsNone(pkt.roll_deg)
            self.assertIsNone(pkt.pitch_deg)  # whole Euler set invalidated
            self.assertFalse(pkt.flags & Flags.EULER_FINITE)

    def test_heading_invalid_is_none_not_zero(self):
        for bad in (None, float("nan"), -999.0):  # -999 = firmware sentinel
            pkt = P.decode_telemetry(P.encode_telemetry(**sample(yaw_mag_rel_deg=bad)))
            self.assertIsNone(pkt.yaw_mag_rel_deg)
            self.assertFalse(pkt.flags & Flags.YAW_MAG_VALID)

    def test_heading_flag_false_hides_value(self):
        flags = ALL_OK & ~Flags.YAW_MAG_VALID
        pkt = P.decode_telemetry(P.encode_telemetry(**sample(flags=flags)))
        self.assertIsNone(pkt.yaw_mag_rel_deg)

    def test_out_of_range_rejected_by_encoder(self):
        self.assertEqual(P.encode_angle(95.0, 90.0), P.INVALID_I16)
        self.assertEqual(P.encode_angle(-200.0), P.INVALID_I16)

    def test_out_of_range_on_wire_warns(self):
        raw = bytearray(P.encode_telemetry(**sample()))
        raw[10:12] = struct.pack("<h", 12000)  # pitch 120 deg: impossible
        pkt = P.decode_telemetry(bytes(raw))
        self.assertIn("pitch_out_of_range", pkt.warnings)
        self.assertIn("euler_flag_inconsistent", pkt.warnings)
        self.assertIsNone(pkt.pitch_deg)

    def test_flag_claims_valid_but_sentinel(self):
        raw = bytearray(P.encode_telemetry(**sample()))
        raw[14:16] = b"\x00\x80"
        pkt = P.decode_telemetry(bytes(raw))
        self.assertIsNone(pkt.yaw_mag_rel_deg)
        self.assertIn("yaw_mag_flag_inconsistent", pkt.warnings)

    def test_reserved_flag_warns(self):
        pkt = P.decode_telemetry(P.encode_telemetry(**sample(flags=ALL_OK | Flags.RESERVED_7)))
        self.assertIn("reserved_flag_set", pkt.warnings)

    def test_bad_quaternion_not_packed(self):
        for q in (None, Quaternion(float("nan"), 0, 0, 0), Quaternion(2, 0, 0, 0)):
            pkt = P.decode_telemetry(P.encode_telemetry(**sample(quaternion=q)))
            self.assertIsNone(pkt.quaternion)
            self.assertFalse(pkt.flags & Flags.QUAT_VALID)


class MalformedPackets(unittest.TestCase):
    def test_sizes(self):
        good = P.encode_telemetry(**sample())
        for n, kind in ((0, "truncated"), (1, "truncated"), (19, "truncated")):
            with self.assertRaises(P.ProtocolError) as cm:
                P.decode_telemetry(good[:n])
            self.assertEqual(cm.exception.kind, kind)
        with self.assertRaises(P.ProtocolError) as cm:
            P.decode_telemetry(good + b"\x00")
        self.assertEqual(cm.exception.kind, "oversized")

    def test_unknown_version(self):
        for v in (0, 2, 255):
            with self.assertRaises(P.ProtocolError) as cm:
                P.decode_telemetry(P.encode_telemetry(**sample(version=v)))
            self.assertEqual(cm.exception.kind, "unknown_version")

    def test_random_bytes_never_crash(self):
        rng = random.Random(42)
        for _ in range(5000):
            data = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 25)))
            try:
                pkt = P.decode_telemetry(data)
            except P.ProtocolError:
                continue
            for v in (pkt.roll_deg, pkt.pitch_deg, pkt.yaw_rel_deg, pkt.yaw_mag_rel_deg):
                self.assertTrue(v is None or math.isfinite(v))


class QuaternionPacking(unittest.TestCase):
    def test_accuracy(self):
        rng = random.Random(1)
        worst_deg = 0.0
        total_deg = 0.0
        n_samples = 20000
        for _ in range(n_samples):
            v = [rng.gauss(0, 1) for _ in range(4)]
            n = math.sqrt(sum(c * c for c in v))
            q = Quaternion(*(c / n for c in v))
            r = P.unpack_quaternion(P.pack_quaternion(q))
            dot = abs(q.w * r.w + q.x * r.x + q.y * r.y + q.z * r.z)
            ang = math.degrees(2 * math.acos(min(1.0, dot)))
            worst_deg = max(worst_deg, ang)
            total_deg += ang
        mean_deg = total_deg / n_samples
        # Theoretical worst-case bound for 10-bit signed Smallest-Three is ~0.274 deg
        self.assertLess(worst_deg, 0.28)
        self.assertLess(mean_deg, 0.10)
        self.assertLess(worst_deg, 0.50)

    def test_identity_and_sign(self):
        r = P.unpack_quaternion(P.pack_quaternion(Quaternion(-1, 0, 0, 0)))
        self.assertAlmostEqual(r.w, 1.0)  # -q == q as rotation, positive largest

    def test_component_error_bound(self):
        q = quat_from_euler(30, -20, 100)
        r = P.unpack_quaternion(P.pack_quaternion(q))
        s = 1 if (q.w * r.w + q.x * r.x + q.y * r.y + q.z * r.z) >= 0 else -1
        for a, b in zip((q.w, q.x, q.y, q.z), (r.w, r.x, r.y, r.z)):
            self.assertLess(abs(a - s * b), 0.003)


class Status(unittest.TestCase):
    def test_roundtrip(self):
        data = P.encode_status(phase=P.Phase.MAG_CAL, mag_cal_seconds_left=17,
                               i2c_errors=3, mag_errors=2**32 - 1,
                               loop_overruns=99, notify_drops=7)
        self.assertEqual(len(data), 20)
        st = P.decode_status(data)
        self.assertEqual((st.phase_name, st.mag_cal_seconds_left, st.i2c_errors,
                          st.mag_errors, st.loop_overruns, st.notify_drops),
                         ("mag_cal", 17, 3, 2**32 - 1, 99, 7))

    def test_unknown_phase_and_errors(self):
        self.assertEqual(P.decode_status(P.encode_status(phase=42)).phase_name, "unknown")
        with self.assertRaises(P.ProtocolError):
            P.decode_status(P.encode_status(phase=4)[:10])
        with self.assertRaises(P.ProtocolError):
            P.decode_status(P.encode_status(phase=4, version=9))


class CalibrationPacket(unittest.TestCase):
    def test_roundtrip(self):
        payload = P.encode_calibration(
            mount_yaw_deg=90.0,
            mount_pitch_deg=5.5,
            mount_roll_deg=-10.2,
            mag_offset_x=12.34,
            mag_offset_y=-5.67,
            mag_offset_z=45.89,
            mag_scale_x=0.98,
            mag_scale_y=1.02,
            mag_scale_z=1.00,
        )
        self.assertEqual(len(payload), 32)
        decoded = P.decode_calibration(payload)
        self.assertEqual(decoded.version, 1)
        self.assertAlmostEqual(decoded.mount_yaw_deg, 90.0, places=1)
        self.assertAlmostEqual(decoded.mount_pitch_deg, 5.5, places=1)
        self.assertAlmostEqual(decoded.mount_roll_deg, -10.2, places=1)
        self.assertAlmostEqual(decoded.mag_offset_x, 12.34, places=2)
        self.assertAlmostEqual(decoded.mag_offset_y, -5.67, places=2)
        self.assertAlmostEqual(decoded.mag_offset_z, 45.89, places=2)
        self.assertAlmostEqual(decoded.mag_scale_x, 0.98, places=2)
        self.assertAlmostEqual(decoded.mag_scale_y, 1.02, places=2)
        self.assertAlmostEqual(decoded.mag_scale_z, 1.00, places=2)

    def test_errors(self):
        with self.assertRaises(P.ProtocolError):
            P.decode_calibration(b"")
        with self.assertRaises(P.ProtocolError):
            P.decode_calibration(b"\x01" * 15)  # truncated
        with self.assertRaises(P.ProtocolError):
            P.decode_calibration(b"\x99" + b"\x00" * 31)  # invalid version


if __name__ == "__main__":
    unittest.main()

