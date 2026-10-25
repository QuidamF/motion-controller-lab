"""Generate reference packets (hex) for contract validation without hardware.

Usage:  python tools/gen_vectors.py
Prints named test vectors; the firmware encoder (Phase 3) must reproduce
the same bytes for the same inputs.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from motion_gateway import protocol as P  # noqa: E402
from motion_gateway.protocol import Flags, Quaternion  # noqa: E402

OK = 0x7F
VECTORS = {
    "nominal_identity": dict(sequence=1234, timestamp_ms=45678, roll_deg=1.23, pitch_deg=-3.45,
                             yaw_rel_deg=15.67, yaw_mag_rel_deg=14.81,
                             quaternion=Quaternion(1, 0, 0, 0), flags=OK),
    "heading_invalid": dict(sequence=1, timestamp_ms=36000, roll_deg=0.0, pitch_deg=0.0,
                            yaw_rel_deg=0.0, yaw_mag_rel_deg=-999.0,
                            quaternion=Quaternion(1, 0, 0, 0),
                            flags=OK & ~(Flags.MAG_COVERAGE_VALID | Flags.MAG_CALIBRATED)),
    "extremes": dict(sequence=65535, timestamp_ms=0xFFFFFFFF, roll_deg=-180.0, pitch_deg=90.0,
                     yaw_rel_deg=180.0, yaw_mag_rel_deg=-180.0,
                     quaternion=Quaternion(0.5, 0.5, -0.5, 0.5), flags=OK),
    "euler_nan": dict(sequence=7, timestamp_ms=100, roll_deg=float("nan"), pitch_deg=0.0,
                      yaw_rel_deg=0.0, yaw_mag_rel_deg=None, quaternion=None, flags=OK),
}

if __name__ == "__main__":
    for name, kw in VECTORS.items():
        data = P.encode_telemetry(**kw)
        pkt = P.decode_telemetry(data)
        print(f"{name:18s} {data.hex()}  flags=0x{int(pkt.flags):02x} "
              f"r={pkt.roll_deg} p={pkt.pitch_deg} y={pkt.yaw_rel_deg} m={pkt.yaw_mag_rel_deg}")
    st = P.encode_status(phase=P.Phase.RUNNING, i2c_errors=3, mag_errors=1,
                         loop_overruns=12, notify_drops=2)
    print(f"{'status_running':18s} {st.hex()}")
