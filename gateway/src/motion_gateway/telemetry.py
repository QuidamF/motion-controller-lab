"""Normalization and link statistics (no BLE, no WebSocket, no I/O).

The gateway never recomputes orientation; it only validates, tracks the
sequence, measures rate/jitter and converts packets to the JSON schema
documented in docs/websocket-protocol.md.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any, Optional

from .protocol import Flags, StatusPacket, TelemetryPacket

SCHEMA_VERSION = 1


# ---------------------------------------------------------------- Sequence
@dataclass
class SequenceResult:
    extended: int  # unwrapped (monotonic) sequence
    lost: int  # packets missing just before this one
    duplicate: bool
    reordered: bool  # older than the last accepted packet -> should be dropped
    reset: bool  # large backwards jump, treated as device restart


class SequenceTracker:
    """Unwraps the u16 sequence and classifies gaps.

    A backwards jump larger than `reset_threshold` (default 1000 packets,
    ~20 s at 50 Hz) is interpreted as a device restart, not reordering.
    """

    def __init__(self, reset_threshold: int = 1000) -> None:
        self.reset_threshold = reset_threshold
        self._last_raw: Optional[int] = None
        self._extended = 0
        self.received = 0
        self.lost = 0
        self.duplicates = 0
        self.reordered = 0
        self.resets = 0

    def reset(self) -> None:
        """Call on a new BLE connection: continuity across links is unknown."""
        self._last_raw = None

    def update(self, raw: int) -> SequenceResult:
        raw &= 0xFFFF
        if self._last_raw is None:
            self._last_raw = raw
            self._extended = raw
            self.received += 1
            return SequenceResult(self._extended, 0, False, False, False)

        delta = (raw - self._last_raw) & 0xFFFF  # 0..65535 forward distance
        if delta == 0:
            self.duplicates += 1
            return SequenceResult(self._extended, 0, True, False, False)
        if delta < 0x8000:  # forward
            lost = delta - 1
            self._extended += delta
            self._last_raw = raw
            self.received += 1
            self.lost += lost
            return SequenceResult(self._extended, lost, False, False, False)
        back = 0x10000 - delta
        if back > self.reset_threshold:
            self.resets += 1
            self._last_raw = raw
            self._extended += 1  # keep extended monotonic across restarts
            self.received += 1
            return SequenceResult(self._extended, 0, False, False, True)
        self.reordered += 1
        return SequenceResult(self._extended - back, 0, False, True, False)


# ---------------------------------------------------------------- Rate
class RateMeter:
    """Packets per second over a sliding window of host timestamps."""

    def __init__(self, window_s: float = 2.0) -> None:
        self.window_s = window_s
        self._times: deque[float] = deque()

    def add(self, t: float) -> None:
        self._times.append(t)
        self._trim(t)

    def _trim(self, now: float) -> None:
        while self._times and now - self._times[0] > self.window_s:
            self._times.popleft()

    def rate(self, now: Optional[float] = None) -> float:
        if now is not None:
            self._trim(now)
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / span if span > 0 else 0.0


# ---------------------------------------------------------------- Latency
class LatencyEstimator:
    """Observable (relative) latency.

    Device and host clocks are not synchronized, so absolute one-way latency
    is NOT measurable. We track offset = t_host_ms - t_device_ms; its minimum
    over a window approximates the fastest delivery, and
    (offset - min_offset) is the extra delay of each packet (jitter /
    queueing). Clock drift (~ppm) is negligible over the window.
    """

    def __init__(self, window: int = 500) -> None:
        self._offsets: deque[float] = deque(maxlen=window)

    def reset(self) -> None:
        self._offsets.clear()

    def add(self, host_time_s: float, device_ms: int) -> float:
        off = host_time_s * 1000.0 - device_ms
        self._offsets.append(off)
        return off - min(self._offsets)

    def stats(self) -> dict[str, Optional[float]]:
        if not self._offsets:
            return {"extra_delay_ms_mean": None, "extra_delay_ms_max": None}
        base = min(self._offsets)
        extras = [o - base for o in self._offsets]
        return {
            "extra_delay_ms_mean": round(sum(extras) / len(extras), 2),
            "extra_delay_ms_max": round(max(extras), 2),
        }


# ---------------------------------------------------------------- Normalizer
def _validity(flags: Flags) -> dict[str, bool]:
    return {
        "reference_captured": bool(flags & Flags.REFERENCE_CAPTURED),
        "euler_finite": bool(flags & Flags.EULER_FINITE),
        "quat_valid": bool(flags & Flags.QUAT_VALID),
        "mag_data_ready": bool(flags & Flags.MAG_DATA_READY),
        "mag_coverage_valid": bool(flags & Flags.MAG_COVERAGE_VALID),
        "mag_calibrated": bool(flags & Flags.MAG_CALIBRATED),
        "yaw_mag_valid": bool(flags & Flags.YAW_MAG_VALID),
    }


@dataclass
class Normalizer:
    device_id: str = "unknown"
    seq: SequenceTracker = field(default_factory=SequenceTracker)
    rate: RateMeter = field(default_factory=RateMeter)
    latency: LatencyEstimator = field(default_factory=LatencyEstimator)
    rejected: dict[str, int] = field(default_factory=dict)
    warnings: dict[str, int] = field(default_factory=dict)
    last_status: Optional[StatusPacket] = None

    def on_new_link(self) -> None:
        self.seq.reset()
        self.latency.reset()

    def count_rejected(self, kind: str) -> None:
        self.rejected[kind] = self.rejected.get(kind, 0) + 1

    def telemetry(self, pkt: TelemetryPacket, host_time_s: float) -> Optional[dict[str, Any]]:
        """Return the JSON-ready message, or None if the packet must be dropped."""
        sr = self.seq.update(pkt.sequence)
        if sr.duplicate or sr.reordered:
            self.count_rejected("duplicate" if sr.duplicate else "reordered")
            return None
        if sr.reset:
            self.latency.reset()
        for w in pkt.warnings:
            self.warnings[w] = self.warnings.get(w, 0) + 1
        self.rate.add(host_time_s)
        extra = self.latency.add(host_time_s, pkt.timestamp_ms)

        q = pkt.quaternion
        return {
            "type": "telemetry",
            "schema_version": SCHEMA_VERSION,
            "device_id": self.device_id,
            "sequence": sr.extended,
            "sequence_raw": pkt.sequence,
            "lost_before": sr.lost,
            "timestamp_ms": pkt.timestamp_ms,
            "host_time": round(host_time_s, 6),
            "extra_delay_ms": round(extra, 2),
            "orientation": {
                "roll_deg": pkt.roll_deg,
                "pitch_deg": pkt.pitch_deg,
                "yaw_rel_deg": pkt.yaw_rel_deg,
                "yaw_mag_rel_deg": pkt.yaw_mag_rel_deg,
            },
            "quaternion": None if q is None else {
                "w": round(q.w, 4), "x": round(q.x, 4), "y": round(q.y, 4), "z": round(q.z, 4),
            },
            "validity": _validity(pkt.flags),
            "warnings": list(pkt.warnings),
        }

    def stats(self, now: Optional[float] = None) -> dict[str, Any]:
        s = self.seq
        total = s.received + s.lost
        st = self.last_status
        return {
            "rate_hz": round(self.rate.rate(now), 2),
            "received": s.received,
            "lost": s.lost,
            "loss_ratio": round(s.lost / total, 5) if total else 0.0,
            "duplicates": s.duplicates,
            "reordered": s.reordered,
            "device_resets": s.resets,
            "rejected": dict(self.rejected),
            "warnings": dict(self.warnings),
            **self.latency.stats(),
            "device": None if st is None else {
                "phase": st.phase_name,
                "mag_cal_seconds_left": st.mag_cal_seconds_left,
                "i2c_errors": st.i2c_errors,
                "mag_errors": st.mag_errors,
                "loop_overruns": st.loop_overruns,
                "notify_drops": st.notify_drops,
            },
        }
