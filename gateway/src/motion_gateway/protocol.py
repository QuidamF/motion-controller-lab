"""Motion Controller Lab - BLE binary protocol v1 (pure functions, no I/O).

Specification: docs/ble-protocol.md. All multi-byte fields are little-endian.

Telemetry packet (20 bytes):
    0  u8   protocol_version (=1)
    1  u8   flags
    2  u16  sequence (wraps at 65535)
    4  u32  timestamp_ms (device millis() of the sample)
    8  i16  roll        0.01 deg, INVALID_I16 if unavailable
    10 i16  pitch       0.01 deg
    12 i16  yaw_rel     0.01 deg
    14 i16  yaw_mag_rel 0.01 deg
    16 u32  quat_packed (smallest-three, see pack_quaternion)

Status packet (20 bytes):
    0 u8 version, 1 u8 phase, 2 u8 mag_cal_seconds_left, 3 u8 reserved,
    4 u32 i2c_errors, 8 u32 mag_errors, 12 u32 loop_overruns,
    16 u16 notify_drops, 18 u16 reserved
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import Optional

# ---------------------------------------------------------------- UUIDs
SERVICE_UUID = "4d430001-8f3a-4c2b-9e71-2a5c6d7e8f90"
TELEMETRY_CHAR_UUID = "4d430002-8f3a-4c2b-9e71-2a5c6d7e8f90"
STATUS_CHAR_UUID = "4d430003-8f3a-4c2b-9e71-2a5c6d7e8f90"
CALIBRATION_CHAR_UUID = "4d430004-8f3a-4c2b-9e71-2a5c6d7e8f90"

# ---------------------------------------------------------------- Constants
PROTOCOL_VERSION = 1
STATUS_VERSION = 1
CALIBRATION_VERSION = 1
TELEMETRY_SIZE = 20
STATUS_SIZE = 20
CALIBRATION_SIZE = 32
INVALID_I16 = -32768  # 0x8000
ANGLE_SCALE = 100.0  # LSB per degree (0.01 deg resolution)
MOUNT_ANGLE_SCALE = 10.0  # LSB per degree (0.1 deg resolution)

_TELEMETRY_STRUCT = struct.Struct("<BBHIhhhhI")
_STATUS_STRUCT = struct.Struct("<BBBBIIIHH")
_CALIBRATION_STRUCT = struct.Struct("<BBhhhffffff")
assert _TELEMETRY_STRUCT.size == TELEMETRY_SIZE
assert _STATUS_STRUCT.size == STATUS_SIZE
assert _CALIBRATION_STRUCT.size == CALIBRATION_SIZE

_Q_COMPONENT_MAX = 1.0 / math.sqrt(2.0)  # smallest-three bound
_Q_BITS = 10
_Q_QMAX = (1 << (_Q_BITS - 1)) - 1  # 511
_Q_NORM_TOLERANCE = 0.02

# Physical ranges per field (degrees), inclusive.
ANGLE_LIMITS = {
    "roll": 180.0,
    "pitch": 90.0,
    "yaw_rel": 180.0,
    "yaw_mag_rel": 180.0,
}


class Flags(IntFlag):
    REFERENCE_CAPTURED = 1 << 0
    EULER_FINITE = 1 << 1
    QUAT_VALID = 1 << 2
    MAG_DATA_READY = 1 << 3
    MAG_COVERAGE_VALID = 1 << 4
    MAG_CALIBRATED = 1 << 5
    YAW_MAG_VALID = 1 << 6
    RESERVED_7 = 1 << 7


class Phase(IntEnum):
    BOOT = 0
    GYRO_CAL = 1
    MAG_CAL = 2
    SETTLING = 3
    RUNNING = 4
    FATAL = 255


class ProtocolError(ValueError):
    """Packet cannot be decoded. `kind` is a stable machine-readable tag."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


# ---------------------------------------------------------------- Data types
@dataclass(frozen=True)
class Quaternion:
    w: float
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class TelemetryPacket:
    version: int
    flags: Flags
    sequence: int
    timestamp_ms: int
    roll_deg: Optional[float]
    pitch_deg: Optional[float]
    yaw_rel_deg: Optional[float]
    yaw_mag_rel_deg: Optional[float]
    quaternion: Optional[Quaternion]
    warnings: tuple[str, ...] = ()


@dataclass(frozen=True)
class StatusPacket:
    version: int
    phase: int
    phase_name: str
    mag_cal_seconds_left: int
    i2c_errors: int
    mag_errors: int
    loop_overruns: int
    notify_drops: int


# ---------------------------------------------------------------- Angles
def encode_angle(value: Optional[float], limit: float = 180.0) -> int:
    """Degrees -> i16 centidegrees. None / NaN / Inf / out of range -> INVALID_I16."""
    if value is None or not math.isfinite(value) or abs(value) > limit + 1e-6:
        return INVALID_I16
    raw = int(round(value * ANGLE_SCALE))
    return max(-32767, min(32767, raw))


def decode_angle(raw: int) -> Optional[float]:
    return None if raw == INVALID_I16 else raw / ANGLE_SCALE


# ---------------------------------------------------------------- Quaternion
def pack_quaternion(q: Optional[Quaternion]) -> Optional[int]:
    """Smallest-three packing into 32 bits.

    bits 31..30: index (0=w,1=x,2=y,3=z) of the omitted largest-|.| component
    bits 29..20, 19..10, 9..0: remaining components in w,x,y,z order,
    signed 10-bit, scaled so +-511 == +-1/sqrt(2). The quaternion is negated
    if needed so that the omitted component is >= 0 (q and -q are the same
    rotation). Returns None if q is missing, non-finite or not unit-norm.
    """
    if q is None:
        return None
    comps = [q.w, q.x, q.y, q.z]
    if not all(math.isfinite(c) for c in comps):
        return None
    norm = math.sqrt(sum(c * c for c in comps))
    if abs(norm - 1.0) > _Q_NORM_TOLERANCE:
        return None
    comps = [c / norm for c in comps]
    idx = max(range(4), key=lambda i: abs(comps[i]))
    if comps[idx] < 0:
        comps = [-c for c in comps]
    packed = idx << 30
    shift = 20
    for i, c in enumerate(comps):
        if i == idx:
            continue
        v = int(round(c / _Q_COMPONENT_MAX * _Q_QMAX))
        v = max(-_Q_QMAX, min(_Q_QMAX, v))
        packed |= (v & 0x3FF) << shift
        shift -= 10
    return packed


def unpack_quaternion(packed: int) -> Quaternion:
    idx = (packed >> 30) & 0x3
    small = []
    for shift in (20, 10, 0):
        v = (packed >> shift) & 0x3FF
        if v & 0x200:
            v -= 0x400
        small.append(v / _Q_QMAX * _Q_COMPONENT_MAX)
    largest = math.sqrt(max(0.0, 1.0 - sum(c * c for c in small)))
    comps = small[:idx] + [largest] + small[idx:]
    return Quaternion(*comps)


# ---------------------------------------------------------------- Telemetry
def encode_telemetry(
    *,
    sequence: int,
    timestamp_ms: int,
    roll_deg: Optional[float],
    pitch_deg: Optional[float],
    yaw_rel_deg: Optional[float],
    yaw_mag_rel_deg: Optional[float],
    quaternion: Optional[Quaternion],
    flags: int,
    version: int = PROTOCOL_VERSION,
) -> bytes:
    """Reference encoder (mirrors the firmware). Used by simulator and tests.

    Validity bits derived from data are enforced here exactly as the firmware
    does: EULER_FINITE is cleared if any Euler angle is unavailable,
    QUAT_VALID is cleared if the quaternion cannot be packed, and
    YAW_MAG_VALID is cleared if the heading is unavailable.
    """
    roll = encode_angle(roll_deg, ANGLE_LIMITS["roll"])
    pitch = encode_angle(pitch_deg, ANGLE_LIMITS["pitch"])
    yaw = encode_angle(yaw_rel_deg, ANGLE_LIMITS["yaw_rel"])
    mag = encode_angle(yaw_mag_rel_deg, ANGLE_LIMITS["yaw_mag_rel"])
    qp = pack_quaternion(quaternion)

    flags = int(flags) & 0xFF
    if INVALID_I16 in (roll, pitch, yaw):
        flags &= ~Flags.EULER_FINITE
    if qp is None:
        flags &= ~Flags.QUAT_VALID
        qp = 0
    if mag == INVALID_I16:
        flags &= ~Flags.YAW_MAG_VALID
    return _TELEMETRY_STRUCT.pack(
        version & 0xFF, flags & 0xFF, sequence & 0xFFFF, timestamp_ms & 0xFFFFFFFF,
        roll, pitch, yaw, mag, qp,
    )


def decode_telemetry(data: bytes) -> TelemetryPacket:
    """Decode and validate a telemetry notification.

    Raises ProtocolError for: wrong size ("truncated"/"oversized"), unknown
    version ("unknown_version"). Inconsistencies that do not make the packet
    unusable are reported in `warnings` and the affected value is set to None.
    """
    if len(data) < 1:
        raise ProtocolError("truncated", "empty packet")
    if data[0] != PROTOCOL_VERSION:
        raise ProtocolError("unknown_version", f"unsupported protocol_version {data[0]}")
    if len(data) < TELEMETRY_SIZE:
        raise ProtocolError("truncated", f"expected {TELEMETRY_SIZE} bytes, got {len(data)}")
    if len(data) > TELEMETRY_SIZE:
        raise ProtocolError("oversized", f"expected {TELEMETRY_SIZE} bytes, got {len(data)}")

    ver, flags_raw, seq, ts, r, p, y, m, qp = _TELEMETRY_STRUCT.unpack(data)
    flags = Flags(flags_raw)
    warnings: list[str] = []
    if flags & Flags.RESERVED_7:
        warnings.append("reserved_flag_set")

    def checked(raw: int, name: str) -> Optional[float]:
        val = decode_angle(raw)
        if val is not None and abs(val) > ANGLE_LIMITS[name] + 0.005:
            warnings.append(f"{name}_out_of_range")
            return None
        return val

    roll, pitch, yaw = checked(r, "roll"), checked(p, "pitch"), checked(y, "yaw_rel")
    mag = checked(m, "yaw_mag_rel")

    euler_ok = bool(flags & Flags.EULER_FINITE)
    if euler_ok and None in (roll, pitch, yaw):
        warnings.append("euler_flag_inconsistent")
    if not euler_ok:
        roll = pitch = yaw = None

    mag_ok = bool(flags & Flags.YAW_MAG_VALID)
    if mag_ok and mag is None:
        warnings.append("yaw_mag_flag_inconsistent")
    if not mag_ok:
        mag = None  # never expose an invalid heading as a number

    quat = unpack_quaternion(qp) if flags & Flags.QUAT_VALID else None

    return TelemetryPacket(
        version=ver, flags=flags, sequence=seq, timestamp_ms=ts,
        roll_deg=roll, pitch_deg=pitch, yaw_rel_deg=yaw, yaw_mag_rel_deg=mag,
        quaternion=quat, warnings=tuple(warnings),
    )


# ---------------------------------------------------------------- Status
def encode_status(
    *, phase: int, mag_cal_seconds_left: int = 0, i2c_errors: int = 0,
    mag_errors: int = 0, loop_overruns: int = 0, notify_drops: int = 0,
    version: int = STATUS_VERSION,
) -> bytes:
    return _STATUS_STRUCT.pack(
        version & 0xFF, phase & 0xFF, mag_cal_seconds_left & 0xFF, 0,
        i2c_errors & 0xFFFFFFFF, mag_errors & 0xFFFFFFFF,
        loop_overruns & 0xFFFFFFFF, notify_drops & 0xFFFF, 0,
    )


def decode_status(data: bytes) -> StatusPacket:
    if len(data) < 1:
        raise ProtocolError("truncated", "empty status packet")
    if data[0] != STATUS_VERSION:
        raise ProtocolError("unknown_version", f"unsupported status version {data[0]}")
    if len(data) != STATUS_SIZE:
        kind = "truncated" if len(data) < STATUS_SIZE else "oversized"
        raise ProtocolError(kind, f"expected {STATUS_SIZE} bytes, got {len(data)}")
    ver, phase, secs, _r0, i2c, mag, overruns, drops, _r1 = _STATUS_STRUCT.unpack(data)
    try:
        name = Phase(phase).name.lower()
    except ValueError:
        name = "unknown"
    return StatusPacket(ver, phase, name, secs, i2c, mag, overruns, drops)


# ---------------------------------------------------------------- Calibration
@dataclass(frozen=True)
class CalibrationPayload:
    version: int
    flags: int
    mount_yaw_deg: float
    mount_pitch_deg: float
    mount_roll_deg: float
    mag_offset_x: float
    mag_offset_y: float
    mag_offset_z: float
    mag_scale_x: float
    mag_scale_y: float
    mag_scale_z: float


def encode_calibration(
    *,
    mount_yaw_deg: float = 0.0,
    mount_pitch_deg: float = 0.0,
    mount_roll_deg: float = 0.0,
    mag_offset_x: float = 0.0,
    mag_offset_y: float = 0.0,
    mag_offset_z: float = 0.0,
    mag_scale_x: float = 1.0,
    mag_scale_y: float = 1.0,
    mag_scale_z: float = 1.0,
    flags: int = 0x07,  # bit 0: mag_valid, bit 1: mount_valid, bit 2: save_nvs
    version: int = CALIBRATION_VERSION,
) -> bytes:
    """Encode calibration and mounting angles into 32-byte binary BLE payload."""
    def enc_mount(deg: float) -> int:
        raw = int(round(deg * MOUNT_ANGLE_SCALE))
        return max(-32767, min(32767, raw))

    return _CALIBRATION_STRUCT.pack(
        version & 0xFF,
        flags & 0xFF,
        enc_mount(mount_yaw_deg),
        enc_mount(mount_pitch_deg),
        enc_mount(mount_roll_deg),
        float(mag_offset_x),
        float(mag_offset_y),
        float(mag_offset_z),
        float(mag_scale_x),
        float(mag_scale_y),
        float(mag_scale_z),
    )


def decode_calibration(data: bytes) -> CalibrationPayload:
    """Decode and validate a 32-byte calibration payload."""
    if len(data) < 1:
        raise ProtocolError("truncated", "empty calibration packet")
    if data[0] != CALIBRATION_VERSION:
        raise ProtocolError("unknown_version", f"unsupported calibration version {data[0]}")
    if len(data) != CALIBRATION_SIZE:
        kind = "truncated" if len(data) < CALIBRATION_SIZE else "oversized"
        raise ProtocolError(kind, f"expected {CALIBRATION_SIZE} bytes, got {len(data)}")

    ver, flags, my_raw, mp_raw, mr_raw, ox, oy, oz, sx, sy, sz = _CALIBRATION_STRUCT.unpack(data)
    return CalibrationPayload(
        version=ver,
        flags=flags,
        mount_yaw_deg=my_raw / MOUNT_ANGLE_SCALE,
        mount_pitch_deg=mp_raw / MOUNT_ANGLE_SCALE,
        mount_roll_deg=mr_raw / MOUNT_ANGLE_SCALE,
        mag_offset_x=ox,
        mag_offset_y=oy,
        mag_offset_z=oz,
        mag_scale_x=sx,
        mag_scale_y=sy,
        mag_scale_z=sz,
    )

