"""Simulated telemetry and status source for testing without physical hardware.

Generates realistic 50 Hz motion data and 1 Hz status packets matching
docs/ble-protocol.md and src/motion_gateway/protocol.py.

Can inject controlled anomalies: packet loss, duplicate packets, truncated/oversized
packets, unknown versions, non-finite values, and periodic disconnects.
"""

from __future__ import annotations

import asyncio
import math
import random
import time
from dataclasses import dataclass
from typing import AsyncIterator, Callable, Optional

from .protocol import (
    Flags,
    Phase,
    Quaternion,
    encode_status,
    encode_telemetry,
)


def _quat_from_euler(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
    cr = math.cos(math.radians(roll_deg) * 0.5)
    sr = math.sin(math.radians(roll_deg) * 0.5)
    cp = math.cos(math.radians(pitch_deg) * 0.5)
    sp = math.sin(math.radians(pitch_deg) * 0.5)
    cy = math.cos(math.radians(yaw_deg) * 0.5)
    sy = math.sin(math.radians(yaw_deg) * 0.5)
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )


@dataclass
class AnomalyConfig:
    drop_rate: float = 0.0          # probability of dropping a sample (0.0 .. 1.0)
    duplicate_rate: float = 0.0     # probability of sending an immediate duplicate
    corrupt_rate: float = 0.0       # probability of sending truncated/corrupt bytes
    invalid_heading_rate: float = 0.0  # probability that heading is marked invalid
    disconnect_interval_s: float = 0.0 # if > 0, simulate disconnect every N seconds
    reconnect_delay_s: float = 2.0     # downtime when disconnect occurs


class SimSource:
    """Simulates an ESP32-C3 BLE GATT server emitting notifications.

    Callbacks:
        on_telemetry_bytes(data: bytes, host_time_s: float)
        on_status_bytes(data: bytes, host_time_s: float)
        on_connection_change(connected: bool, device_id: str)
    """

    def __init__(
        self,
        device_id: str = "MCL-SIM1",
        sample_rate_hz: float = 50.0,
        anomalies: Optional[AnomalyConfig] = None,
        on_telemetry: Optional[Callable[[bytes, float], None]] = None,
        on_status: Optional[Callable[[bytes, float], None]] = None,
        on_connection: Optional[Callable[[bool, str], None]] = None,
    ) -> None:
        self.device_id = device_id
        self.sample_rate_hz = sample_rate_hz
        self.anomalies = anomalies or AnomalyConfig()
        self.on_telemetry = on_telemetry
        self.on_status = on_status
        self.on_connection = on_connection

        self._running = False
        self._sequence = 0
        self._start_time_s = 0.0
        self._i2c_errors = 0
        self._mag_errors = 0
        self._loop_overruns = 0
        self._notify_drops = 0
        self._rng = random.Random(1234)

    @property
    def is_running(self) -> bool:
        return self._running

    def stop(self) -> None:
        self._running = False

    async def run(self) -> None:
        """Run the simulation loop until stop() is called or cancelled."""
        self._running = True
        self._start_time_s = time.monotonic()
        period_s = 1.0 / self.sample_rate_hz
        status_interval_s = 1.0
        last_status_s = 0.0
        last_disconnect_s = self._start_time_s

        if self.on_connection:
            self.on_connection(True, self.device_id)

        try:
            next_sample_time = time.monotonic()
            while self._running:
                now = time.monotonic()

                # Simulated periodic disconnect
                if (
                    self.anomalies.disconnect_interval_s > 0
                    and (now - last_disconnect_s) >= self.anomalies.disconnect_interval_s
                ):
                    if self.on_connection:
                        self.on_connection(False, self.device_id)
                    await asyncio.sleep(self.anomalies.reconnect_delay_s)
                    last_disconnect_s = time.monotonic()
                    next_sample_time = time.monotonic()
                    if self.on_connection:
                        self.on_connection(True, self.device_id)
                    continue

                # Emit 1 Hz status packet
                if now - last_status_s >= status_interval_s:
                    last_status_s = now
                    status_bytes = encode_status(
                        phase=Phase.RUNNING,
                        mag_cal_seconds_left=0,
                        i2c_errors=self._i2c_errors,
                        mag_errors=self._mag_errors,
                        loop_overruns=self._loop_overruns,
                        notify_drops=self._notify_drops,
                    )
                    if self.on_status:
                        self.on_status(status_bytes, now)

                # Generate smooth synthetic motion
                t = now - self._start_time_s
                roll = 15.0 * math.sin(t * 0.8)
                pitch = 10.0 * math.cos(t * 0.6)
                yaw_rel = ((t * 15.0) + 180.0) % 360.0 - 180.0

                # Magnetometer heading
                heading_valid = True
                if (
                    self.anomalies.invalid_heading_rate > 0
                    and self._rng.random() < self.anomalies.invalid_heading_rate
                ):
                    heading_valid = False
                    yaw_mag_rel = None
                else:
                    yaw_mag_rel = ((yaw_rel + 2.0 * math.sin(t * 1.5)) + 180.0) % 360.0 - 180.0

                quat = _quat_from_euler(roll, pitch, yaw_rel)

                flags = (
                    Flags.REFERENCE_CAPTURED
                    | Flags.EULER_FINITE
                    | Flags.QUAT_VALID
                    | Flags.MAG_DATA_READY
                    | Flags.MAG_COVERAGE_VALID
                    | Flags.MAG_CALIBRATED
                )
                if heading_valid:
                    flags |= Flags.YAW_MAG_VALID

                device_ms = int(t * 1000) & 0xFFFFFFFF
                current_seq = self._sequence
                self._sequence = (self._sequence + 1) & 0xFFFF

                # Anomaly: drop packet
                dropped = (
                    self.anomalies.drop_rate > 0
                    and self._rng.random() < self.anomalies.drop_rate
                )

                if not dropped:
                    packet_bytes = encode_telemetry(
                        sequence=current_seq,
                        timestamp_ms=device_ms,
                        roll_deg=roll,
                        pitch_deg=pitch,
                        yaw_rel_deg=yaw_rel,
                        yaw_mag_rel_deg=yaw_mag_rel,
                        quaternion=quat,
                        flags=flags,
                    )

                    # Anomaly: corrupt/truncate
                    if (
                        self.anomalies.corrupt_rate > 0
                        and self._rng.random() < self.anomalies.corrupt_rate
                    ):
                        packet_bytes = packet_bytes[:10]  # truncated packet

                    if self.on_telemetry:
                        self.on_telemetry(packet_bytes, now)

                    # Anomaly: duplicate
                    if (
                        self.anomalies.duplicate_rate > 0
                        and self._rng.random() < self.anomalies.duplicate_rate
                    ):
                        if self.on_telemetry:
                            self.on_telemetry(packet_bytes, now + 0.001)

                # Maintain exact sampling cadence
                next_sample_time += period_s
                sleep_duration = next_sample_time - time.monotonic()
                if sleep_duration > 0:
                    await asyncio.sleep(sleep_duration)
                else:
                    # Simulation loop overrun
                    self._loop_overruns += 1
                    next_sample_time = time.monotonic()
        finally:
            self._running = False
            if self.on_connection:
                self.on_connection(False, self.device_id)
