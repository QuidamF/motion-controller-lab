"""SQLite-based persistent calibration database for Motion Controller Lab.

Stores hard-iron/soft-iron magnetic calibration, gyro bias, and device profiles
indexed by device_id (BLE MAC address or unique name).
Allows peripherals to be calibrated once via a graphical wizard, persisting across restarts.
"""

from __future__ import annotations

import datetime
import logging
import os
import sqlite3
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from typing import Any, Dict, Generator, List, Optional

logger = logging.getLogger("motion_gateway.calibration")

DEFAULT_DB_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "data",
    "calibration.db",
)


@dataclass
class DeviceCalibration:
    device_id: str
    device_name: str = ""
    created_at: str = ""
    updated_at: str = ""
    mag_offset_x: float = 0.0
    mag_offset_y: float = 0.0
    mag_offset_z: float = 0.0
    mag_scale_x: float = 1.0
    mag_scale_y: float = 1.0
    mag_scale_z: float = 1.0
    mag_span_x: float = 0.0
    mag_span_y: float = 0.0
    mag_span_z: float = 0.0
    mag_coverage_pct: float = 0.0
    gyro_bias_x: float = 0.0
    gyro_bias_y: float = 0.0
    gyro_bias_z: float = 0.0
    yaw_offset_deg: float = 0.0
    mount_yaw_deg: float = 0.0
    mount_pitch_deg: float = 0.0
    mount_roll_deg: float = 0.0
    is_valid: bool = True
    notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class CalibrationDatabase:
    """Manages SQLite storage for device calibrations."""

    def __init__(self, db_path: str = DEFAULT_DB_PATH) -> None:
        self.db_path = db_path
        self._ensure_dir()
        self._init_db()

    def _ensure_dir(self) -> None:
        db_dir = os.path.dirname(self.db_path)
        if db_dir and not os.path.exists(db_dir):
            os.makedirs(db_dir, exist_ok=True)

    @contextmanager
    def _connection(self) -> Generator[sqlite3.Connection, None, None]:
        conn = sqlite3.connect(self.db_path, timeout=10.0)
        conn.row_factory = sqlite3.Row
        try:
            yield conn
        finally:
            conn.close()

    def _init_db(self) -> None:
        with self._connection() as conn:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS device_calibrations (
                    device_id TEXT PRIMARY KEY,
                    device_name TEXT,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    mag_offset_x REAL DEFAULT 0.0,
                    mag_offset_y REAL DEFAULT 0.0,
                    mag_offset_z REAL DEFAULT 0.0,
                    mag_scale_x REAL DEFAULT 1.0,
                    mag_scale_y REAL DEFAULT 1.0,
                    mag_scale_z REAL DEFAULT 1.0,
                    mag_span_x REAL DEFAULT 0.0,
                    mag_span_y REAL DEFAULT 0.0,
                    mag_span_z REAL DEFAULT 0.0,
                    mag_coverage_pct REAL DEFAULT 0.0,
                    gyro_bias_x REAL DEFAULT 0.0,
                    gyro_bias_y REAL DEFAULT 0.0,
                    gyro_bias_z REAL DEFAULT 0.0,
                    yaw_offset_deg REAL DEFAULT 0.0,
                    mount_yaw_deg REAL DEFAULT 0.0,
                    mount_pitch_deg REAL DEFAULT 0.0,
                    mount_roll_deg REAL DEFAULT 0.0,
                    is_valid INTEGER DEFAULT 1,
                    notes TEXT DEFAULT ''
                );
                """
            )
            # Automatic column migration for existing tables
            cur = conn.cursor()
            cur.execute("PRAGMA table_info(device_calibrations)")
            existing_cols = {row["name"] for row in cur.fetchall()}
            for col in ("mount_yaw_deg", "mount_pitch_deg", "mount_roll_deg"):
                if col not in existing_cols:
                    conn.execute(f"ALTER TABLE device_calibrations ADD COLUMN {col} REAL DEFAULT 0.0;")
            conn.commit()
            logger.debug("Calibration database initialized at %s", self.db_path)

    def get_calibration(self, device_id: str) -> Optional[Dict[str, Any]]:
        """Retrieve stored calibration profile for device_id, or None if not found."""
        if not device_id:
            return None
        with self._connection() as conn:
            cur = conn.cursor()
            cur.execute(
                "SELECT * FROM device_calibrations WHERE device_id = ? OR device_name = ?",
                (device_id, device_id),
            )
            row = cur.fetchone()
            if row is None:
                return None
            res = dict(row)
            res["is_valid"] = bool(res["is_valid"])
            return res

    def save_calibration(self, device_id: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """Insert or update a calibration profile for device_id."""
        now = datetime.datetime.now(datetime.timezone.utc).isoformat()
        existing = self.get_calibration(device_id)

        created_at = existing["created_at"] if existing else now
        updated_at = now
        device_name = data.get("device_name", existing["device_name"] if existing else device_id)
        notes = data.get("notes", existing["notes"] if existing else "")

        cal = DeviceCalibration(
            device_id=device_id,
            device_name=device_name,
            created_at=created_at,
            updated_at=updated_at,
            mag_offset_x=float(data.get("mag_offset_x", 0.0)),
            mag_offset_y=float(data.get("mag_offset_y", 0.0)),
            mag_offset_z=float(data.get("mag_offset_z", 0.0)),
            mag_scale_x=float(data.get("mag_scale_x", 1.0)),
            mag_scale_y=float(data.get("mag_scale_y", 1.0)),
            mag_scale_z=float(data.get("mag_scale_z", 1.0)),
            mag_span_x=float(data.get("mag_span_x", 0.0)),
            mag_span_y=float(data.get("mag_span_y", 0.0)),
            mag_span_z=float(data.get("mag_span_z", 0.0)),
            mag_coverage_pct=float(data.get("mag_coverage_pct", 0.0)),
            gyro_bias_x=float(data.get("gyro_bias_x", 0.0)),
            gyro_bias_y=float(data.get("gyro_bias_y", 0.0)),
            gyro_bias_z=float(data.get("gyro_bias_z", 0.0)),
            yaw_offset_deg=float(data.get("yaw_offset_deg", 0.0)),
            mount_yaw_deg=float(data.get("mount_yaw_deg", 0.0)),
            mount_pitch_deg=float(data.get("mount_pitch_deg", 0.0)),
            mount_roll_deg=float(data.get("mount_roll_deg", 0.0)),
            is_valid=bool(data.get("is_valid", True)),
            notes=notes,
        )

        with self._connection() as conn:
            conn.execute(
                """
                INSERT INTO device_calibrations (
                    device_id, device_name, created_at, updated_at,
                    mag_offset_x, mag_offset_y, mag_offset_z,
                    mag_scale_x, mag_scale_y, mag_scale_z,
                    mag_span_x, mag_span_y, mag_span_z,
                    mag_coverage_pct,
                    gyro_bias_x, gyro_bias_y, gyro_bias_z,
                    yaw_offset_deg, mount_yaw_deg, mount_pitch_deg, mount_roll_deg,
                    is_valid, notes
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(device_id) DO UPDATE SET
                    device_name=excluded.device_name,
                    updated_at=excluded.updated_at,
                    mag_offset_x=excluded.mag_offset_x,
                    mag_offset_y=excluded.mag_offset_y,
                    mag_offset_z=excluded.mag_offset_z,
                    mag_scale_x=excluded.mag_scale_x,
                    mag_scale_y=excluded.mag_scale_y,
                    mag_scale_z=excluded.mag_scale_z,
                    mag_span_x=excluded.mag_span_x,
                    mag_span_y=excluded.mag_span_y,
                    mag_span_z=excluded.mag_span_z,
                    mag_coverage_pct=excluded.mag_coverage_pct,
                    gyro_bias_x=excluded.gyro_bias_x,
                    gyro_bias_y=excluded.gyro_bias_y,
                    gyro_bias_z=excluded.gyro_bias_z,
                    yaw_offset_deg=excluded.yaw_offset_deg,
                    mount_yaw_deg=excluded.mount_yaw_deg,
                    mount_pitch_deg=excluded.mount_pitch_deg,
                    mount_roll_deg=excluded.mount_roll_deg,
                    is_valid=excluded.is_valid,
                    notes=excluded.notes;
                """,
                (
                    cal.device_id,
                    cal.device_name,
                    cal.created_at,
                    cal.updated_at,
                    cal.mag_offset_x,
                    cal.mag_offset_y,
                    cal.mag_offset_z,
                    cal.mag_scale_x,
                    cal.mag_scale_y,
                    cal.mag_scale_z,
                    cal.mag_span_x,
                    cal.mag_span_y,
                    cal.mag_span_z,
                    cal.mag_coverage_pct,
                    cal.gyro_bias_x,
                    cal.gyro_bias_y,
                    cal.gyro_bias_z,
                    cal.yaw_offset_deg,
                    cal.mount_yaw_deg,
                    cal.mount_pitch_deg,
                    cal.mount_roll_deg,
                    1 if cal.is_valid else 0,
                    cal.notes,
                ),
            )
            conn.commit()

        logger.info("Saved calibration for device '%s' (Coverage: %.1f%%)", device_id, cal.mag_coverage_pct)
        return cal.to_dict()

    def list_calibrations(self) -> List[Dict[str, Any]]:
        """List all calibrated devices stored in the database."""
        with self._connection() as conn:
            cur = conn.cursor()
            cur.execute("SELECT * FROM device_calibrations ORDER BY updated_at DESC")
            rows = cur.fetchall()
            result = []
            for r in rows:
                d = dict(r)
                d["is_valid"] = bool(d["is_valid"])
                result.append(d)
            return result

    def delete_calibration(self, device_id: str) -> bool:
        """Delete a calibration profile by device_id."""
        with self._connection() as conn:
            cur = conn.cursor()
            cur.execute("DELETE FROM device_calibrations WHERE device_id = ?", (device_id,))
            conn.commit()
            deleted = cur.rowcount > 0
            if deleted:
                logger.info("Deleted calibration for device '%s'", device_id)
            return deleted
