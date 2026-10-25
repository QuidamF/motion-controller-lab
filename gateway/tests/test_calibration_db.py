"""Unit tests for CalibrationDatabase (SQLite storage)."""

import os
import tempfile
import unittest

from _path import *
from motion_gateway.calibration_db import CalibrationDatabase


class TestCalibrationDatabase(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.db_path = os.path.join(self.temp_dir.name, "test_cal.db")
        self.db = CalibrationDatabase(db_path=self.db_path)

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def test_init_creates_table(self) -> None:
        self.assertTrue(os.path.exists(self.db_path))
        self.assertEqual(len(self.db.list_calibrations()), 0)

    def test_save_and_get_calibration(self) -> None:
        data = {
            "device_name": "MCL-TEST-1",
            "mag_offset_x": 12.34,
            "mag_offset_y": -5.67,
            "mag_offset_z": 45.89,
            "mag_scale_x": 0.98,
            "mag_scale_y": 1.02,
            "mag_scale_z": 1.00,
            "mag_coverage_pct": 94.5,
            "notes": "Desk calibration test",
        }
        saved = self.db.save_calibration("DEV-001", data)
        self.assertEqual(saved["device_id"], "DEV-001")
        self.assertAlmostEqual(saved["mag_offset_x"], 12.34)
        self.assertAlmostEqual(saved["mag_coverage_pct"], 94.5)

        retrieved = self.db.get_calibration("DEV-001")
        self.assertIsNotNone(retrieved)
        self.assertEqual(retrieved["device_name"], "MCL-TEST-1")
        self.assertAlmostEqual(retrieved["mag_offset_y"], -5.67)
        self.assertTrue(retrieved["is_valid"])

    def test_update_existing_device(self) -> None:
        self.db.save_calibration("DEV-002", {"mag_offset_x": 1.0, "mag_coverage_pct": 50.0})
        first = self.db.get_calibration("DEV-002")
        self.assertIsNotNone(first)
        self.assertAlmostEqual(first["mag_offset_x"], 1.0)

        # Update with better calibration
        self.db.save_calibration("DEV-002", {"mag_offset_x": 2.5, "mag_coverage_pct": 98.0})
        updated = self.db.get_calibration("DEV-002")
        self.assertIsNotNone(updated)
        self.assertAlmostEqual(updated["mag_offset_x"], 2.5)
        self.assertAlmostEqual(updated["mag_coverage_pct"], 98.0)
        self.assertEqual(updated["created_at"], first["created_at"])
        self.assertGreaterEqual(updated["updated_at"], first["updated_at"])

    def test_list_and_delete(self) -> None:
        self.db.save_calibration("DEV-A", {"mag_coverage_pct": 80.0})
        self.db.save_calibration("DEV-B", {"mag_coverage_pct": 90.0})
        devices = self.db.list_calibrations()
        self.assertEqual(len(devices), 2)

        deleted = self.db.delete_calibration("DEV-A")
        self.assertTrue(deleted)
        self.assertIsNone(self.db.get_calibration("DEV-A"))
        self.assertEqual(len(self.db.list_calibrations()), 1)

        # Delete non-existent
        self.assertFalse(self.db.delete_calibration("DEV-NONEXISTENT"))

    def test_mounting_rotation_persistence(self) -> None:
        data = {
            "device_name": "MCL-BREADBOARD-90",
            "mount_yaw_deg": 90.0,
            "mount_pitch_deg": 0.0,
            "mount_roll_deg": 0.0,
            "mag_offset_x": 10.0,
            "mag_coverage_pct": 95.0,
        }
        self.db.save_calibration("DEV-MOUNT", data)
        retrieved = self.db.get_calibration("DEV-MOUNT")
        self.assertIsNotNone(retrieved)
        self.assertAlmostEqual(retrieved["mount_yaw_deg"], 90.0)
        self.assertAlmostEqual(retrieved["mount_pitch_deg"], 0.0)
        self.assertAlmostEqual(retrieved["mount_roll_deg"], 0.0)



if __name__ == "__main__":
    unittest.main()
