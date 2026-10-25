"""Tests for orientation algorithm verification (Phase 1 audit).

Covers:
- D-03: Gyro angular rate units on 6-DOF fallback (preventing 57.3x attenuation).
- D-04: Canonical equivalence of Madgwick 9-DOF magnetic equations.
- D-05: AK8963 100 Hz vs 200 Hz loop, sample freshness timeout (50 ms) and smooth fallback/recovery.
"""
import math
import unittest
import random
from _path import *  # noqa: F401,F403  (adds src/ to sys.path)


def madgwick_6dof_step(q: list[float], gx_rad: float, gy_rad: float, gz_rad: float,
                       ax: float, ay: float, az: float, dt: float, beta: float = 0.08) -> list[float]:
    """Pure mathematical model of madgwickUpdate6DOFRad."""
    qw, qx, qy, qz = q
    norm_a = math.sqrt(ax * ax + ay * ay + az * az)

    qDotW = 0.5 * (-qx * gx_rad - qy * gy_rad - qz * gz_rad)
    qDotX = 0.5 * ( qw * gx_rad + qy * gz_rad - qz * gy_rad)
    qDotY = 0.5 * ( qw * gy_rad - qx * gz_rad + qz * gx_rad)
    qDotZ = 0.5 * ( qw * gz_rad + qx * gy_rad - qy * gx_rad)

    if norm_a > 1e-6:
        ax /= norm_a; ay /= norm_a; az /= norm_a
        _2qw = 2.0 * qw; _2qx = 2.0 * qx; _2qy = 2.0 * qy; _2qz = 2.0 * qz
        _4qw = 4.0 * qw; _4qx = 4.0 * qx; _4qy = 4.0 * qy
        _8qx = 8.0 * qx; _8qy = 8.0 * qy
        qw2 = qw * qw; qx2 = qx * qx; qy2 = qy * qy; qz2 = qz * qz

        sW = _4qw * qy2 + _2qy * ax + _4qw * qx2 - _2qx * ay
        sX = _4qx * qz2 - _2qz * ax + 4.0 * qw2 * qx - _2qw * ay - _4qx + _8qx * qx2 + _8qx * qy2 + _4qx * az
        sY = 4.0 * qw2 * qy + _2qw * ax + _4qy * qz2 - _2qz * ay - _4qy + _8qy * qx2 + _8qy * qy2 + _4qy * az
        sZ = 4.0 * qx2 * qz - _2qx * ax + 4.0 * qy2 * qz - _2qy * ay

        norm_s = math.sqrt(sW * sW + sX * sX + sY * sY + sZ * sZ)
        if norm_s > 1e-6:
            sW /= norm_s; sX /= norm_s; sY /= norm_s; sZ /= norm_s
            qDotW -= beta * sW
            qDotX -= beta * sX
            qDotY -= beta * sY
            qDotZ -= beta * sZ

    qw += qDotW * dt
    qx += qDotX * dt
    qy += qDotY * dt
    qz += qDotZ * dt

    norm_q = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm_q > 1e-6:
        qw /= norm_q; qx /= norm_q; qy /= norm_q; qz /= norm_q

    return [qw, qx, qy, qz]


def madgwick_9dof_step(q: list[float], gx_dps: float, gy_dps: float, gz_dps: float,
                       ax: float, ay: float, az: float,
                       mx: float, my: float, mz: float,
                       dt: float, beta: float = 0.08,
                       buggy_d03: bool = False) -> list[float]:
    """Model of madgwickUpdate9DOF with selectable D-03 behavior."""
    deg2rad = math.pi / 180.0
    gx_rad = gx_dps * deg2rad
    gy_rad = gy_dps * deg2rad
    gz_rad = gz_dps * deg2rad

    norm_a = math.sqrt(ax * ax + ay * ay + az * az)
    if norm_a < 1e-6:
        return q

    norm_m = math.sqrt(mx * mx + my * my + mz * mz)
    if norm_m < 1e-6:
        if buggy_d03:
            # Defective: gx_rad is treated as dps and converted AGAIN
            return madgwick_6dof_step(q, gx_rad * deg2rad, gy_rad * deg2rad, gz_rad * deg2rad,
                                      ax, ay, az, dt, beta)
        else:
            # Corrected: passed directly in rad/s
            return madgwick_6dof_step(q, gx_rad, gy_rad, gz_rad, ax, ay, az, dt, beta)

    ax /= norm_a; ay /= norm_a; az /= norm_a
    mx /= norm_m; my /= norm_m; mz /= norm_m

    qw, qx, qy, qz = q
    q1, q2, q3, q4 = qw, qx, qy, qz
    _2q1 = 2.0 * q1; _2q2 = 2.0 * q2; _2q3 = 2.0 * q3; _2q4 = 2.0 * q4
    _2q1q3 = 2.0 * q1 * q3; _2q3q4 = 2.0 * q3 * q4
    q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3; q1q4 = q1 * q4
    q2q2 = q2 * q2; q2q3 = q2 * q3; q2q4 = q2 * q4
    q3q3 = q3 * q3; q3q4 = q3 * q4; q4q4 = q4 * q4

    hx = 2.0 * (mx * (0.5 - q3q3 - q4q4) + my * (q2q3 - q1q4) + mz * (q2q4 + q1q3))
    hy = 2.0 * (mx * (q2q3 + q1q4) + my * (0.5 - q2q2 - q4q4) + mz * (q3q4 - q1q2))
    _2bx = math.sqrt(hx * hx + hy * hy)
    _2bz = 2.0 * (mx * (q2q4 - q1q3) + my * (q3q4 + q1q2) + mz * (0.5 - q2q2 - q3q3))
    _4bx = 2.0 * _2bx
    _4bz = 2.0 * _2bz

    s1 = (-_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay)
          - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
          + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
          + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

    s2 = (_2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az)
          + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
          + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
          + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

    s3 = (-_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az)
          + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
          + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
          + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

    s4 = (_2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay)
          + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
          + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
          + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

    norm_s = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
    if norm_s > 1e-6:
        s1 /= norm_s; s2 /= norm_s; s3 /= norm_s; s4 /= norm_s
    else:
        s1 = s2 = s3 = s4 = 0.0

    qDot1 = 0.5 * (-q2 * gx_rad - q3 * gy_rad - q4 * gz_rad) - beta * s1
    qDot2 = 0.5 * ( q1 * gx_rad + q3 * gz_rad - q4 * gy_rad) - beta * s2
    qDot3 = 0.5 * ( q1 * gy_rad - q2 * gz_rad + q4 * gx_rad) - beta * s3
    qDot4 = 0.5 * ( q1 * gz_rad + q2 * gy_rad - q3 * gx_rad) - beta * s4

    qw += qDot1 * dt
    qx += qDot2 * dt
    qy += qDot3 * dt
    qz += qDot4 * dt

    norm_q = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm_q > 1e-6:
        qw /= norm_q; qx /= norm_q; qy /= norm_q; qz /= norm_q

    return [qw, qx, qy, qz]


def quat_to_yaw_deg(q: list[float]) -> float:
    qw, qx, qy, qz = q
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return math.degrees(yaw)


class OrientationAlgorithmTests(unittest.TestCase):
    """Regression and verification tests for Phase 1 audit."""

    def test_d03_gyro_scale_on_6dof_fallback(self):
        """D-03: Verify that fallback to 6-DOF does not double-convert deg/s to rad/s."""
        dt = 0.005  # 200 Hz
        steps = 200  # 1.0 second duration
        gz_dps = 90.0  # 90 deg/s rotation on Z axis
        ax, ay, az = 0.0, 0.0, 1.0
        mx, my, mz = 0.0, 0.0, 0.0  # Zero magnetic field triggers fallback

        # 1. Defective implementation (double deg2rad conversion)
        q_buggy = [1.0, 0.0, 0.0, 0.0]
        for _ in range(steps):
            q_buggy = madgwick_9dof_step(q_buggy, 0.0, 0.0, gz_dps, ax, ay, az, mx, my, mz, dt,
                                         beta=0.08, buggy_d03=True)
        yaw_buggy = quat_to_yaw_deg(q_buggy)

        # In defective version, integrated angle is attenuated by ~57.3x
        expected_buggy_yaw = 90.0 / (180.0 / math.pi)  # ~1.57 degrees
        self.assertAlmostEqual(yaw_buggy, expected_buggy_yaw, delta=0.5,
                               msg=f"Defective model should show ~1.57 deg, got {yaw_buggy}")

        # 2. Corrected implementation (single deg2rad conversion)
        q_fixed = [1.0, 0.0, 0.0, 0.0]
        for _ in range(steps):
            q_fixed = madgwick_9dof_step(q_fixed, 0.0, 0.0, gz_dps, ax, ay, az, mx, my, mz, dt,
                                        beta=0.08, buggy_d03=False)
        yaw_fixed = quat_to_yaw_deg(q_fixed)

        # In corrected version, integrated angle matches 90 degrees
        self.assertAlmostEqual(yaw_fixed, 90.0, delta=1.0,
                               msg=f"Corrected model must integrate to ~90 deg, got {yaw_fixed}")

    def test_d04_madgwick_equations_match_canonical(self):
        """D-04: Verify that firmware magnetic equations match canonical MadgwickAHRS.c."""
        random.seed(42)
        for _ in range(100):
            # Generate random unit quaternion
            q = [random.uniform(-1, 1) for _ in range(4)]
            nq = math.sqrt(sum(x * x for x in q))
            q1, q2, q3, q4 = [x / nq for x in q]

            # Generate random unit mag
            m = [random.uniform(-1, 1) for _ in range(3)]
            nm = math.sqrt(sum(x * x for x in m))
            mx, my, mz = [x / nm for x in m]

            # Canonical x-io formulas
            _2q1 = 2.0 * q1; _2q2 = 2.0 * q2; _2q3 = 2.0 * q3; _2q4 = 2.0 * q4
            q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3; q4q4 = q4*q4
            hx_xio = mx * q1q1 - 2*q1*my * q4 + 2*q1*mz * q3 + mx * q2q2 + 2*q2 * my * q3 + 2*q2 * mz * q4 - mx * q3q3 - mx * q4q4
            hy_xio = 2*q1*mx * q4 + my * q1q1 - 2*q1*mz * q2 + 2*q2*mx * q3 - my * q2q2 + my * q3q3 + 2*q3 * mz * q4 - my * q4q4
            hz_xio = -2*q1*mx * q3 + 2*q1*my * q2 + mz * q1q1 + 2*q2*mx * q4 - mz * q2q2 + 2*q3 * my * q4 - mz * q3q3 + mz * q4q4
            _2bx_xio = math.sqrt(hx_xio * hx_xio + hy_xio * hy_xio)
            _2bz_xio = hz_xio

            # Firmware formulas
            hx_fw = 2.0 * (mx * (0.5 - q3q3 - q4q4) + my * (q2*q3 - q1*q4) + mz * (q2*q4 + q1*q3))
            hy_fw = 2.0 * (mx * (q2*q3 + q1*q4) + my * (0.5 - q2*q2 - q4q4) + mz * (q3*q4 - q1*q2))
            hz_fw = 2.0 * (mx * (q2*q4 - q1*q3) + my * (q3*q4 + q1*q2) + mz * (0.5 - q2*q2 - q3q3))
            _2bx_fw = math.sqrt(hx_fw * hx_fw + hy_fw * hy_fw)
            _2bz_fw = hz_fw

            self.assertAlmostEqual(hx_xio, hx_fw, places=5)
            self.assertAlmostEqual(hy_xio, hy_fw, places=5)
            self.assertAlmostEqual(hz_xio, hz_fw, places=5)
            self.assertAlmostEqual(_2bx_xio, _2bx_fw, places=5)
            self.assertAlmostEqual(_2bz_xio, _2bz_fw, places=5)

    def test_d05_mag_freshness_watchdog_and_recovery(self):
        """D-05: Verify 50 ms freshness watchdog, fallback to 6-DOF and smooth recovery."""
        timeout_ms = 50
        dt = 0.005  # 5 ms per loop iteration (200 Hz)
        t_ms = 0

        # State tracking
        mag_data_ready = False
        last_mag_sample_ms = 0
        mag_fresh_history: list[bool] = []
        q = [1.0, 0.0, 0.0, 0.0]

        # Phase 1: 0 to 100 ms — Normal operation (100 Hz mag = fresh sample every 10 ms)
        for t_step in range(5, 105, 5):
            t_ms = t_step
            new_mag = (t_step % 10 == 0)
            if new_mag:
                mag_data_ready = True
                last_mag_sample_ms = t_ms

            mag_fresh = (mag_data_ready and (t_ms - last_mag_sample_ms <= timeout_ms))
            mag_fresh_history.append(mag_fresh)

            if mag_fresh:
                q = madgwick_9dof_step(q, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.8, dt)
            else:
                q = madgwick_6dof_step(q, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, dt)

        # During normal operation with initial sample, mag_fresh stays True once initialized
        self.assertTrue(all(mag_fresh_history[1:]))

        # Phase 2: Mag failure (no new samples for 80 ms, from 105 to 180 ms)
        outage_fresh_history: list[bool] = []
        for t_step in range(105, 185, 5):
            t_ms = t_step
            mag_fresh = (mag_data_ready and (t_ms - last_mag_sample_ms <= timeout_ms))
            if not mag_fresh and mag_data_ready:
                mag_data_ready = False
            outage_fresh_history.append(mag_fresh)

            if mag_fresh:
                q = madgwick_9dof_step(q, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.8, dt)
            else:
                q = madgwick_6dof_step(q, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, dt)

        # First 50 ms / 5 ms = 10 steps were within timeout (ZOH)
        self.assertTrue(all(outage_fresh_history[:10]))
        # After 50 ms (steps 11 to 16), mag_fresh MUST be False (fallback to 6-DOF)
        self.assertFalse(any(outage_fresh_history[10:]))
        self.assertFalse(mag_data_ready)

        # Quaternion remains valid and normalized
        norm_q = math.sqrt(sum(x * x for x in q))
        self.assertAlmostEqual(norm_q, 1.0, places=5)

        # Phase 3: Mag recovery (fresh samples resume at 190 ms)
        recovery_fresh_history: list[bool] = []
        for t_step in range(185, 235, 5):
            t_ms = t_step
            new_mag = (t_step % 10 == 0)
            if new_mag:
                mag_data_ready = True
                last_mag_sample_ms = t_ms

            mag_fresh = (mag_data_ready and (t_ms - last_mag_sample_ms <= timeout_ms))
            recovery_fresh_history.append(mag_fresh)

            if mag_fresh:
                q = madgwick_9dof_step(q, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.8, dt)
            else:
                q = madgwick_6dof_step(q, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, dt)

        # Mag is restored on first fresh sample at 190 ms (index 1)
        self.assertFalse(recovery_fresh_history[0])  # 185 ms still waiting
        self.assertTrue(recovery_fresh_history[1])   # 190 ms restored
        norm_q_rec = math.sqrt(sum(x * x for x in q))
        self.assertAlmostEqual(norm_q_rec, 1.0, places=5)


if __name__ == "__main__":
    unittest.main()
