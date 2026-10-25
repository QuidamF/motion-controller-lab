# BLE protocol v1 — Motion Controller Lab

Reference implementation: [protocol.py](../gateway/src/motion_gateway/protocol.py).
Test vectors: `python gateway/tools/gen_vectors.py`.

## GATT

| Element | UUID | Properties |
|---|---|---|
| Service | `4d430001-8f3a-4c2b-9e71-2a5c6d7e8f90` | — |
| Telemetry | `4d430002-8f3a-4c2b-9e71-2a5c6d7e8f90` | Notify, 20 B, ~50 Hz |
| Status | `4d430003-8f3a-4c2b-9e71-2a5c6d7e8f90` | Read + Notify, 20 B, 1 Hz |

- Device name: `MCL-XXXX` (last 2 MAC bytes), configurable with `BLE_DEVICE_NAME`.
- Service UUID goes in the advertising data; the name goes in the scan response.
- Both packets are ≤ 20 bytes, so they fit the default ATT MTU (23). No MTU negotiation is required.

## Telemetry packet (20 bytes, little-endian)

| Off | Type | Field | Unit / scale | Invalid |
|---|---|---|---|---|
| 0 | u8 | protocol_version | = 1 | other → packet rejected |
| 1 | u8 | flags | see below | — |
| 2 | u16 | sequence | +1 per generated packet, wraps | — |
| 4 | u32 | timestamp_ms | device `millis()` at the sample | wraps after ~49.7 days |
| 8 | i16 | roll | 0.01°, [-180, 180] | `0x8000` |
| 10 | i16 | pitch | 0.01°, [-90, 90] | `0x8000` |
| 12 | i16 | yaw_rel | 0.01°, [-180, 180] | `0x8000` |
| 14 | i16 | yaw_mag_rel | 0.01°, [-180, 180] | `0x8000` |
| 16 | u32 | quat_packed | smallest-three | ignored if `QUAT_VALID`=0 |

### Flags

| Bit | Name | Meaning |
|---|---|---|
| 0 | REFERENCE_CAPTURED | relative reference captured (`referenceCaptured`) |
| 1 | EULER_FINITE | roll, pitch and yaw_rel are finite and within range |
| 2 | QUAT_VALID | absolute quaternion is finite with norm ≈ 1 |
| 3 | MAG_DATA_READY | at least one valid magnetometer sample has been read |
| 4 | MAG_COVERAGE_VALID | the calibration had enough per-axis coverage |
| 5 | MAG_CALIBRATED | hard/soft-iron calibration applied (equals bit 4 in firmware v5.1) |
| 6 | YAW_MAG_VALID | the heading in this packet is valid (`yawMagValid`) |
| 7 | reserved | must be 0; the receiver warns if it is set |

The flags report availability and finiteness only, **not** accuracy. The firmware has no filter-convergence indicator.

### Rules for invalid values

- `NaN`, `Inf`, out-of-range values and the firmware sentinel `-999` are encoded as `0x8000`, and the corresponding flag is cleared.
- If `EULER_FINITE`=0, the receiver discards all three Euler angles.
- If `YAW_MAG_VALID`=0, the heading is `None`/`null`, **never 0**.
- A flag that says valid while the field holds the sentinel produces the warning `*_flag_inconsistent` and the value is `None`.

### Quaternion (smallest-three)

- Bits 31-30: index (0=w, 1=x, 2=y, 3=z) of the largest-|·| component, which is omitted. The quaternion is negated so that this component is ≥ 0.
- Bits 29-20, 19-10, 9-0: the remaining components in w, x, y, z order. Each is a signed 10-bit value; ±511 ↔ ±1/√2.
- Reconstruction: omitted component = √(1 − Σ²).
- Measured error over 50,000 random quaternions: worst-case 0.2199° (theoretical bound < 0.28°, test `test_accuracy`); mean error 0.0845°; < 0.003 per component.
- It is the **absolute** quaternion from the Madgwick filter (`qw..qz` in the CSV), not the relative one.

## Status packet (20 bytes, little-endian)

| Off | Type | Field |
|---|---|---|
| 0 | u8 | version (=1) |
| 1 | u8 | phase: 0 boot, 1 gyro_cal, 2 mag_cal, 3 settling, 4 running, 255 fatal |
| 2 | u8 | mag_cal_seconds_left |
| 3 | u8 | reserved |
| 4 | u32 | i2c_errors |
| 8 | u32 | mag_errors |
| 12 | u32 | loop_overruns (IMU cycles with elapsed > 2× period) |
| 16 | u16 | notify_drops (notifications rejected by the BLE stack via onStatus) |
| 18 | u16 | reserved |

## Angle conventions

- Roll, pitch and yaw_rel are ZYX Euler angles of `q_rel = conj(q_ref) ⊗ q`, where the reference is captured at the end of `setup()`.
- Rotations about the sensor X, Y and Z axes follow the right-hand rule.
- yaw_rel proviene del cuaternión fusionado por el filtro Madgwick (9-DOF MARG completo cuando la calibración y cobertura magnética son válidas; fallback limpio a 6-DOF mediante watchdog de caducidad de 50 ms).
- yaw_mag_rel es el rumbo magnético con compensación de inclinación (*tilt-compensated magnetic heading*) relativo al primer rumbo válido. Provee una referencia directa de brújula para diagnóstico y contraste.
- Remapeo de ejes de hardware: Los ejes del magnetómetro AK8963 están alineados con la terna del cuerpo del MPU-9250 ($X_{body} = +Y_{mag}, Y_{body} = +X_{mag}, Z_{body} = -Z_{mag}$).

## Rate and loss

- The IMU runs at 200 Hz; one packet is generated every 4 valid samples (~50 Hz), always with the latest sample.
- If `notify()` fails, the packet is dropped without retry. The sequence number still advances, so the receiver sees the gap.
- On reconnection the gateway resets sequence continuity, so gaps between links are not counted as loss.
- A backwards jump of more than 1000 packets is treated as a device restart.
