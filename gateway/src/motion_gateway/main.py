"""Main entry point for Motion Controller Lab Gateway.

Orchestrates BLE/Sim source -> Normalizer -> WebSocket server -> Local subscribers.
Handles clean shutdown on Ctrl+C.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
import time
from typing import Optional

from .ble_client import BleClient
from .calibration_db import CalibrationDatabase
from .protocol import (
    ProtocolError,
    decode_status,
    decode_telemetry,
    encode_calibration,
)
from .sim_source import AnomalyConfig, SimSource
from .telemetry import Normalizer
from .websocket_server import WebSocketServer

logger = logging.getLogger("motion_gateway")


class GatewayOrchestrator:
    def __init__(
        self,
        sim_mode: bool = False,
        name: Optional[str] = None,
        address: Optional[str] = None,
        ws_host: str = "127.0.0.1",
        ws_port: int = 8765,
        device_id: str = "MCL-0001",
        anomaly_config: Optional[AnomalyConfig] = None,
    ) -> None:
        self.sim_mode = sim_mode
        self.device_id = device_id
        self.normalizer = Normalizer(device_id=device_id)
        self.cal_db = CalibrationDatabase()
        self.ws_server = WebSocketServer(
            host=ws_host,
            port=ws_port,
            on_client_message=self._on_client_command,
        )
        self.source: Optional[SimSource | BleClient] = None
        self.connection_status = "disconnected"
        self._running = False

        if sim_mode:
            self.source = SimSource(
                device_id=device_id,
                anomalies=anomaly_config or AnomalyConfig(),
                on_telemetry=self._on_telemetry_bytes,
                on_status=self._on_status_bytes,
                on_connection=self._on_connection_change,
            )
        else:
            self.source = BleClient(
                name=name,
                address=address,
                on_telemetry=self._on_telemetry_bytes,
                on_status=self._on_status_bytes,
                on_connection=self._on_connection_change,
            )

    def _on_connection_change(self, connected: bool, device_id: str) -> None:
        self.connection_status = "connected" if connected else "disconnected"
        if connected:
            self.device_id = device_id
            self.normalizer.device_id = device_id
            self.normalizer.on_new_link()
            logger.info("Gateway: Device connected: %s", device_id)

            # Check persistent calibration profile in SQLite
            cal = self.cal_db.get_calibration(device_id)
            if cal:
                logger.info(
                    "Gateway: Stored calibration profile loaded for %s (Coverage: %.1f%%)",
                    device_id,
                    cal.get("mag_coverage_pct", 0.0),
                )
                # Auto-inject calibration over BLE characteristic
                asyncio.create_task(self.inject_calibration_to_device(cal))
            else:
                logger.info(
                    "Gateway: Device %s has NO stored calibration in SQLite. Calibration recommended.",
                    device_id,
                )
        else:
            logger.info("Gateway: Device disconnected: %s", device_id)

        # Broadcast connection state update via WebSocket
        self.broadcast_gateway_status()

    async def inject_calibration_to_device(self, cal: dict) -> bool:
        """Pack and send calibration data over BLE characteristic to ESP32."""
        if not cal:
            return False
        try:
            payload = encode_calibration(
                mount_yaw_deg=float(cal.get("mount_yaw_deg", 0.0)),
                mount_pitch_deg=float(cal.get("mount_pitch_deg", 0.0)),
                mount_roll_deg=float(cal.get("mount_roll_deg", 0.0)),
                mag_offset_x=float(cal.get("mag_offset_x", 0.0)),
                mag_offset_y=float(cal.get("mag_offset_y", 0.0)),
                mag_offset_z=float(cal.get("mag_offset_z", 0.0)),
                mag_scale_x=float(cal.get("mag_scale_x", 1.0)),
                mag_scale_y=float(cal.get("mag_scale_y", 1.0)),
                mag_scale_z=float(cal.get("mag_scale_z", 1.0)),
            )
            if hasattr(self.source, "write_calibration"):
                success = await self.source.write_calibration(payload)
                if success:
                    logger.info("Auto-injected calibration to device %s via BLE", self.device_id)
                return success
        except Exception as exc:
            logger.error("Failed to inject calibration: %s", exc)
        return False


    def _on_telemetry_bytes(self, data: bytes, host_time_s: float) -> None:
        try:
            packet = decode_telemetry(data)
        except ProtocolError as exc:
            self.normalizer.count_rejected(exc.kind)
            logger.debug("Rejected telemetry packet: %s (%s)", exc.kind, exc)
            return

        normalized = self.normalizer.telemetry(packet, host_time_s)
        if normalized is not None:
            self.ws_server.broadcast_telemetry(normalized)

    def _on_status_bytes(self, data: bytes, host_time_s: float) -> None:
        try:
            st = decode_status(data)
            self.normalizer.last_status = st
            self.broadcast_gateway_status()
        except ProtocolError as exc:
            self.normalizer.count_rejected(exc.kind)
            logger.debug("Rejected status packet: %s (%s)", exc.kind, exc)

    def broadcast_gateway_status(self) -> None:
        now = time.monotonic()
        stats = self.normalizer.stats(now=now)
        cal = self.cal_db.get_calibration(self.device_id)
        status_msg = {
            "type": "status",
            "schema_version": 1,
            "device_id": self.device_id,
            "gateway": {
                "connection": self.connection_status,
                "mode": "simulated" if self.sim_mode else "ble",
                "ws_clients": self.ws_server.client_count,
            },
            "calibration": {
                "is_calibrated": cal is not None,
                "profile": cal,
            },
            "stats": stats,
            "timestamp": round(time.time(), 3),
        }
        self.ws_server.broadcast_status(status_msg)

    async def _on_client_command(self, cmd_dict: dict, websocket) -> None:
        """Handle incoming RPC commands from WebSocket subscribers."""
        cmd = cmd_dict.get("command")
        if not cmd:
            return

        logger.debug("Received WebSocket client command: %s", cmd)

        if cmd == "get_calibration":
            dev_id = cmd_dict.get("device_id") or self.device_id
            cal = self.cal_db.get_calibration(dev_id)
            await self.ws_server.send_to_client(
                websocket,
                {
                    "type": "calibration_data",
                    "device_id": dev_id,
                    "found": cal is not None,
                    "calibration": cal,
                },
            )

        elif cmd == "save_calibration":
            dev_id = cmd_dict.get("device_id") or self.device_id
            cal_payload = cmd_dict.get("calibration", {})
            saved = self.cal_db.save_calibration(dev_id, cal_payload)
            # Inject newly saved calibration into connected hardware
            asyncio.create_task(self.inject_calibration_to_device(saved))
            # Notify sender
            await self.ws_server.send_to_client(
                websocket,
                {
                    "type": "save_calibration_ack",
                    "status": "ok",
                    "device_id": dev_id,
                    "calibration": saved,
                },
            )
            # Broadcast updated status to all clients
            self.broadcast_gateway_status()

        elif cmd == "list_calibrations":
            devices = self.cal_db.list_calibrations()
            await self.ws_server.send_to_client(
                websocket,
                {
                    "type": "calibration_list",
                    "devices": devices,
                },
            )

        elif cmd == "delete_calibration":
            dev_id = cmd_dict.get("device_id")
            deleted = self.cal_db.delete_calibration(dev_id) if dev_id else False
            await self.ws_server.send_to_client(
                websocket,
                {
                    "type": "delete_calibration_ack",
                    "status": "ok" if deleted else "not_found",
                    "device_id": dev_id,
                },
            )
            self.broadcast_gateway_status()

    async def run(self) -> None:
        self._running = True
        await self.ws_server.start()

        source_task = asyncio.create_task(self.source.run())
        status_task = asyncio.create_task(self._periodic_status_loop())

        try:
            await asyncio.gather(source_task, status_task)
        except asyncio.CancelledError:
            pass
        finally:
            await self.shutdown()

    async def _periodic_status_loop(self) -> None:
        """Periodically broadcast status to WebSocket and log diagnostic summary."""
        while self._running:
            try:
                await asyncio.sleep(1.0)
                self.broadcast_gateway_status()

                # 1-line readable summary to console
                s = self.normalizer.stats(now=time.monotonic())
                loss_pct = s["loss_ratio"] * 100.0
                delay = s.get("extra_delay_ms_mean")
                delay_str = f"{delay:.1f}ms" if delay is not None else "n/a"
                sys.stdout.write(
                    f"\r[{self.device_id}] Rate: {s['rate_hz']:5.1f} Hz | "
                    f"Recv: {s['received']:6d} | Lost: {s['lost']:3d} ({loss_pct:4.1f}%) | "
                    f"Jitter: {delay_str:>6s} | WS Clients: {self.ws_server.client_count} | "
                    f"State: {self.connection_status:<12s}"
                )
                sys.stdout.flush()
            except asyncio.CancelledError:
                break
            except Exception as exc:
                logger.error("Error in status loop: %s", exc)

    async def shutdown(self) -> None:
        self._running = False
        print("\nShutting down gateway...")
        if self.source:
            self.source.stop()
        await self.ws_server.stop()
        print("Gateway stopped cleanly.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Motion Controller Lab - BLE GATT to WebSocket Gateway",
    )
    parser.add_argument("--sim", action="store_true", help="Run simulated source instead of BLE")
    parser.add_argument("--name", type=str, default=None, help="BLE target device name filter")
    parser.add_argument("--address", type=str, default=None, help="BLE target device address/MAC")
    parser.add_argument("--ws-host", type=str, default="127.0.0.1", help="WebSocket host")
    parser.add_argument("--ws-port", type=int, default=8765, help="WebSocket port")
    parser.add_argument("--device-id", type=str, default="MCL-0001", help="Default device ID")
    parser.add_argument("--sim-drops", type=float, default=0.0, help="Simulated drop probability (0.0..1.0)")
    parser.add_argument("--sim-corrupt", type=float, default=0.0, help="Simulated corruption probability")
    parser.add_argument("--sim-duplicates", type=float, default=0.0, help="Simulated duplicate probability")
    parser.add_argument("--sim-disconnects", type=float, default=0.0, help="Simulated disconnect interval (s)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose debug logging")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    anomalies = AnomalyConfig(
        drop_rate=args.sim_drops,
        corrupt_rate=args.sim_corrupt,
        duplicate_rate=args.sim_duplicates,
        disconnect_interval_s=args.sim_disconnects,
    )

    orchestrator = GatewayOrchestrator(
        sim_mode=args.sim,
        name=args.name,
        address=args.address,
        ws_host=args.ws_host,
        ws_port=args.ws_port,
        device_id=args.device_id,
        anomaly_config=anomalies,
    )

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    main_task = loop.create_task(orchestrator.run())

    def request_shutdown() -> None:
        main_task.cancel()

    # Register OS signals
    try:
        loop.add_signal_handler(signal.SIGINT, request_shutdown)
        loop.add_signal_handler(signal.SIGTERM, request_shutdown)
    except NotImplementedError:
        # Windows event loop may not support add_signal_handler
        pass

    try:
        loop.run_until_complete(main_task)
    except (KeyboardInterrupt, asyncio.CancelledError):
        loop.run_until_complete(orchestrator.shutdown())
    finally:
        loop.close()


if __name__ == "__main__":
    main()
