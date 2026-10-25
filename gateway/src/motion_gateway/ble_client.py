"""BLE GATT client using Bleak for communication with ESP32-C3 Motion Controller.

Discovers the device by service UUID or name, subscribes to telemetry and status
notifications, and implements controlled exponential backoff reconnection.
"""

from __future__ import annotations

import asyncio
import logging
import random
import time
from enum import Enum
from typing import Callable, Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice

from .protocol import (
    SERVICE_UUID,
    STATUS_CHAR_UUID,
    TELEMETRY_CHAR_UUID,
)

logger = logging.getLogger("motion_gateway.ble")


class ConnectionState(str, Enum):
    STOPPED = "stopped"
    SCANNING = "scanning"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    BACKOFF = "backoff"


class BleClient:
    """Manages BLE discovery, connection, subscription, and backoff reconnection."""

    def __init__(
        self,
        name: Optional[str] = None,
        address: Optional[str] = None,
        on_telemetry: Optional[Callable[[bytes, float], None]] = None,
        on_status: Optional[Callable[[bytes, float], None]] = None,
        on_connection: Optional[Callable[[bool, str], None]] = None,
        min_backoff_s: float = 1.0,
        max_backoff_s: float = 30.0,
    ) -> None:
        self.target_name = name
        self.target_address = address
        self.on_telemetry = on_telemetry
        self.on_status = on_status
        self.on_connection = on_connection
        self.min_backoff_s = min_backoff_s
        self.max_backoff_s = max_backoff_s

        self._state = ConnectionState.STOPPED
        self._running = False
        self._client: Optional[BleakClient] = None
        self._device_id = "unknown"
        self._backoff = min_backoff_s
        self._disconnect_event = asyncio.Event()

    @property
    def state(self) -> ConnectionState:
        return self._state

    @property
    def device_id(self) -> str:
        return self._device_id

    def stop(self) -> None:
        """Signal client to stop running and disconnect."""
        self._running = False
        self._disconnect_event.set()

    async def run(self) -> None:
        """Supervise the discovery and connection lifecycle with exponential backoff."""
        self._running = True
        logger.info("BLE supervisor started (target_name=%s, target_address=%s)",
                    self.target_name, self.target_address)

        while self._running:
            try:
                device = await self._discover_device()
                if not self._running:
                    break
                if device is None:
                    await self._sleep_backoff("No matching BLE device found")
                    continue

                self._device_id = device.name or device.address
                connected = await self._connect_and_stream(device)
                if not self._running:
                    break

                if not connected:
                    await self._sleep_backoff("Failed to establish/maintain BLE connection")
                else:
                    # Reset backoff after successful connection session
                    self._backoff = self.min_backoff_s

            except asyncio.CancelledError:
                break
            except Exception as exc:
                logger.error("Unexpected error in BLE supervisor: %s", exc)
                await self._sleep_backoff(str(exc))

        self._state = ConnectionState.STOPPED
        await self._disconnect_client()
        logger.info("BLE supervisor stopped")

    async def _discover_device(self) -> Optional[BLEDevice]:
        """Scan for candidate devices matching service UUID, name, or address."""
        self._state = ConnectionState.SCANNING
        logger.info("Scanning for Motion Controller device...")

        def match_filter(device: BLEDevice, adv_data: Any) -> bool:
            if self.target_address and device.address.lower() == self.target_address.lower():
                return True
            if self.target_name and device.name:
                if self.target_name.lower() in device.name.lower():
                    return True
            # Match by service UUID in advertisement
            if adv_data.service_uuids:
                for u in adv_data.service_uuids:
                    if u.lower() == SERVICE_UUID.lower():
                        return True
            # Match default device name prefix "MCL-" if no specific filter is set
            if not self.target_name and not self.target_address and device.name:
                if device.name.startswith("MCL-"):
                    return True
            return False

        try:
            device = await BleakScanner.find_device_by_filter(match_filter, timeout=5.0)
            if device:
                logger.info("Discovered candidate: %s [%s]", device.name, device.address)
            return device
        except Exception as exc:
            logger.warning("Scan error: %s", exc)
            return None

    async def _connect_and_stream(self, device: BLEDevice) -> bool:
        """Connect to device, subscribe to GATT notifications, and wait for disconnect."""
        self._state = ConnectionState.CONNECTING
        self._disconnect_event.clear()
        logger.info("Connecting to %s...", device.address)

        def on_disconnected(client: BleakClient) -> None:
            logger.warning("BLE device %s disconnected", client.address)
            self._disconnect_event.set()
            if self.on_connection:
                self.on_connection(False, self._device_id)

        try:
            async with BleakClient(device, disconnected_callback=on_disconnected) as client:
                self._client = client
                self._state = ConnectionState.CONNECTED
                logger.info("BLE connected to %s", device.address)

                if self.on_connection:
                    self.on_connection(True, self._device_id)

                # Subscribe to telemetry characteristic
                await client.start_notify(TELEMETRY_CHAR_UUID, self._handle_telemetry_notification)

                # Subscribe to status characteristic if available
                try:
                    await client.start_notify(STATUS_CHAR_UUID, self._handle_status_notification)
                except Exception as exc:
                    logger.debug("Status notification not subscribed: %s", exc)

                # Initial read of status characteristic
                try:
                    init_status = await client.read_gatt_char(STATUS_CHAR_UUID)
                    if init_status and self.on_status:
                        self.on_status(bytes(init_status), time.monotonic())
                except Exception:
                    pass

                # Wait until disconnection event is triggered or client is stopped
                await self._disconnect_event.wait()
                return True

        except Exception as exc:
            logger.warning("Connection failure to %s: %s", device.address, exc)
            return False
        finally:
            self._client = None
            if self.on_connection:
                self.on_connection(False, self._device_id)

    async def _disconnect_client(self) -> None:
        if self._client and self._client.is_connected:
            try:
                await self._client.disconnect()
            except Exception:
                pass
        self._client = None

    async def _sleep_backoff(self, reason: str) -> None:
        self._state = ConnectionState.BACKOFF
        jitter = random.uniform(0.0, 0.5)
        sleep_time = min(self._backoff + jitter, self.max_backoff_s)
        logger.info("%s. Reconnecting in %.1fs...", reason, sleep_time)
        self._backoff = min(self._backoff * 2.0, self.max_backoff_s)
        try:
            await asyncio.sleep(sleep_time)
        except asyncio.CancelledError:
            pass

    def _handle_telemetry_notification(self, char: BleakGATTCharacteristic, data: bytearray) -> None:
        host_time = time.monotonic()
        if self.on_telemetry:
            self.on_telemetry(bytes(data), host_time)

    def _handle_status_notification(self, char: BleakGATTCharacteristic, data: bytearray) -> None:
        host_time = time.monotonic()
        if self.on_status:
            self.on_status(bytes(data), host_time)

    async def write_calibration(self, payload: bytes) -> bool:
        """Write 32-byte calibration payload to CALIBRATION_CHAR_UUID on connected device."""
        if not self._client or not self._client.is_connected:
            logger.warning("Cannot write calibration: BLE client is not connected")
            return False
        try:
            from .protocol import CALIBRATION_CHAR_UUID
            await self._client.write_gatt_char(CALIBRATION_CHAR_UUID, payload, response=True)
            logger.info("Successfully injected calibration to device via BLE (%d bytes)", len(payload))
            return True
        except Exception as exc:
            logger.error("Failed to write calibration to BLE characteristic: %s", exc)
            return False

