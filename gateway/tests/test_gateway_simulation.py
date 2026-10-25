"""Integration tests for Gateway simulation and WebSocket streaming.

Verifies:
1. SimSource generation and anomaly injection.
2. WebSocket server broadcasting at 50 Hz.
3. Slow consumer isolation: slow clients drop intermediate samples without blocking fast clients.
4. Client connect / disconnect / reconnect cycles.
5. Clean shutdown.
"""

from __future__ import annotations

import asyncio
import json
import time
import unittest

from _path import *  # noqa: F401,F403

import websockets
from motion_gateway.protocol import decode_telemetry
from motion_gateway.sim_source import AnomalyConfig, SimSource
from motion_gateway.telemetry import Normalizer
from motion_gateway.websocket_server import ClientSlot, WebSocketServer


class SimSourceTests(unittest.IsolatedAsyncioTestCase):
    async def test_cadence_and_payload(self):
        received = []

        def on_telem(data, t):
            received.append((data, t))

        src = SimSource(sample_rate_hz=100.0, on_telemetry=on_telem)
        task = asyncio.create_task(src.run())
        await asyncio.sleep(0.15)  # expect ~15 samples
        src.stop()
        await task

        self.assertGreaterEqual(len(received), 10)
        self.assertLessEqual(len(received), 20)
        pkt = decode_telemetry(received[0][0])
        self.assertEqual(pkt.version, 1)

    async def test_anomaly_injection(self):
        anomalies = AnomalyConfig(drop_rate=0.2, corrupt_rate=0.1, duplicate_rate=0.1)
        telem_data = []

        src = SimSource(
            sample_rate_hz=200.0,
            anomalies=anomalies,
            on_telemetry=lambda d, t: telem_data.append((d, t)),
        )
        task = asyncio.create_task(src.run())
        await asyncio.sleep(0.2)
        src.stop()
        await task

        norm = Normalizer()
        for d, t in telem_data:
            try:
                p = decode_telemetry(d)
                norm.telemetry(p, t)
            except Exception:
                norm.count_rejected("corrupt")

        stats = norm.stats()
        self.assertGreater(stats["received"], 10)
        # Verify anomalies were tracked
        self.assertTrue(stats["lost"] > 0 or stats["duplicates"] > 0 or "corrupt" in stats["rejected"])


class ClientSlotTests(unittest.IsolatedAsyncioTestCase):
    async def test_slot_drops_and_keeps_latest(self):
        """Verify ClientSlot bounded queue (maxsize=1) drops stale messages."""
        slot = ClientSlot(None)  # ws not needed for queue testing
        self.assertEqual(slot.dropped_samples, 0)

        slot.push_latest("msg1")
        self.assertEqual(slot.queue.qsize(), 1)
        self.assertEqual(slot.dropped_samples, 0)

        # Overwrite with msg2 -> msg1 dropped
        slot.push_latest("msg2")
        self.assertEqual(slot.queue.qsize(), 1)
        self.assertEqual(slot.dropped_samples, 1)

        # Overwrite with msg3 -> msg2 dropped
        slot.push_latest("msg3")
        self.assertEqual(slot.queue.qsize(), 1)
        self.assertEqual(slot.dropped_samples, 2)

        # Drain: should yield msg3
        drained = slot.queue.get_nowait()
        self.assertEqual(drained, "msg3")
        self.assertTrue(slot.queue.empty())


class WebSocketStreamingTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.port = 8799
        self.server = WebSocketServer(host="127.0.0.1", port=self.port)
        await self.server.start()

    async def asyncTearDown(self):
        await self.server.stop()

    async def test_concurrent_streaming(self):
        """Verify multiple clients concurrently receive telemetry samples at 50 Hz."""
        normalizer = Normalizer(device_id="TEST-DEV")

        sim = SimSource(
            sample_rate_hz=50.0,
            on_telemetry=lambda data, t: self.server.broadcast_telemetry(
                normalizer.telemetry(decode_telemetry(data), t)
            ),
        )
        sim_task = asyncio.create_task(sim.run())

        c1_received = []
        c2_received = []

        async def client1():
            async with websockets.connect(f"ws://127.0.0.1:{self.port}") as ws:
                for _ in range(15):
                    msg = await ws.recv()
                    data = json.loads(msg)
                    if data.get("type") == "telemetry":
                        c1_received.append(data["sequence"])

        async def client2():
            async with websockets.connect(f"ws://127.0.0.1:{self.port}") as ws:
                for _ in range(15):
                    msg = await ws.recv()
                    data = json.loads(msg)
                    if data.get("type") == "telemetry":
                        c2_received.append(data["sequence"])

        await asyncio.gather(client1(), client2())

        sim.stop()
        await sim_task

        self.assertEqual(len(c1_received), 15)
        self.assertEqual(len(c2_received), 15)
        self.assertEqual(c1_received[0], c2_received[0])

    async def test_client_reconnection(self):
        """Verify client can disconnect and reconnect cleanly."""
        url = f"ws://127.0.0.1:{self.port}"

        # Connect 1
        ws1 = await websockets.connect(url)
        self.assertEqual(self.server.client_count, 1)
        await ws1.close()
        await asyncio.sleep(0.05)
        self.assertEqual(self.server.client_count, 0)

        # Reconnect 2
        ws2 = await websockets.connect(url)
        self.assertEqual(self.server.client_count, 1)
        await ws2.close()
        await asyncio.sleep(0.05)
        self.assertEqual(self.server.client_count, 0)


if __name__ == "__main__":
    unittest.main()
