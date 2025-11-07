#!/usr/bin/env python3
"""
Mock Monitor WebSocket server for GUI testing.

This helper streams synthetic heartbeat and diagnostics events so that
`acoustics_gui` can exercise the Monitor WebSocket intake without any
hardware online.

Usage:
    python acoustics/tools/monitor/mock_ws_server.py \
        --devices acoustics/state/devices.json \
        --host 127.0.0.1 --port 48080

Requirements:
    pip install websockets
"""

from __future__ import annotations

import argparse
import asyncio
import json
import pathlib
import random
import signal
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import List

try:
    import websockets
except ImportError as exc:  # pragma: no cover - dependency notice
    raise SystemExit(
        "The mock server requires the 'websockets' package. "
        "Install it via 'pip install websockets'."
    ) from exc


@dataclass
class MockConfig:
    host: str
    port: int
    path: str
    devices: List[str]
    heartbeat_interval: float
    diagnostics_every: int


def load_device_ids(path: pathlib.Path) -> List[str]:
    if not path.exists():
        return ["device-sim"]
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except json.JSONDecodeError:
        return ["device-sim"]

    device_ids: List[str] = []
    if isinstance(data, list):
        for entry in data:
            if isinstance(entry, dict):
                device_id = entry.get("id") or entry.get("device_id")
                if isinstance(device_id, str) and device_id:
                    device_ids.append(device_id)
    elif isinstance(data, dict):
        for entry in data.get("devices", []):
            if isinstance(entry, dict):
                device_id = entry.get("id") or entry.get("device_id")
                if isinstance(device_id, str) and device_id:
                    device_ids.append(device_id)

    return device_ids or ["device-sim"]


async def emit_events(websocket: websockets.WebSocketServerProtocol, config: MockConfig) -> None:
    counter = 0
    await websocket.send(json.dumps({"type": "hello", "device_count": len(config.devices)}))
    while True:
        device_id = random.choice(config.devices)
        now = datetime.now(timezone.utc)
        heartbeat = {
            "type": "heartbeat",
            "device_id": device_id,
            "latency_ms": round(random.uniform(20.0, 85.0), 2),
            "queue_depth": random.randint(0, 3),
            "is_playing": random.choice([True, False]),
            "timestamp": now.isoformat()
        }
        await websocket.send(json.dumps(heartbeat))

        if config.diagnostics_every > 0 and counter % config.diagnostics_every == 0:
            diag = {
                "type": "diagnostics",
                "device_id": device_id,
                "severity": random.choice(["info", "warn", "critical"]),
                "reason": random.choice([
                    "High latency spike",
                    "Queue overflow",
                    "Low RSSI detected"
                ]),
                "recommended_action": "Inspect the unit on site",
                "timestamp": now.isoformat()
            }
            await websocket.send(json.dumps(diag))

        counter += 1
        await asyncio.sleep(config.heartbeat_interval)


async def handler(websocket: websockets.WebSocketServerProtocol, path: str, config: MockConfig) -> None:
    if path != config.path:
        await websocket.close(code=1008, reason="Unsupported path")
        return
    try:
        await emit_events(websocket, config)
    except websockets.ConnectionClosedOK:
        pass
    except websockets.ConnectionClosedError:
        pass


async def main(args: argparse.Namespace) -> None:
    devices = load_device_ids(args.devices)
    config = MockConfig(
        host=args.host,
        port=args.port,
        path=args.path,
        devices=devices,
        heartbeat_interval=args.interval,
        diagnostics_every=args.diag_interval,
    )
    server = await websockets.serve(
        lambda ws, p: handler(ws, p, config),
        args.host,
        args.port,
    )
    print(f"Mock Monitor WS listening on ws://{args.host}:{args.port}{args.path} ({len(devices)} devices)")

    stop = asyncio.Future()

    def _signal_handler(*_: object) -> None:
        if not stop.done():
            stop.set_result(None)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            asyncio.get_event_loop().add_signal_handler(sig, _signal_handler)
        except NotImplementedError:
            signal.signal(sig, lambda *_: _signal_handler())

    await stop
    server.close()
    await server.wait_closed()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mock Monitor WebSocket server.")
    parser.add_argument("--host", default="127.0.0.1", help="Bind address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=48080, help="Bind port (default: 48080)")
    parser.add_argument("--path", default="/ws/events", help="WebSocket path expected by the GUI")
    parser.add_argument(
        "--devices",
        type=pathlib.Path,
        default=pathlib.Path("acoustics/state/devices.json"),
        help="Path to devices.json for seeding device IDs",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Seconds between heartbeats (default: 1.0)",
    )
    parser.add_argument(
        "--diag-interval",
        type=int,
        default=5,
        help="Emit a diagnostics event every N heartbeats (default: 5, 0 disables)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    asyncio.run(main(parse_args()))
