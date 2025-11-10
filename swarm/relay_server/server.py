# 実行方法
# uvicorn server:app --host 0.0.0.0 --port 8765
# uv run uvicorn relay_server.server:app --reload --host 0.0.0.0 --port 8765

import asyncio
import json
from collections import defaultdict
from typing import Optional, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect

from .cubes import (
    CubeStatus,
    connect_cube,
    disconnect_cube,
    read_battery_level,
    register_notification_handler,
    scan_cubes,
    set_led,
    set_motor,
)

app = FastAPI()

# グローバル変数としてキューブの状態を保持
cube_statuses = {}

# WebSocket 毎の購読状態 (position 通知)
target_subscribers: dict[str, Set[WebSocket]] = defaultdict(set)
websocket_subscriptions: dict[WebSocket, Set[str]] = defaultdict(set)

# Define a WebSocket endpoint
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    await websocket.send_text(json.dumps({
        "type": "system",
        "payload": {
            "status": "connected",
            "message": "WebSocket connection established."
        }
    }))
    try:
        while True:
            # Receive a message from the client
            data = await websocket.receive_text()
            message = json.loads(data)

            # Process the message based on its type
            response = await process_message(message, websocket)

            # Send the response back to the client
            if response is not None:
                await websocket.send_text(json.dumps(response))
    except WebSocketDisconnect:
        print("Client disconnected")
    finally:
        cleanup_websocket(websocket)

async def process_message(message, websocket: WebSocket):
    message_type = message.get("type")
    payload = message.get("payload", {})

    if message_type == "command":
        return await handle_command(payload)
    elif message_type == "query":
        return await handle_query(payload, websocket)
    elif message_type == "system":
        return handle_system(payload)
    else:
        return {"type": "error", "payload": {"message": "Unknown message type"}}

async def handle_command(payload):
    cmd = payload.get("cmd")
    target = payload.get("target")
    params = payload.get("params", {})
    require_result = payload.get("require_result", True)

    def build_result(status, message=None):
        result_payload = {
            "cmd": cmd,
            "target": target,
            "status": status
        }
        if message:
            result_payload["message"] = message
        if status != "success" or require_result:
            return {"type": "result", "payload": result_payload}
        return None

    if cmd == "connect":
        # 既に接続済みか確認
        if target in cube_statuses:
            return build_result("success", "Device already connected")

        # キューブをスキャンして接続
        dev_list = await scan_cubes([target])
        if not dev_list:
            return build_result("error", "Device not found")
        cube = await connect_cube(dev_list[0])
        if cube is None:
            return build_result("error", "Failed to connect")
        # 状態を登録
        cube_status = CubeStatus(cube, target)
        cube_statuses[target] = cube_status
        loop = asyncio.get_running_loop()

        def schedule_push():
            loop.call_soon_threadsafe(lambda: asyncio.create_task(broadcast_position_update(target)))

        await register_notification_handler(cube_status.cube, cube_status, schedule_push)
        return build_result("success")
    elif cmd == "disconnect":
        # キューブを切断
        cube_status = cube_statuses.get(target)
        if cube_status:
            success = await disconnect_cube(cube_status)
            if success:
                del cube_statuses[target]
                await notify_subscription_termination(target, "Device disconnected")
                return build_result("success")
            else:
                return build_result("error", "Failed to disconnect")
        return build_result("error", "Device not connected")
    elif cmd == "move":
        # モーター制御
        cube_status = cube_statuses.get(target)
        if cube_status:
            left_speed = params.get("left_speed", 0)
            right_speed = params.get("right_speed", 0)
            await set_motor(cube_status.cube, left_speed, right_speed)
            return build_result("success")
        return build_result("error", "Device not connected")
    elif cmd == "led":
        # LED制御
        cube_status = cube_statuses.get(target)
        if cube_status:
            r = params.get("r", 0)
            g = params.get("g", 0)
            b = params.get("b", 0)
            await set_led(cube_status.cube, r, g, b)
            return build_result("success")
        return build_result("error", "Device not connected")
    else:
        return build_result("error", "Unknown command")

async def handle_query(payload, websocket: WebSocket):
    info = payload.get("info")
    target = payload.get("target")
    notify_requested = payload.get("notify")

    cube_status = cube_statuses.get(target)
    if not cube_status:
        remove_subscription(websocket, target)
        return {
            "type": "response",
            "payload": {
                "info": info,
                "target": target,
                "message": "Device not connected"
            }
        }

    if info == "battery":
        try:
            battery_level = await read_battery_level(cube_status.cube)
        except asyncio.TimeoutError:
            print(f"Timed out while reading battery level for {target}")
            return {
                "type": "response",
                "payload": {
                    "info": info,
                    "target": target,
                    "battery_level": None,
                    "message": "Timed out while reading battery level"
                }
            }
        except Exception as exc:
            print(f"Failed to read battery level for {target}: {exc}")
            return {
                "type": "response",
                "payload": {
                    "info": info,
                    "target": target,
                    "battery_level": None,
                    "message": "Failed to read battery level"
                }
            }
        return {
            "type": "response",
            "payload": {
                "info": info,
                "target": target,
                "battery_level": battery_level
            }
        }
    elif info == "position":
        if notify_requested is True:
            add_subscription(websocket, target)
        else:
            remove_subscription(websocket, target)
        notify_active = is_subscribed(websocket, target)
        return {
            "type": "response",
            "payload": {
                "info": info,
                "target": target,
                "notify": notify_active,
                "position": {
                    "x": cube_status.x,
                    "y": cube_status.y,
                    "angle": cube_status.angle,
                    "on_mat": cube_status.on_mat
                }
            }
        }
    else:
        return {
            "type": "response",
            "payload": {
                "info": info,
                "target": target,
                "message": "Unknown query"
            }
        }

def handle_system(payload):
    status = payload.get("status")
    message = payload.get("message")

    # Example: Handle system messages
    return {
        "type": "system",
        "payload": {
            "status": status,
            "message": message
        }
    }

def add_subscription(websocket: WebSocket, target: str):
    target_subscribers[target].add(websocket)
    websocket_subscriptions[websocket].add(target)

def remove_subscription(websocket: WebSocket, target: Optional[str]):
    if target is None:
        return
    subscribers = target_subscribers.get(target)
    if subscribers:
        subscribers.discard(websocket)
        if not subscribers:
            target_subscribers.pop(target, None)
    targets = websocket_subscriptions.get(websocket)
    if targets:
        targets.discard(target)
        if not targets:
            websocket_subscriptions.pop(websocket, None)

def is_subscribed(websocket: WebSocket, target: str) -> bool:
    targets = websocket_subscriptions.get(websocket)
    return target in targets if targets else False

def cleanup_websocket(websocket: WebSocket):
    targets = websocket_subscriptions.pop(websocket, set())
    for target in targets:
        subscribers = target_subscribers.get(target)
        if subscribers:
            subscribers.discard(websocket)
            if not subscribers:
                target_subscribers.pop(target, None)

async def broadcast_position_update(target: str):
    subscribers = list(target_subscribers.get(target, set()))
    if not subscribers:
        return
    cube_status = cube_statuses.get(target)
    if cube_status is None:
        return
    payload = {
        "info": "position",
        "target": target,
        "notify": True,
        "position": {
            "x": cube_status.x,
            "y": cube_status.y,
            "angle": cube_status.angle,
            "on_mat": cube_status.on_mat
        }
    }
    message = json.dumps({"type": "response", "payload": payload})
    for websocket in subscribers:
        try:
            await websocket.send_text(message)
        except Exception:
            cleanup_websocket(websocket)

async def notify_subscription_termination(target: str, reason: str):
    subscribers = list(target_subscribers.get(target, set()))
    if not subscribers:
        return
    payload = {
        "info": "position",
        "target": target,
        "notify": False,
        "message": reason
    }
    message = json.dumps({"type": "response", "payload": payload})
    for websocket in subscribers:
        try:
            await websocket.send_text(message)
        finally:
            remove_subscription(websocket, target)
