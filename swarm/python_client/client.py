import asyncio
import websockets
import json
import math
from control import drive_go_to_goal

# WebSocket サーバーの URL
WEBSOCKET_URL = "ws://localhost:8765/ws"

# Toio のターゲット ID
TOIO_ID = "F3H"

# WebSocket を通じてコマンドを送信する関数
def create_command(cmd, target, params=None):
    return {
        "type": "command",
        "payload": {
            "cmd": cmd,
            "target": target,
            "params": params or {}
        }
    }

# WebSocket を通じてクエリを送信する関数
def create_query(info, target):
    return {
        "type": "query",
        "payload": {
            "info": info,
            "target": target
        }
    }

# WebSocket を通じてメッセージを送信
def send_message(websocket, message):
    return websocket.send(json.dumps(message))

# WebSocket を通じてメッセージを受信
def receive_message(websocket):
    return websocket.recv()



# WebSocket メッセージを受信して処理する関数
async def websocket_listener(websocket, position_data):
    while True:
        message = await websocket.recv()
        # print("Received message:", message)
        try:
            data = json.loads(message)
            if data.get("type") == "response" and data["payload"].get("info") == "position":
                position = data["payload"].get("position", {})
                position_data["x"] = position.get("x")
                position_data["y"] = position.get("y")
                position_data["angle"] = position.get("angle")
                position_data["on_mat"] = position.get("on_mat")
                # print(f"Updated Position: x={position_data['x']}, y={position_data['y']}, angle={position_data['angle']}, on_mat={position_data['on_mat']}")
        except json.JSONDecodeError:
            print("Invalid JSON received.")

# Toio を回転させながら動かす制御ループ
async def control_toio():
    position_data = {"x": None, "y": None, "angle": None}  # 座標データを共有

    async with websockets.connect(WEBSOCKET_URL) as websocket:
        # 接続確認
        response = await receive_message(websocket)
        print("Connected to server:", response)

        # Toio を接続
        connect_command = create_command("connect", TOIO_ID)
        await send_message(websocket, connect_command)
        response = await receive_message(websocket)
        data = json.loads(response)
        print("Connect response data:", data)
        if( data.get("payload", {}).get("status") != "success"):
            print("Failed to connect to Toio.")
            return

        #待つ
        await asyncio.sleep(1)

        # WebSocket リスナーを並列で実行
        listener_task = asyncio.create_task(websocket_listener(websocket, position_data))
        angle = 0  # 回転角度の初期値

        # 制御ループ
        try:
            while True:
                # 現在地を取得するためのクエリを送信
                query_command = create_query("position", TOIO_ID)
                await send_message(websocket, query_command)

                # 現在の座標を使用して制御
                if position_data["x"] is not None and position_data["y"] is not None:
                    print(f"Using Position: x={position_data['x']}, y={position_data['y']}, angle={position_data['angle']}")

                    # drive_go_to_goal を使用してモーター速度を計算
                    left_speed, right_speed = await drive_go_to_goal(
                        position_data["x"],
                        position_data["y"],
                        position_data["angle"],
                        xg=330,  # 目標 x 座標
                        yg=330,  # 目標 y 座標
                        vmax=70,
                        wmax=60,
                        k_r=0.5,
                        k_a=1.2,
                        stop_dist=20
                    )

                    # モーター制御コマンドを送信
                    move_command = create_command("move", TOIO_ID, {
                        "left_speed": left_speed,
                        "right_speed": right_speed
                    })
                    await send_message(websocket, move_command)

                # LED 制御コマンドを送信
                led_command = create_command("led", TOIO_ID, {
                    "r": int((math.sin(math.radians(angle)) + 1) * 127),
                    "g": int((math.cos(math.radians(angle)) + 1) * 127),
                    "b": 127
                })
                await send_message(websocket, led_command)

                # 次の角度に進む
                angle = (angle + 1) % 360
                await asyncio.sleep(0.1)  # 20Hz
        except KeyboardInterrupt:
            print("Stopping control loop...")
        finally:
            listener_task.cancel()  # リスナータスクをキャンセル
            # Toio を切断
            disconnect_command = create_command("disconnect", TOIO_ID)
            await send_message(websocket, disconnect_command)
            # response = await receive_message(websocket)
            # print("Disconnect response:", response)

if __name__ == "__main__":
    asyncio.run(control_toio())


