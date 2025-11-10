# Transport Layer (ToioClient)

ToioClient は Boost.Asio/Beast を用いて Toio Relay Server (`/ws`) との WebSocket 通信を行う下位レイヤーです。CLI やミドルウェアからは単一サーバーへの接続・コマンド送信・受信ハンドリングを担うコンポーネントとして利用します。

## 役割
- WebSocket の確立／切断 (`connect`, `close`) と受信スレッド (`reader_loop`) の管理。
- `send_command`, `send_query` といった JSON メッセージの直列化・送信。
- `connect_cube`, `send_move`, `set_led`, `query_position` などの高水準 API。
- 受信 JSON を呼び出し側コールバック (`set_message_handler`) へそのまま渡す。

## 接続フロー
1. `resolver_.resolve(host, port)` で DNS 解決し、`asio::connect` で TCP を確立。
2. ハンドシェイク時に User-Agent を `toio-cpp-client/0.1` に設定。
3. 接続後は `connected_` と `running_` を立て、専用スレッドで `reader_loop` を起動。
4. `close()` は `websocket_.close(normal)` → 受信スレッド join → フラグを落とす。

API 呼び出し前には `ensure_connected()` が状態をチェックし、未接続なら `runtime_error` を投げます。

## メッセージモデル

### command

```json
{
  "type": "command",
  "payload": {
    "cmd": "move",
    "target": "cube-id",
    "params": { "left_speed": -30, "right_speed": 30 },
    "require_result": true
  }
}
```

`cmd` 例: `connect`, `disconnect`, `move`, `led`。`require_result=false` で成功レスポンスを省略可能。

### query

```json
{
  "type": "query",
  "payload": {
    "info": "position",
    "target": "cube-id",
    "notify": true
  }
}
```

`info` 例: `battery`, `position`。`notify=true` で購読開始、`false/省略` で単発クエリ。

サーバーからの `result` / `response` / `system` / `error` は JSON 文字列のまま `MessageHandler` に渡されます。ハンドラ未設定時は標準出力ログにフォールバックします。
