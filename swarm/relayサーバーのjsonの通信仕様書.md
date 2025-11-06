# Toio 中継サーバー JSON 通信仕様

## 1. 概要
Toio 中継サーバーは WebSocket を介してクライアントと Toio デバイス間のコマンド／状態を双方向に中継します。本仕様書は Python 製リレーサーバー (`toio/relay-server/server.py`) の現在の実装と一致する JSON メッセージ形式を定義します。

---

## 2. 接続情報とシーケンス
- **プロトコル**: WebSocket
- **エンドポイント**: `ws://<host>:8765/ws`
- **データ形式**: UTF-8 エンコードの JSON 文字列

接続フロー:
1. クライアントが WebSocket を確立すると、サーバーは `type: "system"` / `status: "connected"` を即時送信して接続完了を通知します。
2. 以降はクライアント主導で `command` / `query` を送信し、サーバーは対になる `result` / `response` を返します。
3. 不正なメッセージ型や JSON 解析エラー時には `type: "error"` が返ります。

---

## 3. 共通メッセージ構造
すべてのメッセージは下記の 2 フィールドを持ちます。

| フィールド | 型     | 説明                                               |
|-----------|--------|----------------------------------------------------|
| `type`    | string | メッセージ種別 (`command`, `result`, `query`, `response`, `system`, `error`) |
| `payload` | object | 種別ごとのデータ。本仕様書の各節を参照してください。             |

未定義の `type` を受信した場合、サーバーは `type: "error"` を返します。

---

## 4. メッセージタイプ詳細

### 4.1 system
- **方向**: サーバー → クライアント (通知)。クライアントが送った場合は内容をそのままエコーします。
- **目的**: 接続状態やサーバー状態のブロードキャスト。

| フィールド  | 型     | 必須 | 説明                                                |
|------------|--------|------|-----------------------------------------------------|
| `status`   | string | ✅   | `"connected"`, `"started"`, `"stopped"`, `"error"` など。 |
| `message`  | string | ✅   | 状態に関する補足メッセージ。                        |

> 備考: `connected` はハンドシェイク用、`error` はサーバー内部エラーや Toio 接続失敗理由を説明する際に使用します。クライアントは未知の `status` が来ても無視できるように実装してください。

---

### 4.2 command
- **方向**: クライアント → サーバー
- **目的**: Toio への操作要求

| フィールド  | 型     | 必須 | 説明                                      |
|------------|--------|------|-------------------------------------------|
| `cmd`      | string | ✅   | 実行するコマンド名                        |
| `target`   | string | ✅   | Toio デバイス ID (例: BLE MAC 末尾)        |
| `params`   | object | 任意 | コマンド固有パラメータ。省略時は空オブジェクト。 |
| `require_result` | bool | 任意 | `true` (デフォルト) なら通常通り `result` を返却。`false` の場合は成功時レスポンスを省略し、失敗時のみ `result` を返します。 |

サポートされる `cmd`:

| cmd         | 説明                         | `params`                                      |
|-------------|------------------------------|-----------------------------------------------|
| `connect`   | 指定 ID の Toio と接続       | なし                                          |
| `disconnect`| 接続済み Toio を切断         | なし                                          |
| `move`      | 左右モーター速度を制御       | `left_speed`, `right_speed` (整数)。値域は Toio SDK (`set_motor`) の仕様に従う。 |
| `led`       | LED カラーを制御             | `r`, `g`, `b` (0〜255 の整数)                 |

未知の `cmd` は `status: "error"` の `result` が返ります。

---

### 4.3 result
- **方向**: サーバー → クライアント
- **目的**: `command` に対する同期応答

| フィールド | 型     | 必須 | 説明                                                        |
|-----------|--------|------|-------------------------------------------------------------|
| `cmd`     | string | ✅   | 対応するコマンド名                                          |
| `target`  | string | ✅   | 操作対象 ID                                                |
| `status`  | string | ✅   | `"success"` or `"error"`                                    |
| `message` | string | 任意 | 追加情報。`success` でも補足 (例: 既に接続済み) を返すことがあります。 |

`status: "error"` の場合でも HTTP レベルは 200 で返るため、クライアントは JSON 内容で成否を判断してください。`require_result: false` のコマンドが成功すると `result` は送信されませんが、失敗した場合は `status: "error"` の `result` が必ず返ります。

---

### 4.4 query
- **方向**: クライアント → サーバー
- **目的**: Toio 状態の取得

| フィールド | 型     | 必須 | 説明                                  |
|-----------|--------|------|---------------------------------------|
| `info`    | string | ✅   | 要求する情報種別 (`battery`, `position`) |
| `target`  | string | ✅   | 状態を問い合わせる Toio ID            |
| `notify`  | bool   | 任意 | `info: "position"` の場合に使用。`true` でサーバー側の更新通知を購読し、`false` または省略で単発レスポンス。サーバーは `response.notify` で現在の購読状態を通知します。 |

サーバーは接続済みデバイスのキャッシュ状態を返します。未接続の場合は `message: "Device not connected"` を含む `response` を返します。

---

### 4.5 response
- **方向**: サーバー → クライアント
- **目的**: `query` の結果送信

成功時 (単発クエリ):
| フィールド        | 型          | 必須 | 説明                         |
|------------------|-------------|------|------------------------------|
| `info`           | string      | ✅   | クエリ種別 (`battery` 等)    |
| `target`         | string      | ✅   | 対象 Toio ID                 |
| `notify`         | bool        | 任意 | 現在の購読状態。購読中は `true`、単発応答は `false` または省略。 |
| `battery_level`  | number/int  | 条件 | `info` が `battery` の場合   |
| `position`       | object      | 条件 | `info` が `position` の場合 (`x`, `y`, `angle`, `on_mat`) |

挙動:
- 1 回目の `response` で最新位置を返却したあと、サーバーは Toio から位置更新通知を受けるたびに同じ `type: "response"` を送信します。
- `notify: false` (または省略) の `query` を再送すると購読を解除します。解除が完了したレスポンスは `notify: false` になります。Toio の切断 (`disconnect` コマンド) や WebSocket Close でも自動解除されます。

失敗時:
| フィールド | 型     | 必須 | 説明                                           |
|-----------|--------|------|------------------------------------------------|
| `info`    | string | ✅   | クエリ種別                                     |
| `target`  | string | ✅   | 対象 Toio ID                                   |
| `message` | string | ✅   | エラー内容 (`Device not connected`, `Unknown query` など) |

---

### 4.6 error
- **方向**: サーバー → クライアント
- **目的**: プロトコルレベルの例外通知 (例: 未知の `type`, JSON 解析失敗)

| フィールド | 型     | 必須 | 説明                         |
|-----------|--------|------|------------------------------|
| `message` | string | ✅   | エラー理由 (`Unknown message type` など) |

クライアントは `error` を受信した場合、送信したメッセージを再確認して再送／切断を判断します。

---

## 5. メッセージ例

### 5.1 接続時の system 通知
```json
{
  "type": "system",
  "payload": {
    "status": "connected",
    "message": "WebSocket connection established."
  }
}
```

### 5.2 connect 成功/再接続
```json
{
  "type": "command",
  "payload": {
    "cmd": "connect",
    "target": "685"
  }
}
```
```json
{
  "type": "result",
  "payload": {
    "cmd": "connect",
    "target": "685",
    "status": "success"
  }
}
```
`require_result` で応答を抑制する例:
```json
{
  "type": "command",
  "payload": {
    "cmd": "move",
    "target": "685",
    "require_result": false,
    "params": {
      "left_speed": 30,
      "right_speed": 30
    }
  }
}
```
すでに接続済みの場合:
```json
{
  "type": "result",
  "payload": {
    "cmd": "connect",
    "target": "685",
    "status": "success",
    "message": "Device already connected"
  }
}
```

### 5.3 disconnect
```json
{
  "type": "command",
  "payload": {
    "cmd": "disconnect",
    "target": "d8J"
  }
}
```
```json
{
  "type": "result",
  "payload": {
    "cmd": "disconnect",
    "target": "d8J",
    "status": "success"
  }
}
```

### 5.4 move / led
```json
{
  "type": "command",
  "payload": {
    "cmd": "move",
    "target": "685",
    "params": {
      "left_speed": 50,
      "right_speed": 75
    }
  }
}
```
```json
{
  "type": "command",
  "payload": {
    "cmd": "led",
    "target": "J9r",
    "params": {
      "r": 255,
      "g": 0,
      "b": 0
    }
  }
}
```

### 5.5 query / response
```json
{
  "type": "query",
  "payload": {
    "info": "battery",
    "target": "685"
  }
}
```
```json
{
  "type": "response",
  "payload": {
    "info": "battery",
    "target": "685",
    "battery_level": 85
  }
}
```
位置情報:
```json
{
  "type": "response",
  "payload": {
    "info": "position",
    "target": "d8J",
    "position": {
      "x": 150,
      "y": 200,
      "angle": 90,
      "on_mat": true
    }
  }
}
```
購読モード:
```json
{
  "type": "query",
  "payload": {
    "info": "position",
    "target": "d8J",
    "notify": true
  }
}
```
最新位置の初回レスポンス:
```json
{
  "type": "response",
  "payload": {
    "info": "position",
    "target": "d8J",
    "notify": true,
    "position": {
      "x": 150,
      "y": 200,
      "angle": 90,
      "on_mat": true
    }
  }
}
```
その後、位置が変わるたびに `notify: true` を含む `response` がサーバーからプッシュされます。購読解除したい場合は `notify: false` の `query` を送信すると、解除完了レスポンスは `notify: false` になります。

### 5.6 query 失敗 / error 応答
```json
{
  "type": "response",
  "payload": {
    "info": "battery",
    "target": "685",
    "message": "Device not connected"
  }
}
```
```json
{
  "type": "error",
  "payload": {
    "message": "Unknown message type"
  }
}
```

---

## 6. 実装／拡張に関する注意
- `params` は省略可能ですが、JSON スキーマを作成する場合は空オブジェクトを許容してください。
- `result` / `response` で追加フィールドが増える場合があります。未使用フィールドは無視し、必要なキーのみ参照してください。
- 新しい `cmd` や `info` を追加する際は、本仕様のテーブルに追記し、`status` と `message` の扱いを明示してください。
- 帯域削減が必要な場合は `require_result: false` を活用し、失敗検知が必要なコマンドでは `status: "error"` の `result` が届く前提でリトライ戦略を設計してください。
- 位置情報のポーリングを避けたい場合は `notify: true` を利用して購読し、不要になったら `notify: false` を送信してサーバー側の購読リソースを解放してください。
