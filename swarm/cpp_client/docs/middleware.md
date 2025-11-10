# Middleware Layer (FleetManager & ServerSession)

FleetManager / ServerSession は複数の Toio Relay Server をまとめ、Cube ごとの最新状態と共通 API を提供するレイヤーです。CLI を含む上位アプリケーションはこのミドルウェアを通じて `moveall`, `batteryall`, `subscribeall` などの操作を行います。

## コンポーネント

### FleetManager
- サーバー横断のオーケストレーション。`server_id` と `cube_id` をキーに `ServerSession` を管理。
- 主な API:
  - `use(server_id, cube_id)` / `active_target()` … カレントターゲット管理。
  - `connect`, `disconnect`, `move`, `set_led`, `query_battery`, `query_position`.
  - `move_all`, `set_led_all`, `query_battery_all`, `query_position_all`,
    `toggle_subscription_all`.
  - `snapshot()` … `CubeState` 配列を返し、UI で `status` 表示に利用。
- 受信した `CubeState` 更新イベントを保持し、CLI などへ通知できるコールバックを備える。

### ServerSession
- ToioClient を 1 サーバーにつき 1 つ保持し、接続ライフサイクルと送受信を担当。
- `connect_cube`, `disconnect_cube`, `send_move`, `set_led`, `query_*` などの薄いラッパーで ToioClient API をまとめている。
- `set_message_handler` で受信 JSON を解析し、`CubeState` の `connected` / `battery` / `position(on_mat)` などを更新して FleetManager へ渡す。
- `cube_ids()` / `snapshot()` により、管理中の Cube を列挙したり状態配列を提供する。

### CubeState
```cpp
struct CubeState {
  std::string server_id;
  std::string cube_id;
  bool connected = false;
  std::optional<Position> position; // x, y, angle, on_mat, sensor timestamp
  std::optional<int> battery_percent;
  struct { uint8_t r, g, b; } led = {0, 0, 0};
  std::chrono::steady_clock::time_point last_update;
};
```
- 最新値のみ保持するシンプルな設計。購読で届いた `position` も単発クエリも同じ場所に上書きする。
- Thread-safety は `std::shared_mutex` で確保し、読み取りは `shared_lock`、書き込みは `unique_lock` で行う。

## CLI との統合
1. 起動時に `--fleet-config path` を指定すると YAML を読み込み、サーバーごとに `ServerSession` を生成する。
2. CLI の `tokenize` → `switch/case` は維持しつつ、実処理はすべて FleetManager の API に委譲。
3. `status` コマンドは `snapshot()` の内容を整形表示し、`moveall`/`ledall`/`posall` などは `*_all` ヘルパーを呼ぶだけで済む。

## 設定ファイル仕様
FleetManager は YAML で渡されたサーバー一覧を元に初期化する。人間が編集しやすく、複数サーバー構成の管理が容易。

```yaml
servers:
  - id: 5b-00
    host: 127.0.0.1
    port: 8765
    endpoint: /ws
    default_require_result: true
    cubes:
      - id: F3H
        auto_connect: true
        auto_subscribe: true
        initial_led: [0, 0, 255]
      - id: J2T
        auto_connect: false
        auto_subscribe: false
  - id: remote-lab
    host: relay.example.com
    port: 9000
    endpoint: /custom
    cubes:
      - id: LAB01
```

### フィールド
- `servers[].id`: CLI やログでサーバーを識別するキー。
- `host`/`port`/`endpoint`: ToioClient 接続先。
- `default_require_result`: 省略時 false。コマンド送信時の `require_result` 既定値。
- `cubes[]`: サーバー配下の Cube 列挙。
  - `auto_connect`: 起動直後に `connect_cube` を呼ぶか（省略時 true）。
  - `auto_subscribe`: 起動直後に `query_position(..., true)` を送るか（省略時 true）。
  - `initial_led`: `[R,G,B]` (0-255)。接続確認に使える任意フィールド。省略時は 0-255 のランダムな RGB が自動で設定される。

### CLI での利用
- `./toio_cli --fleet-config configs/fleet.yaml` で YAML を読み込み、複数サーバーを同時制御。
- CLI は常に YAML 設定ファイル経由で構成を読み込む。ローカル検証用には `configs/minimal.yaml` をそのまま利用できる。
- 設定ファイルなしのモードは廃止されており、必ず `--fleet-config` を指定して起動する。

## 今後の検討事項
- ServerSession の再接続ポリシー（指数バックオフ・最大リトライ回数）。
- CLI 以外（GUI, Python スクリプト等）から FleetManager を利用する際の API 形態。
- LED カラーの初期値をサーバーから通知しない場合の扱い。
