# Toio CLI Middleware Design

複数の toio サーバーを CLI から横断的に扱うミドルウェア案を整理する。既存の `swarm/cpp_client/src/main.cpp` の CLI 振る舞いを踏襲しつつ、共通 API と各 toio の最新状態配列を保持するレイヤーを新設する。

## 要件整理
- インターフェースは現状どおり CLI で提供する。将来 REST / WebSocket へ拡張できる設計にしておく。
- 状態配列は「最新のみ」を保持すればよい。履歴やロギングは別手段に任せる。
- cube_id の衝突は起こらないため、各 Cube は `server_id` と `cube_id` の組をキーにして十分。
- 追跡対象フィールドは位置・バッテリー・LED カラー・接続状態。購読を ON にしている Cube では随時更新し、購読していない Cube は明示的クエリ時に更新する。

## コンポーネント
### FleetManager
- 役割: サーバー横断のオーケストレーション。CLI からのリクエストを受けて適切な ServerSession に委譲しつつ、全 Cube の最新状態スナップショットを返す。
- 主な公開 API（CLI から呼ばれる想定）:
  - `use(server_id, cube_id)` / `active_target()` … CLI カーソル管理。
  - `connect(server_id, cube_id)` / `disconnect(...)`
  - `move(server_id, cube_id, left, right, require)` / `move_all(left, right, require)`
  - `set_led(...)` / `set_led_all(...)`
  - `query_battery(...)`, `query_position(...)`, `toggle_subscription(...)`
  - `list_cubes()` … 状態配列の取得。
- 既存 `main.cpp` のコマンドパーサーはこの API を呼び出すだけに簡略化できる。

### ServerSession
- 1 toio サーバー = 1 `ToioClient` を保持し、接続ライフサイクルと送受信の直列化を担当。
- 役割:
  - CLI 起動時に設定ファイル（例: `fleet.yaml`）を読み込んで `host/port/endpoint` を設定。
  - `connect_cube`, `disconnect_cube`, `send_move`, `set_led`, `query_*` など `ToioClient` の薄いラッパー。
  - 送信はスレッドセーフなキュー + worker thread で直列化し、`ToioClient` の `write_mutex_` 争奪を防ぐ。
  - `set_message_handler` を使って受信 JSON を解析し、CubeState の該当フィールドを更新して FleetManager へ通知。
- 各 ServerSession は `std::unordered_map<std::string, CubeState>` を持ち、`cube_id -> state` を最新化。FleetManager からは読み取り専用で参照する。

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
- 位置とバッテリー、LED は取得できた時点で上書きする。`connected` は `result`/`system` メッセージを監視。
- FleetManager はすべての CubeState から配列（`std::vector<CubeSnapshot>`) を生成し CLI へ表示する。CLI 側は JSON 文字列に整形して `status` コマンドなどに利用できる。

## CLI との統合
1. 起動時フロー
   - `main.cpp` で引数 `--fleet-config path` を受け取る（従来の `--host`/`--id` も後方互換として FleetManager 初期化に渡せる）。
   - FleetManager が各 ServerSession を生成し、`auto_connect` / `auto_subscribe` を設定。
   - 既存の `subscriptions`/`known_cubes`/`active_cube` ロジックは FleetManager 内部に移行する。

2. コマンド処理
- CLI は `tokenize` → `switch/case` を維持しつつ、実際の仕事は FleetManager へ委譲。
- `moveall`, `ledall`, `batteryall`, `posall`, `subscribeall`, `unsubscribeall`
  など複数 Cube 対象コマンドは FleetManager の `*_all` ヘルパーが `servers`
  を横断。
   - `status`（新設想定）で FleetManager の `list_cubes()` を要約表示すると状態配列の可視化が容易。

3. 受信ハンドラ
   - ServerSession は `ToioClient::set_message_handler` で受信 JSON を解析し、`target` / `payload.target` を抽出して CubeState を更新。
   - 更新後に FleetManager へ `on_cube_state(server_id, cube_state)` コールバックを呼び、CLI が購読していれば表示できる。

## 設定ファイル仕様
CLI の柔軟性を保つため、FleetManager へ渡す設定は YAML ファイルで表現する。人が手で編集しやすく、コメントや階層を自然に記述できるため複数サーバー構成の管理がしやすい。

```yaml
servers:
  - id: 5b-00               # CLI で参照する server_id
    host: 127.0.0.1
    port: 8765
    endpoint: /ws
    default_require_result: true  # 省略時は false
    cubes:
      - id: F3H
        auto_connect: true        # CLI 起動時に connect/send_query を実行
        auto_subscribe: true      # 起動直後に position notify を有効化
        initial_led: [0, 0, 255]  # 任意: connect 後にセットする色
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

### フィールド一覧
- `servers[]`: 1 つの toio 中継サーバーに対応。配列順に接続を試みる。
  - `id` (必須): CLI 内でサーバーを識別する文字列。`use <server>:<cube>` のような拡張を見据えている。
  - `host` / `port` / `endpoint` (必須): `ToioClient` の接続先。
  - `default_require_result` (任意): そのサーバーへのコマンド送信時の `require_result` 既定値。
  - `cubes[]`: サーバー配下で管理する Cube 群。
    - `id` (必須): Cube ID。
    - `auto_connect` (任意, 既定 false): 起動時に `connect_cube` を呼ぶか。
    - `auto_subscribe` (任意, 既定 false): 起動直後に `query_position(..., true)` を送るか。
    - `initial_led` (任意): `[R,G,B]` の 0-255 配列。`auto_connect` と組み合わせて接続確認に利用。

### CLI からの利用
- `./toio_cli --fleet-config fleet.yaml` を指定すると、FleetManager が YAML を読み込み ServerSession を初期化する。
- 従来の `--host`, `--port`, `--endpoint`, `--id` を指定した場合は「単一サーバー構成」として扱い、内部的に `servers` を動的生成する。`--subscribe` は `auto_subscribe=true` 相当。
- 設定ファイルを省略した場合は既存 CLI と同じ挙動を維持しつつ、`status` コマンドなど FleetManager ベースの機能は利用可能。

## ステートマネジメント
- 単一の最新値のみ保持する方針のため、`CubeState` は常に上書き更新で OK。
- CLI 表示用には `std::vector<CubeState> snapshot()` を FleetManager が提供し、`main.cpp` から `status` コマンドで呼び出す。
- Thread safety: ServerSession 内部では `std::shared_mutex` で CubeState を保護し、FleetManager 側で `shared_lock` による読み出しを行う。

## 開発ステップ案
1. FleetManager / ServerSession / CubeState のヘッダー定義を `include/toio/middleware/` 以下に追加。
2. 既存 `main.cpp` を FleetManager 経由の呼び出しに段階的に置き換え（CLI 表層はそのまま）。
3. 状態表示用 `status` コマンドを追加し、CubeState の中身を CLI で確認できるようにする。
4. 必要に応じて将来の REST / WebSocket 公開に備え、FleetManager の API を CLI から独立した形でまとめておく。

## オープン課題
- ServerSession の接続再試行ポリシー（指数バックオフ、最大リトライ回数など）をどうするか。
- CLI 以外（例: GUI, python スクリプト）から FleetManager を利用する際の API 形態。
- LED カラーの既定値（接続直後にサーバーから通知されない場合はどう扱うか）。

この方針に沿って実装すれば、複数 toio サーバーを単一 CLI から扱いつつ、各 Cube の最新状態を簡潔にトラッキングできる。
