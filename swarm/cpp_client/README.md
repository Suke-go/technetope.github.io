# Toio C++ Client

シンプルな CLI から Toio 中継サーバーへ WebSocket で接続し、コマンド／クエリ／通知購読を行うクライアントです。全体設計は `docs/transport.md` / `docs/middleware.md` / `docs/cli.md` に記載しています。

## 必要要件
- C++20 対応コンパイラ (clang++ 15 以降推奨)
- CMake 3.20+
- Boost 1.78+ (`system`, `thread` コンポーネント)
- `nlohmann_json` (システムに無い場合は CMake の `FetchContent` で自動取得します)

## ビルド手順
```bash
cd swarm/cpp_client
cmake -S . -B build
cmake --build build
```

### 実行例
```bash
./build/toio_cli --fleet-config configs/fleet.yaml
```

`--fleet-config` は必須引数です。手元で試す場合は `configs/minimal.yaml` をコピーして編集するか、そのまま指定すると最小構成で起動できます。

起動後は以下のようなコマンドを入力できます。
```
help            # コマンド一覧
status          # 状態スナップショットを表示
use F3H         # 操作対象 Cube を切り替え
connect         # アクティブ Cube を接続
disconnect      # アクティブ Cube を切断
move -30 30 0   # 左右モーター出力、末尾0でresultレス省略
moveall -30 30 0# 既知すべての Cube に move を一括送信
stop            # move 0 0 のショートカット
led 255 0 0     # LED を赤に
ledall 0 0 255  # 既知すべての Cube を同じ色に
battery         # 電池クエリ
batteryall      # 既知すべての Cube に電池クエリ
pos             # 位置を単発クエリ
posall          # 全 Cube の位置を単発クエリ
subscribe       # 位置購読開始（notify true）
subscribeall    # 全 Cube の位置購読開始
unsubscribe     # 購読解除
unsubscribeall  # 全 Cube の購読解除
exit            # 終了
```

受信した `result` / `response` / `system` / `error` は JSON 文字列として標準出力に表示され、ターゲットがある場合は `[RECV][CubeID] {...}` の形式になります。

## アーキテクチャ概要

```
┌─────────────────────────────────────────────┐
│                 CLI (main.cpp)              │
│  - 引数解析 / REPL                          │
│  - FleetManager API へのコマンド委譲        │
│  - status/batteryall などの集約表示         │
└───────────────────┬─────────────────────────┘
                    │
┌───────────────────▼─────────────────────────┐
│   Middleware (FleetManager / ServerSession) │
│  - server_id/cube_id の管理                 │
│  - 状態スナップショット (CubeState)         │
│  - 受信 JSON -> リアルタイム反映            │
└───────────────────┬─────────────────────────┘
                    │
┌───────────────────▼─────────────────────────┐
│        Transport (ToioClient)               │
│  - Boost.Asio/Beast による WebSocket 接続   │
│  - JSON コマンド/クエリの送受信             │
│  - メッセージ/ログハンドラの提供           │
└───────────────────┬─────────────────────────┘
                    │
            Toio Relay Server (/ws)

静的ライブラリ `toio_lib` は Transport + Middleware レイヤーをまとめたもので、CLI を含むアプリケーションから再利用できます。
```

- `ToioClient` が単一サーバーへの WebSocket を張り、`ServerSession` がサーバー単位の複数 Cube を束ねます。
- `FleetManager` は複数の ServerSession を保持し、CLI からの `moveall`/`posall` などの一括操作を提供します。
- 到着した `response(info=="position")` や `result(cmd=="connect")` は `CubeState` に即時反映され、`status` コマンドで確認できます。

## 詳細ドキュメント
- Transport (ToioClient の通信仕様・メッセージモデル): `docs/transport.md`
- Middleware (FleetManager / ServerSession / YAML 設定): `docs/middleware.md`
- CLI (起動方法・REPL コマンド): `docs/cli.md`
- Coding Guidelines (アーキテクチャ/スタイル規約): `docs/coding_guidelines.md`
