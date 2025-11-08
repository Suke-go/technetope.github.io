# Toio C++ Client

シンプルな CLI から Toio 中継サーバーへ WebSocket で接続し、コマンド／クエリ／通知購読を行うクライアントです。`DESIGN.md` にある方針をそのまま形にした実装になっています。

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
./build/toio_cli --id F3H --host 127.0.0.1 --port 8765 --subscribe
```

複数 Cube を同時に扱う場合は `--id` を複数回指定します。

```bash
./build/toio_cli --id 56f --id j2T --host 127.0.0.1 --port 8765
```

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
