# CLI Layer

`toio_cli` は FleetManager を利用して複数の Toio サーバー／Cube を操作する対話型ツールです。ここでは起動方法、コマンド、入出力のルールをまとめます。

## 起動方法

```bash
./build/toio_cli --fleet-config configs/fleet.yaml
```

`--fleet-config` は必須で、指定した YAML に従って FleetManager が初期化されます。`configs/minimal.yaml` を渡すとローカル検証用の最小構成が読み込まれます。

### 主な引数

| 引数 | 説明 |
|------|------|
| `--fleet-config <path>` | 必須。YAML (例: `configs/fleet.yaml`) を読み込み。 |
| `--help` / `-h` | 使い方を表示して終了。 |

## コマンド一覧

```
help            # コマンド一覧
status          # 状態スナップショット表示
use F3H         # 操作対象 Cube を切り替え（server:cube 形式も可）
connect         # アクティブ Cube を接続
disconnect      # アクティブ Cube を切断
move -30 30 0   # 左右モーター、末尾0で result レスポンスを抑制
moveall -30 30 0# すべての Cube に move
stop            # move 0 0 のショートカット
led 255 0 0     # LED を赤に
ledall 0 0 255  # すべての Cube を同じ色に
battery         # アクティブ Cube の電池クエリ
batteryall      # すべての Cube の電池クエリ
pos             # アクティブ Cube の位置クエリ
posall          # すべての Cube の位置クエリ
subscribe       # 位置購読開始（notify true）
subscribeall    # すべての Cube の購読開始
unsubscribe     # 位置購読解除
unsubscribeall  # すべての Cube の購読解除
exit / quit     # 切断して終了
```

`status` は `FleetManager::snapshot()` の内容を表形式で表示し、`CubeState` の `connected`, `battery`, `position(on_mat)` を確認できます。

## 入出力

- CLI から送信したコマンドの応答 (`result`) やクエリ結果 (`response`)、サーバー通知 (`system`/`error`) は `ToioClient` の `MessageHandler` から `[RECV][server:cube] {...}` 形式で標準出力に表示されます。
- `require_result=false` のコマンドは成功時に `result` が返らないため、CLI でもレスポンス待ちをおこないません。失敗時のみ `status: "error"` の `result` が到着します。

## 設定ファイル (YAML) の位置

```
configs/
  fleet.yaml    # 実運用向けの複数サーバー設定
  minimal.yaml  # 単体テスト・ドキュメント向けの最小構成
```

`fleet.yaml` の構造や各フィールドの説明は `docs/middleware.md` を参照してください。
