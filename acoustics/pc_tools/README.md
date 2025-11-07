# PC Tools Module

Utilities for the sequencing, monitoring, and real-time control workflow live here.  
PCツール群は C++17 + CMake を想定した再構築中であり、以下のサブモジュールに分割する。

- `scheduler/`: タイムライン編集、OSCバンドル生成、コマンド送出（`CLI11`, `nlohmann/json`, `asio`, `fmt`, `spdlog`）。`acoustics_scheduler` ライブラリとして共通ロジックを切り出しており、CLI/GUI/サービスから同一コードパスで利用できる。
- `monitor/`: 遅延計測・心拍受信・ログ集約（`asio`, `spdlog`）。`DeviceRegistry` が `/announce` を永続化し、CSV ログを追記する。
- `libs/`: 共有ユーティリティ（OSC パケット・トランスポート、DeviceRegistry、暗号フック）を配置予定

セットアップ手順例:
1. `cmake -S acoustics/pc_tools -B build -DCMAKE_BUILD_TYPE=Release`
2. `cmake --build build` で `agent_a_scheduler` / `agent_a_monitor` をビルド。
4. テスト実装後は `ctest --test-dir build` で `Catch2` ベースのテストを実行。

実行例:
- Scheduler: `./build/scheduler/agent_a_scheduler acoustics/pc_tools/scheduler/examples/basic_timeline.json --host 192.168.10.255 --port 9000 --bundle-spacing 0.02 --target-map mappings/voices.json --default-targets dev-001,dev-002,dev-003 --osc-config acoustics/secrets/osc_config.json`
  - `--osc-config`（デフォルト `acoustics/secrets/osc_config.json`）でファームウェアと共有する鍵/IV を読み込み、AES-256-CTR を常時有効化する。CLI 実行前に `acoustics/tools/secrets/gen_headers.py` で JSON → `Secrets.h` を生成しておくこと。
  - `--dry-run` で送信せず内容を確認。表示にはタイムタグ、ターゲット ID、プリセット ID が含まれる。
  - `--base-time 2024-05-01T21:00:00Z` でリードタイムの基準時刻を指定。タイムラインおよびオーバーライド値は 3 秒以上でなければならない。
- Monitor: `./build/monitor/agent_a_monitor --host 0.0.0.0 --port 19100 --registry state/devices.json --csv logs/heartbeat.csv`
  - `--count` で受信パケット上限、`Ctrl+C` または `SIGINT` で停止。
  - `/announce` を受け取ったデバイスは `state/devices.json` に追記される。

## Testing
- `cmake -S acoustics/pc_tools -B build/acoustics` で依存解決とビルド設定を実施。テストのみ再ビルドする場合は `cmake --build build/acoustics --target acoustics_scheduler_tests` を利用。
- すべての Catch2 テストは `ctest --test-dir build/acoustics --output-on-failure` で実行。失敗ケースのみ確認する際は `ctest --test-dir build/acoustics --rerun-failed --output-on-failure` が便利。
- 個別テストやタグ指定が必要なら `build/acoustics/scheduler/acoustics_scheduler_tests -v [scheduler]` のように Catch2 バイナリを直接起動。
- CI に組み込む場合も同じ手順（`cmake` → `cmake --build` → `ctest`）をジョブ内で順に呼び出すだけで再現可能。

### ファイアウォール設定例

モニタは UDP/19100 を待ち受けるため、ホスト OS のファイアウォールでポートを開放する必要があります。

- **nftables**
  ```bash
  sudo nft insert rule inet filter input udp dport 19100 accept
  sudo nft list chain inet filter input
  ```
- **iptables**
  ```bash
  sudo iptables -I INPUT -p udp --dport 19100 -j ACCEPT
  sudo iptables-save | sudo tee /etc/iptables/iptables.rules
  ```

いずれの場合も、拒否ルールより前に配置されていることを確認してください。

各ディレクトリに `README.md` / `USAGE.md` を配置し、依存バージョンとコマンド例を併記すること。  
旧Python原型コードは段階的に削除予定のため、C++実装を追加したら不要なスクリプトを整理する。
