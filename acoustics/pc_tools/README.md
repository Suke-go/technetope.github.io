# PC Tools Module

Utilities for the sequencing, monitoring, and real-time control workflow live here.  
PCツール群は C++17 + CMake を想定した再構築中であり、以下のサブモジュールに分割する。

- `scheduler/`: タイムライン編集、OSCバンドル生成、コマンド送出（`CLI11`, `nlohmann/json`, `asio`, `fmt`, `spdlog`）
- `monitor/`: 遅延計測・心拍受信・ログ集約（`asio`, `spdlog`）。`DeviceRegistry` が `/announce` を永続化し、CSV ログを追記する。
- `libs/`: 共有ユーティリティ（OSC パケット・トランスポート、DeviceRegistry、暗号フック）を配置予定

セットアップ手順例:
1. `cmake -S acoustics/pc_tools -B build -DCMAKE_BUILD_TYPE=Release`
2. `cmake --build build` で `agent_a_scheduler` / `agent_a_monitor` をビルド。
4. テスト実装後は `ctest --test-dir build` で `Catch2` ベースのテストを実行。

実行例:
- Scheduler: `./build/scheduler/agent_a_scheduler acoustics/pc_tools/scheduler/examples/basic_timeline.json --host 192.168.10.255 --port 9000 --bundle-spacing 0.02`
  - `--dry-run` で送信せず内容を確認、`--base-time 2024-05-01T21:00:00Z` でリードタイムの基準時刻を指定。
- Monitor: `./build/monitor/agent_a_monitor --host 0.0.0.0 --port 9100 --registry state/devices.json --csv logs/heartbeat.csv`
  - `--count` で受信パケット上限、`Ctrl+C` または `SIGINT` で停止。
  - `/announce` を受け取ったデバイスは `state/devices.json` に追記される。

### ファイアウォール設定例

モニタは UDP/9100 を待ち受けるため、ホスト OS のファイアウォールでポートを開放する必要があります。

- **nftables**
  ```bash
  sudo nft insert rule inet filter input udp dport 9100 accept
  sudo nft list chain inet filter input
  ```
- **iptables**
  ```bash
  sudo iptables -I INPUT -p udp --dport 9100 -j ACCEPT
  sudo iptables-save | sudo tee /etc/iptables/iptables.rules
  ```

いずれの場合も、拒否ルールより前に配置されていることを確認してください。

各ディレクトリに `README.md` / `USAGE.md` を配置し、依存バージョンとコマンド例を併記すること。  
旧Python原型コードは段階的に削除予定のため、C++実装を追加したら不要なスクリプトを整理する。
