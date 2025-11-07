# Troubleshooting Guide

## 1. よくある症状と対処
- **Wi-Fi に参加しない**
  - `acoustics/secrets/osc_config.json`（およびそこから生成された `Secrets.h`）の SSID/Password、2.4GHz 強度、MAC フィルタ登録を確認。
  - シリアルログで `[WiFi] Connecting to ...` が繰り返される場合は AP 側ログも確認する。
- **NTP 同期が完了しない**
  - ファイアウォールで UDP/123 が閉じていないか。
  - 企業ネットワークではローカル NTP (例: `192.168.0.1`) を `NtpClient` のホストに設定してビルドする。
- **OSC 受信エラー (`AES decrypt failed`)**
  - `--osc-config` で指定している JSON とファームウェアに書き込んだ `osc_config.json` が同じか。変更後は `python3 acoustics/tools/secrets/gen_headers.py` で `Secrets.h` を再生成する。
  - 同一キーで複数端末へ送る場合は、パケット毎に異なる nonce を付与する（CTR モードの要件）。
- **再生予定時刻より遅れて鳴る**
  - `timetag` が Stick 側の NTP 時刻よりも過去の場合、直ちに再生する。PC 側が十分なリードタイム（> 300ms）を確保しているか確認。
  - Wi-Fi の再送・干渉で遅延していないか、別チャネルを試す。
- **音が出ない / ノイズが入る**
  - `manifest.json` のファイルパスが `/presets/...` になっているか。
  - WAV フォーマット（16bit, 44.1kHz 推奨）を確認。サンプルレート不一致時はノイズ化する。
- **心拍が届かない**
  - `HEARTBEAT_REMOTE_HOST` の IP を最新にする。DHCP 環境では固定 IP/MDNS の検討が必要。

## 2. 診断フロー
1. `pio device monitor -b 115200` で直近ログを取得。
2. Wi-Fi 接続後は `heartbeat` JSON に RSSI、キュー長、再生状態が載るため PC 側で収集。
3. `pc_tools/monitor/` の遅延計測スクリプトで Timetag と再生完了までの差分を記録。
4. `osc_sync_results.md` にズレ・ドロップ率を表形式で追記。
5. 問題が解決しない場合は `acoustics/process/issue_template.md` で Issue を起票し、ログと設定ファイルを添付。

## 3. 既知のバグ／注意事項
- 同一 AES キーで連続した `nonce` を再利用するとセキュリティリスク。将来対応策：パケットヘッダに IV を添付して再生側で更新。
- `StickCP2.Speaker` は WAV データを RAM 上に保持するため、大容量ファイルでは PSRAM の空き容量を確認する。長尺音源を扱う場合は `playRaw()` を用いたストリーミング実装や分割再生を検討。
- 再生中に `SPIFFS` のファイル操作を行うとバッファ枯渇の恐れ有り。プリセット更新は停止状態で実施。
