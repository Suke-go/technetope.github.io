# OSC Synchronization Test Results

| Date | Firmware | PC Tool | Devices | Drift (ms) | Start Offset (ms) | Packet Loss (%) | Notes / Logs |
|------|----------|---------|---------|------------|-------------------|-----------------|--------------|
| TBD  | v0.1.0   | scheduler@TBD | 3 | Pending | Pending | 0 | `logs/osc_sync/YYYYMMDD` |

Metrics to capture each run:
- Clock drift after NTP 同期（`ntp_diff_ms` 推移）
- OSC timetag 到着遅延（最小/平均/最大/標準偏差）
- 再生開始オフセット（Stick 同士のズレ, 目標 ±5ms）
- パケットロスまたは再送回数

生データ（CSV/JSON）と収集スクリプトへのリンクを列 `Notes / Logs` に記載する。
