# Load / Endurance Test Notes

| Date | Devices | Duration | Workload | Result | Notes |
|------|---------|----------|----------|--------|-------|
| TBD  | 1       | 4h       | `/acoustics/play` 60s interval, loop off | Pending | 後述のスクリプトで実施予定 |
| TBD  | 3       | 2h       | 同期再生 (Timetag リード 500ms) | Pending | RSSI・再接続回数を記録 |
| TBD  | 5       | 8h       | 5分ループ, 心拍監視 | Pending | ログを `logs/load_test/YYYYMMDD` に保存 |

## 実施手順（準備中）
1. `pc_tools/scripts/replay_bundle.py` で想定タイムラインを送出。
2. 各 Stick でシリアルログを保存 (`monitor --raw --logfile` 推奨)。
3. 心拍 JSON を PC 側で収集し、CPU/RSSI/Queue 長をグラフ化。
4. 異常停止時は `tests/load_test.md` に原因・再現手順・対策案を追記。

※ まだ実機テスト未実施のため、完了後に表を更新し、エビデンス（ログ/動画リンク）を貼付する。
