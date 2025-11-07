# Frog Round Validation (Missing Fundamental Presets)

目的は、`acoustics/sound_assets/f0/` に配置した「ミッシングファンダメンタル」系プリセットを使い、複数台の M5StickC Plus2 で輪唱を確実に再生できることを検証する。以下の手順は 2–4 台のデバイスを想定しているが、最大 30 台まで拡張できるように設計している。

## 1. 事前準備
- **ファームウェア**  
  - `acoustics/secrets/osc_config.json` に Wi-Fi / OSC 設定を反映済みで、`pio run` もしくは `python3 acoustics/tools/secrets/gen_headers.py` によって `include/Secrets.h` を生成できていること。  
  - `acoustics/firmware/data/manifest.json` にプリセット ID（例: `frog_voice_a`〜`frog_voice_d`）と `/presets` パスが登録されていること。
- **サウンドアセット**  
  - `tools/sound_gen/generate_missing_f0.py` を利用して `acoustics/sound_assets/f0/` に必要な WAV を生成済み。  
  - 同じ ID を `acoustics/sound_assets/manifest.json` にも追記し、ゲイン値を整えておく（推奨 0.75〜0.85）。  
  - 最新アセットを各デバイスへアップロード済み：  
    ```sh
    (cd acoustics/firmware && pio run -t uploadfs)
    ```
- **タイムライン素材**  
  - `acoustics/pc_tools/scheduler/examples/` を参考に、輪唱用タイムライン（例: `frog_round.json`）を用意する。  
    - `default_lead_time` は 3.5 秒以上。  
    - 各イベントは `targets` にデバイス ID を指定し、`/acoustics/play` の `args` でプリセット ID を渡す。  
    - 同期検証用に、冒頭で全デバイス一斉に短い「ピン」音を鳴らすイベントを入れておくと比較がしやすい。

## 2. タイムライン例

あらかじめサンプルを用意したい場合は `acoustics/tests/test01/` ディレクトリを使用できる。  
`frog_round.json`（タイムライン）と `targets.json`（声部→デバイス ID マッピング）を同梱しており、`--target-map` オプションで参照する。

```json
{
  "version": "1.1",
  "default_lead_time": 4.0,
  "events": [
    { "offset": -0.5, "address": "/acoustics/play", "targets": ["dev-004b12c48530", "dev-004b12c3c510"], "args": ["round_ping", 0, 0.7, 0] },
    { "offset": 0.0, "address": "/acoustics/play", "targets": ["dev-004b12c48530"], "args": ["frog_voice_a", 0, 0.82, 0] },
    { "offset": 2.0, "address": "/acoustics/play", "targets": ["dev-004b12c3c510"], "args": ["frog_voice_b", 0, 0.82, 0] },
    { "offset": 4.0, "address": "/acoustics/play", "targets": ["dev-004b12c58aa0"], "args": ["frog_voice_c", 0, 0.82, 0] },
    { "offset": 6.0, "address": "/acoustics/play", "targets": ["dev-004b12c6fbc0"], "args": ["frog_voice_d", 0, 0.82, 0] },
    { "offset": 16.0, "address": "/acoustics/stop", "args": [] }
  ]
}
```
- `offset` はリードタイム後を基準とした秒数。輪唱のインターバルは実際の楽曲テンポに合わせて調整する。  
- `round_ping` などの短音で、デバイス間の遅延・音量バランスを測る。

## 3. 検証手順
1. **NTP・ネットワーク確認**  
   - `chronyc tracking` で PC の同期状態を確認。  
   - 各デバイスが NTP に到達できることを `pio device monitor` で起動ログから確認。
2. **モニタ開始**  
   ```sh
   mkdir -p acoustics/logs
   ./build/acoustics/monitor/agent_a_monitor \
     --port 19100 \
     --csv acoustics/logs/heartbeat.csv \
     --registry state/devices.json
   ```
   - 既存ログを残す場合は `--csv` パスを日付入りにする (`logs/heartbeat_YYYYMMDD.csv`)。
3. **デバイス ID 確認**  
   - モニタ出力または `state/devices.json` から対象 ID を控える。  
   - 任意で `acoustics/sound_assets/manifest.json` にコメントを入れてプリセットと ID の対応表を残す。
4. **スケジューラ ドライラン**  
   ```sh
   ./build/acoustics/scheduler/agent_a_scheduler \
     path/to/frog_round.json \
     --host 255.255.255.255 \
     --port 9000 \
     --bundle-spacing 0.05 \
     --osc-config acoustics/secrets/osc_config.json \
     --dry-run
   ```
   - `--osc-config` にはファームウェアと共有する JSON（通常 `acoustics/secrets/osc_config.json`）を指定し、CLI/FW で同じ鍵・IV を参照する。
   - タイムタグ、ターゲット、プリセット ID が期待通りか確認。  
   - `--base-time 2025-11-02T04:00:00Z` のように基準時刻を固定すると繰り返し試験しやすい。
5. **本番送信**  
   - `--dry-run` を外し、同コマンド（`--osc-config` を含む）で送信。  
   - 送信ログにエラーがないことを確認。
6. **聴感・収録チェック**  
   - 物理的に各デバイスを配置し、輪唱が正しい順序／テンポで聞こえるか評価。  
   - 可能なら PC またはレコーダでステレオ/マルチマイク録音を取得しておく（後段のタイミング解析に使用）。
7. **モニタ結果確認**  
   - `tail -f acoustics/logs/heartbeat.csv` で遅延（latency_ms）が許容値 (<20ms 推奨) に収まるか見る。  
   - `state/devices.json` に latest_heartbeat や alias が反映されているか確認。

## 4. 成否判定
- **同期性**: `round_ping` の再生タイミングが録音波形で ±15 ms 以内に揃っている。  
- **輪唱進行**: 各声部が指定どおりの順番・間隔（例: 2 秒刻み）で開始し、重なりが崩れていない。  
- **音質**: ミッシングファンダメンタル特有の倍音構成が崩れていない（低音感が失われていない）。  
- **再現性**: 同じタイムラインを 3 回以上連続実行し、失敗（再生漏れ／誤順序／遅延閾値超過）が発生しない。

## 5. 追加観察ポイント
- **ゲイン調整**: 必要に応じてタイムライン側の振幅（OSC 第3引数）や manifest の `gain` を調整し、各声部の聴感レベルを揃える。  
- **デバイス欠席時**: タイムライン中の一部ターゲットがオフラインでも、他デバイスの進行が影響を受けないこと。  
- **復帰シナリオ**: 再生途中で電源を再投入した場合、NTP 同期完了後に次ジョブで正常動作するかを確認。  
- **ログ整理**: 実施日ごとに `logs/` と `state/` をバックアップし、異常時は `acoustics/tests/osc_sync_results.md` にメモを書く。

## 6. 成果物
- タイムライン JSON（Git 管理する場合は `pc_tools/scheduler/examples/` 配下へ）。  
- 心拍 CSV・録音データ・観察メモ（`acoustics/tests/` または `logs/` へ格納）。  
- 必要に応じて `requirements_frog_round.md` に検証結果の要約を追記する。
