# MOTHER Earth (Missing Fundamental) Playback Integration

このドキュメントは、M5StickC Plus2 ベースの音響モジュールで **MOTHER Earth (Missing Fundamental ver.)** の A セクションを再生するための実装手順をまとめたものです。右手（メロディ）と左手（ベース）を別デバイスに割り当て、OSC タイムラインでペダル指示を再現します。

## 必要な音源ファイル

- `mother_waves_mf` パイプライン、または下記「音源生成の手順」で得た WAV を `acoustics/firmware/data/presets/mother_mf/` に配置してください。
- ファイル名は下記のプリセット ID と合わせます。

| Melody | Bass  |
|--------|-------|
| A4_mf  | A2_mf |
| Csharp5_mf | Csharp3_mf |
| E5_mf  | E3_mf |
| Fsharp5_mf | — |
| D5_mf  | D3_mf |
| B4_mf  | B2_mf |
| G4_mf  | — |

> ベース側はペダル操作再現のため Stop を送らず、Play のみ送出します。

## 音源生成の手順

1. **周波数テーブルを確認する**  
   12-TET (A4 = 440 Hz) 基準での目標周波数は次のとおりです。

   | ID | 周波数 [Hz] | 備考 |
   |----|-------------|------|
   | A4_mf | 440.000 | 右手スタート音 |
   | Csharp5_mf | 554.365 | 右手 |
   | E5_mf | 659.255 | 右手 |
   | Fsharp5_mf | 739.989 | 右手 |
   | D5_mf | 587.330 | 右手 |
   | B4_mf | 493.883 | 右手 |
   | G4_mf | 392.000 | 右手 |
   | A2_mf | 110.000 | 左手ベース |
   | Csharp3_mf | 138.591 | 左手ベース |
   | E3_mf | 164.814 | 左手ベース |
   | D3_mf | 146.832 | 左手ベース |
   | B2_mf | 123.471 | 左手ベース |

2. **欠落基音トーンを生成する**  
   `acoustics/tools/sound_gen/generate_missing_f0.py` を利用すると、上記 12 音を一括で出力できます。  
   16000 Hz / 1.0 秒 / 8 partials の例:
   ```bash
   python3 acoustics/tools/sound_gen/generate_missing_f0.py \
     --output-dir acoustics/sound_assets/presets/mother_mf \
     --sample-rate 16000 \
     --duration 1.0 \
     --harmonics 8 \
     --amplitude 0.95 \
     --fade-ms 12 \
     A4_mf=440 \
     Csharp5_mf=554.365 \
     E5_mf=659.255 \
     Fsharp5_mf=739.989 \
     D5_mf=587.330 \
     B4_mf=493.883 \
     G4_mf=392.000 \
     A2_mf=110.000 \
     Csharp3_mf=138.591 \
     E3_mf=164.814 \
     D3_mf=146.832 \
     B2_mf=123.471
   ```

3. **ファームウェア側にコピーする**  
   ```bash
   rsync -av --delete acoustics/sound_assets/presets/mother_mf/ \
         acoustics/firmware/data/presets/mother_mf/
   ```
   frog 系や `sample.wav` など共通プリセットは `acoustics/firmware/data/presets/` 直下に配置します。

4. **SPIFFS に反映する**  
   1 台ずつ同じ SPIFFS イメージを焼き込みます。
   ```bash
   cd acoustics/firmware
   pio run -t uploadfs
   ```
   その後 `pio run -t upload` でファームを書き込んでください。

## タイミング仕様

- テンポ 125 BPM → 四分音符 480 ms、八分音符 240 ms。
- A セクション 1 ループ = 3.84 秒。スクリプトはループ数分だけ `n * 3.84 s + オフセット` で積み上げます。
- 右手は毎回 `/acoustics/stop` → `/acoustics/play` を同一オフセットに積み、`SoundTimeline::fromJsonFile` 内の `std::stable_sort` によって Stop が必ず先に処理されます。
- 左手は `/acoustics/play` のみでペダル保持を再現。`--final-stop-delay` で余韻を締めるタイミングを調整できます (既定 960 ms)。

## プリセット登録

`acoustics/firmware/data/manifest.json` にメロディ／ベース双方のプリセットをまとめて定義しています。`sample_rate` を調整した場合はここを更新してください。共通カタログは `acoustics/sound_assets/manifest.json` です。

## 実行ステップ

1. **音源を準備・配置する**  
   上の手順で `data/presets/mother_mf/` を更新し、`pio run -t uploadfs` → `pio run -t upload` を実行して各デバイスに反映させます。

2. **タイムラインを生成する**  
   例として 2 周分を生成し、既存ファイルを上書きします。
   ```bash
   python3 acoustics/tools/timeline/generate_mother_mf_timeline.py \
     --loops 2 \
     --melody-gain 1.0 \
     --bass-gain 1.0 \
     --output acoustics/pc_tools/scheduler/examples/mother_mf_a_section.json
   ```
   ループ数やゲインを変えたい場合は各オプションを調整します。

3. **ターゲットを 2 台に設定する**  
   タイムラインは論理ターゲット `mother_melody` と `mother_bass` を使用します。  
   `acoustics/tests/test01/targets_mother.json` を以下の内容で利用し、Stick を複数登録します（例ではメロディ側に 2 台、ベース側に 1 台）。
   ```json
   {
     "mother_melody": ["dev-004b12c3c510", "dev-004b12c3a830"],
     "mother_bass": ["dev-004b12c29994"]
   }
   ```

4. **送信内容をドライランで確認する**  
   ```bash
   ./build/scheduler/agent_a_scheduler \
     acoustics/pc_tools/scheduler/examples/mother_mf_a_section.json \
     --target-map acoustics/tests/test01/targets_mother.json \
     --host 255.255.255.255 \
     --port 9000 \
     --lead-time 4.0 \
     --osc-config acoustics/secrets/osc_config.json \
     --dry-run
   ```
   `offset` が 0.24 秒刻みで並び、ターゲットが 2 台だけになっていることを確認します。

5. **本番送信する**  
   ドライランで問題なければ `--dry-run` を外して再実行します。  
   `--osc-config` は必須であり、ファームウェアに書き込んだ JSON（`acoustics/secrets/osc_config.json`）を指すようにしてください。  
   `acoustics/tools/scheduler/run_mother_mf.sh` / `.py` を使うと、この JSON を自動解決して送信してくれます。鍵を確認したいだけなら `python3 acoustics/tools/scheduler/show_osc_keys.py` が便利です。

## 再生ロジックのポイント

- メロディ側には各イベントの直前に `/acoustics/stop` を送信し、音の切り替えを明瞭化しています。
- ベース側は `/acoustics/play` のみ送信し、SoundTimeline で同一オフセット内のイベント順序を保持するために `std::stable_sort` を適用しました。
- タイムライン末尾で両手に Stop を送ってフェードアウトを保証します。不要な場合は `--final-stop-delay 0` を指定して停止コマンドを省略できます。

## 追加テスト

- `agent_a_scheduler --dry-run` で送出内容を確認し、`offset` が想定どおり 240 ms 刻みで並んでいるかをチェックします。
- 実機テストでは、Stick の画面に表示される現在再生プリセットがタイムラインに追従しているか確認してください。
  - メロディ用デバイスで Stop が効いているか (連打しても前音が残らない)。
  - ベース用デバイスは余韻が自然に重なっているか。
- 音源長を確認する場合は `soxi -D acoustics/sound_assets/presets/mother_mf/A4_mf.wav` のように秒数を測り、1.2 秒前後で揃っているかチェックしてください。

## 今後の改善アイデア

- WAV 長さを自動測定し `length_ms` を更新するスクリプトを追加する。
- ベースの残響時間を調整した WAV を別途用意し、ペダルの踏み替えに合わせて長さを変える。
- GUI ツール (`pc_tools/gui`) と連携してライブでループ回数やゲインを変更できるようにする。
