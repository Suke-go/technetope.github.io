# M5 Scheduler/Firmware Alignment Plan

本ドキュメントは現在のM5StickC Plus2向け音響スタックに対して、暗号化・ターゲット配信・リードタイム・再生安全策・Secrets管理を統一するための作業計画です。エンジニアが迷わず実装できるよう、背景・目的・操作するファイル・検証方法をすべて列挙します。  
参照ルール: `acoustics/process/coding_rules.md`（命名/ログ/テスト方針）、`docs/masterdocs.md`（全体要件）、`acoustics/pc_tools/README.md` と `acoustics/firmware/README.md`（既存ワークフロー）。

---

## 1. 暗号化フロー完全統一 (Do)

### 1.1 目的
- 送信側（Agent A Scheduler）と受信側（Firmware）が常に AES-256-CTR を使用し、暗号化を“オプション”ではなく“必須”にする。
- 鍵・IVの定義場所を1か所に集約し、CLI/ファームともに同じ値を参照。

### 1.2 作業ステップ
1. **Secrets JSON設計**
   - 新規ファイル: `acoustics/secrets/osc_config.example.json`
   - フィールド: `wifi.primary`, `wifi.secondary`, `osc.listen_port`, `osc.key_hex`, `osc.iv_hex`, `ntp.server`, `ntp.offset`, `ntp.interval_ms`, `heartbeat.host`, `heartbeat.port`.
   - `.gitignore` に `acoustics/secrets/osc_config.json` を追加。

2. **生成スクリプト追加**
   - 位置: `acoustics/tools/secrets/gen_headers.py`（新規Python）
   - 処理: `osc_config.json` から `acoustics/firmware/include/Secrets.h` を自動生成（現行テンプレート互換）。
   - 生成ログをINFOで出力し、`platformio.ini` の `extra_scripts` でビルド前に実行。

3. **Scheduler側読み込み**
   - ファイル: `acoustics/pc_tools/scheduler/src/main.cpp`
   - 追加: `--osc-config` オプションをデフォルト `acoustics/secrets/osc_config.json` とし、CLI引数未指定でも自動でキー/IVをロード。
   - `--osc-encrypt` フラグを廃止し、キー読込に失敗した場合のみエラー終了。

4. **ドキュメント更新**
   - `acoustics/README.md` と `docs/masterdocs.md` に「Secrets生成手順」を追記。
   - `acoustics/tools/scheduler/show_osc_keys.py` を新JSONフォーマットに対応させ、キー確認を容易にする。

### 1.3 参考ファイル
- 既存の鍵利用箇所: `acoustics/firmware/src/main.cpp:311-317`, `acoustics/pc_tools/libs/src/OscTransport.cpp:80-144`, `acoustics/pc_tools/scheduler/src/main.cpp:363-427`.
- `OscEncryptor` 実装: `acoustics/pc_tools/libs/src/OscEncryptor.cpp`.

### 1.4 検証
- PlatformIOビルド時に `Secrets.h` が生成されるか確認 (`pio run -v` ログ)。
- Scheduler実行: `./build/.../agent_a_scheduler timeline.json --target-map ...` を `osc_config.json` のみで起動し、送信パケットが暗号化されているかWiresharkで確認（先頭8バイトがカウンタになっていること）。
- Firmwareシリアルログで `[OSC] AES decrypt failed` が出ないことを確認。

### 1.5 工数目安
- JSON/スクリプト/CLI改修/ドキュメントで 1.5–2.0 人日。

---

## 2. ターゲット配信のユニキャスト化 (Plan→Do)

### 2.1 目的
- GUI/CLIから論理ターゲットを指定すると、SchedulerがデバイスごとのユニキャストOSCバンドルを送信し、M5側は自分宛でないパケットを復号すらしない。
- Device Registry をGUIで編集できるようにし、IP/MACを手入力しなくて済む導線を整備。

### 2.2 作業ステップ
1. **Device Registry整備**
   - 既存ファイル `acoustics/state/devices.json` を正式仕様化し、構造（`device_id`, `ip`, `port`, `last_seen`, `tags`）を `docs/masterdocs.md` に記載。
   - Monitor (`acoustics/pc_tools/monitor/src/main.cpp`) が heartbeatを受けた際にIP/ポート情報を更新するよう修正。

2. **Scheduler送信処理改修**
   - ファイル: `acoustics/pc_tools/scheduler/src/main.cpp`
   - `TargetResolver` から得た `targetId` を軸に `std::unordered_map<device_id, Message>` を作成。
   - 各 `device_id` に対して `asio::ip::udp::endpoint` を `devices.json` から解決し、`OscSender` を都度生成するか、送信前に `setEndpoint()` を切り替える。
   - バンドル複製時に `osc::Bundle` の中身もデバイス別にフィルタ。

3. **Firmwareフィルタ（保険）**
   - `acoustics/firmware/src/modules/OscReceiver.cpp`: `/acoustics/play` で第2引数に `device_id` を追加し、ローカルIDと一致しない場合はログだけ残してスキップ。  
   - この実装はユニキャストが失敗した場合の最終防衛線として利用。

4. **GUI連携準備**
   - `acoustics/pc_tools/gui/src/main.cpp`: `devices.json` を読み取って一覧化、タグ編集を行うUIを追加（詳細は別タスクだが契機を記載）。
   - REST/IPCで Scheduler に即時プレイ命令を送れるよう API 仕様書を `acoustics/docs/gui_spec.md` に追記。

### 2.3 参考ファイル
- `TargetResolver`: `acoustics/pc_tools/scheduler/src/TargetResolver.cpp`.
- OSCバンドル構造: `acoustics/pc_tools/libs/src/OscPacket.cpp`.
- Firmware `device_id` 生成: `acoustics/firmware/src/main.cpp:288-305`.

### 2.4 検証
- Monitorで `devices.json` にIP/ポートが記録されることを確認。
- Schedulerを `--dry-run` で実行し、ログに `target=device_x` が1つずつ表示されること。
- 実機テスト: 2台以上のM5を接続し、異なるターゲットに異なるプリセットを送って意図通り再生することを確認。
- GUI側でRegistryのタグを変更→Scheduler再実行→マッピングが反映されることを確認。

### 2.5 工数目安
- Registry/Monitor 0.5人日、Scheduler送信2.0人日、Firmwareフィルタ0.5人日、GUI連携土台1.0人日 計4人日前後。

---

## 3. リードタイム既定値の堅牢化 (Plan→Do)

### 3.1 目的
- タイムラインJSONが `default_lead_time` を欠いても、システムが最低3秒のリードタイムで動作し続けるようにする。

### 3.2 作業ステップ
1. `acoustics/pc_tools/scheduler/include/acoustics/scheduler/SoundTimeline.h`
   - `double defaultLeadTime_{kMinimumLeadTimeSeconds};` に初期化変更。
2. `SoundTimeline::fromJsonFile`（`src/SoundTimeline.cpp`）
   - JSON値を読む際に `std::max(parsed, kMinimumLeadTimeSeconds)`、値が小さかった場合は `spdlog::warn`。
3. `acoustics/pc_tools/scheduler/tests/SoundTimelineTests.cpp`
   - 「`default_lead_time` 未指定→スケジュール成功、lead=3秒」となるテストケースを追加。

### 3.3 検証
- 新テストを `ctest` で実行。
- JSONから `default_lead_time: 2.0` を読み込んだ際に警告が出て3.0秒に補正されるかログを確認。

### 3.4 工数目安
- コード/テスト含め 0.5人日。

---

## 4. 再生暴発対策 (Plan→Do)

### 4.1 目的
- 過去時刻の再生要求やループ暴走を即時検出・排除し、状況をGUI/モニタで可視化する。

### 4.2 作業ステップ
1. **PlaybackQueue拡張**
   - ファイル: `acoustics/firmware/src/modules/PlaybackQueue.{h,cpp}`
   - 追加フィールド: `static constexpr size_t kMaxQueueSize = 32;`
   - `push()` 内で `queue_.size() >= kMaxQueueSize` の場合は最も古い/または新規を破棄しログ出力。
   - `push()` で `item.start_time_us < now_us - 1'000'000` の場合は登録せず `/diagnostics/reject` を送れるよう追加引数（NTPクライアント参照）を検討。

2. **Preset長メタデータ**
   - `acoustics/firmware/src/modules/PresetStore.{h,cpp}` に `duration_ms` フィールドを追加（`manifest.json` に `duration_ms` を追加）。
   - `AudioPlayer` は `current_preset_duration_ms_` を保持。

3. **playbackTask更新**
   - `acoustics/firmware/src/main.cpp:90-150` 付近で、現在再生中アイテムの期待終了時刻（`start_time_us + duration_ms * 1000`）を越えたら `audio_player.stop()`。

4. **OscReceiverタイムタグ検証**
   - `acoustics/firmware/src/modules/OscReceiver.cpp`
   - `now_us - reject_window` より古い timetag を検出したら `queue.push()` せず、`HeartbeatPublisher` 経由で `/diagnostics/reject` を即送信（新API追加）。

5. **診断メッセージ**
   - `HeartbeatPublisher` に `/diagnostics/reject` 送信用メソッドを追加。
   - Monitor (`acoustics/pc_tools/monitor/src/main.cpp`) で該当メッセージを受信し、CSV/ログ/GUIに表示。

### 4.3 検証
- 単体: `PlaybackQueue` に対してユニットテストを追加（Queue満杯/過去時刻破棄）。
- 実機: 意図的に遅延パケットを送信し、M5ログ・Monitorログ双方で拒否が可視化されるか確認。

### 4.4 工数目安
- Queue + Preset/Audio + Diagnostics全体で 2.5人日。

---

## 5. GUI連携の基盤 (Plan→Do)

### 5.1 目的
- 将来GUIからの操作を前提に、デバイスレジストリ・即時送信API・診断イベント表示の土台を整える。

### 5.2 作業ステップ
1. `gui_spec.md` を更新し、以下を定義:
   - デバイス一覧エンドポイント（例: REST `/devices`）。
   - 即時プレイAPI（POST `/play` with `{device_id, preset_id, lead_time}`）。
   - 診断イベント購読（WebSocket `/ws/diagnostics` 等）。
2. `acoustics/pc_tools/gui/src/main.cpp`
   - `devices.json` のポーリング/キャッシュ更新処理を追加。
   - 診断イベントを受信してダッシュボード表示。
3. Scheduler/MonitorとGUI間のIPCに使用するポート・認証方法を `docs/masterdocs.md` に記載。

### 5.3 工数目安
- 仕様書/土台実装で 1.0–1.5人日。

---

## 6. 作業順序の提案
1. **Secrets一元化 & 暗号化常時オン**（他タスクの前提）
2. **ターゲットユニキャスト化**（Scheduler/Firmware大改修）
3. **リードタイム既定値保護**（早めに実施し落とし穴を塞ぐ）
4. **再生暴発対策 + 診断通知**
5. **GUI基盤**

各ステップ後には `git status` で差分を確認し、`ctest`（PCツール）と `pio run`（Firmware）を通すこと。ログの読み方や動作確認手順は `acoustics/tests/frog_round_validation.md` を併用すると効率的です。

---

## 7. 検証用チェックリスト
- [ ] `pio run` 成功 / `Secrets.h` 自動生成確認。
- [ ] `cmake --build build/acoustics && ctest` パス。
- [ ] 新しい Scheduler で `--dry-run` ログにデバイス別バンドルが表示。
- [ ] 実機で遅延パケットを送信し、`/diagnostics/reject` が Monitor/GUI に反映。
- [ ] GUIでデバイスタグ編集 → `state/devices.json` に反映。
- [ ] ドキュメント (`README.md`, `masterdocs.md`, `gui_spec.md`) が最新内容を説明。

以上を完了すると、全デバイスが同一暗号化仕様でターゲット配信され、遅延耐性とGUIフレンドリな運用が実現できます。

