# Agent B 作業指示書 (M5StickC Plus2 ファームウェア担当)

## 役割概要
- M5StickC Plus2 上で動作するファームウェアを開発し、OSCバンドルの受信・キュー管理・I2S再生・状態報告を実装する。
- 詳細仕様は `acoustics/docs/masterdocs.md` の「サブシステム別 要件定義 > M5StickC Plus2側」と `acoustics/docs/device_setup.md` を参照。

## 作業ステップ
1. **リファレンス読解**
   - `acoustics/docs/masterdocs.md`, `acoustics/process/coding_rules.md`, `acoustics/docs/troubleshooting.md` を確認。
2. **環境構築 (Week 1)**
   - PlatformIO/Arduinoツールチェーンを準備し、`M5Unified`, `M5GFX`, `M5StickCPlus2` を導入。
   - `acoustics/firmware/` をチェックアウトし、サンプルビルド→Flash→シリアルログ確認。
3. **基盤機能実装 (Week 1-2)**
   - `src/modules/` に NTP同期モジュール、HOLD(GPIO4)制御、プリセット読み込み処理を作成。
   - Wi-Fi接続・再接続ロジックを実装し、`docs/device_setup.md` に設定手順を追記。
4. **OSC処理と再生 (Week 2-3)**
   - バンドル受信→暗号化復号→Timetag比較→PlaybackQueue投入の流れを実装。
   - I2S出力 (`audio.setPinout` 調整) と FreeRTOSタスクの優先度設定を行い、短尺サンプルの再生を確認。
   - デバイスIDアドバタイズ: 起動時に固有ID（MACアドレス＋設定ファイル由来の別名）をブロードキャストし、PC側のディスカバリに応答する。
5. **フィードバックと耐久性 (Week 3-4)**
   - 心拍メッセージやエラーレポートをPC側へ返す仕組みを追加。
   - 長時間テストや再起動試験を行い、`acoustics/tests/load_test.md` に結果を記録。

## テスト計画
- 単体: NTP同期、SPIFFS読み込み、OSCパーサ、PlaybackQueue。
- ハード結合: 1台で起動→再生→停止→再起動を繰り返し、HOLD制御とリソース解放を確認。
- 小規模インテグレーション: 3〜5台で同時再生してズレを測り、`acoustics/tests/osc_sync_results.md` へ記録。
- セキュリティ: 暗号化キーの設定・ローテーション手順を検証し、想定外コマンドを拒否できるか確認。
- デバイスID検証: PC側が意図したIDのみを受け付けるか、ID未登録の場合に安全に拒否できるかをテストログに記録。

## 成果物・提出先
- ソースコード・設定: `acoustics/firmware/` 配下。
- 手順書更新: `acoustics/docs/device_setup.md`, `acoustics/docs/troubleshooting.md`。
- テストログ: `acoustics/tests/` 配下へ追加。

## レポート方法
- 各機能完成時に `acoustics/process/issue_template.md` に基づくIssueを起票。
- テスト結果はログ・動画リンクを添えて共有し、週次で状況報告。

---
### 2025-11-01 進捗メモ (Codex)
- PlatformIO 設定・ライブラリ依存を `acoustics/firmware/platformio.ini` に定義。
- `src/main.cpp` 及び各モジュール（NTP/StickCP2/Wi-Fi/OSC/AES/I2S/Heartbeat）を実装し、FreeRTOS タスク分割を済ませた。
- `include/Secrets.example.h` を追加し、機密情報は `Secrets.h` に切り出す運用に変更。
- `data/manifest.json` を含む SPIFFS 用アセット雛形とロードロジック (`PresetStore`) を整備。
- `docs/device_setup.md` / `docs/troubleshooting.md` / `tests/*.md` を更新し、セットアップ〜テスト計画を反映。
