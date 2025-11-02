# Firmware Module

M5StickC Plus2 用のファームウェア実装一式。Wi-Fi 接続、NTP 同期、OSC バンドル復号、再生キュー、I2S 出力、心拍レポートを FreeRTOS タスクで分担しています。

## ディレクトリ構成
- `platformio.ini` : PlatformIO 向けターゲット (`espressif32@6.7.0`) と依存ライブラリ (`M5Unified`, `M5StickCPlus2`, `NTPClient`, `ArduinoJson`, `CNMAT/OSC`) を定義。
- `include/Secrets.example.h` : Wi-Fi / OSC キー設定のサンプル。運用環境では `Secrets.h` を作成し、リポジトリにコミットしない。
- `data/manifest.json` : SPIFFS に焼き込むプリセットマニフェストの雛形。`/presets/*.wav` へのパスとゲインを保持。
- `data/presets/` : サンプルサウンドなど SPIFFS に載せる WAV ファイルを置く。
- `src/main.cpp` : ブートストラップ。各モジュールを初期化し FreeRTOS タスクを起動。
- `src/modules/` : 機能別クラス群
  - `WiFiManager` : 接続／再接続ロジック
  - `NtpClient` : NTP 同期・現在時刻取得
  - `PresetStore` : SPIFFS マニフェスト読み込み
  - `OscReceiver` : AES-CTR 復号・OSC Bundle 解析
  - `PlaybackQueue` : Timetag に基づく再生キュー
  - `AudioPlayer` : `StickCP2.Speaker` を利用した HAT SPK2 再生
  - `HeartbeatPublisher` : PC 側への状態レポート

## ビルド・フラッシュ手順
1. **前提ソフト** : PlatformIO CLI または VSCode 拡張、`espressif32` ボードパッケージ、CH9102 ドライバ。
2. **Secrets 設定** : `include/Secrets.example.h` を `include/Secrets.h` としてコピーし、Wi-Fi SSID/Password、OSC AES キー/IV、心拍送信先、NTP サーバ (`NTP_SERVER` など) を環境に合わせて埋める。
3. **プリセット配置** : `data/manifest.json` に再生したいファイルを登録し、`data/presets/` 下に WAV を配置。`pio run -t uploadfs` で SPIFFS に書き込む。
4. **ビルド & 書き込み**
   ```sh
   pio run
   pio run -t upload
   ```
5. **シリアルモニタ** : `pio device monitor -b 115200` でログ確認（Wi-Fi 接続、NTP 同期、OSC メッセージ、心拍送出）。

## 実装のキーポイント
- `M5StickCPlus2` ライブラリ経由で `StickCP2.begin()` を呼び出し、HAT SPK2 へのI2Sルーティングと電源保持を自動設定。
- FreeRTOS タスクで Wi-Fi/NTP/OSC/Playback/Heartbeat を独立処理。`playbackTask` が NTP 時刻に合わせてキューを消化。
- OSC パケットは AES-CTR (256bit) で復号後、`/acoustics/play`/`/acoustics/stop` のメッセージを処理。Timetag (`osctime_t` / 64bit) をマイクロ秒に変換して再生時刻を決定。
- Audio 再生は `StickCP2.Speaker` の `playWav` を利用し、SPIFFS 上の WAV ファイルを PSRAM に読み込んでから再生。`sample_test` プリセットで動作確認できる。
- RTC8563 を利用して起動時に時刻をシードし、NTP 同期成功時に RTC を更新することで再起動後も時刻を保持する。
- 心拍 (`/heartbeat`) は NTP 同期完了後にのみ送信し、OSC 引数として `[device_id, seq(int32), seconds(int32), micros(int32), queue_size(int32), playing(bool)]` を送出する。PC 側は `/announce` と合わせて `DeviceRegistry` に登録しレイテンシ測定を行う。

## 次のアクション
- `docs/device_setup.md` に沿って Wi-Fi 設定／SPIFFS へのアセット配置手順を追加する。
- `acoustics/tests/load_test.md` `osc_sync_results.md` に実機検証ログを記録。
- 暗号鍵管理・ローテーション手順を `docs/troubleshooting.md`／`process/` 系資料へ展開。
- 初回セットアップ時は `acoustics/tests/audio_smoke_test.md` に従って `sample_test` プリセットで音出し確認を行う。
