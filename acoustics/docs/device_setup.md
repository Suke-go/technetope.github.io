# M5StickC Plus2 Setup Guide

最新版ファームウェアを PlatformIO でビルドし、プリセット音源を SPIFFS に焼き込むまでの手順をまとめる。

## 1. 前提環境
- CH9102 USB シリアルドライバ（Win/macOS 公式配布版）
- PlatformIO CLI もしくは VSCode + PlatformIO 拡張
- Git で `biotope/acoustics/` を取得済み
- USB-C ケーブル（データ対応）・安定した 5V 電源

## 2. リポジトリ初期化
1. `acoustics/firmware/` で依存をインストール（`pio pkg install` は初回ビルド時に自動取得される）。
2. `include/Secrets.example.h` を `include/Secrets.h` にコピーし、以下を設定。
   - `WIFI_PRIMARY_SSID` / `WIFI_PRIMARY_PASS`
   - 必要に応じてセカンダリ SSID
   - `OSC_LISTEN_PORT`, `HEARTBEAT_REMOTE_HOST`, `OSC_AES_KEY`, `OSC_AES_IV`
3. `data/manifest.json` にプリセット一覧を記述し、対応する WAV ファイルを `data/presets/` に配置。

```jsonc
{
  "presets": [
    {
      "id": "id_bell_a",
      "file": "/presets/bell_a.wav",
      "sample_rate": 44100,
      "gain": 0.85
    }
  ]
}
```

## 3. SPIFFS へのアセット書き込み
1. `pio run -t buildfs`
2. デバイスを USB 接続し、`pio run -t uploadfs`
3. シリアルモニタで `SPIFFS` がマウントされ、`manifest.json` が読み込まれることを確認

## 4. ファームウェアのビルドと書き込み
1. `pio run` でビルド。初回はライブラリ解決に時間がかかる。
2. `pio run -t upload` でフラッシュ。ボードは `m5stick-c`（Plus2）を指定済み。
3. 必要に応じて `pio device monitor -b 115200` でログ監視。
   - Wi-Fi 接続成功 → NTP 同期 → OSC 待ち受け開始 → 心拍送信が順に表示される。

## 5. 動作確認チェックリスト
- `StickCP2` ライブラリが HOLD(GPIO4) を維持するため、電源が落ちない。
- `WiFiManager` の再接続ログが異常なく循環する（SSID/PSK 誤り時は 5 秒ごとにリトライ）。
- `NtpClient` が `Synced` ログを出力し、`nowMicros()` が単調増加する。
- PC 側から `/acoustics/play` メッセージ（サンプルスクリプト）を送信すると、該当プリセットが再生される。
- `/acoustics/stop` でキューと再生が停止する。

## 6. トラブルシュート
- **書き込み失敗 (`Failed to write to target RAM`)** : CH9102 ドライバ再インストール、USB ケーブル交換、`upload_speed` を `921600` まで下げる。
- **Wi-Fi 未接続** : Secrets の SSID/Password を確認。職場ネットワークでは MAC フィルタや 5GHz 制限に注意。
- **NTP 同期不可** : ポート 123 のファイアウォール設定を確認。ローカル NTP サーバーを `NtpClient` に設定する。
- **音が出ない** : `StickCP2.begin()` 時に `config.external_speaker.hat_spk2 = true` を設定したか確認。SPIFFS 上の WAV ファイルパスが `/` から始まっているか、PSRAM 容量に余裕があるかをチェック。
- **暗号化エラー** : PC 側と AES キー/IV が一致しているか、Timetag 形式 (秒・分解能) が期待通りか確認。

運用現場の手順書（写真付き配線図、SSID 一覧など）は別紙 `docs/process/` 系に追記する。
