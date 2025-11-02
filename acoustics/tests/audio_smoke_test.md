# Audio Smoke Test

この手順は SPIFFS に配置した `sample_test` プリセットで HAT SPK2 から音が出るか確認するための最小テストです。

## 事前準備
1. `acoustics/firmware/data/presets/sample.wav` にテスト用の WAV が存在することを確認する。
2. `acoustics/firmware/data/manifest.json` に以下のエントリが含まれていることを確認する。
   ```json
   {
     "id": "sample_test",
     "file": "/presets/sample.wav",
     "sample_rate": 44100,
     "gain": 1.0
   }
   ```

## 手順
1. SPIFFS を書き込む  
   ```sh
   pio run -t uploadfs
   ```
2. ファームウェアを書き込む  
   ```sh
   pio run -t upload
   ```
3. シリアルモニタを開きログを確認  
   ```sh
   pio device monitor -b 115200
   ```
   - `PresetStore` が `sample_test` を読み込んだことを確認する。
4. 任意の方法で再生をトリガーする
   - **手動トリガー**: 一時的に `setup()` の末尾で `audio_player.play(*preset, 1.0f);` を呼び出して再ビルド。
   - **OSC トリガー**: PC から `/acoustics/play` `("sample_test", <timetag>, 1.0)` を送信する。
5. SPK2 から音声が再生されることを確認する。

## 後始末
- 手動トリガーを追加した場合はコードから削除してから再ビルドすること。
- ログと結果を `acoustics/tests/osc_sync_results.md` などへ記録する。
