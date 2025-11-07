# Test 01 — Frog Round (Missing Fundamental)

`test01` は、ミッシングファンダメンタル系プリセットを用いた 4 声の輪唱検証セット。  
タイムラインは論理ターゲット（`voice_a`〜`voice_d`）を使用し、PC スケジューラの `--target-map` 機能で実機 ID に割り当てる。  
単体検証時はすべての `voice_*` を同じデバイス ID に割り当てれば、1 台で「カエルのうた」の各声部を順番に再生できる。

## 構成ファイル
- `frog_round.json` — `/acoustics/play` イベントと停止コマンドを含むタイムライン。
- `targets.json` — 各声部に割り当てるデバイス ID。一時的に変更する場合は編集し、コミット時に戻すか別ファイルとして扱う。

## 実行例
```sh
# ビルド済みを前提とする
./build/acoustics/scheduler/agent_a_scheduler \
  acoustics/tests/test01/frog_round.json \
  --host 255.255.255.255 \
  --port 9000 \
  --bundle-spacing 0.05 \
  --target-map acoustics/tests/test01/targets.json \
  --osc-config acoustics/secrets/osc_config.json \
  --dry-run

# 問題なければ --dry-run を外して送信
```

- `round_ping` イベント（offset = -0.5）は本番開始直前に全デバイスで短音を鳴らすことで、録音した波形から同期誤差を計測できるようにするため。  
- 音量は `args` 内の第3引数（ゲイン）で微調整する。根本的なラウドネス調整は manifest 側の `gain` で行う。
- `--osc-config` にはファームウェアと共有する JSON（通常は `acoustics/secrets/osc_config.json`）を指定し、暗号キー/IV を一元管理する。

## 運用メモ
- `targets.json` の ID は `state/devices.json` や `logs/heartbeat*.csv` に合わせて更新する。  
- 追加デバイスを使う場合は `voice_e` などを追記し、タイムラインにもイベントを追加する。  
- 実施結果や録音は `acoustics/tests/` 直下の検証ログへ格納すること。 `frog_round_validation.md` のチェックリストを併用する。
