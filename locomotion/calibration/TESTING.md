# 実機テスト手順書

**対象**: locomotion/calibration モジュール  
**対象カメラ**: Intel RealSense D415  
**対象プラットフォーム**: Apple Silicon macOS (M1/M2/M3), Linux  
**最終更新**: 2025-11-09

---

## 1. 前提条件

### 1.1 ハードウェア要件

- **Intel RealSense D415** カメラ（ファームウェア 5.16.0.1 以上推奨）
- **USB 3.0 ケーブル**（USB 3.0 ポートへの接続推奨）
- **ChArUco ボード**（5×7グリッド、45mm正方形、33mmマーカー）
- **toio プレイマット**（A3 Simple Playmat #01推奨）
- **カメラマウント**（高さ約2600mm、床面に対して垂直）

### 1.2 ソフトウェア要件

- **OS**: macOS 13.0+ (Ventura/Sonoma) または Linux (Ubuntu 20.04+)
- **依存ライブラリ**:
  - librealsense2 (2.54.0+)
  - OpenCV 4.5.0+ (aruco モジュール含む)
  - spdlog
  - nlohmann/json
- **ビルドツール**: CMake 3.20+, C++20対応コンパイラ

### 1.3 環境セットアップ

#### macOS (Apple Silicon)

```bash
# 依存ライブラリのインストール
brew install librealsense opencv spdlog nlohmann-json

# カメラプロセスの停止（毎回実行推奨）
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true
```

#### Linux

```bash
# 依存ライブラリのインストール
sudo apt update
sudo apt install librealsense2-dev libopencv-dev libspdlog-dev nlohmann-json3-dev

# udev ルールの設定（RealSense デバイスアクセス用）
# 詳細: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
```

---

## 2. ビルド手順

### 2.1 ビルドの実行

```bash
cd /Users/ksk432/technetope/locomotion/calibration

# ビルドディレクトリの作成
cmake -B build -DLOCOMOTION_BUILD_TESTS=ON

# ビルド実行
cmake --build build

# ビルド結果の確認
ls -la build/*calibration* build/*qc
```

### 2.2 ビルドターゲット

- `capture_calibration`: CLI キャリブレーションツール
- `capture_calibration_interactive`: インタラクティブGUIキャリブレーションツール
- `run_calibration_qc`: QC自動化ツール
- `test_floor_plane_estimator`: ユニットテスト
- `test_calibration_pipeline_integration`: 統合テスト

---

## 3. テスト準備

### 3.1 設定ファイルの確認

キャリブレーション設定ファイルを確認・準備します。

```bash
# 設定ファイルの確認
cat config/calibration_config.json

# 低解像度設定（テスト用）
cat calibration_config_low_res.json

# toioプレイマットレイアウトの確認
cat config/toio_playmat.json
```

**重要な設定項目:**

- `playmat_layout_path`: toioプレイマットレイアウトJSONのパス（相対パス可）
- `board_mount_label`: ChArUcoボードのマウントラベル（`center_mount_nominal`）
- `min_charuco_corners`: 最小検出コーナー数（デフォルト: 12）
- `max_reprojection_error_id`: 最大再投影誤差（デフォルト: 8.0 ID units）

### 3.2 カメラ接続の確認

```bash
# macOS: カメラプロセスを停止
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true

# RealSense デバイスの列挙（sudo必須、macOS）
sudo /opt/homebrew/bin/rs-enumerate-devices

# USB接続の確認（macOS）
system_profiler SPUSBDataType | grep -A 5 "Intel RealSense"

# USB3接続の確認（"Up to 5 Gb/s" と表示されればOK）
```

**注意事項:**

- macOSでは **sudo 実行が必須**（USBアクセスのため）
- USB 3.0接続推奨（USB 2.0では15 FPS制限）
- カメラはMac本体のUSB-Cポートに直挿し推奨

### 3.3 ChArUcoボードの準備

1. **ボードの印刷**
   ```bash
   # マーカーシートの生成（必要に応じて）
   python tools/generate_marker_sheet.py --output markers_a4_45mm.png
   ```

2. **ボードの配置**
   - toioプレイマット上の中央付近に配置
   - 床面に対して平行（傾き ≤ 5°）
   - カメラから40-80cmの距離
   - 照明条件: 均一な明るさ、グレア・影を避ける

3. **ボードサイズの確認**
   - 正方形サイズ: 45mm
   - マーカーサイズ: 33mm
   - グリッド: 5列×7行

---

## 4. テスト手順

### 4.1 基本キャリブレーションテスト（CLI）

#### テスト1: パス解決の検証

**目的**: 設定ファイルからの相対パス解決が正しく動作することを確認

```bash
cd /Users/ksk432/technetope

# プロジェクトルートから実行（相対パス解決のテスト）
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true
sudo ./locomotion/calibration/build/capture_calibration \
  locomotion/calibration/calibration_config_low_res.json \
  locomotion/calibration/calib_result.json
```

**期待される結果:**
- `playmat_layout_path: "config/toio_playmat.json"` が正しく解決される
- ログに "Loaded playmat layout from 'config/toio_playmat.json' (base dir: '...')" と表示
- エラーなくキャリブレーションが完了

**検証項目:**
- [ ] パス解決が成功（エラーメッセージなし）
- [ ] toio_playmat.json が正しく読み込まれる
- [ ] キャリブレーション結果JSONが生成される

#### テスト2: 基本的なキャリブレーション実行

**目的**: キャリブレーションパイプラインが正常に動作することを確認

```bash
cd /Users/ksk432/technetope/locomotion/calibration

# カメラプロセスの停止
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true

# キャリブレーション実行
sudo ./build/capture_calibration \
  calibration_config_low_res.json \
  calib_result.json
```

**期待される結果:**
- ChArUcoボードが検出される（12個以上のコーナー）
- 床面推定が成功（標準偏差 ≤ 8.0mm）
- ホモグラフィ計算が成功（再投影誤差 ≤ 8.0 ID units）
- `calib_result.json` が生成される

**検証項目:**
- [ ] キャリブレーションが成功（exit code 0）
- [ ] 出力JSONファイルが生成される
- [ ] JSONスキーマ v2.0 に準拠している
- [ ] `reprojection_error_px` が含まれている
- [ ] `toio_coordinate_transform` セクションが含まれている
- [ ] `validation` セクションが含まれている

#### テスト3: キャリブレーション結果の検証

**目的**: 生成されたキャリブレーション結果の品質を検証

```bash
# 結果JSONの確認
cat calib_result.json | jq '.'

# 検証結果の確認
cat calib_result.json | jq '.validation'

# 品質メトリクスの確認
cat calib_result.json | jq '.quality_metrics'

# toio座標変換の確認
cat calib_result.json | jq '.toio_coordinate_transform'
```

**検証項目:**

1. **再投影誤差**
   - `quality_metrics.reprojection_error_px` ≤ 8.0 pixels
   - `quality_metrics.reprojection_error_id` ≤ 8.0 ID units
   - `quality_metrics.reprojection_error_floor_mm` が記録されている

2. **床面推定**
   - `floor_plane.std_mm` ≤ 8.0mm
   - `floor_plane.inlier_ratio` ≥ 0.7
   - `camera.camera_height_mm` が 2500-2700mm の範囲内

3. **toio座標変換**
   - `toio_coordinate_transform.transform_error_id` ≤ 2.0 ID units
   - `toio_coordinate_transform.coverage_area_toio_id` が正しく設定されている
   - `toio_coordinate_transform.playmat_id` が "a3_simple" である

4. **検証結果**
   - `validation.passed` が `true`
   - `validation.checks` の各項目が "PASS" または "WARN"
   - `validation.warnings` が空または許容可能な警告のみ

### 4.2 インタラクティブキャリブレーションテスト

#### テスト4: インタラクティブツールの動作確認

**目的**: GUIツールが正常に動作し、リアルタイムフィードバックが表示されることを確認

```bash
cd /Users/ksk432/technetope/locomotion/calibration

# カメラプロセスの停止
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true

# インタラクティブツールの起動
sudo ./build/capture_calibration_interactive \
  calibration_config_low_res.json \
  calib_result.json
```

**操作手順:**

1. **ウィンドウが開くことを確認**
   - カラー/深度プレビューが表示される
   - ステータスパネルが表示される
   - インストラクションパネルが表示される

2. **ChArUco検出の確認**
   - ボードをカメラの視野内に配置
   - ステータスインジケーターの色を確認:
     - 🔴 Red: < 8コーナー検出
     - 🟡 Yellow: 8-11コーナー検出
     - 🟢 Green: ≥ 12コーナー検出（キャプチャ可能）
   - オーバーレイ表示（Dキーでトグル）でコーナーが表示される

3. **深度ヒートマップの確認**
   - 深度プレビューが表示される（設定で有効化されている場合）
   - カラーコード:
     - Blue: 0-150mm（検出ゾーン以下）
     - Green: 150-300mm（足検出ゾーン）← ターゲット
     - Yellow: 300-450mm（足より上）
     - Red: 450mm+（範囲外）

4. **キャプチャの実行**
   - ステータスが 🟢 Green になるまでボードを調整
   - `SPACE` キーでキャプチャ
   - "Capturing frame..." トーストが表示される
   - 処理完了後、結果が表示される

5. **カメラ高さの確認**
   - ステータスパネルに "Height: X.XX m" が表示される
   - キャリブレーション結果がある場合、その値が使用される
   - 結果がない場合、深度から推定された値が表示される

**検証項目:**
- [ ] ウィンドウが正常に開く
- [ ] リアルタイムプレビューが表示される（10-15 FPS）
- [ ] ステータスインジケーターが正しく色分けされる
- [ ] ChArUco検出オーバーレイが表示される
- [ ] 深度ヒートマップが表示される（有効化されている場合）
- [ ] カメラ高さが表示される
- [ ] キャプチャが正常に実行される
- [ ] キャリブレーション結果JSONが生成される

**キーボードショートカット:**
- `SPACE`: キャプチャ実行（ステータスがGreenの時のみ）
- `S`: デバッグフレーム保存
- `D`: 検出オーバーレイのトグル
- `Q` / `ESC`: 終了

### 4.3 QC自動化ツールのテスト

#### テスト5: QCツールの実行

**目的**: QC自動化ツールが正常に動作し、レポートが生成されることを確認

```bash
cd /Users/ksk432/technetope/locomotion/calibration

# カメラプロセスの停止
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true

# QCツールの実行
sudo ./build/run_calibration_qc \
  config/calibration_config.json \
  qc_report.md \
  qc_report.json
```

**期待される結果:**
- RealSense デバイス接続チェックが成功
- キャリブレーションが実行される
- 結果が検証される
- MarkdownレポートとJSONレポートが生成される

**検証項目:**
- [ ] デバイス接続チェックが成功
- [ ] キャリブレーションが実行される
- [ ] 検証結果が正しく判定される
- [ ] Markdownレポートが生成される
- [ ] JSONレポートが生成される
- [ ] レポートに推奨アクションが含まれている

**レポートの確認:**

```bash
# Markdownレポートの確認
cat qc_report.md

# JSONレポートの確認
cat qc_report.json | jq '.'
```

### 4.4 toio座標変換のテスト

#### テスト6: 座標変換の精度検証

**目的**: 画像ピクセル → toio座標への変換が正確に動作することを確認

**手動テスト手順:**

1. **キャリブレーション結果の読み込み**
   ```bash
   # キャリブレーション結果を確認
   cat calib_result.json | jq '.toio_coordinate_transform'
   ```

2. **既知点での検証**
   - ChArUcoボードのコーナー位置を画像上で確認
   - 対応するtoio座標を計算
   - 期待値との誤差を確認（目標: ≤ 2.0 ID units）

3. **カバレッジエリアの確認**
   ```bash
   # カバレッジエリアの確認
   cat calib_result.json | jq '.toio_coordinate_transform.coverage_area_toio_id'
   ```
   - 期待値: `{"min": {"x": 34.0, "y": 35.0}, "max": {"x": 339.0, "y": 250.0}}`
   - 許容誤差: ±1.0 ID units

**検証項目:**
- [ ] 変換誤差が ≤ 2.0 ID units
- [ ] カバレッジエリアが期待範囲内
- [ ] ラウンドトリップ誤差が ≤ 2.0 pixels（実装されている場合）

### 4.5 エッジケースのテスト

#### テスト7: パス解決のエッジケース

**目的**: 様々なパス指定パターンで正しく動作することを確認

```bash
# テスト1: 絶対パス指定
sudo ./build/capture_calibration \
  /Users/ksk432/technetope/locomotion/calibration/calibration_config_low_res.json \
  /tmp/calib_result.json

# テスト2: 相対パス指定（異なるディレクトリから実行）
cd /Users/ksk432/technetope
sudo ./locomotion/calibration/build/capture_calibration \
  locomotion/calibration/calibration_config_low_res.json \
  /tmp/calib_result.json

# テスト3: 設定ファイルが存在しない場合
sudo ./build/capture_calibration \
  nonexistent_config.json \
  /tmp/calib_result.json
# → デフォルト値が使用されることを確認
```

**検証項目:**
- [ ] 絶対パス指定が正しく動作する
- [ ] 相対パス指定が設定ファイル基準で解決される
- [ ] 設定ファイルが存在しない場合、デフォルト値が使用される
- [ ] エラーメッセージが分かりやすい

#### テスト8: 検証機能のテスト

**目的**: キャリブレーション検証機能が正しく動作することを確認

```bash
# キャリブレーション実行後、検証結果を確認
cat calib_result.json | jq '.validation'

# 警告が含まれている場合の確認
# （例: 変換誤差が大きい場合、カメラ高さが範囲外の場合など）
```

**検証項目:**
- [ ] 検証結果がJSONに含まれている
- [ ] 警告が適切に記録されている
- [ ] 検証チェックが正しく判定されている

---

## 5. トラブルシューティング

### 5.1 よくある問題と対処法

#### 問題1: カメラが検出されない

**症状:**
```
Failed to start RealSense pipeline: No device connected
```

**対処法:**
```bash
# macOS: カメラプロセスを停止
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true

# デバイスの確認
sudo /opt/homebrew/bin/rs-enumerate-devices

# USB接続の確認
system_profiler SPUSBDataType | grep -A 5 "Intel RealSense"

# カメラの抜き差し
# → USB-Cポートに直挿し（ハブ経由でない）
```

#### 問題2: パス解決エラー

**症状:**
```
Failed to load playmat layout 'config/toio_playmat.json': Failed to open playmat layout file
```

**対処法:**
```bash
# 設定ファイルのパスを確認
cat calibration_config.json | jq '.playmat_layout_path'

# 絶対パスに変更（一時的な対処）
# または、設定ファイルと同じディレクトリから実行

# パスの存在確認
ls -la config/toio_playmat.json
```

#### 問題3: ChArUcoボードが検出されない

**症状:**
```
ChArUco board not detected in current frame
```

**対処法:**
- ボードの位置を調整（カメラから40-80cm）
- 照明条件を改善（均一な明るさ、グレアを避ける）
- ボードの傾きを確認（床面に対して平行）
- ボードのサイズを確認（45mm正方形、33mmマーカー）
- カメラの焦点距離を確認

#### 問題4: 床面推定が失敗する

**症状:**
```
Floor plane estimation failed
Floor plane std exceeds threshold
```

**対処法:**
- 深度データの品質を確認（ノイズ、外乱の有無）
- カメラ高さを確認（2500-2700mmの範囲内）
- 床面が平らで不透明であることを確認
- 設定パラメータを調整:
  - `floor_z_min_mm`: 2400.0
  - `floor_z_max_mm`: 2800.0
  - `floor_inlier_threshold_mm`: 8.0
  - `floor_ransac_iterations`: 500

#### 問題5: 再投影誤差が大きい

**症状:**
```
reprojection error exceeds threshold
```

**対処法:**
- ChArUcoボードの検出品質を改善（より多くのコーナーを検出）
- ボードの配置を調整（カメラに対して平行）
- サブピクセル補正が有効であることを確認
- ホモグラフィRANSAC閾値を調整（`homography_ransac_thresh_px`）

#### 問題6: toio座標変換の誤差が大きい

**症状:**
```
transform_error_at_correspondences exceeds threshold
```

**対処法:**
- toio_playmat.json の対応点を確認・更新
- ChArUcoボードの物理的な配置位置を確認
- ボードマウントラベルが正しいことを確認（`board_mount_label`）
- 対応点の数を増やす（3点以上推奨）

### 5.2 ログレベルの調整

デバッグ時に詳細なログを出力する場合:

```json
{
  "log_level": "debug"
}
```

または、環境変数で設定:

```bash
export SPDLOG_LEVEL=debug
sudo ./build/capture_calibration ...
```

### 5.3 デバッグフレームの保存

インタラクティブツールで `S` キーを押すと、デバッグフレームが保存されます:

```bash
# デバッグフレームの確認
ls -la logs/calibration_debug/

# フレームの確認
# （カラー画像と深度画像がPNG形式で保存される）
```

---

## 6. 検証チェックリスト

### 6.1 基本機能

- [ ] パス解決が正しく動作する（相対パス、絶対パス）
- [ ] キャリブレーションパイプラインが正常に実行される
- [ ] ChArUcoボードが検出される（12個以上のコーナー）
- [ ] 床面推定が成功する（標準偏差 ≤ 8.0mm）
- [ ] ホモグラフィ計算が成功する（再投影誤差 ≤ 8.0 ID units）
- [ ] キャリブレーション結果JSONが生成される

### 6.2 toio座標変換

- [ ] 画像ピクセル → toio座標変換が動作する
- [ ] toio座標 → 画像ピクセル変換が動作する
- [ ] カバレッジエリアチェックが動作する
- [ ] 変換誤差が ≤ 2.0 ID units
- [ ] カバレッジエリアが期待範囲内

### 6.3 検証機能

- [ ] 変換誤差の検証が動作する
- [ ] カバレッジエリアの検証が動作する
- [ ] ラウンドトリップ誤差の検証が動作する（実装されている場合）
- [ ] 検証結果がJSONに含まれている
- [ ] 警告が適切に記録されている

### 6.4 インタラクティブツール

- [ ] ウィンドウが正常に開く
- [ ] リアルタイムプレビューが表示される
- [ ] ステータスインジケーターが正しく色分けされる
- [ ] ChArUco検出オーバーレイが表示される
- [ ] 深度ヒートマップが表示される
- [ ] カメラ高さが表示される
- [ ] キャプチャが正常に実行される

### 6.5 QCツール

- [ ] デバイス接続チェックが成功する
- [ ] キャリブレーションが実行される
- [ ] 検証結果が正しく判定される
- [ ] Markdownレポートが生成される
- [ ] JSONレポートが生成される

---

## 7. パフォーマンス検証

### 7.1 処理時間の測定

```bash
# キャリブレーション実行時間の測定
time sudo ./build/capture_calibration \
  calibration_config_low_res.json \
  calib_result.json

# 期待値:
# - キャリブレーション1回: ≤ 200ms
# - 5回のキャリブレーション: ≤ 2 minutes
```

### 7.2 フレームレートの確認

インタラクティブツールで確認:
- プレビューフレームレート: ≥ 10 FPS（目標: 15 FPS）
- キャプチャレスポンス: ≤ 500ms
- キャリブレーション処理: ≤ 5s

---

## 8. テスト結果の記録

### 8.1 テスト結果テンプレート

```markdown
## テスト実行記録

**日付**: 2025-11-09
**環境**: macOS 14.0 (Apple Silicon M2)
**カメラ**: Intel RealSense D415 (FW 5.16.0.1)
**USB接続**: USB 3.0 (直挿し)

### テスト結果

| テストID | テスト項目 | 結果 | 備考 |
|---------|-----------|------|------|
| 1 | パス解決 | PASS | 相対パス解決が正常に動作 |
| 2 | 基本キャリブレーション | PASS | 再投影誤差: 5.2px |
| 3 | キャリブレーション結果検証 | PASS | 全ての検証項目がPASS |
| 4 | インタラクティブツール | PASS | GUIが正常に動作 |
| 5 | QCツール | PASS | レポートが正常に生成 |
| 6 | toio座標変換 | PASS | 変換誤差: 1.8 ID units |

### 検出された問題

- なし

### 推奨アクション

- なし
```

### 8.2 結果ファイルの保存

```bash
# テスト結果の保存
mkdir -p test_results/$(date +%Y%m%d)
cp calib_result.json test_results/$(date +%Y%m%d)/
cp qc_report.md test_results/$(date +%Y%m%d)/
cp qc_report.json test_results/$(date +%Y%m%d)/
```

---

## 9. 参考資料

- [README.md](README.md) - モジュールの概要
- [REQUIREMENTS_CALIBRATION_V2.md](REQUIREMENTS_CALIBRATION_V2.md) - 要件仕様
- [STATUS.md](STATUS.md) - 実装状況
- [../../docs/apple_silicon_realsense.md](../../docs/apple_silicon_realsense.md) - Apple Silicon環境での注意事項
- [capture_calibration_interactive_requirements.md](capture_calibration_interactive_requirements.md) - インタラクティブツールの要件

---

## 10. 次のステップ

テストが成功したら、以下を検討してください:

1. **人検出機能の実装**: toio座標変換を使用した人検出パイプラインの実装
2. **パフォーマンス最適化**: 処理時間の短縮、メモリ使用量の削減
3. **エラーハンドリングの強化**: カメラ切断時の自動再接続など
4. **複数人対応**: 複数人の検出・トラッキング機能の実装

---

**最終更新**: 2025-11-09  
**作成者**: AI Assistant  
**レビュー状況**: DRAFT - 実機テスト後に更新予定

