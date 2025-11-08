# Locomotion Calibration Module

本ディレクトリは toio×RealSense プロジェクトのキャリブレーション処理（`rs2::align`＋ChArUco ホモグラフィ推定＋床面フィット）を実装するための雛形です。

## 推奨依存ライブラリ
- **Intel RealSense SDK (librealsense)** – RealSense D435 デバイス制御。  
  GitHub: https://github.com/IntelRealSense/librealsense
- **OpenCV + contrib（aruco モジュール）** – ChArUco ボード検出・`findHomography`。  
  GitHub: https://github.com/opencv/opencv  
  contrib: https://github.com/opencv/opencv_contrib
- **spdlog** – ランタイムロギング。  
  GitHub: https://github.com/gabime/spdlog
- （任意）**Eigen3** – 後続で床面フィットや最適化を行う際に利用可能。  
  GitHub: https://gitlab.com/libeigen/eigen

上記はいずれも MIT/BSD 系ライセンスで、CMake から `find_package` 可能です。

## 環境構築（macOS / Linux）
1. **RealSense SDK**  
   - macOS: `brew install librealsense`  
   - Ubuntu: `sudo apt install librealsense2-dev`
2. **OpenCV（aruco 含む）**  
   - macOS: `brew install opencv`（Homebrew 版は contrib を同梱）  
   - Ubuntu: `sudo apt install libopencv-dev`
3. **spdlog**  
   - `brew install spdlog` または `sudo apt install libspdlog-dev`
4. （任意）`vcpkg` / `conan` を導入し、依存ライブラリのマルチプラットフォーム管理を行う

## ビルド方法

```bash
cd locomotion/calibration
cmake -S . -B build
cmake --build build
```

ルートプロジェクトへ統合する場合は、上記 CMakeLists を親プロジェクトから `add_subdirectory(locomotion/calibration)` してください。

## ディレクトリ構成（主要ファイル）

```
locomotion/calibration/
 ├─ CMakeLists.txt
 ├─ include/locomotion/calibration/
 │   ├─ CalibrationPipeline.h        # RealSense 取得＋処理パイプライン
 │   ├─ CalibrationSession.h         # リトライ制御／結果集計
 │   ├─ CalibrationResult.h
 │   ├─ CharucoDetector.h            # Charuco 検出ラッパ
 │   ├─ FloorPlaneEstimator.h        # 床面フィット（RANSAC 実装予定）
 │   └─ PlaymatLayout.h              # プレイマットレイアウト読み込み
 ├─ src/
 │   ├─ CalibrationPipeline.cpp
 │   ├─ CalibrationSession.cpp
 │   ├─ CharucoDetector.cpp
 │   ├─ FloorPlaneEstimator.cpp      # いまはプレースホルダ、将来 RANSAC 実装
 │   └─ PlaymatLayout.cpp
 ├─ config/                          # toio プレイマットレイアウト JSON
 └─ tools/                           # キャリブレーション CLI / スクリプト
```

## toio プレイマットと Charuco レイアウトの用意手順

1. **プレイマット実測**  
   - 参考 PDF `locomotion/toio_playmat.pdf` の Start/End ID（例: `Start (34,35)`, `End (339,250)` for face #01）を基準に、実際のマットで位置・向きを確認します。  
   - メジャーや定規で横幅・縦幅を測り、ID↔mm 換算の誤差をメモしておきます（後で JSON 更新）。
   - 連番マット（TMD01SS）の場合は、各面（#01-#12）の座標範囲をPDFから確認してください。

2. **ArUco / Charuco マーカーの作成**  
   - `python locomotion/calibration/tools/generate_marker_sheet.py --output markers_a4_45mm.png --metadata markers_a4_45mm.json` を実行。  
   - `markers_a4_45mm.png` を A4 用紙にスケール 100% で印刷し、45 mm 角になっているか定規で確認。  
   - 必要に応じて `--marker-size-mm` などを調整して再印刷。

3. **マーカー貼り付け**  
   - プレイマット上の所定位置（例: 中央付近の 2×2 マスに収まるよう）へマーカーを貼付し、toio が通行できるようテープで固定します。  
   - マーカー ID と貼付位置（Position ID 座標）をメモしておくと後で検証に使えます。

4. **レイアウト JSON の更新**  
   - `locomotion/calibration/config/toio_playmat.json` を開き、実測値で `position_id_extent`, `id_per_mm`, `board_to_position_id` の対応点を更新します。  
   - Charuco ボードを中心に貼る場合は `charuco_5x7_45mm` を使い、他の配置にしたい場合は `charuco_mounts` に新しい `label` を追加します。

5. **実行時設定**  
   - キャリブレーション実装から `CalibrationConfig::playmat_layout_path` に JSON のパス（既定は `config/toio_playmat.json`）、`board_mount_label` に利用したマウントのラベル（既定は `center_mount_nominal`）を指定します。  
   - RealSense から Charuco ボードを検出すると、レイアウト JSON をもとに Position ID 座標へ変換されます。

## キャリブレーション実行 CLI

単体でビルドした場合は `capture_calibration` が生成され、下記のようにキャリブレーションを実行できます。

```bash
cmake -S locomotion/calibration -B build
cmake --build build
./build/capture_calibration calibration_config.json calib_result.json
```

- `calibration_config.json` は `CalibrationConfig` および `SessionConfig` の上書き値を記述した任意の JSON（存在しない場合はデフォルト値を利用）。  
- 出力 `calib_result.json` にはホモグラフィ・床平面・再投影誤差などが保存されます。  
- 実行には RealSense D435 と Charuco ボードが視野内にある環境が必要です。

## テスト手順

実機テストの詳細な手順については、[TESTING.md](TESTING.md) を参照してください。

**クイックスタート:**
```bash
# ビルド
cmake -B build -DLOCOMOTION_BUILD_TESTS=ON
cmake --build build

# 実機テスト（macOS、sudo必須）
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true
sudo ./build/capture_calibration calibration_config_low_res.json calib_result.json
```

## 参考資料
- [TESTING.md](TESTING.md) - 実機テスト手順書
- [REQUIREMENTS_CALIBRATION_V2.md](REQUIREMENTS_CALIBRATION_V2.md) - 要件仕様
- [STATUS.md](STATUS.md) - 実装状況
- RealSense Align サンプル: https://github.com/IntelRealSense/librealsense/tree/master/examples/align  
- OpenCV + Charuco キャリブレーション例: https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/camera_calibration/charuco_diamond.cpp  
- ORCA / RVO2 ライブラリ: https://github.com/snape/RVO2
