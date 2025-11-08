# Calibration Workflow Blueprint

本ドキュメントは、RealSense D415 と toio プレイマット環境におけるキャリブレーション／検証フローを共有するためのワークフロー定義です。開発者（Codex）と QC 担当（ユーザー）が同じ手順で動作確認できるよう、依存リポジトリ、実装ステップ、検証タスクを整理しています。

**関連ドキュメント:**
- [implementation_requirements.md](locomotion/calibration/implementation_requirements.md) - 詳細な技術仕様とクラス設計
- 本ドキュメント - 実装の優先順位とQCフロー

**想定デバイス:** RealSense D415（implementation_requirements.mdと統一）

---

## 1. 参照コード（git subtree）

`external_references/` 以下にサブツリーとして取り込んだ OSS を活用します。

| Path | Upstream | 用途 |
| ---- | -------- | ---- |
| `external_references/Perception_Using_PCD` | https://github.com/ysm10801/Perception_Using_PCD | RealSense 点群生成＋RANSAC 実装の参考 |
| `external_references/PointCloud-Playground` | https://github.com/Loozin031/PointCloud-Playground | Open3D を用いた平面検出パラメータの参考 |
| `external_references/charuco_calibration` | https://github.com/CopterExpress/charuco_calibration | Charuco 検出処理の参考 |
| `external_references/stereo-calib` | https://github.com/shubhamwagh/stereo-calib | Charuco でのホモグラフィ／保存形式の参考 |

更新が必要な場合は `git subtree pull --prefix=... <repo> <branch> --squash` を実行してください。

---

## 2. 実装タスクと依存関係

### タスク実行順序

```
2.1 CharucoDetector 実装 (独立、最優先)
  ↓
2.2 FloorPlaneEstimator 実装 (独立、並行可能)
  ↓
2.3 CalibrationPipeline 拡張 (2.1に依存)
  ↓
2.4 QC スクリプト (全タスクの統合)
```

### 実装方針の前提

**Intrinsics について:**
- RealSense D415 の**内蔵Intrinsics（ファクトリーキャリブレーション済み）を使用**する
- `rs2_intrinsics` から `fx`, `fy`, `cx`, `cy`, `distortion_model`, `coeffs[]` を取得
- 独自の再キャリブレーションツールは実装しない（将来的な検証用として保留）

---

### 2.1 `CharucoDetector` 実装

**目的:** Charuco ボード検出を独立クラスとして実装し、再利用可能にする。

**完了条件:**
- [ ] サブピクセル補正済み 2D 点列を返す
- [ ] ボード座標系の 3D 点列を返す
- [ ] 有効コーナー数が `min_charuco_corners`（既定12）以上の場合のみ成功
- [ ] 検出失敗時は `std::nullopt` を返し、Info レベルでログ出力

**参考:**
- `external_references/charuco_calibration/src/` の検出ロジック
- `external_references/stereo-calib/` の Charuco 処理

**既存コードの再利用:**
- OpenCV の `cv::aruco::detectMarkers` + `cv::aruco::interpolateCornersCharuco`
- サブピクセル補正: `cv::cornerSubPix`

**自作が必要な部分:**
- `CharucoDetector` クラス設計（`include/locomotion/calibration/CharucoDetector.h`）
- `CalibrationConfig` からパラメータを受け取る構造
  - `charuco_enable_subpixel_refine`
  - `charuco_subpixel_window`
  - `charuco_subpixel_max_iterations`
  - `charuco_subpixel_epsilon`
- 検出統計情報の返却（有効マーカー枚数、補間成功率）

**API 設計例:**
```cpp
struct CharucoDetectionResult {
    std::vector<cv::Point2f> corners_2d;  // サブピクセル補正済み
    std::vector<cv::Point3f> corners_3d;  // ボード座標系(mm)
    int detected_markers;
    int interpolated_corners;
};

class CharucoDetector {
public:
    explicit CharucoDetector(const CharucoConfig& config);
    std::optional<CharucoDetectionResult> Detect(const cv::Mat& color);
};
```

**推奨パラメータ:**
- ボード仕様: 5×7 マス、マス一辺 45mm、マーカー一辺 33mm（`CalibrationConfig` と統一）
- ArUco 辞書: `DICT_4X4_50`
- サブピクセル補正ウィンドウ: 5×5
- 最小採用コーナー数: 12

**成果物:**
- `locomotion/calibration/include/locomotion/calibration/CharucoDetector.h`
- `locomotion/calibration/src/CharucoDetector.cpp`
- ユニットテスト: `locomotion/calibration/test/test_charuco_detector.cpp`

---

### 2.2 `FloorPlaneEstimator` 実装

**目的:** 深度フレームから床平面を RANSAC で推定し、法線/距離/インライヤ統計を返す。

**完了条件:**
- [ ] 平面方程式 `ax + by + cz + d = 0` を返す
- [ ] 平面の標準偏差 `plane_std_mm` を算出
- [ ] インライヤ比率 `inlier_ratio` を算出
- [ ] 法線の向きがカメラを向くように符号調整

**参考:**
- librealsense `examples/pointcloud/rs-pointcloud.cpp`（`rs2::pointcloud` の使い方）
- `external_references/Perception_Using_PCD/src/segmentation/plane_segmentation.cpp`（RANSAC ロジック）
- `external_references/PointCloud-Playground/scripts/plane_detection.py`（パラメータ設定の参考値）

**既存コードの再利用:**
- RealSense から点群を得る処理は librealsense サンプルに準拠
- RANSAC の主要ロジック（サンプリング、距離評価）は `Perception_Using_PCD` を C++20 スタイルに移植

**自作が必要な部分:**
- `FloorPlaneEstimator` クラス設計（`src/FloorPlaneEstimator.h/.cpp`）
- 点群ダウンサンプリング（グリッドベース）
- 範囲フィルタ（カメラから 0.3〜1.5m の範囲に限定してテーブルなどの外乱を除去）
- 結果を `cv::Vec4f plane`, `plane_std_mm`, `inlier_ratio` に整形

**処理フロー:**
1. `rs2::pointcloud` で XYZ 点群生成
2. 範囲フィルタ適用（Z方向: 300mm〜1500mm）
3. グリッドダウンサンプリング（4×4 グリッド、848×480 → 212×120 相当）
4. RANSAC で平面推定
   - ランダムサンプリング（3点）→ 平面候補 → 距離閾値でインライヤ算出
   - 反復回数分繰り返し、最もインライヤ数が多いモデルを選択
5. ベストモデル選択後、全インライヤで最小二乗再フィット
6. 標準偏差計算（全インライヤの平面からの距離）

**推奨パラメータ:**
- ダウンサンプリング: 4×4 グリッド
- 高さフィルタ: カメラから 0.3〜1.5m 範囲
- RANSAC 反復: 500 回（`CalibrationConfig::floor_ransac_iterations`）
- インライヤ閾値: 8mm（`CalibrationConfig::floor_inlier_threshold_mm`）

**API 設計例:**
```cpp
struct FloorPlaneEstimate {
    cv::Vec4f plane;        // [a, b, c, d] where ax+by+cz+d=0
    double plane_std_mm;    // 標準偏差（mm）
    double inlier_ratio;    // インライヤ比率 [0.0, 1.0]
    int total_points;       // フィルタ後の総点数
};

class FloorPlaneEstimator {
public:
    explicit FloorPlaneEstimator(const FloorPlaneConfig& config);
    std::optional<FloorPlaneEstimate> Estimate(
        const rs2::depth_frame& depth,
        const rs2_intrinsics& intrinsics);
};
```

**成果物:**
- `locomotion/calibration/src/FloorPlaneEstimator.h`
- `locomotion/calibration/src/FloorPlaneEstimator.cpp`
- ユニットテスト: `locomotion/calibration/test/test_floor_plane_estimator.cpp`（モック点群で検証）

---

### 2.3 `CalibrationPipeline` 拡張

**目的:** CharucoDetector と FloorPlaneEstimator を統合し、完全なキャリブレーション結果を出力する。

**完了条件:**
- [ ] RealSense 内蔵 Intrinsics を取得して使用
- [ ] 内蔵 distortion モデルに応じた歪み補正を適用
- [ ] CharucoDetector でコーナー検出
- [ ] ホモグラフィ推定（`cv::findHomography` RANSAC）
- [ ] FloorPlaneEstimator で床面推定
- [ ] 結果を `calib_result.json` に保存（スキーマ v2.0）

**既存コードの再利用:**
- 現行の `CalibrationPipeline` のフレーム取得・Align処理を維持
- OpenCV の `cv::undistort` または `initUndistortRectifyMap` + `remap`
- JSON 書き込みは既存の `CalibrationSession::SaveResultJson` を拡張

**自作が必要な部分:**
- RealSense Intrinsics 取得処理
  ```cpp
  rs2::video_stream_profile color_profile = ...;
  rs2_intrinsics intrinsics = color_profile.get_intrinsics();
  ```
- Intrinsics を `cv::Mat` 形式に変換
  ```cpp
  cv::Mat K = (cv::Mat_<double>(3,3) <<
      intrinsics.fx, 0, intrinsics.cx,
      0, intrinsics.fy, intrinsics.cy,
      0, 0, 1);
  cv::Mat distCoeffs = cv::Mat(1, 5, CV_64F);
  for (int i = 0; i < 5; ++i) {
      distCoeffs.at<double>(i) = intrinsics.coeffs[i];
  }
  ```
- 歪み補正の適用（Charuco 検出前に実施）
- `CalibrationSnapshot` の拡張
  - `intrinsics` フィールド追加
  - `floor_plane` フィールド追加

**結果 JSON フォーマット（スキーマ v2.0）:**
```json
{
  "schema_version": "2.0",
  "timestamp": "2025-01-08T12:34:56Z",
  "intrinsics": {
    "fx": 615.123,
    "fy": 615.456,
    "cx": 320.789,
    "cy": 240.012,
    "distortion_model": "brown_conrady",
    "distortion_coeffs": [0.01, -0.02, 0.001, 0.0005, 0.03]
  },
  "homography_color_to_position": [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]
  ],
  "floor_plane": {
    "coefficients": [0.0, 0.0, 1.0, -500.0],
    "std_mm": 3.2,
    "inlier_ratio": 0.94
  },
  "reprojection_error_id": 3.2,
  "charuco_corners": 42,
  "validation": {
    "passed": true,
    "checks": {
      "reprojection_error": "PASS",
      "floor_plane_std": "PASS",
      "charuco_corners": "PASS"
    }
  }
}
```

**歪み補正の影響について:**
- 歪み補正後は画像サイズが変わる可能性がある（トリミングまたはパディング）
- ホモグラフィは補正後の座標系で計算されるため、実行時も同じ補正を適用する必要がある
- `calib_result.json` に保存される Intrinsics は補正**前**の値（RealSense 内蔵値）
- 補正後の新しい Intrinsics を算出する場合は `cv::getOptimalNewCameraMatrix` を使用

**成果物:**
- `locomotion/calibration/src/CalibrationPipeline.cpp`（拡張）
- `locomotion/calibration/src/CalibrationSession.cpp`（JSON保存部分の拡張）
- 統合テスト: `locomotion/calibration/test/test_calibration_pipeline_integration.cpp`

---

### 2.4 QC 用スクリプト

**目的:** 全キャリブレーションプロセスを自動実行し、合格/不合格を判定する。

**完了条件:**
- [ ] RealSense 接続確認
- [ ] Intrinsics 取得と検証
- [ ] Charuco 検出（再投影誤差が閾値以下か確認）
- [ ] ホモグラフィ適用後のランドマーク誤差を数値化
- [ ] 深度フレームで床面推定 → 残差・インライヤ率を表示
- [ ] QC レポート（JSON または Markdown）生成

**実装方法:**
- CMake の `ctest` に統合し、`ctest -L qc` で実行可能にする
- C++ で `tools/run_calibration_qc.cpp` を実装（Bash/Python より統合が容易）
- または Python スクリプト `tools/run_calibration_qc.py` でラップ

**QC 判定基準:**

| 項目 | 合格基準 | 測定方法 |
| ---- | -------- | -------- |
| RealSense 接続 | デバイス検出成功 | `rs2::context::query_devices()` |
| Intrinsics 取得 | `fx`, `fy`, `cx`, `cy` が妥当な範囲 | 600 < fx < 650, 600 < fy < 650 (D415 想定) |
| Charuco 検出 | 検出コーナー数 ≥ 12 | `CharucoDetector::Detect()` |
| 再投影誤差 | RMS < 8.0 (Position ID 単位) | `CalibrationSnapshot::reprojection_error` |
| 床面推定 標準偏差 | std < 8.0 mm | `FloorPlaneEstimate::plane_std_mm` |
| 床面推定 インライヤ比 | inlier_ratio > 0.8 | `FloorPlaneEstimate::inlier_ratio` |

**QC レポート例（Markdown）:**
```markdown
# Calibration QC Report

**Date:** 2025-01-08 12:34:56
**Device:** RealSense D415 (Serial: 123456789)

## Test Results

| Test | Status | Value | Threshold | Notes |
| ---- | ------ | ----- | --------- | ----- |
| RealSense Connection | ✅ PASS | - | - | Device detected |
| Intrinsics | ✅ PASS | fx=615.1, fy=615.4 | 600-650 | Valid range |
| Charuco Detection | ✅ PASS | 42 corners | ≥12 | Good detection |
| Reprojection Error | ✅ PASS | 3.2 ID units | <8.0 | Excellent |
| Floor Plane Std | ✅ PASS | 3.2 mm | <8.0 | Stable |
| Floor Inlier Ratio | ✅ PASS | 0.94 | >0.8 | High confidence |

## Overall Result: ✅ PASS

All calibration metrics are within acceptable ranges.
```

**既存コードの再利用:**
- `CalibrationPipeline` を直接呼び出す
- librealsense のデバイス検出ロジック

**自作が必要な部分:**
- QC 判定ロジック（閾値との比較）
- レポート生成（JSON/Markdown）
- CMake テストとの統合

**成果物:**
- `locomotion/calibration/tools/run_calibration_qc.cpp`（または `.py`）
- `locomotion/calibration/CMakeLists.txt`（ctest 統合）
- QC レポートサンプル: `docs/qc_report_sample.md`

---

## 3. QC フロー

| フェーズ | デベロッパ作業 | QC 担当作業 | 合格基準 |
| -------- | --------------- | ----------- | -------- |
| CharucoDetector | ユニットテストとサンプル画像で動作確認 | 実機データで検出率を検証 | 検出コーナー数 ≥ 12 |
| FloorPlaneEstimator | モック点群でRANSAC動作確認 | 実際の床環境で `run_calibration_qc` を実行 | std < 8mm, inlier_ratio > 0.8 |
| CalibrationPipeline | 統合テストで全フロー確認 | toio プレイマットで試験実行 | 再投影誤差 < 8.0 ID units |
| 回帰テスト | ユニット/統合テストを再実行 | QC スクリプトで全項目再検証 | 全項目 PASS |

**不合格時の対応フロー:**
1. QC レポートを確認し、どの項目が不合格かを特定
2. 不合格項目に応じた対処:
   - Charuco 検出失敗 → ボードの配置、照明条件を調整
   - 再投影誤差が大きい → Charuco ボードの平坦性を確認、設置距離を調整
   - 床面推定失敗 → 床面の平坦性を確認、範囲フィルタのパラメータを調整
3. 調整後、再度 QC スクリプトを実行
4. 3 回連続で不合格の場合はデベロッパに報告

---

## 4. 実装タイムラインと優先度

### マイルストーン 1: 基本検出機能（Week 1-2）
- **優先度: 最高**
- タスク 2.1: CharucoDetector 実装
- タスク 2.2: FloorPlaneEstimator 実装
- ユニットテスト整備

### マイルストーン 2: 統合とキャリブレーション（Week 3）
- **優先度: 高**
- タスク 2.3: CalibrationPipeline 拡張
- 統合テスト整備
- JSON スキーマ確定

### マイルストーン 3: QC 自動化（Week 4）
- **優先度: 中**
- タスク 2.4: QC スクリプト実装
- CI/CD 統合
- ドキュメント整備

### 将来タスク（優先度: 低、上記完了後に着手）
- 深度クラスタリング（足/頭検出）
- PlaymatLayout 実装（複数マット対応）
- Intrinsics 再キャリブレーションツール（検証用）
- プロジェクタ連携のための外部パラメータ推定

---

## 5. パラメータ一覧と推奨値

### CharucoDetector パラメータ

| パラメータ | 推奨値 | 説明 |
| ---------- | ------ | ---- |
| `charuco_squares_x` | 5 | ボード列数 |
| `charuco_squares_y` | 7 | ボード行数 |
| `charuco_square_length_mm` | 45.0 | マス一辺長 |
| `charuco_marker_length_mm` | 33.0 | マーカー一辺長 |
| `aruco_dictionary` | `DICT_4X4_50` | ArUco 辞書 |
| `min_charuco_corners` | 12 | 最小採用コーナー数 |
| `charuco_enable_subpixel_refine` | true | サブピクセル補正 |
| `charuco_subpixel_window` | 5 | サブピクセル補正ウィンドウ |
| `charuco_subpixel_max_iterations` | 30 | 補正の最大反復回数 |
| `charuco_subpixel_epsilon` | 0.1 | 補正の収束閾値 |

### FloorPlaneEstimator パラメータ

| パラメータ | 推奨値 | 説明 |
| ---------- | ------ | ---- |
| `floor_inlier_threshold_mm` | 8.0 | インライヤ判定閾値 |
| `floor_ransac_iterations` | 500 | RANSAC 反復回数 |
| `floor_min_inlier_ratio` | 0.7 | 最小インライヤ比率 |
| `floor_z_min_mm` | 300.0 | 範囲フィルタ下限 |
| `floor_z_max_mm` | 1500.0 | 範囲フィルタ上限 |
| `floor_downsample_grid` | 4 | ダウンサンプリンググリッド |

### CalibrationPipeline パラメータ

| パラメータ | 推奨値 | 説明 |
| ---------- | ------ | ---- |
| `color_width` | 1280 | カラー画像幅 |
| `color_height` | 720 | カラー画像高さ |
| `depth_width` | 848 | 深度画像幅 |
| `depth_height` | 480 | 深度画像高さ |
| `fps` | 30 | ストリーム FPS |
| `homography_ransac_thresh_px` | 3.0 | findHomography 閾値 |
| `max_reprojection_error_id` | 8.0 | Position ID 単位の許容誤差 |
| `session_attempts` | 5 | キャリブレーション試行回数 |
| `enable_floor_plane_fit` | true | 床平面推定の有効化 |

---

## 6. トラブルシューティング

### Charuco 検出が失敗する
**原因:**
- 照明条件が悪い（反射、影）
- ボードが歪んでいる、汚れている
- カメラとボードの距離が不適切

**対処:**
- 均一な照明を確保
- ボードを平坦な面に貼り付ける
- カメラから 50-80cm の距離で試行
- `min_charuco_corners` を一時的に下げる（例: 8）

### 再投影誤差が大きい
**原因:**
- Charuco ボードの印刷精度が低い
- ボードが平坦でない
- toio プレイマットとの対応点が不正確

**対処:**
- 高品質プリンタで再印刷（300dpi 以上）
- ボードを硬い板に貼り付ける
- PlaymatLayout の対応点を再測定

### 床面推定が不安定
**原因:**
- 床面が不均一（カーペット、凹凸）
- 深度ノイズが多い
- 範囲フィルタが不適切

**対処:**
- 平坦な床面で試行
- `floor_inlier_threshold_mm` を緩和（例: 10.0）
- 範囲フィルタの上下限を調整
- RANSAC 反復回数を増加（例: 1000）

---

## 付録: データ管理

### キャリブレーションデータの保存場所
- 結果: `locomotion/calibration/data/calib_result.json`
- QC レポート: `locomotion/calibration/data/qc_reports/YYYYMMDD_HHMMSS.md`
- デバッグ画像: `locomotion/calibration/data/debug/` (オプション)

### バージョン管理
- `calib_result.json` は git で管理**しない**（`.gitignore` に追加）
- スキーマバージョンを明記し、後方互換性を保つ
- QC レポートは一時的なもので、必要に応じて保存

---

## 7. 現在の実装進捗（2025-01-08時点）

### ✅ 完了したタスク

| タスク | 状態 | 備考 |
| ------ | ---- | ---- |
| 2.1 CharucoDetector 実装 | ✅ 完了 | 公開API、サブピクセル補正対応 |
| 2.2 FloorPlaneEstimator 実装 | ✅ 完了 | Intrinsics対応RANSAC＋標準偏差/インライヤ率 |
| 2.3 CalibrationPipeline 拡張 | ✅ 完了 | Intrinsics取得・歪み補正・床面推定・JSON v2.0 |
| PlaymatLayout 実装 | ✅ 完了 | JSON読み込み、アフィン変換対応 |
| CalibrationSession 実装 | ✅ 完了 | 統計評価、JSON出力（v2.0対応要確認） |
| capture_calibration ツール | ✅ 完了 | CLI実装済み |

### 🔧 未完了・要修正項目（優先順）

#### 🔴 優先度: 最高（必須）

1. **QC スクリプトと自動レポート**
   - `tools/run_calibration_qc.*` を実装し、RealSense接続→キャプチャ→判定→レポート生成を自動化
   - `ctest -L qc` に統合し、QC担当が sudo 1 コマンドで確認できるようにする

2. **統合テスト／データ保存性の向上**
   - Charuco/深度のモックデータを整備し、`test_floor_plane_estimator.cpp` や統合テストを有効化
   - `SessionConfig::save_intermediate_snapshots` を活用してリトライ毎のスナップショットを保存、QC 解析に活かす

#### 🟡 優先度: 高

3. **ヘッダー配置の整理（任意）**
   - 内部専用クラス（例: `FloorPlaneEstimator`）を `src/` 配下に移動するか、公開API化の方針を明記

### Apple Silicon環境での開発注意

**重要:** macOS Apple Silicon環境でRealSenseを使用する場合、特別な対処が必要です。

```bash
# 開発時に毎回実行
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true
# カメラ抜き差し後、即座にsudoで実行
sudo ./capture_calibration
```

**主な問題:**
- sudo実行が必須（`libusb_init failed` 回避）
- macOSのカメラプロセスと競合
- USB3接続推奨（USB2だと15 FPS制限）

**詳細:** [docs/apple_silicon_realsense.md](docs/apple_silicon_realsense.md)

### 次のアクション

**推奨される作業順序:**
1. ✅ ドキュメント整備（本ファイルとimplementation_requirements.md）→ **完了**
2. ✅ CalibrationPipelineにIntrinsics処理を追加 → **完了（歪み補正＋床面推定連携）**
3. ✅ CalibrationConfigの拡張 → **完了（RANSAC/閾値/seed追加）**
4. ✅ FloorPlaneEstimatorの拡張 → **完了（Intrinsics対応RANSAC）**
5. ⬜ 統合テストで全フロー検証
6. ⬜ QCスクリプト実装

**検証コマンド:**
```bash
# ビルド
cmake -B build -DLOCOMOTION_BUILD_TESTS=ON
cmake --build build

# モックテスト
cd build && ctest

# 実機テスト（Apple Silicon環境）
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true
sudo ./build/capture_calibration
```

---

本ワークフローに沿って実装と QC を往復することで、座標系統一と床検出を堅実に進められます。追加の要望や変更があれば、このファイルを更新して合意形成に使ってください。
