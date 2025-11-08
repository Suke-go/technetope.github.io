# キャリブレーション実装要件定義書

**対象モジュール:** `locomotion/calibration`
**言語 / 標準:** C++20（libc++/libstdc++ いずれも許容）
**依存:** librealsense2, OpenCV（aruco + core + imgproc + calib3d）, spdlog,（任意）Eigen3
**目的:** toio プレイマット上の絶対座標系に RealSense センサ出力を正規化するためのオンライン・オフライン併用キャリブレーション機能を提供する。

**関連ドキュメント:**
- [workingflow.md](../../../workingflow.md) - 実装の優先順位、タイムライン、QCフロー
- 本ドキュメント - 詳細な技術仕様とクラス設計

**実装方針:**
- RealSense D415 の内蔵 Intrinsics（ファクトリーキャリブレーション済み）を使用
- Charuco 検出前に歪み補正を適用
- JSON スキーマ v2.0 による構造化された結果保存

---

## 1. スコープと前提
- **入力:** RealSense D415 から取得するカラー（BGR8）/深度（Z16）フレーム。  
- **出力:**  
  - toio プレイマットの Position ID 座標系への 3×3 ホモグラフィ行列 `H_color_to_position`。  
  - 床平面方程式 \(ax + by + cz + d = 0\)。  
  - 再投影誤差および検証メトリクス。  
- **利用場面:** デプロイ初期のオフラインキャリブレーション、および運用中の軽量リファイン。  
- **非対象:** カメラ内パラ推定、BLE 通信、toio 制御ロジック。

---

## 2. アーキテクチャ概要
### 2.1 モジュール構成
```
locomotion/calibration/
 ├─ include/locomotion/calibration/
 │   ├─ CalibrationPipeline.h
 │   └─ CharucoDetector.h                   (新規、再利用可能な公開API)
 ├─ src/
 │   ├─ CalibrationPipeline.cpp
 │   ├─ CharucoDetector.cpp                 (新規)
 │   ├─ FloorPlaneEstimator.h/.cpp          (新規、内部実装)
 │   ├─ CalibrationSession.h/.cpp           (新規、I/O + ロギング制御)
 │   └─ PlaymatLayout.h/.cpp                (新規、座標変換管理)
 ├─ config/
 │   ├─ toio_playmat.json                   (Position ID レイアウト定義)
 │   └─ calibration_config_sample.json      (設定サンプル)
 ├─ tools/
 │   ├─ capture_calibration.cpp             (CLI ユーティリティ)
 │   └─ generate_marker_sheet.py            (Charuco ボード生成)
 └─ test/
     ├─ test_charuco_detector.cpp
     ├─ test_floor_plane_estimator.cpp
     ├─ test_playmat_layout.cpp
     └─ test_calibration_pipeline_integration.cpp
```

### 2.2 主要クラス責務
- **CalibrationPipeline**
  - RealSense パイプライン管理（ストリーム設定、フレーム取得、リソース解放）。
  - RealSense 内蔵 Intrinsics の取得と歪み補正の適用。
  - 各処理ステップを連結し、`CalibrationSnapshot` を生成する。

- **CharucoDetector（新設）**
  - 歪み補正済みカラー画像から ChArUco 検出・補間を実行。
  - サブピクセル補正による高精度なコーナー位置推定。
  - 検出統計情報（有効マーカー数、補間成功率）を返却。
  - 再利用可能な公開 API として提供。

- **FloorPlaneEstimator（新設）**
  - 深度点群から RANSAC で床平面を推定。
  - 範囲フィルタとダウンサンプリングによる外乱除去。
  - 法線方向の安定化、標準偏差・インライヤ比率の算出。

- **PlaymatLayout（新設）**
  - toio プレイマットレイアウト定義 JSON の読み込み。
  - Charuco ボード座標（mm）→ Position ID のアフィン変換を提供。
  - 複数マット構成と将来のマット間オフセット対応。

- **CalibrationSession（新設）**
  - 繰り返し実行／結果の統計集計・閾値判定。
  - ベストスナップショットの選択とバリデーション。
  - 結果のシリアライズ（JSON スキーマ v2.0）とログ出力。

---

## 3. 実装詳細要件
### 3.1 RealSense 入力と Intrinsics 管理
- `rs2::config` でカラー/深度を有効化し、`rs2::pipeline::start` で開始する。
- フレーム待受は `wait_for_frames`（タイムアウト 500 ms）を基本とし、`RS2_EXCEPTION_TYPE_PIPELINE_BUSY` を受けた場合の再試行を実装する。
- `rs2::align`（カラー基準）を再利用し、カラーと深度の解像度が一致することを assert する。
- `FrameBundle` はメモリコピー（`clone()`）でバッファ破棄の副作用を避ける。

**Intrinsics 取得と歪み補正:**
- パイプライン起動後、カラーストリームの `rs2::video_stream_profile` から `rs2_intrinsics` を取得する。
  ```cpp
  rs2::video_stream_profile color_profile =
      pipeline_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  rs2_intrinsics intrinsics = color_profile.get_intrinsics();
  ```
- `rs2_intrinsics` から OpenCV の `cv::Mat` 形式に変換:
  ```cpp
  cv::Mat K = (cv::Mat_<double>(3,3) <<
      intrinsics.fx, 0, intrinsics.cx,
      0, intrinsics.fy, intrinsics.cy,
      0, 0, 1);
  std::vector<double> distCoeffs(intrinsics.coeffs, intrinsics.coeffs + 5);
  ```
- カラーフレーム取得後、Charuco 検出の**前に**必ず歪み補正を適用する。
- 歪み補正は `cv::undistort` または `cv::initUndistortRectifyMap` + `cv::remap` を使用。
- 補正後の画像サイズは元のサイズを維持する（`cv::getOptimalNewCameraMatrix` は使用しない）。
- 取得した Intrinsics は `CalibrationSnapshot` および結果 JSON に記録する。

### 3.2 ChArUco 検出
- `CharucoDetector::Detect(const cv::Mat& color)` は**歪み補正済み**のカラー画像を入力として受け取り、以下を返すこと:
  - サブピクセル補正済み 2D 点列（`std::vector<cv::Point2f>`）。
  - ボード座標系の 3D 点列（`std::vector<cv::Point3f>`）。
  - 有効マーカー枚数、補間成功率などの統計情報。
- 検出パラメータは `CalibrationConfig` の `charuco_enable_subpixel_refine` / `charuco_subpixel_*` で外部指定可能にする。
- 有効頂点数が設定閾値（既定 12）未満の場合は `std::nullopt` を返す。
- 検出に失敗した場合はログ（Info レベル）に記録し、キャリブレーションループが続行できるようにする。
- toio Position ID への変換に必要なマット基準点（原点座標、X/Y 方向ベクトル、ID スケール）は `CalibrationConfig::playmat_layout_path` が指す外部ファイル（例: `config/toio_playmat.json`）から読み込む。
- レイアウト JSON には以下を含める:  
  - `playmats[i].position_id_extent.min/max`: PDF 記載の Start/End ID  
  - `playmats[i].id_per_mm`: 公称 ID↔mm 換算（実測で更新）  
  - `charuco_mounts[j].board_to_position_id.correspondences`: Charuco ボード座標（mm）と Position ID の対応点（最低 3 点、推奨 4 点以上）
- レイアウト情報を扱う `PlaymatLayout` コンポーネントを実装し、以下を提供する:  
  - JSON 読み込みとバリデーション。  
  - Charuco ボード座標（mm）→ Position ID のアフィン変換生成（最小二乗）。  
  - 複数マット構成（`playmats` 配列）と将来のマット間オフセット対応。  
  - `CalibrationPipeline` から利用可能な API:  
    ```cpp
    PlaymatLayout layout = PlaymatLayout::Load(config.playmat_layout_path);
    cv::Point2f playmat_pt = layout.TransformBoardPoint("a3_simple", "center_mount_nominal", board_pt_mm);
    ```
- `CalibrationConfig::board_mount_label` で使用する対応関係を指定し、異なる Charuco 設置場所を柔軟に扱えるようにする。
- 開発・検証用の ArUco シートは `tools/generate_marker_sheet.py` で生成し、45 mm 角マーカー（A4／300 dpi）を既定テンプレートとする。

### 3.3 ホモグラフィ推定
- `cv::findHomography` を RANSAC モードで呼び出し、閾値は Pixel 空間で `CalibrationConfig::homography_ransac_thresh_px` を使用する。
- 変換先は toio Position ID 座標系（例: PDF 参照の開始点 `Start (x=34, y=35)`、終了点 `End (x=339, y=250)` for face #01）とする。
- 連番マット（TMD01SS）の場合は、各面の座標範囲をPDFから確認し、使用する面に応じて設定を更新すること。
- `PlaymatLayout` が提供する Charuco ボード座標（mm）→ Position ID のアフィン変換を適用して対応点を生成し、`H_color_to_position` を推定する。
- マットごとのオフセット・回転は `PlaymatLayout` 内部で処理される。
- `CalibrationSnapshot::reprojection_error` は Position ID 座標系の平均二乗平方根（RMS）として算出する。
- `CalibrationSession` は複数スナップショットから中央値／分散を評価し、`max_reprojection_error_id` 以下であることを保証する。

### 3.4 床平面推定
- `FloorPlaneEstimator` は以下のパラメータを持つ:
  - ダウンサンプリング: グリッドベース（既定 4×4、848×480 → 212×120 相当）
  - 範囲フィルタ: Z方向 300mm〜1500mm（`floor_z_min_mm` / `floor_z_max_mm`）
  - RANSAC 反復回数（既定 500、`floor_ransac_iterations`）
  - インライヤー閾値（既定 8.0mm、`floor_inlier_threshold_mm`）
  - 最小インライヤ比率（既定 0.7、`floor_min_inlier_ratio`）

**処理フロー:**
1. `rs2::pointcloud` で XYZ 点群生成
2. 範囲フィルタ適用（テーブルなどの外乱を除去）
3. グリッドダウンサンプリング
4. RANSAC で平面推定（3点サンプリング → 距離評価 → ベストモデル選択）
5. 全インライヤで最小二乗再フィット
6. 標準偏差計算（全インライヤの平面からの距離）

- 深度→点群変換には RealSense の `rs2_intrinsics` を使用。
- 推定後、法線の向きがカメラを向くように符号調整（`normal.z < 0` になるよう調整）。
- 平面方程式 `ax + by + cz + d = 0`、標準偏差 `plane_std_mm`、インライヤ比率 `inlier_ratio` を算出して `CalibrationSnapshot` へ付与する。
- インライヤ比率が `floor_min_inlier_ratio` 未満の場合は推定失敗とみなし `std::nullopt` を返す。

### 3.5 セッション管理と永続化
- `CalibrationSession` は以下の責務を持つ:
  - `Run()` で `N` 回（設定、既定 5）スナップショットを取得し、統計評価を実施。
  - `CalibrationSnapshot` の配列から最良値を選び、`CalibrationResult`（RMS、インライヤー率など）を整形する。
  - JSON 永続化・ログ出力を担当し、成功・失敗を呼び出し元へ返す。
- ベスト採用条件:
  - 再投影誤差 < `max_reprojection_error_id`。  
  - 平面の標準偏差 < `max_plane_std_mm`（設定）。  
  - タイムスタンプが最新。
- 永続化フォーマット（スキーマ v2.0）:
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
- スキーマバージョンを明記し、後方互換性を保つ。
- `intrinsics` には RealSense 内蔵の値（補正前）を記録する。
- `validation` には各検証項目の合否を記録し、QC 判定を容易にする。
- コンフィグとの整合性を検証し、不整合があればエラーログを出力する。
- CLI ユーティリティ `capture_calibration` を提供し、以下のワークフローを実装する:
  1. JSON/CLI 引数から `CalibrationConfig` を構築。
  2. `CalibrationSession` を起動し、結果を `calib_result.json` などへ保存。
  3. 失敗時はリトライ回数・理由をログ出力し、必要なら非ゼロ終了コードを返す。

### 3.6 ロギング・エラーハンドリング
- `spdlog` レベル設定（Info/Debug/Warning/Error）を `CalibrationConfig::log_level`（enum）で指定。  
- RealSense 例外はキャッチしてリトライ回数を制限（3 回）。  
- ホモグラフィ推定失敗時は調整済みパラメータをログに吐き、デバッグ用の画像ダンプ（`tools/debug/`）をオプションで出力。

### 3.7 テスト要件
- **ユニットテスト:**
  - **CharucoDetector:**
    - サンプル画像（5×7 Charuco ボード、距離60cm相当）で検出を実行
    - 期待値: 検出コーナー数 ≥ 20（理想的には 24）
    - サブピクセル補正の有効性を検証（補正前後の座標差 > 0.1px）
  - **FloorPlaneEstimator:**
    - 既知の平面点群（z=500mm、法線[0,0,1]）を入力
    - 期待値: 推定平面係数が ±5mm 以内で一致
    - インライヤ比率 > 0.95
    - 標準偏差 < 2.0mm
  - **PlaymatLayout:**
    - サンプル JSON を読み込み、アフィン変換の精度を検証
    - 既知の対応点を変換し、誤差 < 1.0 Position ID 単位

- **統合テスト:**
  - **モックモード（デフォルト）:**
    - RealSense が接続されていなくてもコンフィグ検証とエラー処理が動作する
    - モック画像・深度データで全フローを実行し、JSON 出力を検証
  - **実機モード（`REAL_DEVICE_TESTS=ON`）:**
    - RealSense D415 接続を確認
    - `CalibrationSession` が 5 回試行で 1 回以上成功することを確認
    - 処理時間を計測し、1回あたり < 200ms を目標とする（≤ 150ms は努力目標）

- **CI 連携:**
  - ハード依存部分は `REAL_DEVICE_TESTS` フラグで分岐
  - デフォルトではモックデータを使用し、高速実行（< 5秒）を保証
  - 実機テストは別ジョブで手動トリガーまたは夜間実行

### 3.8 パフォーマンス要件
- **目標:** キャリブレーション 1 回の処理時間 ≤ 150 ms（カラー 1280×720）
  - 内訳想定: 歪み補正 20ms、Charuco検出 50ms、ホモグラフィ 10ms、床面推定 60ms、その他 10ms
  - ≤ 200ms であれば許容可能
  - 初期実装ではパフォーマンスよりも正確性を優先する
- メモリ確保は極力初期化時に行い、ループ内では再利用する。
- RANSAC で利用する乱数生成は `std::mt19937` を共有し、シードを `CalibrationConfig::random_seed` で制御可能に（再現性のため）。
- プロファイリング結果に基づいて、必要に応じて最適化を実施する（例: SIMD、マルチスレッド、OpenCL）。

---

## 4. 実装ステップ推奨順
1. `CharucoDetector` クラスの切り出しとユニットテスト整備。  
2. `FloorPlaneEstimator` の RANSAC 実装とベンチマーク。  
3. `CalibrationPipeline` をこれらヘルパーに委譲し、構成要素を疎結合化。  
4. `CalibrationSession` で反復実行・統計評価を追加。  
5. CLI ツール `capture_calibration` を実装し、設定読み込み〜結果保存を一連で動かす。  
6. CI でモックテストを実行し、実機用テストは別ジョブで手動トリガー。

---

## 5. 保守・将来拡張
- プロジェクタ連携を想定し、`CalibrationSnapshot` にカメラ外部パラメータ（`R`, `t`）を追加できるよう空欄を確保。  
- BLE 制御側と共有するため、`calib.json` のスキーマを OpenAPI/JSON Schema で整備。  
- 長期運用でのドリフト補正のため、既知 toio をアンカーとして `CalibrationSession` で利用できるホットリロード API を準備。

---

## 付録 A: コンフィグ項目一覧（抜粋）

| キー | 型 | 既定値 | 説明 |
|------|----|--------|------|
| `color_width` | int | 1280 | カラー画像幅 |
| `color_height` | int | 720 | カラー画像高さ |
| `depth_width` | int | 848 | 深度画像幅 |
| `depth_height` | int | 480 | 深度画像高さ |
| `fps` | int | 30 | ストリーム FPS |
| `charuco_squares_x` | int | 5 | ChArUco ボード列数 |
| `charuco_squares_y` | int | 7 | ChArUco ボード行数 |
| `charuco_square_length_mm` | float | 45.0 | チェス盤マス一辺長 |
| `charuco_marker_length_mm` | float | 33.0 | マーカー一辺長 |
| `min_charuco_corners` | int | 12 | ホモグラフィ最小採用点数 |
| `homography_ransac_thresh_px` | double | 3.0 | `findHomography` のしきい値（px） |
| `max_reprojection_error_id` | double | 8.0 | Position ID 単位の許容誤差 |
| `charuco_enable_subpixel_refine` | bool | true | Charuco 検出時のサブピクセル補正を有効化 |
| `charuco_subpixel_window` | int | 5 | サブピクセル補正ウィンドウ（`Size(w,w)`） |
| `charuco_subpixel_max_iterations` | int | 30 | サブピクセル補正の最大反復回数 |
| `charuco_subpixel_epsilon` | double | 0.1 | サブピクセル補正の収束閾値 |
| `enable_floor_plane_fit` | bool | true | 床平面推定スイッチ |
| `floor_inlier_threshold_mm` | double | 8.0 | 平面インライヤー閾値 |
| `floor_ransac_iterations` | int | 500 | RANSAC 反復数 |
| `floor_min_inlier_ratio` | double | 0.7 | 最小インライヤ比率 |
| `floor_z_min_mm` | double | 300.0 | 範囲フィルタ下限 |
| `floor_z_max_mm` | double | 1500.0 | 範囲フィルタ上限 |
| `floor_downsample_grid` | int | 4 | ダウンサンプリンググリッド |
| `max_plane_std_mm` | double | 8.0 | 平面標準偏差の許容上限 |
| `session_attempts` | int | 5 | キャリブレーション試行回数 |
| `random_seed` | uint64 | 42 | 乱数シード |
| `log_level` | string | `"info"` | spdlog ログレベル |
| `aruco_dictionary` | string | `"DICT_4X4_50"` | OpenCV ArUco 辞書名 |
| `playmat_layout_path` | string | `"config/toio_playmat.json"` | Position ID レイアウト定義ファイル |
| `board_mount_label` | string | `"center_mount_nominal"` | レイアウト JSON 内の Charuco マウント識別子 |

---

## 付録 B: 実装進捗と今後の対応

### 実装済みコンポーネント（2025-11-08時点）

✅ **完了:**
- `CharucoDetector` クラス（公開API、サブピクセル補正対応）
- `FloorPlaneEstimator` クラス（Intrinsics対応RANSAC + 標準偏差/インライヤ率算出）
- `PlaymatLayout` クラス（JSON読み込み、アフィン変換）
- `CalibrationSession` クラス（統計評価、JSON v2.0 + validation 出力）
- `CalibrationPipeline` Intrinsics取得・歪み補正・床面推定
- `capture_calibration` CLIツール

### 🔧 残作業・次の一歩

1. **QC スクリプト / 自動レポート**
   - `tools/run_calibration_qc.*` を追加し、RealSense接続→パイプライン実行→閾値判定→Markdown/JSON レポートを一括実行
   - `ctest -L qc` に統合し、QC 担当が sudo 実行だけで判定できるようにする

2. **統合テストとログ整備**
   - Charuco/深度のモックフィクスチャを用意し、`test_floor_plane_estimator.cpp` / `test_calibration_pipeline_integration.cpp` を有効化
   - `SessionConfig::save_intermediate_snapshots` を活用し、失敗時のデバッグ情報を自動保存

3. **ヘッダー/内部実装の整理（オプション）**
   - `FloorPlaneEstimator.h` など内部専用クラスを `src/` 配下に移すか、公開APIとしての意図を明文化

### Apple Silicon macOS 環境での注意事項

**背景:** RealSense D415 を Apple Silicon Mac で使用する際、特有の問題が発生します。詳細は [docs/apple_silicon_realsense.md](../../../docs/apple_silicon_realsense.md) を参照。

**主要な問題:**
- `libusb_init failed: LIBUSB_ERROR_OTHER` → **sudo実行が必須**
- `AppleUSBHostUserClient::openGated ... provider is already opened` → **DriverKit導入または sudo + カメラプロセス停止**
- USB2接続での速度制限（15 FPS頭打ち） → **USB3接続を確認**

**開発環境での対処:**
```bash
# 毎回実行する「儀式」
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true
# カメラを抜き差し後、即座にsudoで実行
sudo ./capture_calibration
```

**CI/テストへの影響:**
- 実機テストは `REAL_DEVICE_TESTS=ON` でビルドし、**手動実行または夜間実行**を推奨
- macOS CI環境では sudo 権限とデバイスアクセスが制限されるため、デフォルトはモックテストのみ
- 実機テストの成功率は環境依存（USB接続状態、macOSバージョン、DriverKit設定）

**推奨設定:**
- RealSenseアクセスには必ず sudo を使用
- USB3接続を確認（`system_profiler SPUSBDataType` で "5 Gb/s" を確認）
- Intel RealSense DriverKit を導入してシステム設定で許可
- 開発時は `~/bin/realsense-run.sh` などのラッパースクリプトを使用（docs参照）

### 次のステップ

**推奨される実装順序:**
1. QC ランナー（`tools/run_calibration_qc.*`）の実装と ctest 連携
2. モック/実機テストの整備（`test_floor_plane_estimator.cpp` など）
3. リトライ結果の永続化・レポート整形を `CalibrationSession` に追加
4. 必要に応じて内部ヘッダー配置の整理

**検証方法:**
```bash
# ビルド
cmake -B build -DLOCOMOTION_BUILD_TESTS=ON
cmake --build build

# モックテスト（sudo不要）
cd build && ctest

# 実機テスト（sudo必須、Apple Silicon）
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true
sudo ./build/capture_calibration --config config/calibration_config_sample.json
```

---

## 付録 C: 参考文献リンク
- librealsense Align サンプル
  https://github.com/IntelRealSense/librealsense/tree/master/examples/align
- OpenCV Charuco サンプル
  https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/camera_calibration/charuco_diamond.cpp
- RANSAC 平面フィット参考
  https://pointclouds.org/documentation/tutorials/random_sample_consensus.php
- Apple Silicon でのRealSense運用
  [docs/apple_silicon_realsense.md](../../../docs/apple_silicon_realsense.md)
