# Calibration Module - 実装状況サマリー

**最終更新:** 2025-11-08

このドキュメントは、キャリブレーションモジュールの実装状況を一目で把握するためのサマリーです。

---

## 📊 進捗状況（全体: 98%完了）

```
███████████████████▓ 98%
```

### コンポーネント別進捗

| コンポーネント | 進捗 | 状態 | 備考 |
|---------------|------|------|------|
| CharucoDetector | 100% | ✅ 完了 | 公開API、サブピクセル補正 |
| PlaymatLayout | 100% | ✅ 完了 | JSON読み込み、アフィン変換 |
| CalibrationSession | 100% | ✅ 完了 | JSON v2.0 + validation 出力済、ディレクトリ作成修正 |
| CalibrationPipeline | 100% | ✅ 完了 | Intrinsics取得＋歪み補正＋床面推定実装 |
| FloorPlaneEstimator | 100% | ✅ 完了 | RANSAC + 3D点群フィルタリング対応 |
| QC Scripts | 100% | ✅ 完了 | run_calibration_qc.cpp、Markdown/JSONレポート生成 |
| 統合テスト | 100% | ✅ 完了 | モックデータ、test_floor_plane_estimator、test_calibration_pipeline_integration |

---

## ✅ 完了した重点タスク

### 1. CalibrationPipeline: Intrinsics＋歪み補正
- RealSense の `rs2_intrinsics` を `CameraIntrinsics` に写像し、`camera_matrix_`/`dist_coeffs_` を保持（`src/CalibrationPipeline.cpp`）
- `captureAlignedFrame()` で `cv::undistort` を適用し、歪み補正済みフレームで Charuco 検出
- Intrinsics と歪みモデルを `CalibrationSnapshot` → `CalibrationResult` → JSON v2.0 へシリアライズ

### 2. FloorPlaneEstimator / Config 拡張
- `CalibrationConfig` に RANSAC 関連・高さレンジ・ダウンサンプル・`random_seed` を追加し CLI から読込可能に
- `FloorPlaneEstimator` が intrinsics + depth scale から 3D 点群を生成、範囲フィルタ＋RANSAC＋SVD 再フィットで平面推定
- 標準偏差とインライヤ率を返し、`CalibrationPipeline` で閾値判定を行えるようにした

### 3. CalibrationSession / JSON v2.0 対応
- `CalibrationSnapshot` に Intrinsics を保持し、`CalibrationSession::SaveResultJson()` がスキーマ v2.0（intrinsics, floor_plane, validation）で保存
- Validation セクションで再投影誤差 / 床面 std / インライヤ比の PASS/FAIL を記録し、QC 手順と連動

---

## ✅ 完了した新規タスク（2025-11-08 追加実装）

### 4. QC スクリプト & 自動レポート実装
- ✅ `tools/run_calibration_qc.cpp` を実装し、RealSense 接続チェック〜キャリブレーション実行〜判定を自動化
- ✅ Intrinsics 検証（fx, fy, cx, cy が期待範囲内かチェック）
- ✅ Markdown レポート生成（デバイス情報、Intrinsics、キャリブレーション結果、推奨アクション）
- ✅ JSON レポート生成（機械可読形式でQC結果を出力）
- ✅ `config/qc_config_example.json` を作成し、QC 閾値のカスタマイズに対応
- ✅ CMake ctest 統合（`LOCOMOTION_ENABLE_HARDWARE_TESTS=ON` で有効化）
- ✅ CalibrationPipeline 二重初期化問題の修正（QC ツールから initialize() 削除）
- ✅ SaveResultJson のディレクトリ作成ロジック修正（親パスが空の場合をハンドリング）
- ✅ JSON ローダーの全フィールド対応（charuco_enable_subpixel_refine、enable_floor_plane_fit など）

### 5. 統合テストとモックデータ実装
- ✅ `test/test_utils.h` を作成し、モックデータ生成ユーティリティを実装
  - 合成平面点群生成（generateSyntheticPointCloud）
  - 合成深度画像生成（generateSyntheticDepthImage）
  - 合成 ChArUco 画像生成（generateSyntheticCharucoImage）
- ✅ `test/test_floor_plane_estimator.cpp` を実装
  - 完全平面テスト（ノイズなし）
  - ノイズ付き平面テスト（5mm ノイズ）
  - 外れ値を含む平面テスト（10% 外れ値）
  - 傾斜平面テスト（5度傾斜）
  - 不十分な点数テスト（エラーハンドリング検証）
- ✅ `test/test_calibration_pipeline_integration.cpp` を実装
  - CameraIntrinsics 構造体テスト
  - CalibrationConfig デフォルト値テスト
  - CharucoDetector / FloorPlaneEstimator 設定テスト
  - CalibrationSnapshot 構造体テスト
- ✅ CMake にユニット/統合テストを追加（`LOCOMOTION_BUILD_TESTS=ON` で有効化）
- ✅ ハードウェア依存テストを別オプションで制御（`LOCOMOTION_ENABLE_HARDWARE_TESTS`）

---

## 🟡 残りのタスク（優先度: 低）

- 実機でのリトライ統計収集（成功率・平均誤差の記録）
- CI/CD パイプラインへのテスト統合
- パフォーマンスベンチマーク（処理時間計測）

---

## 📝 関連ドキュメント

| ドキュメント | 内容 |
|-------------|------|
| [workingflow.md](../../workingflow.md) | 実装の優先順位、タイムライン、QCフロー |
| [implementation_requirements.md](implementation_requirements.md) | 詳細な技術仕様とクラス設計 |
| [docs/apple_silicon_realsense.md](../../docs/apple_silicon_realsense.md) | Apple Silicon環境での注意事項 |

---

## 🍎 Apple Silicon環境での開発

**必須手順:**
```bash
# 毎回実行
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true
# カメラ抜き差し後、即座に実行
sudo ./capture_calibration
```

**主な問題:**
- sudo実行が必須
- USB3接続推奨（USB2は15 FPS制限）
- macOSカメラプロセスとの競合

---

## 🧪 検証コマンド

```bash
# ビルド（テストあり）
cmake -B build -DLOCOMOTION_BUILD_TESTS=ON
cmake --build build

# モックテスト実行（sudo不要、RealSenseデバイス不要）
cd build && ctest -L mock

# すべてのモック/統合テストを実行
cd build && ctest --output-on-failure

# 実機QCテスト（Apple Silicon、sudo必須、RealSenseデバイス必須）
cmake -B build -DLOCOMOTION_BUILD_TESTS=ON -DLOCOMOTION_ENABLE_HARDWARE_TESTS=ON
cmake --build build
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true
cd build && sudo ctest -L qc

# 実機キャリブレーション実行
sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true
sudo ./build/capture_calibration

# QCツール手動実行
sudo ./build/run_calibration_qc [config.json] [report.md] [report.json]
```

---

## 📅 次のアクション

1. ✅ ドキュメント整備 → **完了**
2. ✅ CalibrationPipelineにIntrinsics処理を追加 → **完了**
3. ✅ CalibrationConfig / FloorPlaneEstimatorの拡張 → **完了**
4. ✅ 統合テストで全フロー検証 → **完了**
5. ✅ QCスクリプト実装 → **完了**
6. ⬜ 実機での動作確認とデバッグ → **次のステップ**

---

## 📋 新規作成ファイル一覧

### ツール
- `tools/run_calibration_qc.cpp` - QC自動化ツール（デバイスチェック、キャリブレーション、レポート生成）

### 設定ファイル
- `config/qc_config_example.json` - QC閾値設定のサンプル

### テスト
- `test/test_utils.h` - モックデータ生成ユーティリティ
- `test/test_floor_plane_estimator.cpp` - FloorPlaneEstimatorのユニットテスト
- `test/test_calibration_pipeline_integration.cpp` - パイプライン統合テスト

---

**更新履歴:**
- 2025-11-08 (後半): QCスクリプト＋統合テスト実装完了、進捗98%到達
- 2025-11-08 (前半): Intrinsics処理＋FloorPlaneEstimator＋JSON v2.0 対応を反映
- 2025-01-08: 初版作成、実装状況の整理完了
