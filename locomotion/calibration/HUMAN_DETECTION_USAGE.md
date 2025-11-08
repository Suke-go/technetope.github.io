# Human Detection System Usage

## 概要

RealSense D415の深度画像を使用して人間を検出し、toio座標系に変換するシステムです。

## 基本的な使用方法

```cpp
#include "locomotion/calibration/HumanDetector.h"
#include "locomotion/calibration/HumanDetectionConverter.h"
#include "locomotion/calibration/CalibrationPipeline.h"
#include "locomotion/robot/MotionPlanner.h"

// 1. キャリブレーションパイプラインの初期化
CalibrationPipeline pipeline(config);
pipeline.initialize();
auto snapshot = pipeline.ProcessFrame(frame_bundle);

if (!snapshot) {
    // キャリブレーションが必要
    return;
}

// 2. 人間検出器の初期化
HumanDetector detector;

// 3. 人間検出の実行
HumanDetectionResult result = detector.Detect(frame_bundle, *snapshot);

if (result.has_valid_detections) {
    for (const auto& human : result.humans) {
        std::cout << "Human ID: " << human.id << std::endl;
        std::cout << "Position (toio): " << human.toio_position.x << ", " 
                  << human.toio_position.y << std::endl;
        std::cout << "Motion State: " 
                  << (human.motion_state == HumanMotionState::MOVING ? "MOVING" : "STANDING") 
                  << std::endl;
        std::cout << "Velocity: " << human.velocity_mm_per_s.x << ", " 
                  << human.velocity_mm_per_s.y << " mm/s" << std::endl;
    }
}

// 4. MotionPlannerとの統合
std::vector<locomotion::robot::DynamicObstacle> obstacles =
    ConvertToDynamicObstacles(result, 0.8f, 5);

// 5. MotionPlannerで使用
MotionPlanner planner(config);
auto commands = planner.Plan(robot_intents, obstacles, delta_time);
```

## 設定パラメータ

```cpp
HumanDetectionConfig config;

// Coarse-to-fine設定
config.coarse_scale_factor = 4;  // 低解像度検出の縮小倍率
config.fine_roi_margin_px = 20;  // ROIマージン

// 深度フィルタ
config.min_depth_mm = 2300.0;      // 最小深度
config.max_depth_mm = 2800.0;      // 最大深度
config.foot_depth_min_mm = 2300.0; // 足の深度範囲（最小）
config.foot_depth_max_mm = 2450.0; // 足の深度範囲（最大）
config.head_depth_min_mm = 2400.0; // 頭の深度範囲（最小）
config.head_depth_max_mm = 2600.0; // 頭の深度範囲（最大）

// サイズフィルタ
config.min_person_height_mm = 1200.0; // 最小身長
config.max_person_height_mm = 2000.0; // 最大身長
config.min_foot_area_pixels = 100;    // 最小足面積

// 動き判定
config.movement_threshold_mm = 30.0;           // 移動判定の閾値
config.movement_history_frames = 5;            // 動き判定に使用するフレーム数
config.standing_velocity_threshold_mm_per_s = 50.0; // 静止判定の速度閾値

// トラッキング
config.tracking_max_distance_mm = 200.0;       // トラッキング最大距離
config.max_tracked_humans = 10;                // 最大追跡人数
config.tracking_timeout_seconds = 2.0;         // トラッキングタイムアウト

// パフォーマンス
config.enable_coarse_to_fine = true;  // Coarse-to-fine有効化
config.skip_frames = 0;               // スキップフレーム数（0=全フレーム処理）

HumanDetector detector(config);
```

## パフォーマンス

- 目標処理時間: < 50ms/フレーム
- 目標フレームレート: ≥ 15 FPS
- Coarse-to-fineにより約16倍高速化（4倍縮小時）

## 注意事項

1. キャリブレーションが必要: `CalibrationSnapshot`が有効である必要があります
2. 深度画像の品質: ノイズが多い場合は検出精度が低下する可能性があります
3. 照明条件: 暗い環境では深度センサーの精度が低下する可能性があります
4. 複数人の検出: 最大10人まで同時に追跡可能（設定で変更可能）

## トラブルシューティング

### 人間が検出されない
- 深度範囲の設定を確認（`min_depth_mm`, `max_depth_mm`）
- カメラの高さを確認（約2600mmが推奨）
- 照明条件を確認

### 検出精度が低い
- モルフォロジー処理のパラメータを調整
- サイズフィルタの設定を確認
- Coarse-to-fineの設定を調整

### パフォーマンスが低い
- `skip_frames`を増やしてフレームスキップを有効化
- `coarse_scale_factor`を増やして低解像度検出を強化
- `enable_coarse_to_fine`が有効であることを確認

