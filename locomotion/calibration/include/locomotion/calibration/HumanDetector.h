#pragma once

#include <chrono>
#include <vector>

#include <opencv2/core.hpp>

#include "locomotion/calibration/CalibrationPipeline.h"

namespace locomotion::calibration {

enum class HumanMotionState {
  STANDING,  // 静止中（立っている）
  MOVING,    // 移動中（足を動かしている）
  UNKNOWN    // 判定不可
};

struct HumanDetection {
  int id{0};                                    // トラッキングID
  cv::Point2f head_position_px;                 // 頭位置（画像座標）
  cv::Point2f foot_center_px;                   // 足の中心位置（画像座標）
  cv::RotatedRect ellipse_px;                   // 楕円領域（画像座標）
  cv::Point2f toio_position;                    // toio座標系での位置
  cv::RotatedRect ellipse_toio;                 // 楕円領域（toio座標系）
  cv::Point2f velocity_mm_per_s;                // 速度（mm/s、toio座標系）
  HumanMotionState motion_state{HumanMotionState::UNKNOWN};
  double confidence{0.0};                       // 検出確信度 (0-1)
  double height_above_floor_mm{0.0};            // 床からの高さ
  std::chrono::system_clock::time_point timestamp;
};

struct HumanDetectionResult {
  std::vector<HumanDetection> humans;
  std::chrono::system_clock::time_point timestamp;
  double processing_time_ms{0.0};               // 処理時間
  bool has_valid_detections{false};
};

struct HumanDetectionConfig {
  // Coarse-to-fine設定
  int coarse_scale_factor{4};                   // 粗検出の縮小倍率 (640x480 -> 160x120)
  int fine_roi_margin_px{20};                   // 詳細検出時のROIマージン

  // 深度フィルタ
  double min_depth_mm{2300.0};                  // 最小深度（床+150mm）
  double max_depth_mm{2800.0};                  // 最大深度（床+800mm、頭部含む）
  double foot_depth_min_mm{2300.0};             // 足の深度範囲（最小）
  double foot_depth_max_mm{2450.0};             // 足の深度範囲（最大）
  double head_depth_min_mm{2400.0};             // 頭の深度範囲（最小）
  double head_depth_max_mm{2600.0};             // 頭の深度範囲（最大）

  // サイズフィルタ
  double min_person_height_mm{1200.0};          // 最小身長
  double max_person_height_mm{2000.0};          // 最大身長
  double min_foot_diameter_mm{50.0};            // 最小足サイズ
  double max_foot_diameter_mm{200.0};           // 最大足サイズ
  int min_foot_area_pixels{100};                // 最小足面積（ピクセル）

  // 楕円推定
  double ellipse_aspect_ratio{0.4};             // 楕円の縦横比（高さ/幅）
  double ellipse_vertical_offset{0.3};          // 頭から楕円中心までのオフセット（身長比）

  // 動き判定
  double movement_threshold_mm{30.0};           // 移動判定の閾値（mm）
  int movement_history_frames{60};              // 動き判定に使用するフレーム数（60フレーム ≈ 2秒@30fps）
  double standing_velocity_threshold_mm_per_s{50.0}; // 静止判定の速度閾値
  
  // 時間重み付き速度計算
  double velocity_alpha{0.1};                   // 指数減衰係数（0.0-1.0、小さいほど過去を重視）
  double velocity_half_life_seconds{0.5};       // 速度の半減期（秒）
  double position_smoothing_alpha{0.3};         // 位置の平滑化係数（0.0-1.0）
  
  // ノイズ対策
  double noise_threshold_mm{5.0};               // ノイズと見なす移動距離の閾値（mm）
  double outlier_threshold_std{3.0};            // 外れ値判定の標準偏差倍率
  bool enable_outlier_rejection{true};          // 外れ値除去を有効化

  // トラッキング
  double tracking_max_distance_mm{200.0};       // トラッキング最大距離
  int max_tracked_humans{10};                   // 最大追跡人数
  double tracking_timeout_seconds{2.0};         // トラッキングタイムアウト

  // パフォーマンス
  bool enable_coarse_to_fine{true};             // Coarse-to-fine有効化
  int skip_frames{0};                           // スキップフレーム数（0=全フレーム処理）

  // モルフォロジー処理
  int morphology_kernel_size{5};                // モルフォロジーカーネルサイズ
  int morphology_iterations{2};                 // モルフォロジー反復回数
  
  // フォールバック設定
  double head_to_foot_search_radius_mm{150.0};  // 頭からの検索半径（15cm）
  double average_person_height_mm{1650.0};      // 平均身長（フォールバック用）
};

class HumanDetector {
 public:
  explicit HumanDetector(HumanDetectionConfig config = {});

  // フレーム処理（毎フレーム呼び出し）
  HumanDetectionResult Detect(const FrameBundle& frame,
                              const CalibrationSnapshot& calibration);

  // 設定
  void SetConfig(HumanDetectionConfig config);
  [[nodiscard]] const HumanDetectionConfig& GetConfig() const noexcept;

  // リセット（トラッキング状態をクリア）
  void Reset();

  // トラッキング履歴から速度を取得（HumanDetectionConverter用）
  cv::Point2f GetVelocity(int human_id) const;

 private:
  // Coarse検出（低解像度）
  std::vector<cv::Rect> CoarseDetect(const cv::Mat& color_resized,
                                      const cv::Mat& depth_resized);

  // Fine検出（高解像度、ROI内）
  std::vector<HumanDetection> FineDetect(const FrameBundle& frame,
                                          const std::vector<cv::Rect>& coarse_rois,
                                          const CalibrationSnapshot& calibration);

  // 頭と足の位置推定
  bool EstimateHeadAndFoot(const cv::Mat& depth_roi,
                           const cv::Rect& roi,
                           const CalibrationSnapshot& calibration,
                           cv::Point2f& head_pos,
                           cv::Point2f& foot_center,
                           double& height_mm,
                           double& confidence,
                           int tracking_id = -1);  // トラッキングID（フォールバック用）
  
  // 頭中心からの半径内で足を検出
  bool DetectFootNearHead(const cv::Mat& depth_roi,
                          const cv::Rect& roi,
                          const cv::Point2f& head_pos_px,
                          const CalibrationSnapshot& calibration,
                          double search_radius_mm,
                          cv::Point2f& foot_center,
                          double& confidence);

  // 楕円領域の推定
  cv::RotatedRect EstimateEllipse(const cv::Point2f& head_pos,
                                   const cv::Point2f& foot_center,
                                   double height_mm);

  // 動きの判定（時間重み付き速度計算を使用）
  HumanMotionState DetermineMotionState(int human_id,
                                         const cv::Point2f& current_position,
                                         double delta_time_seconds);

  // トラッキング
  int AssignTrackingId(const cv::Point2f& position);
  void UpdateTrackingHistory(int id, const cv::Point2f& position,
                             std::chrono::system_clock::time_point timestamp);
  void CleanupOldTracks(std::chrono::system_clock::time_point current_time);
  
  // 時間重み付き速度計算（指数減衰重み）
  cv::Point2f ComputeWeightedVelocity(const TrackingState& track,
                                       std::chrono::system_clock::time_point current_time) const;
  
  // 位置の平滑化（指数平滑化）
  cv::Point2f SmoothPosition(const cv::Point2f& current_position,
                             const cv::Point2f& previous_smoothed,
                             double alpha) const;
  
  // 外れ値除去
  bool IsOutlier(const cv::Point2f& position,
                 const TrackingState& track) const;
  
  // ノイズフィルタ（小さな動きを無視）
  bool IsNoise(const cv::Point2f& displacement, double delta_time_seconds) const;

  // 深度画像から3D位置を取得（簡易版）
  cv::Point3f DepthTo3D(const cv::Point2f& pixel, uint16_t depth_value,
                        const CameraIntrinsics& intrinsics,
                        double depth_scale_m) const;

  HumanDetectionConfig config_;

  // トラッキング状態
  struct TrackingState {
    cv::Point2f last_position;  // toio座標系での位置（平滑化済み）
    cv::Point2f last_velocity;  // 前回の速度（平滑化済み）
    cv::Point2f last_head_position_px;  // 前回の頭位置（画像座標）
    cv::Point2f last_foot_position_px;  // 前回の足位置（画像座標）
    std::vector<cv::Point2f> position_history;  // 位置履歴（最大60フレーム）
    std::vector<std::chrono::system_clock::time_point> time_history;  // 時刻履歴
    std::chrono::system_clock::time_point last_seen;
    int id;
    HumanMotionState last_motion_state{HumanMotionState::UNKNOWN};
    double last_height_mm{0.0};  // 前回の身長
    double velocity_variance{0.0};  // 速度の分散（ノイズ推定用）
  };
  std::vector<TrackingState> tracked_humans_;
  int next_tracking_id_{1};
  int frame_counter_{0};

  // 作業用バッファ（再利用）
  cv::Mat color_resized_;
  cv::Mat depth_resized_;
  cv::Mat depth_mask_;
  cv::Mat depth_filtered_;
  cv::Mat morphology_kernel_;
};

}  // namespace locomotion::calibration

