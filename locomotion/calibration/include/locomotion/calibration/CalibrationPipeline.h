#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "locomotion/calibration/CameraIntrinsics.h"
#include "locomotion/calibration/CharucoDetector.h"
#include "locomotion/calibration/FloorPlaneEstimator.h"
#include "locomotion/calibration/PlaymatLayout.h"
#include "locomotion/calibration/ToioCoordinateTransform.h"

#include <librealsense2/rs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>

namespace locomotion::calibration {

struct CalibrationConfig {
  int color_width{640};
  int color_height{480};
  int depth_width{640};
  int depth_height{480};
  int fps{15};

  double depth_min_distance_mm{2300.0};
  double depth_max_distance_mm{2800.0};
  double expected_depth_scale_m{0.001};

  int charuco_squares_x{5};             // チェス盤の列数
  int charuco_squares_y{7};             // チェス盤の行数
  float charuco_square_length_mm{45.0F};
  float charuco_marker_length_mm{33.0F};

  int min_charuco_corners{12};
  double homography_ransac_thresh_px{3.0};
  double max_reprojection_error_id{8.0};
  bool charuco_enable_subpixel_refine{true};
  int charuco_subpixel_window{5};
  int charuco_subpixel_max_iterations{30};
  double charuco_subpixel_epsilon{0.1};

  bool enable_floor_plane_fit{true};
  double floor_inlier_threshold_mm{8.0};
  int floor_ransac_iterations{500};
  double floor_min_inlier_ratio{0.7};
  double floor_z_min_mm{2400.0};
  double floor_z_max_mm{2800.0};
  int floor_downsample_grid{4};
  int floor_min_valid_points{100};

  double max_plane_std_mm{8.0};
  int session_attempts{5};
  uint64_t random_seed{42};

  double camera_height_warn_min_mm{2500.0};
  double camera_height_warn_max_mm{2700.0};

  bool enable_spatial_filter{true};
  bool enable_color_auto_exposure{true};

  std::string aruco_dictionary{"DICT_4X4_50"};
  std::string playmat_layout_path{"config/toio_playmat.json"};
  std::string board_mount_label{"center_mount_nominal"};
  std::string log_level{"info"};
};

struct CalibrationSnapshot {
  CameraIntrinsics intrinsics;
  cv::Mat homography_color_to_floor;     // 画像→床(mm) ホモグラフィ
  cv::Mat homography_color_to_toio;      // 画像→toio Position ID 座標
  ToioCoordinateTransform toio_transform;
  cv::Vec4f floor_plane{0.0F, 0.0F, 1.0F, 0.0F};  // 床平面（ax+by+cz+d=0）
  double reprojection_error_px{std::numeric_limits<double>::infinity()};  // 画像ピクセル座標系での誤差
  double reprojection_error_floor_mm{std::numeric_limits<double>::infinity()};
  double reprojection_error_toio{std::numeric_limits<double>::infinity()};
  double floor_plane_std_mm{0.0};
  double inlier_ratio{0.0};
  int floor_inlier_count{0};
  double camera_height_mm{0.0};
  int detected_charuco_corners{0};
  std::chrono::system_clock::time_point timestamp;
};

struct FrameBundle {
  cv::Mat color;
  cv::Mat depth;
  double timestamp_ms{0.0};
};

struct CalibrationValidation {
  double transform_error_at_correspondences{0.0};
  double roundtrip_error_px{0.0};
  bool coverage_area_valid{false};
  std::vector<std::string> warnings;
};

class CalibrationPipeline {
 public:
  explicit CalibrationPipeline(CalibrationConfig config);
  ~CalibrationPipeline();

  CalibrationPipeline(const CalibrationPipeline&) = delete;
  CalibrationPipeline& operator=(const CalibrationPipeline&) = delete;
  CalibrationPipeline(CalibrationPipeline&&) noexcept = default;
  CalibrationPipeline& operator=(CalibrationPipeline&&) noexcept = default;

  bool initialize();
  bool initialize(const std::string& config_file_dir);
  std::optional<CalibrationSnapshot> runOnce();

  bool CaptureFrame(FrameBundle& bundle);
  std::optional<CalibrationSnapshot> ProcessFrame(const FrameBundle& bundle) const;

  [[nodiscard]] const CalibrationConfig& config() const noexcept { return config_; }
  [[nodiscard]] const std::string& camera_model() const noexcept { return camera_model_; }
  [[nodiscard]] const std::string& camera_serial() const noexcept { return camera_serial_; }
  [[nodiscard]] double depth_scale_m() const noexcept { return depth_scale_m_; }

  // Coordinate transformation methods
  // Note: These require a valid calibration snapshot. Use the latest snapshot from ProcessFrame().
  cv::Point2f TransformPixelToToio(cv::Point2f pixel,
                                    const CalibrationSnapshot& snapshot) const;
  cv::Point2f TransformToioToPixel(cv::Point2f toio_pos,
                                    const CalibrationSnapshot& snapshot) const;
  bool IsToioPositionInCoverage(cv::Point2f toio_pos,
                                 const CalibrationSnapshot& snapshot) const;

  // Calibration validation
  CalibrationValidation ValidateCalibration(const CalibrationSnapshot& snapshot) const;

 private:
  bool captureAlignedFrame(FrameBundle& bundle);
  bool computeHomographies(const CharucoDetectionResult& detection,
                           cv::Mat& homography_color_to_floor,
                           cv::Mat& homography_color_to_toio,
                           ToioCoordinateTransform& toio_transform,
                           double& reprojection_error_floor_mm,
                           double& reprojection_error_toio,
                           double& reprojection_error_px) const;
  bool estimateFloorPlane(const FrameBundle& bundle, cv::Vec4f& plane,
                          double& plane_std_mm, double& inlier_ratio,
                          int& inlier_count, double& camera_height_mm) const;
  std::vector<cv::Point2f> boardPointsToPositionId(
      const std::vector<cv::Point3f>& object_points,
      const BoardMount** mount_out = nullptr) const;
  bool warnOnceCameraHeight(double camera_height_mm) const;

  CalibrationConfig config_;

  rs2::pipeline pipeline_;
  rs2::pipeline_profile profile_;
  rs2::align align_to_color_{RS2_STREAM_COLOR};
  rs2::spatial_filter spatial_filter_;
  bool spatial_filter_enabled_{false};

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> board_;
  CharucoDetectorConfig charuco_config_;
  std::unique_ptr<CharucoDetector> charuco_detector_;
  std::unique_ptr<FloorPlaneEstimator> floor_estimator_;

  PlaymatLayout playmat_layout_;
  bool has_playmat_layout_{false};
  mutable bool warned_layout_not_loaded_{false};
  mutable bool warned_mount_missing_{false};
  mutable bool warned_camera_height_{false};

  CameraIntrinsics camera_intrinsics_;
  bool intrinsics_loaded_{false};
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  double depth_scale_m_{0.001};
  std::string camera_model_;
  std::string camera_serial_;
  bool is_initialized_{false};
};

}  // namespace locomotion::calibration
