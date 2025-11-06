#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "locomotion/calibration/CharucoDetector.h"
#include "locomotion/calibration/FloorPlaneEstimator.h"
#include "locomotion/calibration/PlaymatLayout.h"

#include <librealsense2/rs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>

namespace locomotion::calibration {

struct CalibrationConfig {
  int color_width{1280};
  int color_height{720};
  int depth_width{848};
  int depth_height{480};
  int fps{30};

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

  std::string aruco_dictionary{"DICT_4X4_50"};
  std::string playmat_layout_path{"config/toio_playmat.json"};
  std::string board_mount_label{"center_mount_nominal"};
  std::string log_level{"info"};
};

struct CalibrationSnapshot {
  cv::Mat homography_color_to_position;  // 画像→toio Position ID 座標
  cv::Vec4f floor_plane;                 // 床平面（ax+by+cz+d=0）
  double reprojection_error;             // Position ID 座標系での誤差目安
  double floor_plane_std_mm{0.0};
  double inlier_ratio{0.0};
  int detected_charuco_corners{0};
  std::chrono::system_clock::time_point timestamp;
};

struct FrameBundle {
  cv::Mat color;
  cv::Mat depth;
  double timestamp_ms{0.0};
};

class CalibrationPipeline {
 public:
  explicit CalibrationPipeline(CalibrationConfig config);
  ~CalibrationPipeline();

  CalibrationPipeline(const CalibrationPipeline&) = delete;
  CalibrationPipeline& operator=(const CalibrationPipeline&) = delete;

  bool initialize();
  std::optional<CalibrationSnapshot> runOnce();

  [[nodiscard]] const CalibrationConfig& config() const noexcept { return config_; }

 private:
  bool captureAlignedFrame(FrameBundle& bundle);
  bool computeHomography(const std::vector<cv::Point2f>& image_corners,
                         const std::vector<cv::Point3f>& object_points,
                         cv::Mat& homography,
                         double& reprojection_error);
  bool estimateFloorPlane(const FrameBundle& bundle, cv::Vec4f& plane,
                          double& plane_std_mm, double& inlier_ratio);
  std::vector<cv::Point2f> boardPointsToPositionId(
      const std::vector<cv::Point3f>& object_points) const;

  CalibrationConfig config_;

  rs2::pipeline pipeline_;
  rs2::pipeline_profile profile_;
  rs2::align align_to_color_{RS2_STREAM_COLOR};

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> board_;
  CharucoDetectorConfig charuco_config_;
  std::unique_ptr<CharucoDetector> charuco_detector_;
  std::unique_ptr<FloorPlaneEstimator> floor_estimator_;

  PlaymatLayout playmat_layout_;
  bool has_playmat_layout_{false};
  mutable bool warned_layout_not_loaded_{false};
};

}  // namespace locomotion::calibration
