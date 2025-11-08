#pragma once

#include <chrono>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "locomotion/calibration/CameraIntrinsics.h"
#include "locomotion/calibration/ToioCoordinateTransform.h"

namespace locomotion::calibration {

struct CalibrationResult {
  CameraIntrinsics intrinsics;
  cv::Mat homography_color_to_floor;
  cv::Mat homography_color_to_toio;
  ToioCoordinateTransform toio_transform;
  cv::Vec4f floor_plane;
  double reprojection_error_px{0.0};  // 画像ピクセル座標系での誤差
  double reprojection_error_floor_mm{0.0};
  double reprojection_error_toio{0.0};
  double floor_plane_std_mm{0.0};
  double inlier_ratio{0.0};
  int floor_inlier_count{0};
  double camera_height_mm{0.0};
  int detected_charuco_corners{0};
  int capture_count{0};
  std::chrono::system_clock::time_point timestamp;
  std::chrono::system_clock::time_point first_capture_timestamp;
  std::chrono::system_clock::time_point last_capture_timestamp;
};

}  // namespace locomotion::calibration
