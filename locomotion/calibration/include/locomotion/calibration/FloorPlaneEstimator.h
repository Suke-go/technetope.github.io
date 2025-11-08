#pragma once

#include <optional>
#include <random>

#include <opencv2/core.hpp>

#include "locomotion/calibration/CameraIntrinsics.h"

namespace locomotion::calibration {

struct FloorPlaneEstimatorConfig {
  double inlier_threshold_mm{8.0};
  int ransac_iterations{500};
  int min_sample_count{3};
  double min_inlier_ratio{0.7};
  double z_min_mm{300.0};
  double z_max_mm{1500.0};
  int downsample_grid{4};
  int min_valid_points{100};
  uint64_t random_seed{42};
};

struct FloorPlaneEstimate {
  cv::Vec4f plane;
  double plane_std_mm{0.0};
  double inlier_ratio{0.0};
  int inlier_count{0};
  double camera_height_mm{0.0};
};

class FloorPlaneEstimator {
 public:
  explicit FloorPlaneEstimator(FloorPlaneEstimatorConfig config = {});

  void SetConfig(FloorPlaneEstimatorConfig config);
  [[nodiscard]] const FloorPlaneEstimatorConfig& config() const noexcept;

  std::optional<FloorPlaneEstimate> Estimate(const cv::Mat& depth_image,
                                             const CameraIntrinsics& intrinsics,
                                             double depth_scale_m) const;

 private:
  FloorPlaneEstimatorConfig config_{};
  mutable std::mt19937_64 rng_;
};

}  // namespace locomotion::calibration
