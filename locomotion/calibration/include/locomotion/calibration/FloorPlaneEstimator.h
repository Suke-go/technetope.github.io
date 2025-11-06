#pragma once

#include <optional>

#include <opencv2/core.hpp>

namespace locomotion::calibration {

struct FloorPlaneEstimatorConfig {
  double inlier_threshold_mm{8.0};
  int ransac_iterations{500};
  int min_sample_count{3};
};

struct FloorPlaneEstimate {
  cv::Vec4f plane;
  double plane_std_mm{0.0};
  double inlier_ratio{0.0};
};

class FloorPlaneEstimator {
 public:
  explicit FloorPlaneEstimator(FloorPlaneEstimatorConfig config = {});

  void SetConfig(FloorPlaneEstimatorConfig config);
  [[nodiscard]] const FloorPlaneEstimatorConfig& config() const noexcept;

  std::optional<FloorPlaneEstimate> Estimate(const cv::Mat& depth_image) const;

 private:
  FloorPlaneEstimatorConfig config_{};
};

}  // namespace locomotion::calibration
