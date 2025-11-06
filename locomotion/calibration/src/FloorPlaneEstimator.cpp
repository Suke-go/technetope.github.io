#include "locomotion/calibration/FloorPlaneEstimator.h"

#include <spdlog/spdlog.h>

namespace locomotion::calibration {

FloorPlaneEstimator::FloorPlaneEstimator(FloorPlaneEstimatorConfig config)
    : config_(config) {}

void FloorPlaneEstimator::SetConfig(FloorPlaneEstimatorConfig config) {
  config_ = config;
}

const FloorPlaneEstimatorConfig& FloorPlaneEstimator::config() const noexcept {
  return config_;
}

std::optional<FloorPlaneEstimate> FloorPlaneEstimator::Estimate(
    const cv::Mat& depth_image) const {
  if (depth_image.empty()) {
    spdlog::warn("FloorPlaneEstimator received empty depth image.");
    return std::nullopt;
  }

  // TODO: Implement actual RANSAC plane fitting using depth point cloud.
  FloorPlaneEstimate estimate;
  estimate.plane = {0.0F, 0.0F, 1.0F, 0.0F};
  estimate.plane_std_mm = 0.0;
  estimate.inlier_ratio = 0.0;
  return estimate;
}

}  // namespace locomotion::calibration
