#include "locomotion/calibration/FloorPlaneEstimator.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace locomotion::calibration {

namespace {

constexpr double kEpsilon = 1e-6;

struct PlaneModel {
  cv::Vec4d plane;
  std::vector<int> inliers;
};

std::optional<cv::Vec4d> computePlaneFromPoints(const cv::Point3d& a,
                                                const cv::Point3d& b,
                                                const cv::Point3d& c) {
  cv::Point3d v1 = b - a;
  cv::Point3d v2 = c - a;
  cv::Point3d normal = v1.cross(v2);
  double norm = cv::norm(normal);
  if (norm < kEpsilon) {
    return std::nullopt;
  }
  normal /= norm;
  double d = -normal.dot(a);
  return cv::Vec4d(normal.x, normal.y, normal.z, d);
}

double distanceToPlane(const cv::Vec4d& plane, const cv::Point3d& point) {
  double numerator =
      std::abs(plane[0] * point.x + plane[1] * point.y + plane[2] * point.z + plane[3]);
  double denom = std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] +
                           plane[2] * plane[2]);
  if (denom < kEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  return numerator / denom;
}

cv::Vec4d refinePlaneLeastSquares(const std::vector<cv::Point3d>& points,
                                  const std::vector<int>& inliers) {
  if (inliers.size() < 3) {
    return cv::Vec4d(0.0, 0.0, 1.0, 0.0);
  }

  cv::Point3d centroid(0.0, 0.0, 0.0);
  for (int idx : inliers) {
    centroid += points[idx];
  }
  centroid *= 1.0 / static_cast<double>(inliers.size());

  cv::Mat data(static_cast<int>(inliers.size()), 3, CV_64F);
  for (size_t i = 0; i < inliers.size(); ++i) {
    const cv::Point3d& pt = points[inliers[i]];
    data.at<double>(static_cast<int>(i), 0) = pt.x - centroid.x;
    data.at<double>(static_cast<int>(i), 1) = pt.y - centroid.y;
    data.at<double>(static_cast<int>(i), 2) = pt.z - centroid.z;
  }

  cv::Mat w, u, vt;
  cv::SVD::compute(data, w, u, vt, cv::SVD::MODIFY_A);
  cv::Vec3d normal(vt.at<double>(2, 0), vt.at<double>(2, 1), vt.at<double>(2, 2));
  double norm = cv::norm(normal);
  if (norm < kEpsilon) {
    return cv::Vec4d(0.0, 0.0, 1.0, 0.0);
  }
  normal /= norm;

  double d = -normal.dot(cv::Vec3d(centroid.x, centroid.y, centroid.z));
  return cv::Vec4d(normal[0], normal[1], normal[2], d);
}

}  // namespace

FloorPlaneEstimator::FloorPlaneEstimator(FloorPlaneEstimatorConfig config)
    : config_(config), rng_(config.random_seed) {}

void FloorPlaneEstimator::SetConfig(FloorPlaneEstimatorConfig config) {
  config_ = config;
  rng_.seed(config_.random_seed);
}

const FloorPlaneEstimatorConfig& FloorPlaneEstimator::config() const noexcept {
  return config_;
}

std::optional<FloorPlaneEstimate> FloorPlaneEstimator::Estimate(
    const cv::Mat& depth_image, const CameraIntrinsics& intrinsics,
    double depth_scale_m) const {
  if (depth_image.empty()) {
    spdlog::warn("FloorPlaneEstimator received empty depth image.");
    return std::nullopt;
  }
  if (intrinsics.fx == 0.0 || intrinsics.fy == 0.0) {
    spdlog::warn("Invalid intrinsics for floor estimation (fx/fy == 0).");
    return std::nullopt;
  }
  if (depth_scale_m <= 0.0) {
    spdlog::warn("Invalid depth scale value: {}.", depth_scale_m);
    return std::nullopt;
  }

  const int stride = std::max(1, config_.downsample_grid);
  std::vector<cv::Point3d> points;
  points.reserve((depth_image.rows / stride) * (depth_image.cols / stride));

  const double scale_mm = depth_scale_m * 1000.0;

  for (int y = 0; y < depth_image.rows; y += stride) {
    const auto* row_ptr = depth_image.ptr<uint16_t>(y);
    for (int x = 0; x < depth_image.cols; x += stride) {
      uint16_t depth_raw = row_ptr[x];
      if (depth_raw == 0) {
        continue;
      }
      double z_mm = static_cast<double>(depth_raw) * scale_mm;
      if (z_mm < config_.z_min_mm || z_mm > config_.z_max_mm) {
        continue;
      }
      double x_mm = ((static_cast<double>(x) - intrinsics.cx) / intrinsics.fx) * z_mm;
      double y_mm = ((static_cast<double>(y) - intrinsics.cy) / intrinsics.fy) * z_mm;
      points.emplace_back(x_mm, y_mm, z_mm);
    }
  }

  if (points.size() <
      static_cast<size_t>(std::max({config_.min_sample_count, 3, config_.min_valid_points}))) {
    spdlog::warn("Not enough depth samples for plane estimation ({} points, require ≥ {}).",
                 points.size(), config_.min_valid_points);
    return std::nullopt;
  }

  std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

  PlaneModel best_model;
  size_t best_inliers = 0;
  cv::Vec4d best_plane(0.0, 0.0, 1.0, 0.0);

  for (int iter = 0; iter < config_.ransac_iterations; ++iter) {
    size_t idx1 = dist(rng_);
    size_t idx2 = dist(rng_);
    size_t idx3 = dist(rng_);
    if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3) {
      --iter;
      continue;
    }

    auto plane_opt = computePlaneFromPoints(points[idx1], points[idx2], points[idx3]);
    if (!plane_opt) {
      continue;
    }
    const cv::Vec4d& plane = *plane_opt;

    std::vector<int> inliers;
    inliers.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
      double distance = distanceToPlane(plane, points[i]);
      if (distance <= config_.inlier_threshold_mm) {
        inliers.push_back(static_cast<int>(i));
      }
    }

    if (inliers.size() > best_inliers) {
      best_inliers = inliers.size();
      best_plane = plane;
      best_model.plane = plane;
      best_model.inliers = std::move(inliers);
    }
  }

  if (best_inliers < config_.min_valid_points) {
    spdlog::warn(
        "Floor plane estimation failed: only {} inliers found (need ≥ {}).",
        best_inliers, config_.min_valid_points);
    return std::nullopt;
  }

  double inlier_ratio =
      static_cast<double>(best_inliers) / static_cast<double>(points.size());
  if (inlier_ratio < config_.min_inlier_ratio) {
    spdlog::warn("Floor plane estimation rejected: inlier ratio {:.3f} below {:.3f}.",
                 inlier_ratio, config_.min_inlier_ratio);
    return std::nullopt;
  }

  cv::Vec4d refined_plane =
      refinePlaneLeastSquares(points, best_model.inliers);

  if (refined_plane[2] < 0.0) {
    refined_plane *= -1.0;
  }

  double sum_sq = 0.0;
  for (int idx : best_model.inliers) {
    double distance = distanceToPlane(refined_plane, points[idx]);
    sum_sq += distance * distance;
  }
  double plane_std_mm = best_model.inliers.empty()
                            ? 0.0
                            : std::sqrt(sum_sq / static_cast<double>(best_model.inliers.size()));

  double normal_norm =
      std::sqrt(refined_plane[0] * refined_plane[0] + refined_plane[1] * refined_plane[1] +
                refined_plane[2] * refined_plane[2]);
  double camera_height_mm = 0.0;
  if (normal_norm > kEpsilon) {
    camera_height_mm = std::abs(refined_plane[3]) / normal_norm;
  }

  FloorPlaneEstimate estimate;
  estimate.plane = cv::Vec4f(static_cast<float>(refined_plane[0]),
                             static_cast<float>(refined_plane[1]),
                             static_cast<float>(refined_plane[2]),
                             static_cast<float>(refined_plane[3]));
  estimate.plane_std_mm = plane_std_mm;
  estimate.inlier_ratio = inlier_ratio;
  estimate.inlier_count = static_cast<int>(best_model.inliers.size());
  estimate.camera_height_mm = camera_height_mm;
  return estimate;
}

}  // namespace locomotion::calibration
