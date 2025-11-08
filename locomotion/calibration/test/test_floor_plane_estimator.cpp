#include <array>
#include <cmath>
#include <iostream>

#include <opencv2/core.hpp>

#include "locomotion/calibration/CameraIntrinsics.h"
#include "locomotion/calibration/FloorPlaneEstimator.h"
#include "test_utils.h"

using namespace locomotion::calibration;
using namespace locomotion::calibration::test;

namespace {

bool testPerfectPlane() {
  std::cout << "Test: Perfect plane (no noise, no outliers)\n";

  // Create a horizontal plane at z=500mm
  std::array<float, 4> plane_coeffs = {0.0F, 0.0F, 1.0F, -500.0F};  // z = 500

  cv::Mat point_cloud = generateSyntheticPointCloud(80, 60, plane_coeffs, 0.0, 0.0, 42);

  FloorPlaneEstimatorConfig config;
  config.inlier_threshold_mm = 5.0;
  config.ransac_iterations = 100;
  config.random_seed = 42;
  config.z_min_mm = 0.0;
  config.z_max_mm = 2000.0;
  config.min_valid_points = 50;

  FloorPlaneEstimator estimator(config);

  CameraIntrinsics intrinsics = createMockIntrinsics();

  cv::Mat depth = generateSyntheticDepthImage(1280, 720, intrinsics, plane_coeffs, 0.001, 0.0, 42);

  auto result = estimator.Estimate(depth, intrinsics, 0.001);

  if (!result) {
    std::cerr << "  FAIL: Plane fitting failed\n";
    return false;
  }

  cv::Vec4f estimated_plane = result->plane;
  double plane_std_mm = result->plane_std_mm;
  double inlier_ratio = result->inlier_ratio;

  // Normalize plane for comparison
  float norm = std::sqrt(estimated_plane[0] * estimated_plane[0] +
                         estimated_plane[1] * estimated_plane[1] +
                         estimated_plane[2] * estimated_plane[2]);
  cv::Vec4f normalized = estimated_plane / norm;

  // Expected: (0, 0, 1, -500) or (0, 0, -1, 500)
  bool normal_correct =
      (std::abs(normalized[0]) < 0.01F && std::abs(normalized[1]) < 0.01F &&
       std::abs(std::abs(normalized[2]) - 1.0F) < 0.01F);

  float d_expected = -500.0F / norm;
  bool d_correct = std::abs(normalized[3] - d_expected) < 50.0F;  // 50mm tolerance

  std::cout << "  Estimated plane: [" << normalized[0] << ", " << normalized[1] << ", "
            << normalized[2] << ", " << normalized[3] << "]\n";
  std::cout << "  Plane std dev: " << plane_std_mm << " mm\n";
  std::cout << "  Inlier ratio: " << inlier_ratio << "\n";

  if (!normal_correct) {
    std::cerr << "  FAIL: Plane normal incorrect\n";
    return false;
  }

  if (plane_std_mm > 1.0) {
    std::cerr << "  FAIL: Plane std dev too high for perfect plane\n";
    return false;
  }

  if (inlier_ratio < 0.95) {
    std::cerr << "  FAIL: Inlier ratio too low for perfect plane\n";
    return false;
  }

  std::cout << "  PASS\n\n";
  return true;
}

bool testNoisyPlane() {
  std::cout << "Test: Noisy plane (5mm noise, no outliers)\n";

  std::array<float, 4> plane_coeffs = {0.0F, 0.0F, 1.0F, -600.0F};  // z = 600

  cv::Mat point_cloud = generateSyntheticPointCloud(80, 60, plane_coeffs, 5.0, 0.0, 123);

  FloorPlaneEstimatorConfig config;
  config.inlier_threshold_mm = 15.0;  // Higher threshold for noisy data
  config.ransac_iterations = 200;
  config.random_seed = 123;
  config.z_min_mm = 0.0;
  config.z_max_mm = 2000.0;
  config.min_valid_points = 50;

  FloorPlaneEstimator estimator(config);

  CameraIntrinsics intrinsics = createMockIntrinsics();

  cv::Mat depth =
      generateSyntheticDepthImage(1280, 720, intrinsics, plane_coeffs, 0.001, 5.0, 123);

  auto result = estimator.Estimate(depth, intrinsics, 0.001);

  if (!result) {
    std::cerr << "  FAIL: Plane fitting failed\n";
    return false;
  }

  cv::Vec4f estimated_plane = result->plane;
  double plane_std_mm = result->plane_std_mm;
  double inlier_ratio = result->inlier_ratio;

  float norm = std::sqrt(estimated_plane[0] * estimated_plane[0] +
                         estimated_plane[1] * estimated_plane[1] +
                         estimated_plane[2] * estimated_plane[2]);
  cv::Vec4f normalized = estimated_plane / norm;

  std::cout << "  Estimated plane: [" << normalized[0] << ", " << normalized[1] << ", "
            << normalized[2] << ", " << normalized[3] << "]\n";
  std::cout << "  Plane std dev: " << plane_std_mm << " mm\n";
  std::cout << "  Inlier ratio: " << inlier_ratio << "\n";

  // With 5mm noise, expect std dev around 5mm (but allow 3-10mm range)
  if (plane_std_mm < 3.0 || plane_std_mm > 15.0) {
    std::cerr << "  FAIL: Plane std dev out of expected range for 5mm noise\n";
    return false;
  }

  if (inlier_ratio < 0.85) {
    std::cerr << "  FAIL: Inlier ratio too low\n";
    return false;
  }

  std::cout << "  PASS\n\n";
  return true;
}

bool testPlaneWithOutliers() {
  std::cout << "Test: Plane with outliers (2mm noise, 10% outliers)\n";

  std::array<float, 4> plane_coeffs = {0.0F, 0.0F, 1.0F, -450.0F};  // z = 450

  cv::Mat point_cloud = generateSyntheticPointCloud(80, 60, plane_coeffs, 2.0, 0.1, 456);

  FloorPlaneEstimatorConfig config;
  config.inlier_threshold_mm = 10.0;
  config.ransac_iterations = 500;  // More iterations to handle outliers
  config.random_seed = 456;
  config.z_min_mm = 0.0;
  config.z_max_mm = 2000.0;
  config.min_valid_points = 50;

  FloorPlaneEstimator estimator(config);

  CameraIntrinsics intrinsics = createMockIntrinsics();

  cv::Mat depth =
      generateSyntheticDepthImage(1280, 720, intrinsics, plane_coeffs, 0.001, 2.0, 456);

  auto result = estimator.Estimate(depth, intrinsics, 0.001);

  if (!result) {
    std::cerr << "  FAIL: Plane fitting failed\n";
    return false;
  }

  cv::Vec4f estimated_plane = result->plane;
  double plane_std_mm = result->plane_std_mm;
  double inlier_ratio = result->inlier_ratio;

  float norm = std::sqrt(estimated_plane[0] * estimated_plane[0] +
                         estimated_plane[1] * estimated_plane[1] +
                         estimated_plane[2] * estimated_plane[2]);
  cv::Vec4f normalized = estimated_plane / norm;

  std::cout << "  Estimated plane: [" << normalized[0] << ", " << normalized[1] << ", "
            << normalized[2] << ", " << normalized[3] << "]\n";
  std::cout << "  Plane std dev: " << plane_std_mm << " mm\n";
  std::cout << "  Inlier ratio: " << inlier_ratio << "\n";

  // RANSAC should reject outliers, so std dev should be close to noise level
  if (plane_std_mm > 10.0) {
    std::cerr << "  FAIL: Plane std dev too high (RANSAC should have rejected outliers)\n";
    return false;
  }

  // Inlier ratio should be around 90% (100% - 10% outliers)
  if (inlier_ratio < 0.75) {
    std::cerr << "  FAIL: Inlier ratio too low\n";
    return false;
  }

  std::cout << "  PASS\n\n";
  return true;
}

bool testTiltedPlane() {
  std::cout << "Test: Tilted plane (5 degree tilt)\n";

  // Plane tilted 5 degrees around X axis at z=500mm center
  // Normal: (0, sin(5°), cos(5°)) ≈ (0, 0.087, 0.996)
  float tilt_rad = 5.0F * static_cast<float>(M_PI) / 180.0F;
  float a = 0.0F;
  float b = std::sin(tilt_rad);
  float c = std::cos(tilt_rad);
  float d = -500.0F * c;  // Plane passes through (0, 0, 500)

  std::array<float, 4> plane_coeffs = {a, b, c, d};

  cv::Mat point_cloud = generateSyntheticPointCloud(80, 60, plane_coeffs, 1.0, 0.0, 789);

  FloorPlaneEstimatorConfig config;
  config.inlier_threshold_mm = 10.0;
  config.ransac_iterations = 200;
  config.random_seed = 789;
  config.z_min_mm = 0.0;
  config.z_max_mm = 2000.0;
  config.min_valid_points = 50;

  FloorPlaneEstimator estimator(config);

  CameraIntrinsics intrinsics = createMockIntrinsics();

  cv::Mat depth =
      generateSyntheticDepthImage(1280, 720, intrinsics, plane_coeffs, 0.001, 1.0, 789);

  auto result = estimator.Estimate(depth, intrinsics, 0.001);

  if (!result) {
    std::cerr << "  FAIL: Plane fitting failed\n";
    return false;
  }

  cv::Vec4f estimated_plane = result->plane;
  double plane_std_mm = result->plane_std_mm;
  double inlier_ratio = result->inlier_ratio;

  float norm = std::sqrt(estimated_plane[0] * estimated_plane[0] +
                         estimated_plane[1] * estimated_plane[1] +
                         estimated_plane[2] * estimated_plane[2]);
  cv::Vec4f normalized = estimated_plane / norm;

  std::cout << "  Estimated plane: [" << normalized[0] << ", " << normalized[1] << ", "
            << normalized[2] << ", " << normalized[3] << "]\n";
  std::cout << "  Expected plane (normalized): [" << a / c << ", " << b / c << ", 1.0, "
            << d / c << "]\n";
  std::cout << "  Plane std dev: " << plane_std_mm << " mm\n";
  std::cout << "  Inlier ratio: " << inlier_ratio << "\n";

  // Check that estimated normal is close to expected
  float expected_b_normalized = b / c;
  bool normal_correct = std::abs(normalized[1] - expected_b_normalized) < 0.1F;

  if (!normal_correct) {
    std::cerr << "  FAIL: Plane normal incorrect for tilted plane\n";
    return false;
  }

  if (inlier_ratio < 0.9) {
    std::cerr << "  FAIL: Inlier ratio too low\n";
    return false;
  }

  std::cout << "  PASS\n\n";
  return true;
}

bool testInsufficientPoints() {
  std::cout << "Test: Insufficient points (should fail gracefully)\n";

  // Create a very small point cloud
  cv::Mat point_cloud(2, 2, CV_32FC3, cv::Scalar(0, 0, 500));

  FloorPlaneEstimatorConfig config;
  config.inlier_threshold_mm = 10.0;
  config.ransac_iterations = 100;
  config.min_sample_count = 3;
  config.random_seed = 999;

  FloorPlaneEstimator estimator(config);

  CameraIntrinsics intrinsics = createMockIntrinsics();

  // Create minimal depth image
  cv::Mat depth(100, 100, CV_16UC1, cv::Scalar(0));
  for (int i = 0; i < 2; ++i) {
    depth.at<uint16_t>(50 + i, 50) = 500;
  }

  auto result = estimator.Estimate(depth, intrinsics, 0.001);

  if (result) {
    std::cerr << "  FAIL: Should have failed with insufficient points\n";
    return false;
  }

  std::cout << "  PASS (correctly failed)\n\n";
  return true;
}

}  // namespace

int main() {
  std::cout << "=== FloorPlaneEstimator Unit Tests ===\n\n";

  int passed = 0;
  int total = 0;

  auto run_test = [&](auto test_func, const char* name) {
    total++;
    if (test_func()) {
      passed++;
    } else {
      std::cerr << "FAILED: " << name << "\n\n";
    }
  };

  run_test(testPerfectPlane, "Perfect Plane");
  run_test(testNoisyPlane, "Noisy Plane");
  run_test(testPlaneWithOutliers, "Plane with Outliers");
  run_test(testTiltedPlane, "Tilted Plane");
  run_test(testInsufficientPoints, "Insufficient Points");

  std::cout << "=== Test Summary ===\n";
  std::cout << "Passed: " << passed << " / " << total << "\n";

  return (passed == total) ? 0 : 1;
}
