#include <iostream>
#include <string>

#include <opencv2/core.hpp>

#include "locomotion/calibration/CalibrationPipeline.h"
#include "locomotion/calibration/CameraIntrinsics.h"
#include "locomotion/calibration/CharucoDetector.h"
#include "locomotion/calibration/FloorPlaneEstimator.h"

using namespace locomotion::calibration;

namespace {

bool testCameraIntrinsicsStruct() {
  std::cout << "Test: CameraIntrinsics structure\n";

  CameraIntrinsics intrinsics;
  intrinsics.fx = 630.0;
  intrinsics.fy = 630.0;
  intrinsics.cx = 640.0;
  intrinsics.cy = 360.0;
  intrinsics.distortion_model = "brown_conrady";
  intrinsics.distortion_coeffs = {0.1, -0.05, 0.001, 0.002, 0.0};

  if (intrinsics.fx != 630.0 || intrinsics.fy != 630.0) {
    std::cerr << "  FAIL: Focal length incorrect\n";
    return false;
  }

  if (intrinsics.distortion_model != "brown_conrady") {
    std::cerr << "  FAIL: Distortion model incorrect\n";
    return false;
  }

  if (intrinsics.distortion_coeffs.size() != 5) {
    std::cerr << "  FAIL: Distortion coefficients size incorrect\n";
    return false;
  }

  std::cout << "  Intrinsics: fx=" << intrinsics.fx << ", fy=" << intrinsics.fy
            << ", cx=" << intrinsics.cx << ", cy=" << intrinsics.cy << "\n";
  std::cout << "  Distortion model: " << intrinsics.distortion_model << "\n";
  std::cout << "  PASS\n\n";
  return true;
}

bool testCalibrationConfigDefaults() {
  std::cout << "Test: CalibrationConfig default values\n";

  CalibrationConfig config;

  if (config.color_width != 640 || config.color_height != 480) {
    std::cerr << "  FAIL: Color resolution incorrect\n";
    return false;
  }

  if (config.charuco_squares_x != 5 || config.charuco_squares_y != 7) {
    std::cerr << "  FAIL: ChArUco board dimensions incorrect\n";
    return false;
  }

  if (config.floor_inlier_threshold_mm != 8.0) {
    std::cerr << "  FAIL: Floor inlier threshold incorrect\n";
    return false;
  }

  if (config.max_plane_std_mm != 8.0) {
    std::cerr << "  FAIL: Max plane std dev incorrect\n";
    return false;
  }

  if (config.random_seed != 42) {
    std::cerr << "  FAIL: Random seed incorrect\n";
    return false;
  }

  std::cout << "  Color: " << config.color_width << "x" << config.color_height << "\n";
  std::cout << "  ChArUco: " << config.charuco_squares_x << "x" << config.charuco_squares_y
            << "\n";
  std::cout << "  Floor threshold: " << config.floor_inlier_threshold_mm << " mm\n";
  std::cout << "  Random seed: " << config.random_seed << "\n";
  std::cout << "  PASS\n\n";
  return true;
}

bool testCharucoDetectorConfig() {
  std::cout << "Test: CharucoDetector configuration\n";

  CharucoDetectorConfig config;
  config.min_corners = 12;
  config.enable_subpixel_refine = true;
  config.subpixel_window = cv::Size(5, 5);
  config.subpixel_max_iterations = 30;
  config.subpixel_epsilon = 0.1;

  if (config.min_corners != 12) {
    std::cerr << "  FAIL: Min corners incorrect\n";
    return false;
  }

  if (!config.enable_subpixel_refine) {
    std::cerr << "  FAIL: Subpixel refinement should be enabled\n";
    return false;
  }

  std::cout << "  Min corners: " << config.min_corners << "\n";
  std::cout << "  Subpixel refinement: " << (config.enable_subpixel_refine ? "enabled" : "disabled")
            << "\n";
  std::cout << "  PASS\n\n";
  return true;
}

bool testFloorPlaneEstimatorConfig() {
  std::cout << "Test: FloorPlaneEstimator configuration\n";

  FloorPlaneEstimatorConfig config;
  config.inlier_threshold_mm = 8.0;
  config.ransac_iterations = 500;
  config.min_inlier_ratio = 0.7;
  config.z_min_mm = 2400.0;
  config.z_max_mm = 2800.0;
  config.downsample_grid = 4;
  config.random_seed = 42;

  if (config.inlier_threshold_mm != 8.0) {
    std::cerr << "  FAIL: Inlier threshold incorrect\n";
    return false;
  }

  if (config.ransac_iterations != 500) {
    std::cerr << "  FAIL: RANSAC iterations incorrect\n";
    return false;
  }

  if (config.z_min_mm != 2400.0 || config.z_max_mm != 2800.0) {
    std::cerr << "  FAIL: Z range incorrect\n";
    return false;
  }

  std::cout << "  Inlier threshold: " << config.inlier_threshold_mm << " mm\n";
  std::cout << "  RANSAC iterations: " << config.ransac_iterations << "\n";
  std::cout << "  Z range: [" << config.z_min_mm << ", " << config.z_max_mm << "] mm\n";
  std::cout << "  Downsample grid: " << config.downsample_grid << "\n";
  std::cout << "  PASS\n\n";
  return true;
}

bool testCalibrationSnapshotStructure() {
  std::cout << "Test: CalibrationSnapshot structure\n";

  CalibrationSnapshot snapshot;

  // Set intrinsics
  snapshot.intrinsics.fx = 630.0;
  snapshot.intrinsics.fy = 630.0;
  snapshot.intrinsics.cx = 640.0;
  snapshot.intrinsics.cy = 360.0;

  // Set floor plane
  snapshot.floor_plane = cv::Vec4f(0.0F, 0.0F, 1.0F, -500.0F);
  snapshot.homography_color_to_floor = cv::Mat::eye(3, 3, CV_64F);
  snapshot.homography_color_to_toio = cv::Mat::eye(3, 3, CV_64F);
  snapshot.toio_transform.color_to_toio = cv::Mat::eye(3, 3, CV_64F);
  snapshot.toio_transform.playmat_id = "a3_simple";
  snapshot.toio_transform.mount_label = "center_mount_nominal";

  // Set metrics
  snapshot.reprojection_error_floor_mm = 4.1;
  snapshot.reprojection_error_toio = 5.2;
  snapshot.floor_plane_std_mm = 6.8;
  snapshot.inlier_ratio = 0.92;
  snapshot.floor_inlier_count = 200;
  snapshot.camera_height_mm = 2587.0;
  snapshot.detected_charuco_corners = 20;

  if (snapshot.intrinsics.fx != 630.0) {
    std::cerr << "  FAIL: Intrinsics not stored correctly\n";
    return false;
  }

  if (snapshot.floor_plane[2] != 1.0F) {
    std::cerr << "  FAIL: Floor plane not stored correctly\n";
    return false;
  }

  if (snapshot.reprojection_error_toio != 5.2) {
    std::cerr << "  FAIL: Reprojection error not stored correctly\n";
    return false;
  }

  std::cout << "  Intrinsics: fx=" << snapshot.intrinsics.fx << ", fy=" << snapshot.intrinsics.fy
            << "\n";
  std::cout << "  Floor plane: [" << snapshot.floor_plane[0] << ", " << snapshot.floor_plane[1]
            << ", " << snapshot.floor_plane[2] << ", " << snapshot.floor_plane[3] << "]\n";
  std::cout << "  Reprojection error (ID): " << snapshot.reprojection_error_toio << "\n";
  std::cout << "  Floor reprojection error: " << snapshot.reprojection_error_floor_mm << " mm\n";
  std::cout << "  Floor plane std: " << snapshot.floor_plane_std_mm << " mm\n";
  std::cout << "  Inlier ratio: " << snapshot.inlier_ratio << "\n";
  std::cout << "  Detected corners: " << snapshot.detected_charuco_corners << "\n";
  std::cout << "  PASS\n\n";
  return true;
}

}  // namespace

int main() {
  std::cout << "=== Calibration Pipeline Integration Tests ===\n";
  std::cout << "Note: These tests verify data structures and configuration.\n";
  std::cout << "Real device tests require RealSense hardware.\n\n";

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

  run_test(testCameraIntrinsicsStruct, "CameraIntrinsics");
  run_test(testCalibrationConfigDefaults, "CalibrationConfig Defaults");
  run_test(testCharucoDetectorConfig, "CharucoDetector Config");
  run_test(testFloorPlaneEstimatorConfig, "FloorPlaneEstimator Config");
  run_test(testCalibrationSnapshotStructure, "CalibrationSnapshot Structure");

  std::cout << "=== Test Summary ===\n";
  std::cout << "Passed: " << passed << " / " << total << "\n";

  if (passed == total) {
    std::cout << "\n✅ All integration tests passed!\n";
    std::cout << "To test with real hardware, run:\n";
    std::cout << "  sudo killall VDCAssistant AppleCameraAssistant 2>/dev/null || true\n";
    std::cout << "  sudo ./capture_calibration\n";
  }

  return (passed == total) ? 0 : 1;
}
