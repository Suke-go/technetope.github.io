#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "locomotion/calibration/CameraIntrinsics.h"

namespace locomotion::calibration::test {

/**
 * @brief Generate a synthetic planar point cloud for testing floor plane estimation
 *
 * @param width Width of the point cloud grid
 * @param height Height of the point cloud grid
 * @param plane_coeffs Plane equation coefficients [a, b, c, d] for ax+by+cz+d=0
 * @param noise_std_mm Standard deviation of Gaussian noise in mm
 * @param outlier_ratio Fraction of points to make outliers (0.0 to 1.0)
 * @param random_seed Random seed for reproducibility
 * @return cv::Mat Point cloud as CV_32FC3 (x, y, z)
 */
inline cv::Mat generateSyntheticPointCloud(int width, int height,
                                           const std::array<float, 4>& plane_coeffs,
                                           double noise_std_mm = 0.0,
                                           double outlier_ratio = 0.0,
                                           uint64_t random_seed = 42) {
  cv::Mat point_cloud(height, width, CV_32FC3, cv::Scalar(0, 0, 0));

  std::mt19937 rng(random_seed);
  std::normal_distribution<float> noise_dist(0.0F, static_cast<float>(noise_std_mm));
  std::uniform_real_distribution<float> outlier_dist(-500.0F, 500.0F);
  std::uniform_real_distribution<float> outlier_flag_dist(0.0F, 1.0F);

  float a = plane_coeffs[0];
  float b = plane_coeffs[1];
  float c = plane_coeffs[2];
  float d = plane_coeffs[3];

  // Ensure c != 0 for solving z
  if (std::abs(c) < 1e-6F) {
    throw std::invalid_argument("Plane coefficient c must be non-zero for z calculation");
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      float x_world = static_cast<float>(x) * 10.0F;  // 10mm grid spacing
      float y_world = static_cast<float>(y) * 10.0F;

      // Solve for z: z = -(ax + by + d) / c
      float z_plane = -(a * x_world + b * y_world + d) / c;

      bool is_outlier = (outlier_flag_dist(rng) < static_cast<float>(outlier_ratio));

      float z_final;
      if (is_outlier) {
        z_final = z_plane + outlier_dist(rng);
      } else {
        z_final = z_plane + noise_dist(rng);
      }

      point_cloud.at<cv::Vec3f>(y, x) = cv::Vec3f(x_world, y_world, z_final);
    }
  }

  return point_cloud;
}

/**
 * @brief Generate a synthetic depth image from a plane equation
 *
 * @param width Image width
 * @param height Image height
 * @param intrinsics Camera intrinsics
 * @param plane_coeffs Plane equation in camera frame [a, b, c, d]
 * @param depth_scale_m Depth scale factor (e.g., 0.001 for mm)
 * @param noise_std_mm Standard deviation of depth noise in mm
 * @param random_seed Random seed
 * @return cv::Mat Depth image as CV_16UC1 (uint16)
 */
inline cv::Mat generateSyntheticDepthImage(int width, int height,
                                           const CameraIntrinsics& intrinsics,
                                           const std::array<float, 4>& plane_coeffs,
                                           double depth_scale_m = 0.001,
                                           double noise_std_mm = 0.0,
                                           uint64_t random_seed = 42) {
  cv::Mat depth(height, width, CV_16UC1, cv::Scalar(0));

  std::mt19937 rng(random_seed);
  std::normal_distribution<float> noise_dist(0.0F, static_cast<float>(noise_std_mm));

  float a = plane_coeffs[0];
  float b = plane_coeffs[1];
  float c = plane_coeffs[2];
  float d = plane_coeffs[3];

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      // Backproject pixel to normalized ray
      float x_norm = (static_cast<float>(u) - static_cast<float>(intrinsics.cx)) /
                     static_cast<float>(intrinsics.fx);
      float y_norm = (static_cast<float>(v) - static_cast<float>(intrinsics.cy)) /
                     static_cast<float>(intrinsics.fy);

      // Ray direction: (x_norm, y_norm, 1) * depth = (x, y, z)
      // Plane equation: a*x + b*y + c*z + d = 0
      // Substitute: a*(x_norm*z) + b*(y_norm*z) + c*z + d = 0
      // Solve for z: z = -d / (a*x_norm + b*y_norm + c)

      float denominator = a * x_norm + b * y_norm + c;
      if (std::abs(denominator) < 1e-6F) {
        continue;  // Ray parallel to plane
      }

      float z_mm = -d / denominator;

      if (z_mm <= 0.0F) {
        continue;  // Behind camera
      }

      // Add noise
      z_mm += noise_dist(rng);

      // Convert to depth scale
      uint16_t depth_value = static_cast<uint16_t>(z_mm / (depth_scale_m * 1000.0));
      depth.at<uint16_t>(v, u) = depth_value;
    }
  }

  return depth;
}

/**
 * @brief Generate a synthetic ChArUco board image
 *
 * @param width Image width
 * @param height Image height
 * @param squares_x Number of chessboard columns
 * @param squares_y Number of chessboard rows
 * @param square_size Square size in pixels
 * @return cv::Mat Grayscale image with ChArUco board
 */
inline cv::Mat generateSyntheticCharucoImage(int width, int height, int squares_x, int squares_y,
                                             int square_size) {
  cv::Mat image(height, width, CV_8UC1, cv::Scalar(255));

  int board_width = squares_x * square_size;
  int board_height = squares_y * square_size;

  int offset_x = (width - board_width) / 2;
  int offset_y = (height - board_height) / 2;

  // Draw checkerboard pattern
  for (int y = 0; y < squares_y; ++y) {
    for (int x = 0; x < squares_x; ++x) {
      if ((x + y) % 2 == 0) {
        cv::Rect square(offset_x + x * square_size, offset_y + y * square_size, square_size,
                        square_size);
        cv::rectangle(image, square, cv::Scalar(0), -1);
      }
    }
  }

  return image;
}

/**
 * @brief Create mock CameraIntrinsics for testing
 */
inline CameraIntrinsics createMockIntrinsics() {
  CameraIntrinsics intrinsics;
  intrinsics.fx = 630.0;
  intrinsics.fy = 630.0;
  intrinsics.cx = 640.0;
  intrinsics.cy = 360.0;
  intrinsics.distortion_model = "brown_conrady";
  intrinsics.distortion_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0};
  return intrinsics;
}

/**
 * @brief Calculate plane standard deviation from point cloud
 */
inline double calculatePlaneStdDev(const cv::Mat& point_cloud, const cv::Vec4f& plane) {
  if (point_cloud.empty() || point_cloud.type() != CV_32FC3) {
    return std::numeric_limits<double>::infinity();
  }

  std::vector<double> distances;
  distances.reserve(static_cast<size_t>(point_cloud.rows * point_cloud.cols));

  for (int y = 0; y < point_cloud.rows; ++y) {
    for (int x = 0; x < point_cloud.cols; ++x) {
      cv::Vec3f pt = point_cloud.at<cv::Vec3f>(y, x);
      if (pt[2] == 0.0F) continue;  // Skip invalid points

      double dist = std::abs(plane[0] * pt[0] + plane[1] * pt[1] + plane[2] * pt[2] + plane[3]) /
                    std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
      distances.push_back(dist);
    }
  }

  if (distances.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double mean = 0.0;
  for (double d : distances) {
    mean += d;
  }
  mean /= static_cast<double>(distances.size());

  double variance = 0.0;
  for (double d : distances) {
    double diff = d - mean;
    variance += diff * diff;
  }
  variance /= static_cast<double>(distances.size());

  return std::sqrt(variance);
}

}  // namespace locomotion::calibration::test
