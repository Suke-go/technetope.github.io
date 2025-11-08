#include "locomotion/calibration/ToioCoordinateTransform.h"

#include <cmath>

namespace locomotion::calibration {

cv::Point2f TransformImagePixelToToio(cv::Point2f image_pixel,
                                       const cv::Mat& homography_color_to_toio) {
  if (homography_color_to_toio.empty() || homography_color_to_toio.rows != 3 ||
      homography_color_to_toio.cols != 3) {
    return cv::Point2f(0.0f, 0.0f);
  }

  cv::Mat point_homogeneous = (cv::Mat_<double>(3, 1) << image_pixel.x, image_pixel.y, 1.0);
  cv::Mat result_homogeneous = homography_color_to_toio * point_homogeneous;

  double w = result_homogeneous.at<double>(2, 0);
  if (std::abs(w) < 1e-6) {
    return cv::Point2f(0.0f, 0.0f);
  }

  return cv::Point2f(static_cast<float>(result_homogeneous.at<double>(0, 0) / w),
                      static_cast<float>(result_homogeneous.at<double>(1, 0) / w));
}

cv::Point2f TransformFloorToToio(cv::Point2f floor_position_mm,
                                  const ToioCoordinateTransform& transform) {
  // This function would transform floor coordinates to toio coordinates
  // Currently, the homography_color_to_toio already includes this transformation
  // This is a placeholder for future use if we need to transform floor coordinates directly
  (void)floor_position_mm;
  (void)transform;
  return cv::Point2f(0.0f, 0.0f);
}

cv::Point2f TransformToioToImagePixel(cv::Point2f toio_position,
                                       const cv::Mat& homography_toio_to_color) {
  if (homography_toio_to_color.empty() || homography_toio_to_color.rows != 3 ||
      homography_toio_to_color.cols != 3) {
    return cv::Point2f(0.0f, 0.0f);
  }

  cv::Mat point_homogeneous = (cv::Mat_<double>(3, 1) << toio_position.x, toio_position.y, 1.0);
  cv::Mat result_homogeneous = homography_toio_to_color * point_homogeneous;

  double w = result_homogeneous.at<double>(2, 0);
  if (std::abs(w) < 1e-6) {
    return cv::Point2f(0.0f, 0.0f);
  }

  return cv::Point2f(static_cast<float>(result_homogeneous.at<double>(0, 0) / w),
                      static_cast<float>(result_homogeneous.at<double>(1, 0) / w));
}

bool IsToioPositionInCoverage(cv::Point2f toio_position,
                               const ToioCoordinateTransform& transform) {
  if (transform.coverage_area.width <= 0.0 || transform.coverage_area.height <= 0.0) {
    return false;
  }

  return (toio_position.x >= transform.coverage_area.x &&
          toio_position.x <= transform.coverage_area.x + transform.coverage_area.width &&
          toio_position.y >= transform.coverage_area.y &&
          toio_position.y <= transform.coverage_area.y + transform.coverage_area.height);
}

}  // namespace locomotion::calibration

