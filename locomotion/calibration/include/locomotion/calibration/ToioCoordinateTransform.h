#pragma once

#include <string>

#include <opencv2/core.hpp>

namespace locomotion::calibration {

struct ToioCoordinateTransform {
  cv::Mat color_to_toio;          // Combined 3x3 homography (image → toio ID)
  cv::Matx33d board_to_toio;      // Affine transform (board mm → toio ID)
  cv::Rect2d coverage_area;       // Playmat coverage in toio ID units
  double transform_error_id{0.0}; // RMS error at correspondence points
  std::string playmat_id;
  std::string mount_label;
};

// Transform image pixel coordinates to toio Position ID coordinates
cv::Point2f TransformImagePixelToToio(cv::Point2f image_pixel,
                                       const cv::Mat& homography_color_to_toio);

// Transform floor position (mm) to toio Position ID coordinates
cv::Point2f TransformFloorToToio(cv::Point2f floor_position_mm,
                                  const ToioCoordinateTransform& transform);

// Transform toio Position ID coordinates to image pixel coordinates
cv::Point2f TransformToioToImagePixel(cv::Point2f toio_position,
                                       const cv::Mat& homography_toio_to_color);

// Check if a toio position is within the coverage area
bool IsToioPositionInCoverage(cv::Point2f toio_position,
                               const ToioCoordinateTransform& transform);

}  // namespace locomotion::calibration

