#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace locomotion::calibration {

struct LayoutPoint {
  double x{0.0};
  double y{0.0};
};

struct PlaymatExtent {
  LayoutPoint min;
  LayoutPoint max;
};

struct BoardMount {
  std::string playmat_id;
  std::string board_id;
  std::string label;
  cv::Matx33d affine_mm_to_position;
  std::vector<cv::Point2d> board_points_mm;
  std::vector<cv::Point2d> position_id_points;
};

struct PlaymatInfo {
  std::string id;
  std::string label;
  PlaymatExtent extent;
  LayoutPoint id_per_mm;
};

class PlaymatLayout {
 public:
  static PlaymatLayout LoadFromFile(const std::string& path);
  static PlaymatLayout LoadFromFile(const std::string& path, const std::string& base_dir);

  const PlaymatInfo* GetPlaymat(const std::string& playmat_id) const;
  const BoardMount* GetBoardMount(const std::string& mount_label) const;

  cv::Point2f TransformBoardPoint(const std::string& mount_label,
                                  const cv::Point3f& board_point_mm) const;

 private:
  std::vector<PlaymatInfo> playmats_;
  std::vector<BoardMount> mounts_;

  std::unordered_map<std::string, size_t> playmat_index_;
  std::unordered_map<std::string, size_t> mount_index_;
};

}  // namespace locomotion::calibration
