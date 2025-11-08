#include "locomotion/calibration/PlaymatLayout.h"

#include <spdlog/spdlog.h>

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace locomotion::calibration {

namespace {

cv::Matx33d computeAffine(const std::vector<cv::Point2d>& source_mm,
                          const std::vector<cv::Point2d>& target_id) {
  if (source_mm.size() < 3 || target_id.size() < 3 || source_mm.size() != target_id.size()) {
    throw std::runtime_error("Need at least three correspondence points to compute affine transform.");
  }

  cv::Mat A(2 * static_cast<int>(source_mm.size()), 6, CV_64F);
  cv::Mat b(2 * static_cast<int>(source_mm.size()), 1, CV_64F);

  for (size_t i = 0; i < source_mm.size(); ++i) {
    double x = source_mm[i].x;
    double y = source_mm[i].y;
    double tx = target_id[i].x;
    double ty = target_id[i].y;

    int row = static_cast<int>(2 * i);
    A.at<double>(row, 0) = x;
    A.at<double>(row, 1) = y;
    A.at<double>(row, 2) = 1.0;
    A.at<double>(row, 3) = 0.0;
    A.at<double>(row, 4) = 0.0;
    A.at<double>(row, 5) = 0.0;
    b.at<double>(row, 0) = tx;

    A.at<double>(row + 1, 0) = 0.0;
    A.at<double>(row + 1, 1) = 0.0;
    A.at<double>(row + 1, 2) = 0.0;
    A.at<double>(row + 1, 3) = x;
    A.at<double>(row + 1, 4) = y;
    A.at<double>(row + 1, 5) = 1.0;
    b.at<double>(row + 1, 0) = ty;
  }

  cv::Mat solution;
  cv::solve(A, b, solution, cv::DECOMP_SVD);

  return cv::Matx33d(solution.at<double>(0), solution.at<double>(1), solution.at<double>(2),
                     solution.at<double>(3), solution.at<double>(4), solution.at<double>(5), 0.0, 0.0,
                     1.0);
}

std::string resolvePath(const std::string& path, const std::string& base_dir) {
  namespace fs = std::filesystem;
  
  // If path is already absolute, return as-is
  fs::path path_obj(path);
  if (path_obj.is_absolute()) {
    return path;
  }
  
  // If base_dir is provided and not empty, resolve relative to base_dir
  if (!base_dir.empty()) {
    fs::path base_path(base_dir);
    
    // If base_path is a file, get its parent directory
    if (fs::exists(base_path) && fs::is_regular_file(base_path)) {
      base_path = base_path.parent_path();
    }
    
    // If base_path is a directory or absolute, try to resolve
    if (base_path.is_absolute() || fs::exists(base_path)) {
      fs::path resolved = base_path / path;
      resolved = resolved.lexically_normal();
      if (fs::exists(resolved)) {
        return resolved.string();
      }
    }
    
    // Also try as directory path directly
    if (fs::is_directory(base_path) || !fs::exists(base_path)) {
      fs::path resolved = base_path / path;
      resolved = resolved.lexically_normal();
      if (fs::exists(resolved)) {
        return resolved.string();
      }
    }
  }
  
  // Fallback: try current working directory
  if (fs::exists(path_obj)) {
    return fs::absolute(path_obj).string();
  }
  
  // Return original path if nothing works (will fail later with better error message)
  return path;
}

}  // namespace

PlaymatLayout PlaymatLayout::LoadFromFile(const std::string& path) {
  return LoadFromFile(path, "");
}

PlaymatLayout PlaymatLayout::LoadFromFile(const std::string& path, const std::string& base_dir) {
  std::string resolved_path = resolvePath(path, base_dir);
  std::ifstream ifs(resolved_path);
  if (!ifs.good()) {
    throw std::runtime_error("Failed to open playmat layout file: " + resolved_path + 
                             (base_dir.empty() ? "" : " (resolved from base: " + base_dir + ")"));
  }

  nlohmann::json j;
  ifs >> j;

  PlaymatLayout layout;

  if (j.contains("playmats")) {
    for (const auto& node : j["playmats"]) {
      PlaymatInfo info;
      info.id = node.value("id", "");
      info.label = node.value("label", "");
      info.extent.min.x = node["position_id_extent"]["min"].value("x", 0.0);
      info.extent.min.y = node["position_id_extent"]["min"].value("y", 0.0);
      info.extent.max.x = node["position_id_extent"]["max"].value("x", 0.0);
      info.extent.max.y = node["position_id_extent"]["max"].value("y", 0.0);
      info.id_per_mm.x = node["id_per_mm"].value("x", 0.0);
      info.id_per_mm.y = node["id_per_mm"].value("y", 0.0);

      layout.playmat_index_[info.id] = layout.playmats_.size();
      layout.playmats_.push_back(info);
    }
  }

  if (j.contains("charuco_mounts")) {
    for (const auto& node : j["charuco_mounts"]) {
      std::string mount_label = node.value("label", "");
      std::string playmat_id = node.value("playmat_id", "");
      std::string board_id = node.value("board_id", "");

      const auto* playmat = layout.GetPlaymat(playmat_id);
      if (!playmat) {
        spdlog::warn("Mount '{}' references unknown playmat '{}'", mount_label, playmat_id);
        continue;
      }

      std::vector<cv::Point2d> source_mm;
      std::vector<cv::Point2d> target_id;
      if (node.contains("board_to_position_id") &&
          node["board_to_position_id"].contains("correspondences")) {
        for (const auto& corr : node["board_to_position_id"]["correspondences"]) {
          double bx = corr["board_point_mm"].value("x", 0.0);
          double by = corr["board_point_mm"].value("y", 0.0);
          double px = corr["position_id"].value("x", 0.0);
          double py = corr["position_id"].value("y", 0.0);
          source_mm.emplace_back(bx, by);
          target_id.emplace_back(px, py);
        }
      }

      if (source_mm.size() < 3) {
        spdlog::warn("Mount '{}' does not have enough correspondences ({}). Skipping.", mount_label,
                     source_mm.size());
        continue;
      }

      BoardMount mount;
      mount.playmat_id = playmat_id;
      mount.board_id = board_id;
      mount.label = mount_label;
      mount.affine_mm_to_position = computeAffine(source_mm, target_id);
      mount.board_points_mm = source_mm;
      mount.position_id_points = target_id;

      layout.mount_index_[mount.label] = layout.mounts_.size();
      layout.mounts_.push_back(mount);
    }
  }

  return layout;
}

const PlaymatInfo* PlaymatLayout::GetPlaymat(const std::string& playmat_id) const {
  auto it = playmat_index_.find(playmat_id);
  if (it == playmat_index_.end()) {
    return nullptr;
  }
  return &playmats_[it->second];
}

const BoardMount* PlaymatLayout::GetBoardMount(const std::string& mount_label) const {
  auto it = mount_index_.find(mount_label);
  if (it == mount_index_.end()) {
    return nullptr;
  }
  return &mounts_[it->second];
}

cv::Point2f PlaymatLayout::TransformBoardPoint(const std::string& mount_label,
                                               const cv::Point3f& board_point_mm) const {
  const auto* mount = GetBoardMount(mount_label);
  if (!mount) {
    spdlog::warn("Board mount '{}' not found. Returning zero.", mount_label);
    return {};
  }
  const cv::Matx33d& T = mount->affine_mm_to_position;
  cv::Vec3d input(board_point_mm.x, board_point_mm.y, 1.0);
  cv::Vec3d result = T * input;
  return cv::Point2f(static_cast<float>(result[0]), static_cast<float>(result[1]));
}

}  // namespace locomotion::calibration
