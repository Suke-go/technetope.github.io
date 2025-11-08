#include "locomotion/calibration/CalibrationSession.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>

#include <nlohmann/json.hpp>

namespace locomotion::calibration {

namespace {

std::string formatTimestamp(const std::chrono::system_clock::time_point& tp) {
  std::time_t tt = std::chrono::system_clock::to_time_t(tp);
  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &tt);
#else
  gmtime_r(&tt, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

}  // namespace

CalibrationSession::CalibrationSession(CalibrationPipeline pipeline,
                                       SessionConfig session_config)
    : pipeline_(std::move(pipeline)), session_config_(std::move(session_config)) {
  if (session_config_.attempts <= 0) {
    session_config_.attempts = pipeline_.config().session_attempts;
  }
  if (session_config_.max_plane_std_mm <= 0.0) {
    session_config_.max_plane_std_mm = pipeline_.config().max_plane_std_mm;
  }
  if (session_config_.min_inlier_ratio <= 0.0) {
    session_config_.min_inlier_ratio = pipeline_.config().floor_min_inlier_ratio;
  }
}

std::optional<CalibrationResult> CalibrationSession::Run() {
  // Note: pipeline should already be initialized before Run() is called
  // This check is kept for backward compatibility but initialize() should be called
  // with config file directory from the tool that creates the session
  if (!pipeline_.initialize()) {
    spdlog::error("Failed to initialize CalibrationPipeline.");
    return std::nullopt;
  }

  std::optional<CalibrationResult> best;
  int successes = 0;
  std::chrono::system_clock::time_point first_success_ts{};
  std::chrono::system_clock::time_point last_success_ts{};
  bool have_success_ts = false;

  for (int attempt = 0; attempt < session_config_.attempts; ++attempt) {
    auto snapshot = pipeline_.runOnce();
    if (!snapshot) {
      spdlog::info("Attempt {}: ChArUco detection failed.", attempt + 1);
      continue;
    }

    if (snapshot->reprojection_error_toio > pipeline_.config().max_reprojection_error_id) {
      spdlog::warn("Attempt {}: reprojection error {:.3f} exceeds threshold {:.3f}.",
                   attempt + 1, snapshot->reprojection_error_toio,
                   pipeline_.config().max_reprojection_error_id);
      continue;
    }

    if (pipeline_.config().enable_floor_plane_fit &&
        snapshot->floor_plane_std_mm > session_config_.max_plane_std_mm) {
      spdlog::warn("Attempt {}: plane std {:.3f} exceeds threshold {:.3f}.", attempt + 1,
                   snapshot->floor_plane_std_mm, session_config_.max_plane_std_mm);
      continue;
    }

    if (pipeline_.config().enable_floor_plane_fit &&
        snapshot->inlier_ratio < session_config_.min_inlier_ratio) {
      spdlog::warn("Attempt {}: inlier ratio {:.3f} below minimum {:.3f}.", attempt + 1,
                   snapshot->inlier_ratio, session_config_.min_inlier_ratio);
      continue;
    }

    // Optional: Validate calibration quality
    CalibrationValidation validation = pipeline_.ValidateCalibration(*snapshot);
    if (!validation.warnings.empty()) {
      for (const auto& warning : validation.warnings) {
        spdlog::debug("Attempt {} validation warning: {}", attempt + 1, warning);
      }
    }

    CalibrationResult result = SnapshotToResult(*snapshot);
    ++successes;

    if (!have_success_ts) {
      first_success_ts = snapshot->timestamp;
      have_success_ts = true;
    }
    last_success_ts = snapshot->timestamp;

    if (session_config_.save_intermediate_snapshots) {
      CalibrationResult snapshot_result = result;
      snapshot_result.capture_count = successes;
      snapshot_result.first_capture_timestamp = snapshot->timestamp;
      snapshot_result.last_capture_timestamp = snapshot->timestamp;

      namespace fs = std::filesystem;
      fs::path dir = session_config_.snapshot_output_dir.empty()
                         ? fs::path("debug_snapshots")
                         : fs::path(session_config_.snapshot_output_dir);
      try {
        fs::create_directories(dir);
        std::ostringstream oss;
        oss << "capture_" << std::setw(2) << std::setfill('0') << successes << ".json";
        fs::path file_path = dir / oss.str();
        SaveResultJson(snapshot_result, file_path.string(), pipeline_.config(),
                       session_config_, pipeline_.camera_model(),
                       pipeline_.camera_serial(), pipeline_.depth_scale_m());
      } catch (const std::exception& ex) {
        spdlog::warn("Failed to save snapshot JSON: {}", ex.what());
      }
    }

    if (!best || result.timestamp > best->timestamp) {
      best = result;
    }
  }

  if (!best) {
    spdlog::error("CalibrationSession failed. No valid snapshots collected out of {} attempts.",
                  session_config_.attempts);
    return std::nullopt;
  }

  best->capture_count = successes;
  if (have_success_ts) {
    best->first_capture_timestamp = first_success_ts;
    best->last_capture_timestamp = last_success_ts;
  }

  spdlog::info("CalibrationSession succeeded with {} valid snapshots.", successes);
  return best;
}

CalibrationResult CalibrationSession::SnapshotToResult(const CalibrationSnapshot& snapshot) {
  CalibrationResult result;
  result.intrinsics = snapshot.intrinsics;
  result.homography_color_to_floor = snapshot.homography_color_to_floor.clone();
  result.homography_color_to_toio = snapshot.homography_color_to_toio.clone();
  result.toio_transform = snapshot.toio_transform;
  result.toio_transform.color_to_toio = snapshot.toio_transform.color_to_toio.clone();
  result.floor_plane = snapshot.floor_plane;
  result.reprojection_error_px = snapshot.reprojection_error_px;
  result.reprojection_error_floor_mm = snapshot.reprojection_error_floor_mm;
  result.reprojection_error_toio = snapshot.reprojection_error_toio;
  result.floor_plane_std_mm = snapshot.floor_plane_std_mm;
  result.inlier_ratio = snapshot.inlier_ratio;
  result.floor_inlier_count = snapshot.floor_inlier_count;
  result.camera_height_mm = snapshot.camera_height_mm;
  result.detected_charuco_corners = snapshot.detected_charuco_corners;
  result.timestamp = snapshot.timestamp;
  return result;
}

bool CalibrationSession::SaveResultJson(const CalibrationResult& result,
                                        const std::string& path) const {
  return SaveResultJson(result, path, pipeline_.config(), session_config_,
                        pipeline_.camera_model(), pipeline_.camera_serial(),
                        pipeline_.depth_scale_m());
}

bool CalibrationSession::SaveResultJson(const CalibrationResult& result,
                                        const std::string& path,
                                        const CalibrationConfig& pipeline_cfg,
                                        const SessionConfig& session_config,
                                        const std::string& camera_model,
                                        const std::string& camera_serial,
                                        double depth_scale_m) {
  auto matToJson = [](const cv::Mat& mat) {
    nlohmann::json rows = nlohmann::json::array();
    if (mat.empty()) {
      return rows;
    }
    cv::Mat mat64;
    mat.convertTo(mat64, CV_64F);
    for (int r = 0; r < mat64.rows; ++r) {
      nlohmann::json row = nlohmann::json::array();
      for (int c = 0; c < mat64.cols; ++c) {
        row.push_back(mat64.at<double>(r, c));
      }
      rows.push_back(row);
    }
    return rows;
  };

  auto camera_name = camera_model.empty() ? std::string("Intel RealSense D415") : camera_model;

  cv::Vec3d normal(result.floor_plane[0], result.floor_plane[1], result.floor_plane[2]);
  double normal_norm = cv::norm(normal);
  cv::Vec3d normal_unit =
      (normal_norm > 0.0) ? normal / normal_norm : cv::Vec3d(0.0, 0.0, 1.0);
  double distance_from_origin =
      (normal_norm > 0.0) ? std::abs(result.floor_plane[3]) / normal_norm : 0.0;

  nlohmann::json j;
  j["schema_version"] = "2.0";
  j["timestamp"] = formatTimestamp(result.timestamp);

  j["camera"] = {
      {"model", camera_name},
      {"serial_number", camera_serial},
      {"mount_height_mm", result.camera_height_mm},
      {"depth_scale_m", depth_scale_m},
      {"color_resolution",
       {{"width", pipeline_cfg.color_width}, {"height", pipeline_cfg.color_height}}},
      {"depth_resolution",
       {{"width", pipeline_cfg.depth_width}, {"height", pipeline_cfg.depth_height}}},
      {"fps", pipeline_cfg.fps}};

  nlohmann::json intrinsics_json;
  intrinsics_json["fx"] = result.intrinsics.fx;
  intrinsics_json["fy"] = result.intrinsics.fy;
  intrinsics_json["cx"] = result.intrinsics.cx;
  intrinsics_json["cy"] = result.intrinsics.cy;
  intrinsics_json["distortion_model"] = result.intrinsics.distortion_model;
  intrinsics_json["distortion_coeffs"] = result.intrinsics.distortion_coeffs;
  j["intrinsics"] = intrinsics_json;

  nlohmann::json floor_plane_json;
  floor_plane_json["coefficients"] = {result.floor_plane[0], result.floor_plane[1],
                                      result.floor_plane[2], result.floor_plane[3]};
  floor_plane_json["normal_vector"] = {normal_unit[0], normal_unit[1], normal_unit[2]};
  floor_plane_json["distance_from_origin_mm"] = distance_from_origin;
  floor_plane_json["std_mm"] = result.floor_plane_std_mm;
  floor_plane_json["inlier_ratio"] = result.inlier_ratio;
  floor_plane_json["inlier_count"] = result.floor_inlier_count;
  floor_plane_json["camera_height_mm"] = result.camera_height_mm;
  j["floor_plane"] = floor_plane_json;

  j["homography_color_to_floor"] = matToJson(result.homography_color_to_floor);

  nlohmann::json toio_json;
  toio_json["playmat_id"] = result.toio_transform.playmat_id;
  toio_json["board_mount_label"] = result.toio_transform.mount_label;
  if (std::isfinite(result.toio_transform.transform_error_id)) {
    toio_json["transform_error_id"] = result.toio_transform.transform_error_id;
  } else {
    toio_json["transform_error_id"] = nullptr;
  }
  toio_json["transform_color_to_toio"] = matToJson(
      result.toio_transform.color_to_toio.empty() ? result.homography_color_to_toio
                                                  : result.toio_transform.color_to_toio);
  if (result.toio_transform.coverage_area.width > 0.0 &&
      result.toio_transform.coverage_area.height > 0.0) {
    double min_x = result.toio_transform.coverage_area.x;
    double min_y = result.toio_transform.coverage_area.y;
    double max_x = min_x + result.toio_transform.coverage_area.width;
    double max_y = min_y + result.toio_transform.coverage_area.height;
    toio_json["coverage_area_toio_id"] = {
        {"min", {{"x", min_x}, {"y", min_y}}},
        {"max", {{"x", max_x}, {"y", max_y}}}};
  } else {
    toio_json["coverage_area_toio_id"] = nlohmann::json::object();
  }
  j["toio_coordinate_transform"] = toio_json;

  nlohmann::json quality_json;
  quality_json["charuco_corners"] = result.detected_charuco_corners;
  quality_json["reprojection_error_px"] = result.reprojection_error_px;
  quality_json["reprojection_error_id"] = result.reprojection_error_toio;
  quality_json["reprojection_error_floor_mm"] = result.reprojection_error_floor_mm;
  quality_json["floor_plane_std_mm"] = result.floor_plane_std_mm;
  quality_json["floor_inlier_ratio"] = result.inlier_ratio;
  quality_json["floor_inlier_count"] = result.floor_inlier_count;
  quality_json["camera_height_mm"] = result.camera_height_mm;
  quality_json["capture_count"] = result.capture_count;
  if (result.first_capture_timestamp.time_since_epoch().count() != 0) {
    quality_json["timestamp_first"] = formatTimestamp(result.first_capture_timestamp);
  }
  if (result.last_capture_timestamp.time_since_epoch().count() != 0) {
    quality_json["timestamp_last"] = formatTimestamp(result.last_capture_timestamp);
  }
  j["quality_metrics"] = quality_json;

  bool repro_pass =
      result.reprojection_error_toio <= pipeline_cfg.max_reprojection_error_id;
  bool plane_std_pass =
      !pipeline_cfg.enable_floor_plane_fit ||
      result.floor_plane_std_mm <= session_config.max_plane_std_mm;
  bool plane_inlier_pass =
      !pipeline_cfg.enable_floor_plane_fit ||
      result.inlier_ratio >= session_config.min_inlier_ratio;
  bool height_in_range =
      result.camera_height_mm >= pipeline_cfg.camera_height_warn_min_mm &&
      result.camera_height_mm <= pipeline_cfg.camera_height_warn_max_mm;

  nlohmann::json checks;
  checks["reprojection_error"] = repro_pass ? "PASS" : "FAIL";
  checks["floor_plane_std"] =
      pipeline_cfg.enable_floor_plane_fit ? (plane_std_pass ? "PASS" : "FAIL") : "SKIP";
  checks["floor_inlier_ratio"] =
      pipeline_cfg.enable_floor_plane_fit ? (plane_inlier_pass ? "PASS" : "FAIL")
                                          : "SKIP";
  checks["camera_height"] = height_in_range ? "PASS" : "WARN";

  nlohmann::json validation;
  validation["checks"] = checks;
  validation["passed"] = repro_pass && plane_std_pass && plane_inlier_pass;
  nlohmann::json warnings = nlohmann::json::array();
  if (!height_in_range) {
    warnings.push_back("camera_height_out_of_range");
  }
  
  // Add validation warnings for transform error and coverage area
  if (std::isfinite(result.toio_transform.transform_error_id)) {
    if (result.toio_transform.transform_error_id > 2.0) {
      warnings.push_back("transform_error_at_correspondences_exceeds_threshold");
    }
  } else {
    warnings.push_back("transform_error_at_correspondences_not_available");
  }
  
  // Add coverage area validation
  if (result.toio_transform.coverage_area.width > 0.0 &&
      result.toio_transform.coverage_area.height > 0.0) {
    // Coverage area is valid - check if it matches expected playmat extent
    // This is already validated in ValidateCalibration but we add the warning here
  } else {
    warnings.push_back("coverage_area_invalid");
  }
  
  validation["warnings"] = warnings;
  j["validation"] = validation;

  nlohmann::json calib_cfg = {
      {"color_width", pipeline_cfg.color_width},
      {"color_height", pipeline_cfg.color_height},
      {"depth_width", pipeline_cfg.depth_width},
      {"depth_height", pipeline_cfg.depth_height},
      {"fps", pipeline_cfg.fps},
      {"depth_min_distance_mm", pipeline_cfg.depth_min_distance_mm},
      {"depth_max_distance_mm", pipeline_cfg.depth_max_distance_mm},
      {"expected_depth_scale_m", pipeline_cfg.expected_depth_scale_m},
      {"charuco_squares_x", pipeline_cfg.charuco_squares_x},
      {"charuco_squares_y", pipeline_cfg.charuco_squares_y},
      {"charuco_square_length_mm", pipeline_cfg.charuco_square_length_mm},
      {"charuco_marker_length_mm", pipeline_cfg.charuco_marker_length_mm},
      {"min_charuco_corners", pipeline_cfg.min_charuco_corners},
      {"homography_ransac_thresh_px", pipeline_cfg.homography_ransac_thresh_px},
      {"max_reprojection_error_id", pipeline_cfg.max_reprojection_error_id},
      {"charuco_enable_subpixel_refine", pipeline_cfg.charuco_enable_subpixel_refine},
      {"charuco_subpixel_window", pipeline_cfg.charuco_subpixel_window},
      {"charuco_subpixel_max_iterations",
       pipeline_cfg.charuco_subpixel_max_iterations},
      {"charuco_subpixel_epsilon", pipeline_cfg.charuco_subpixel_epsilon},
      {"enable_floor_plane_fit", pipeline_cfg.enable_floor_plane_fit},
      {"floor_inlier_threshold_mm", pipeline_cfg.floor_inlier_threshold_mm},
      {"floor_ransac_iterations", pipeline_cfg.floor_ransac_iterations},
      {"floor_min_inlier_ratio", pipeline_cfg.floor_min_inlier_ratio},
      {"floor_z_min_mm", pipeline_cfg.floor_z_min_mm},
      {"floor_z_max_mm", pipeline_cfg.floor_z_max_mm},
      {"floor_downsample_grid", pipeline_cfg.floor_downsample_grid},
      {"floor_min_valid_points", pipeline_cfg.floor_min_valid_points},
      {"max_plane_std_mm", pipeline_cfg.max_plane_std_mm},
      {"session_attempts", pipeline_cfg.session_attempts},
      {"random_seed", pipeline_cfg.random_seed},
      {"camera_height_warn_min_mm", pipeline_cfg.camera_height_warn_min_mm},
      {"camera_height_warn_max_mm", pipeline_cfg.camera_height_warn_max_mm},
      {"enable_spatial_filter", pipeline_cfg.enable_spatial_filter},
      {"enable_color_auto_exposure", pipeline_cfg.enable_color_auto_exposure},
      {"aruco_dictionary", pipeline_cfg.aruco_dictionary},
      {"playmat_layout_path", pipeline_cfg.playmat_layout_path},
      {"board_mount_label", pipeline_cfg.board_mount_label},
      {"log_level", pipeline_cfg.log_level}};

  nlohmann::json session_cfg_json = {
      {"attempts", session_config.attempts},
      {"max_plane_std_mm", session_config.max_plane_std_mm},
      {"min_inlier_ratio", session_config.min_inlier_ratio},
      {"save_intermediate_snapshots", session_config.save_intermediate_snapshots},
      {"snapshot_output_dir", session_config.snapshot_output_dir}};

  j["config"] = {{"calibration_config", calib_cfg},
                 {"session_config", session_cfg_json}};

  try {
    std::filesystem::path file_path(path);
    auto parent = file_path.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
    std::ofstream ofs(path);
    if (!ofs) {
      spdlog::error("Failed to open file for writing: {}", path);
      return false;
    }
    ofs << j.dump(2);
    spdlog::info("Calibration result saved to {}", path);
    return true;
  } catch (const std::exception& ex) {
    spdlog::error("Failed to save calibration result to {}: {}", path, ex.what());
    return false;
  }
}

}  // namespace locomotion::calibration
