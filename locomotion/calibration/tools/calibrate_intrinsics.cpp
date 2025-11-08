#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <map>

#include <nlohmann/json.hpp>

#include <librealsense2/rs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "locomotion/calibration/CalibrationPipeline.h"
#include "locomotion/calibration/CharucoDetector.h"

using namespace locomotion::calibration;

namespace {

struct IntrinsicsCaptureConfig {
  int required_frames{30};
  int max_attempts{200};
  bool show_progress{true};
};

CalibrationConfig loadCalibrationConfig(const std::filesystem::path& path) {
  CalibrationConfig config;
  if (!std::filesystem::exists(path)) {
    std::cerr << "[WARN] Config file " << path << " not found. Using defaults.\n";
    return config;
  }

  nlohmann::json j;
  std::ifstream ifs(path);
  ifs >> j;

  auto load_int = [&j](const char* key, int& dst) {
    if (j.contains(key)) {
      dst = j[key].get<int>();
    }
  };
  auto load_double = [&j](const char* key, double& dst) {
    if (j.contains(key)) {
      dst = j[key].get<double>();
    }
  };
  auto load_uint64 = [&j](const char* key, uint64_t& dst) {
    if (j.contains(key)) {
      dst = j[key].get<uint64_t>();
    }
  };
  auto load_float = [&j](const char* key, float& dst) {
    if (j.contains(key)) {
      dst = j[key].get<float>();
    }
  };
  auto load_bool = [&j](const char* key, bool& dst) {
    if (j.contains(key)) {
      dst = j[key].get<bool>();
    }
  };
  auto load_string = [&j](const char* key, std::string& dst) {
    if (j.contains(key)) {
      dst = j[key].get<std::string>();
    }
  };

  load_int("color_width", config.color_width);
  load_int("color_height", config.color_height);
  load_int("depth_width", config.depth_width);
  load_int("depth_height", config.depth_height);
  load_int("fps", config.fps);
  load_double("depth_min_distance_mm", config.depth_min_distance_mm);
  load_double("depth_max_distance_mm", config.depth_max_distance_mm);
  load_double("expected_depth_scale_m", config.expected_depth_scale_m);
  load_int("charuco_squares_x", config.charuco_squares_x);
  load_int("charuco_squares_y", config.charuco_squares_y);
  load_float("charuco_square_length_mm", config.charuco_square_length_mm);
  load_float("charuco_marker_length_mm", config.charuco_marker_length_mm);
  load_int("min_charuco_corners", config.min_charuco_corners);
  load_double("homography_ransac_thresh_px", config.homography_ransac_thresh_px);
  load_double("max_reprojection_error_id", config.max_reprojection_error_id);
  load_bool("charuco_enable_subpixel_refine", config.charuco_enable_subpixel_refine);
  load_int("charuco_subpixel_window", config.charuco_subpixel_window);
  load_int("charuco_subpixel_max_iterations", config.charuco_subpixel_max_iterations);
  load_double("charuco_subpixel_epsilon", config.charuco_subpixel_epsilon);
  load_bool("enable_floor_plane_fit", config.enable_floor_plane_fit);
  load_double("floor_inlier_threshold_mm", config.floor_inlier_threshold_mm);
  load_int("floor_ransac_iterations", config.floor_ransac_iterations);
  load_double("floor_min_inlier_ratio", config.floor_min_inlier_ratio);
  load_double("floor_z_min_mm", config.floor_z_min_mm);
  load_double("floor_z_max_mm", config.floor_z_max_mm);
  load_int("floor_downsample_grid", config.floor_downsample_grid);
  load_int("floor_min_valid_points", config.floor_min_valid_points);
  load_double("max_plane_std_mm", config.max_plane_std_mm);
  load_int("session_attempts", config.session_attempts);
  load_uint64("random_seed", config.random_seed);
  load_double("camera_height_warn_min_mm", config.camera_height_warn_min_mm);
  load_double("camera_height_warn_max_mm", config.camera_height_warn_max_mm);
  load_bool("enable_spatial_filter", config.enable_spatial_filter);
  load_bool("enable_color_auto_exposure", config.enable_color_auto_exposure);
  load_string("aruco_dictionary", config.aruco_dictionary);
  load_string("playmat_layout_path", config.playmat_layout_path);
  load_string("board_mount_label", config.board_mount_label);
  load_string("log_level", config.log_level);

  return config;
}

IntrinsicsCaptureConfig loadIntrinsicsCaptureConfig(const nlohmann::json& j) {
  IntrinsicsCaptureConfig cfg;
  if (j.contains("intrinsics_required_frames")) {
    cfg.required_frames = j["intrinsics_required_frames"].get<int>();
  }
  if (j.contains("intrinsics_max_attempts")) {
    cfg.max_attempts = j["intrinsics_max_attempts"].get<int>();
  }
  if (j.contains("intrinsics_show_progress")) {
    cfg.show_progress = j["intrinsics_show_progress"].get<bool>();
  }
  return cfg;
}

rs2::config makeRealSenseConfig(const CalibrationConfig& cfg) {
  rs2::config rs_cfg;
  rs_cfg.enable_stream(RS2_STREAM_COLOR, cfg.color_width, cfg.color_height, RS2_FORMAT_BGR8,
                       cfg.fps);
  return rs_cfg;
}

cv::Ptr<cv::aruco::Dictionary> makeDictionary(const CalibrationConfig& config) {
  using PD = cv::aruco::PredefinedDictionaryType;
  static const std::map<std::string, PD> kMap = {
      {"DICT_4X4_50", PD::DICT_4X4_50},         {"DICT_4X4_100", PD::DICT_4X4_100},
      {"DICT_5X5_50", PD::DICT_5X5_50},         {"DICT_5X5_100", PD::DICT_5X5_100},
      {"DICT_6X6_50", PD::DICT_6X6_50},         {"DICT_6X6_100", PD::DICT_6X6_100},
      {"DICT_APRILTAG_16h5", PD::DICT_APRILTAG_16h5},
      {"DICT_APRILTAG_25h9", PD::DICT_APRILTAG_25h9},
      {"DICT_APRILTAG_36h11", PD::DICT_APRILTAG_36h11}};
  auto it = kMap.find(config.aruco_dictionary);
  PD dict_id = (it != kMap.end()) ? it->second : PD::DICT_4X4_50;
  if (it == kMap.end()) {
    std::cerr << "[WARN] Unknown ArUco dictionary '" << config.aruco_dictionary
              << "'. Falling back to DICT_4X4_50.\n";
  }
  return cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(dict_id));
}

void saveIntrinsicsJson(const std::filesystem::path& path,
                        const CalibrationConfig& config,
                        double rms_error,
                        const cv::Mat& camera_matrix,
                        const cv::Mat& dist_coeffs,
                        int frames_used) {
  nlohmann::json j;
  j["schema_version"] = 1;
  j["camera"] = {
      {"image_width", config.color_width},
      {"image_height", config.color_height},
      {"fps", config.fps},
  };

  nlohmann::json K = nlohmann::json::array();
  for (int r = 0; r < camera_matrix.rows; ++r) {
    nlohmann::json row = nlohmann::json::array();
    for (int c = 0; c < camera_matrix.cols; ++c) {
      row.push_back(camera_matrix.at<double>(r, c));
    }
    K.push_back(row);
  }
  j["camera_matrix"] = K;

  std::vector<double> dist;
  dist.reserve(static_cast<size_t>(dist_coeffs.total()));
  for (int i = 0; i < dist_coeffs.total(); ++i) {
    dist.push_back(dist_coeffs.at<double>(i));
  }
  j["distortion_coefficients"] = dist;

  j["charuco_board"] = {
      {"squares_x", config.charuco_squares_x},
      {"squares_y", config.charuco_squares_y},
      {"square_length_mm", config.charuco_square_length_mm},
      {"marker_length_mm", config.charuco_marker_length_mm},
      {"aruco_dictionary", config.aruco_dictionary},
  };

  j["calibration_report"] = {
      {"frames_used", frames_used},
      {"rms_reprojection_error", rms_error},
  };

  std::filesystem::create_directories(path.parent_path());
  std::ofstream ofs(path);
  ofs << j.dump(2);
  std::cout << "[INFO] Intrinsics saved to " << path << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path config_path{"calibration_config.json"};
  std::filesystem::path output_path{"calibration/intrinsics/d415_intrinsics.json"};

  if (argc > 1) {
    config_path = argv[1];
  }
  if (argc > 2) {
    output_path = argv[2];
  }

  CalibrationConfig calib_config = loadCalibrationConfig(config_path);

  nlohmann::json json_config;
  if (std::filesystem::exists(config_path)) {
    std::ifstream ifs(config_path);
    ifs >> json_config;
  }
  IntrinsicsCaptureConfig capture_config = loadIntrinsicsCaptureConfig(json_config);

  cv::Ptr<cv::aruco::Dictionary> dictionary = makeDictionary(calib_config);
  cv::Ptr<cv::aruco::CharucoBoard> board = cv::makePtr<cv::aruco::CharucoBoard>(
      cv::Size(calib_config.charuco_squares_x, calib_config.charuco_squares_y),
      calib_config.charuco_square_length_mm, calib_config.charuco_marker_length_mm, *dictionary);

  CharucoDetectorConfig detector_config;
  detector_config.min_corners = calib_config.min_charuco_corners;
  detector_config.enable_subpixel_refine = calib_config.charuco_enable_subpixel_refine;
  detector_config.subpixel_window =
      cv::Size(calib_config.charuco_subpixel_window, calib_config.charuco_subpixel_window);
  detector_config.subpixel_max_iterations = calib_config.charuco_subpixel_max_iterations;
  detector_config.subpixel_epsilon = calib_config.charuco_subpixel_epsilon;

  CharucoDetector detector(dictionary, board, detector_config);

  rs2::pipeline pipeline;
  rs2::config rs_cfg = makeRealSenseConfig(calib_config);

  try {
    pipeline.start(rs_cfg);
  } catch (const rs2::error& err) {
    std::cerr << "[ERROR] Failed to start RealSense pipeline: " << err.what() << std::endl;
    return 1;
  }

  std::vector<std::vector<cv::Point2f>> all_charuco_corners;
  std::vector<std::vector<int>> all_charuco_ids;
  int attempt = 0;

  std::cout << "[INFO] Capturing intrinsics frames. Required frames: "
            << capture_config.required_frames << std::endl;

  while (static_cast<int>(all_charuco_corners.size()) < capture_config.required_frames &&
         attempt < capture_config.max_attempts) {
    ++attempt;
    rs2::frameset frames;
    try {
      frames = pipeline.wait_for_frames();
    } catch (const rs2::error& err) {
      std::cerr << "[WARN] Frame capture failed: " << err.what() << std::endl;
      continue;
    }

    rs2::video_frame color = frames.get_color_frame();
    if (!color) {
      continue;
    }

    cv::Mat color_bgr(cv::Size(color.get_width(), color.get_height()), CV_8UC3,
                      const_cast<void*>(color.get_data()), cv::Mat::AUTO_STEP);
    cv::Mat color_copy = color_bgr.clone();

    auto detection = detector.Detect(color_copy);
    if (!detection || detection->detected_charuco_corners < detector_config.min_corners) {
      if (capture_config.show_progress) {
        std::cout << "[INFO] Attempt " << attempt << ": insufficient charuco corners."
                  << std::endl;
      }
      continue;
    }

    all_charuco_corners.push_back(detection->image_points);
    all_charuco_ids.emplace_back(detection->ids.begin(), detection->ids.end());
    if (capture_config.show_progress) {
      std::cout << "[INFO] Captured frame " << all_charuco_corners.size() << " / "
                << capture_config.required_frames
                << " (corners=" << detection->detected_charuco_corners << ")." << std::endl;
    }
  }

  pipeline.stop();

  if (static_cast<int>(all_charuco_corners.size()) < capture_config.required_frames) {
    std::cerr << "[ERROR] Not enough valid frames captured. Needed "
              << capture_config.required_frames << ", collected "
              << all_charuco_corners.size() << std::endl;
    return 1;
  }

  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
  std::vector<cv::Mat> rvecs, tvecs;

  double rms = 0.0;
  try {
    rms = cv::aruco::calibrateCameraCharuco(all_charuco_corners, all_charuco_ids, board,
                                            cv::Size(calib_config.color_width,
                                                     calib_config.color_height),
                                            camera_matrix, dist_coeffs, rvecs, tvecs);
  } catch (const cv::Exception& ex) {
    std::cerr << "[ERROR] Charuco calibration failed: " << ex.what() << std::endl;
    return 1;
  }

  std::cout << "[INFO] Calibration completed. RMS error = " << rms << std::endl;
  saveIntrinsicsJson(output_path, calib_config, rms, camera_matrix, dist_coeffs,
                     static_cast<int>(all_charuco_corners.size()));

  return 0;
}
