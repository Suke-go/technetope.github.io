#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "locomotion/calibration/CalibrationPipeline.h"
#include "locomotion/calibration/CalibrationSession.h"
#include "locomotion/calibration/CharucoDetector.h"

using namespace locomotion::calibration;

namespace {

struct InteractiveUiConfig {
  double preview_scale{1.0};
  bool show_depth{true};
  bool show_instructions{true};
  bool show_detection_overlay{true};
  int capture_goal{5};
  int min_charuco_corners_override{-1};
  bool auto_capture_mode{false};
  double auto_capture_delay_s{2.0};
  std::string debug_frame_dir{"logs/calibration_debug"};
  double status_overlay_alpha{0.85};
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
  auto load_uint64 = [&j](const char* key, uint64_t& dst) {
    if (j.contains(key)) {
      dst = j[key].get<uint64_t>();
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

SessionConfig loadSessionConfig(const nlohmann::json& j, const CalibrationConfig& defaults) {
  SessionConfig config;
  config.attempts = defaults.session_attempts;
  config.max_plane_std_mm = defaults.max_plane_std_mm;
  config.min_inlier_ratio = defaults.floor_min_inlier_ratio;

  if (j.contains("session_attempts")) {
    config.attempts = j["session_attempts"].get<int>();
  }
  if (j.contains("max_plane_std_mm")) {
    config.max_plane_std_mm = j["max_plane_std_mm"].get<double>();
  }
  if (j.contains("min_inlier_ratio")) {
    config.min_inlier_ratio = j["min_inlier_ratio"].get<double>();
  }
  if (j.contains("save_intermediate_snapshots")) {
    config.save_intermediate_snapshots = j["save_intermediate_snapshots"].get<bool>();
  }
  if (j.contains("snapshot_output_dir")) {
    config.snapshot_output_dir = j["snapshot_output_dir"].get<std::string>();
  }
  return config;
}

InteractiveUiConfig loadInteractiveUiConfig(const nlohmann::json& root) {
  InteractiveUiConfig cfg;
  if (!root.contains("interactive_ui")) {
    return cfg;
  }
  const auto& j = root.at("interactive_ui");
  if (j.contains("preview_scale")) {
    cfg.preview_scale = j["preview_scale"].get<double>();
  }
  if (j.contains("show_depth")) {
    cfg.show_depth = j["show_depth"].get<bool>();
  }
  if (j.contains("show_instructions")) {
    cfg.show_instructions = j["show_instructions"].get<bool>();
  }
  if (j.contains("show_detection_overlay")) {
    cfg.show_detection_overlay = j["show_detection_overlay"].get<bool>();
  }
  if (j.contains("capture_goal")) {
    cfg.capture_goal = j["capture_goal"].get<int>();
  }
  if (j.contains("min_charuco_corners")) {
    cfg.min_charuco_corners_override = j["min_charuco_corners"].get<int>();
  }
  if (j.contains("auto_capture_mode")) {
    cfg.auto_capture_mode = j["auto_capture_mode"].get<bool>();
  }
  if (j.contains("auto_capture_delay_s")) {
    cfg.auto_capture_delay_s = j["auto_capture_delay_s"].get<double>();
  }
  if (j.contains("debug_frame_dir")) {
    cfg.debug_frame_dir = j["debug_frame_dir"].get<std::string>();
  }
  if (j.contains("status_overlay_alpha")) {
    cfg.status_overlay_alpha = j["status_overlay_alpha"].get<double>();
  }
  return cfg;
}

cv::aruco::PredefinedDictionaryType parseDictionary(const std::string& name) {
  using PD = cv::aruco::PredefinedDictionaryType;
  static const std::map<std::string, PD> kMap = {
      {"DICT_4X4_50", PD::DICT_4X4_50},         {"DICT_4X4_100", PD::DICT_4X4_100},
      {"DICT_5X5_50", PD::DICT_5X5_50},         {"DICT_5X5_100", PD::DICT_5X5_100},
      {"DICT_6X6_50", PD::DICT_6X6_50},         {"DICT_6X6_100", PD::DICT_6X6_100},
      {"DICT_APRILTAG_16h5", PD::DICT_APRILTAG_16h5},
      {"DICT_APRILTAG_25h9", PD::DICT_APRILTAG_25h9},
      {"DICT_APRILTAG_36h11", PD::DICT_APRILTAG_36h11}};
  auto it = kMap.find(name);
  if (it == kMap.end()) {
    spdlog::warn("Unknown ArUco dictionary '{}', fallback to DICT_4X4_50", name);
    return PD::DICT_4X4_50;
  }
  return it->second;
}

cv::Ptr<cv::aruco::Dictionary> makeDictionary(const CalibrationConfig& config) {
  auto dict_id = parseDictionary(config.aruco_dictionary);
  return cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(dict_id));
}

cv::Ptr<cv::aruco::CharucoBoard> makeBoard(const CalibrationConfig& config,
                                           const cv::Ptr<cv::aruco::Dictionary>& dict) {
  return cv::makePtr<cv::aruco::CharucoBoard>(
      cv::Size(config.charuco_squares_x, config.charuco_squares_y),
      config.charuco_square_length_mm, config.charuco_marker_length_mm, *dict);
}

CharucoDetector makePreviewDetector(const CalibrationConfig& config) {
  auto dictionary = makeDictionary(config);
  auto board = makeBoard(config, dictionary);
  CharucoDetectorConfig detector_cfg;
  detector_cfg.min_corners = std::max(4, config.min_charuco_corners / 2);
  detector_cfg.enable_subpixel_refine = config.charuco_enable_subpixel_refine;
  detector_cfg.subpixel_window =
      cv::Size(config.charuco_subpixel_window, config.charuco_subpixel_window);
  detector_cfg.subpixel_max_iterations = config.charuco_subpixel_max_iterations;
  detector_cfg.subpixel_epsilon = config.charuco_subpixel_epsilon;
  return CharucoDetector(dictionary, board, detector_cfg);
}

constexpr int kPanelGap = 12;

cv::Vec3b lerpColor(const cv::Vec3b& a, const cv::Vec3b& b, double t) {
  t = std::clamp(t, 0.0, 1.0);
  cv::Vec3b out;
  for (int i = 0; i < 3; ++i) {
    out[i] = static_cast<uint8_t>(std::lround(a[i] + (b[i] - a[i]) * t));
  }
  return out;
}

cv::Vec3b colorForHeight(double height_mm) {
  const cv::Vec3b blue(200, 40, 0);    // BGR
  const cv::Vec3b green(0, 200, 0);
  const cv::Vec3b yellow(0, 255, 255);
  const cv::Vec3b red(0, 0, 255);

  if (height_mm <= 0.0) {
    return blue;
  }
  if (height_mm < 150.0) {
    return lerpColor(blue, green, height_mm / 150.0);
  }
  if (height_mm < 300.0) {
    return lerpColor(green, yellow, (height_mm - 150.0) / 150.0);
  }
  if (height_mm < 450.0) {
    return lerpColor(yellow, red, (height_mm - 300.0) / 150.0);
  }
  return red;
}

double estimateCameraHeightFromDepth(const cv::Mat& depth, double depth_scale_m) {
  if (depth.empty() || depth_scale_m <= 0.0) {
    return 0.0;
  }

  std::vector<double> valid_depths;
  valid_depths.reserve(depth.rows * depth.cols / 4);
  const double scale_mm = depth_scale_m * 1000.0;

  for (int y = depth.rows / 4; y < 3 * depth.rows / 4; ++y) {
    const auto* row = depth.ptr<uint16_t>(y);
    for (int x = depth.cols / 4; x < 3 * depth.cols / 4; ++x) {
      uint16_t depth_raw = row[x];
      if (depth_raw > 0) {
        double depth_mm = static_cast<double>(depth_raw) * scale_mm;
        if (depth_mm >= 2300.0 && depth_mm <= 2800.0) {
          valid_depths.push_back(depth_mm);
        }
      }
    }
  }

  if (valid_depths.size() < 100) {
    return 0.0;
  }

  std::sort(valid_depths.begin(), valid_depths.end());
  size_t median_idx = valid_depths.size() / 2;
  return valid_depths[median_idx];
}

cv::Mat renderDepthMap(const cv::Mat& depth,
                       double depth_scale_m,
                       double camera_height_mm) {
  if (depth.empty() || depth_scale_m <= 0.0) {
    return cv::Mat();
  }

  double estimated_height = camera_height_mm;
  if (estimated_height <= 0.0) {
    estimated_height = estimateCameraHeightFromDepth(depth, depth_scale_m);
  }
  if (estimated_height <= 0.0) {
    return cv::Mat();
  }

  cv::Mat heat(depth.size(), CV_8UC3, cv::Scalar(20, 20, 20));
  const double scale_mm = depth_scale_m * 1000.0;

  for (int y = 0; y < depth.rows; ++y) {
    const auto* row = depth.ptr<uint16_t>(y);
    auto* out = heat.ptr<cv::Vec3b>(y);
    for (int x = 0; x < depth.cols; ++x) {
      uint16_t depth_raw = row[x];
      if (depth_raw == 0) {
        out[x] = cv::Vec3b(30, 30, 30);
        continue;
      }
      double depth_mm = static_cast<double>(depth_raw) * scale_mm;
      double height_mm = estimated_height - depth_mm;
      out[x] = colorForHeight(height_mm);
    }
  }
  return heat;
}

cv::Mat composePanels(const cv::Mat& color, const cv::Mat& depth, bool include_depth) {
  if (!include_depth || depth.empty()) {
    return color.clone();
  }

  cv::Mat color_panel = color.clone();
  cv::Mat depth_panel;
  cv::resize(depth, depth_panel, color_panel.size(), 0, 0, cv::INTER_NEAREST);

  const int gap = 12;
  int canvas_width = color_panel.cols + gap + depth_panel.cols;
  int canvas_height = std::max(color_panel.rows, depth_panel.rows);
  cv::Mat canvas(canvas_height, canvas_width, CV_8UC3, cv::Scalar(20, 20, 20));

  color_panel.copyTo(canvas(cv::Rect(0, 0, color_panel.cols, color_panel.rows)));
  depth_panel.copyTo(
      canvas(cv::Rect(color_panel.cols + gap, 0, depth_panel.cols, depth_panel.rows)));
  return canvas;
}

cv::Rect drawTextBlock(cv::Mat& canvas,
                       const std::vector<std::string>& lines,
                       cv::Point preferred_top_left,
                       double font_scale,
                       int thickness,
                       const cv::Scalar& text_color,
                       const cv::Scalar& bg_color,
                       double alpha) {
  if (lines.empty()) {
    return cv::Rect();
  }

  const int padding = 8;
  const int line_gap = 6;
  std::vector<cv::Size> metrics;
  metrics.reserve(lines.size());
  int max_width = 0;
  int total_height = padding;
  for (const auto& line : lines) {
    int baseline = 0;
    cv::Size sz = cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness,
                                  &baseline);
    metrics.push_back(sz);
    max_width = std::max(max_width, sz.width);
    total_height += sz.height + line_gap;
  }
  total_height += padding - line_gap;
  cv::Size box(max_width + padding * 2, total_height);
  cv::Rect rect(preferred_top_left, box);
  if (rect.x + rect.width > canvas.cols) {
    rect.x = std::max(0, canvas.cols - rect.width);
  }
  if (rect.y + rect.height > canvas.rows) {
    rect.y = std::max(0, canvas.rows - rect.height);
  }
  if (rect.x < 0 || rect.y < 0) {
    rect.x = std::max(0, rect.x);
    rect.y = std::max(0, rect.y);
  }

  cv::Mat roi = canvas(rect);
  cv::Mat overlay;
  roi.copyTo(overlay);
  cv::rectangle(overlay, cv::Point(0, 0), cv::Point(rect.width, rect.height), bg_color,
                cv::FILLED);
  cv::addWeighted(overlay, alpha, roi, 1.0 - alpha, 0.0, roi);

  int y = rect.y + padding;
  for (size_t i = 0; i < lines.size(); ++i) {
    y += metrics[i].height;
    cv::putText(canvas, lines[i], cv::Point(rect.x + padding, y),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness, cv::LINE_AA);
    y += line_gap;
  }
  return rect;
}

std::string formatDouble(double value, int precision = 2) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

std::string timestampString() {
  auto now = std::chrono::system_clock::now();
  std::time_t tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &tt);
#else
  localtime_r(&tt, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

bool saveDebugFrame(const FrameBundle& frame,
                    const InteractiveUiConfig& ui_config,
                    std::string& color_path_out,
                    std::string& depth_path_out) {
  namespace fs = std::filesystem;
  try {
    fs::path dir = ui_config.debug_frame_dir.empty() ? fs::path{"logs/calibration_debug"}
                                                     : fs::path{ui_config.debug_frame_dir};
    fs::create_directories(dir);
    std::string ts = timestampString();
    fs::path color_path = dir / ("color_" + ts + ".png");
    fs::path depth_path = dir / ("depth_" + ts + ".png");
    if (!frame.color.empty()) {
      cv::imwrite(color_path.string(), frame.color);
      color_path_out = color_path.string();
    }
    if (!frame.depth.empty()) {
      cv::imwrite(depth_path.string(), frame.depth);
      depth_path_out = depth_path.string();
    }
    return true;
  } catch (const std::exception& ex) {
    spdlog::error("Failed to save debug frame: {}", ex.what());
    return false;
  }
}

struct Toast {
  std::string message;
  std::chrono::steady_clock::time_point expires{};
};

void showToast(Toast& toast, const std::string& msg, std::chrono::milliseconds duration) {
  toast.message = msg;
  toast.expires = std::chrono::steady_clock::now() + duration;
}

void drawToast(const Toast& toast, cv::Mat& canvas) {
  if (toast.message.empty() || std::chrono::steady_clock::now() > toast.expires) {
    return;
  }
  const double font_scale = 0.65;
  const int thickness = 1;
  int baseline = 0;
  cv::Size text_size = cv::getTextSize(toast.message, cv::FONT_HERSHEY_SIMPLEX,
                                       font_scale, thickness, &baseline);
  cv::Point tl((canvas.cols - text_size.width) / 2 - 12,
               canvas.rows - text_size.height - 20);
  cv::Rect rect(tl, cv::Size(text_size.width + 24, text_size.height + 18));
  if (rect.x < 0) rect.x = 0;
  if (rect.y < 0) rect.y = 0;
  if (rect.x + rect.width > canvas.cols) {
    rect.x = std::max(0, canvas.cols - rect.width);
  }
  if (rect.y + rect.height > canvas.rows) {
    rect.y = std::max(0, canvas.rows - rect.height);
  }
  cv::Mat roi = canvas(rect);
  cv::Mat overlay;
  roi.copyTo(overlay);
  cv::rectangle(overlay, cv::Point(0, 0), cv::Point(rect.width, rect.height),
                cv::Scalar(30, 30, 30), cv::FILLED);
  cv::addWeighted(overlay, 0.85, roi, 0.15, 0.0, roi);
  cv::putText(canvas, toast.message,
              cv::Point(rect.x + 12, rect.y + rect.height - 10),
              cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 255, 255), thickness,
              cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path config_path = "calibration_config.json";
  std::filesystem::path output_path = "calib_result.json";
  if (argc > 1) {
    config_path = argv[1];
  }
  if (argc > 2) {
    output_path = argv[2];
  }

  CalibrationConfig calib_config = loadCalibrationConfig(config_path);
  nlohmann::json config_json;
  if (std::filesystem::exists(config_path)) {
    std::ifstream ifs(config_path);
    ifs >> config_json;
  }
  SessionConfig session_config = loadSessionConfig(config_json, calib_config);
  InteractiveUiConfig ui_config = loadInteractiveUiConfig(config_json);

  // Extract config file directory for path resolution
  std::filesystem::path config_dir;
  if (std::filesystem::exists(config_path)) {
    config_dir = std::filesystem::absolute(config_path).parent_path();
  } else {
    config_dir = std::filesystem::current_path();
  }

  CalibrationPipeline pipeline(calib_config);
  if (!pipeline.initialize(config_dir.string())) {
    std::cerr << "[ERROR] Failed to initialize calibration pipeline." << std::endl;
    return 1;
  }

  CharucoDetector preview_detector = makePreviewDetector(calib_config);

  const std::string window_title = "RealSense D415 Calibration - " +
                                   std::to_string(calib_config.color_width) + "x" +
                                   std::to_string(calib_config.color_height) + " @ " +
                                   std::to_string(calib_config.fps) + " FPS";
  cv::namedWindow(window_title, cv::WINDOW_NORMAL);

  const int detection_threshold =
      ui_config.min_charuco_corners_override > 0 ? ui_config.min_charuco_corners_override
                                                 : calib_config.min_charuco_corners;
  const int capture_goal = ui_config.capture_goal > 0 ? ui_config.capture_goal
                                                      : session_config.attempts;

  bool show_overlay = ui_config.show_detection_overlay;
  bool running = true;
  int successful_captures = 0;
  int capture_attempts = 0;
  std::optional<CalibrationResult> last_result;
  std::string last_result_message = "No captures yet.";
  std::string last_error_message;
  std::string last_output_path;
  std::future<std::optional<CalibrationSnapshot>> processing_future;
  bool processing_active = false;
  char spinner_chars[] = {'|', '/', '-', '\\'};
  const size_t spinner_count = sizeof(spinner_chars) / sizeof(spinner_chars[0]);
  size_t spinner_index = 0;
  Toast toast;
  double fps_estimate = 0.0;
  auto last_frame_tp = std::chrono::steady_clock::now();

  FrameBundle frame;
  cv::Mat canvas;

  while (running) {
    FrameBundle bundle;
    if (!pipeline.CaptureFrame(bundle)) {
      canvas = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
      cv::putText(canvas, "Camera capture failed. Check RealSense connection.",
                  cv::Point(20, canvas.rows / 2), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
      cv::imshow(window_title, canvas);
      int key = cv::waitKey(10);
      if (key == 27 || key == 'q' || key == 'Q') {
        break;
      }
      continue;
    }

    frame = std::move(bundle);
    auto now = std::chrono::steady_clock::now();
    double delta_s =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - last_frame_tp).count();
    last_frame_tp = now;
    if (delta_s > 0.0) {
      double inst_fps = 1.0 / delta_s;
      fps_estimate = fps_estimate == 0.0 ? inst_fps : 0.9 * fps_estimate + 0.1 * inst_fps;
    }

    auto detection = preview_detector.Detect(frame.color);
    bool detection_good =
        detection && detection->detected_charuco_corners >= detection_threshold;

    if (processing_active && processing_future.valid() &&
        processing_future.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::ready) {
      processing_active = false;
      auto snapshot_opt = processing_future.get();
      if (!snapshot_opt) {
        last_error_message = "Calibration failed. Board detection lost during capture.";
        showToast(toast, last_error_message, std::chrono::seconds(3));
      } else {
        CalibrationResult result = CalibrationSession::SnapshotToResult(*snapshot_opt);
        bool saved = CalibrationSession::SaveResultJson(result, output_path.string(),
                                                        pipeline.config(), session_config,
                                                        pipeline.camera_model(),
                                                        pipeline.camera_serial(),
                                                        pipeline.depth_scale_m());
        if (saved) {
          last_result = result;
          last_output_path = output_path.string();
          ++successful_captures;
          last_result_message = "OK - RMS " +
                                formatDouble(result.reprojection_error_toio, 2) +
                                " ID, plane std " +
                                formatDouble(result.floor_plane_std_mm, 2) + " mm";
          last_error_message.clear();
          showToast(toast, "Calibration saved to " + last_output_path,
                    std::chrono::seconds(3));
        } else {
          last_error_message = "Failed to write calibration JSON.";
          showToast(toast, last_error_message, std::chrono::seconds(3));
        }
      }
    }

    if (!processing_active) {
      spinner_index = 0;
    } else {
      spinner_index = (spinner_index + 1) % spinner_count;
    }

    cv::Mat color_panel = frame.color.clone();
    if (show_overlay && detection) {
      for (const auto& pt : detection->image_points) {
        cv::circle(color_panel, pt, 4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
      }
      for (size_t i = 0; i < detection->image_points.size(); ++i) {
        const auto& pt = detection->image_points[i];
        int id = detection->ids[i];
        cv::putText(color_panel, std::to_string(id),
                    cv::Point(static_cast<int>(pt.x) + 4, static_cast<int>(pt.y) - 4),
                    cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
      }
    }

    cv::Mat depth_panel;
    double current_camera_height = 0.0;
    if (last_result && last_result->camera_height_mm > 0.0) {
      current_camera_height = last_result->camera_height_mm;
    }
    if (ui_config.show_depth) {
      depth_panel = renderDepthMap(frame.depth, pipeline.depth_scale_m(), current_camera_height);
    }

    canvas = composePanels(color_panel, depth_panel, ui_config.show_depth);

    if (ui_config.preview_scale != 1.0) {
      cv::resize(canvas, canvas, cv::Size(), ui_config.preview_scale, ui_config.preview_scale,
                 cv::INTER_LINEAR);
    }

    std::vector<std::string> status_lines;
    std::string board_status;
    int detected_corners = detection ? detection->detected_charuco_corners : 0;
    if (detection && detected_corners > 0) {
      board_status =
          "Board " + std::string(detection_good ? "detected" : "partial") + " (" +
          std::to_string(detected_corners) + "/" +
          std::to_string(detection_threshold) + ")";
    } else {
      board_status = "Board not detected";
    }
    status_lines.push_back("Status: " + board_status);
    status_lines.push_back("Captures: " + std::to_string(successful_captures) + "/" +
                          std::to_string(capture_goal));
    std::string processing_text = processing_active
                                      ? std::string("Processing ") +
                                            spinner_chars[spinner_index % spinner_count]
                                      : (detection_good ? "Ready" : "Waiting for board...");
    status_lines.push_back("State: " + processing_text);
    status_lines.push_back("Preview FPS: " + formatDouble(fps_estimate, 1));
    if (current_camera_height > 0.0) {
      status_lines.push_back("Height: " + formatDouble(current_camera_height / 1000.0, 2) + " m");
    } else if (ui_config.show_depth && !frame.depth.empty()) {
      double est_height = estimateCameraHeightFromDepth(frame.depth, pipeline.depth_scale_m());
      if (est_height > 0.0) {
        status_lines.push_back("Height (est): " + formatDouble(est_height / 1000.0, 2) + " m");
      }
    }
    if (!last_result_message.empty()) {
      status_lines.push_back("Last: " + last_result_message);
    }
    if (!last_error_message.empty()) {
      status_lines.push_back("Error: " + last_error_message);
    }
    status_lines.push_back("Output: " + output_path.string());
    status_lines.push_back(std::string("Overlay: ") + (show_overlay ? "ON" : "OFF"));

    auto status_rect = drawTextBlock(canvas, status_lines, cv::Point(12, 12), 0.6, 1,
                                     cv::Scalar(255, 255, 255), cv::Scalar(20, 20, 20),
                                     ui_config.status_overlay_alpha);
    // Status indicator color: Red (< 8), Yellow (8-11), Green (≥ 12)
    cv::Scalar indicator_color;
    if (detected_corners < 8) {
      indicator_color = cv::Scalar(0, 0, 255);  // Red (BGR)
    } else if (detected_corners < detection_threshold) {
      indicator_color = cv::Scalar(0, 215, 255);  // Yellow (BGR)
    } else {
      indicator_color = cv::Scalar(0, 200, 0);  // Green (BGR)
    }
    cv::circle(canvas,
               cv::Point(status_rect.x + status_rect.width - 18, status_rect.y + 18), 8,
               indicator_color, cv::FILLED, cv::LINE_AA);

    if (ui_config.show_instructions) {
      std::vector<std::string> instructions = {
          "INSTRUCTIONS:",
          "1. Place ChArUco board 40-80cm from camera",
          "2. Keep board parallel to floor",
          "3. Avoid glare and harsh shadows",
          "4. Press SPACE when indicator is green",
          "CONTROLS:",
          "[SPACE] Capture  [S] Save Frame  [D] Overlay",
          "[Q]/[ESC] Quit"};
      int bottom_y = canvas.rows - 10 - static_cast<int>(instructions.size()) * 18;
      drawTextBlock(canvas, instructions, cv::Point(12, std::max(12, bottom_y)), 0.55, 1,
                    cv::Scalar(230, 230, 230), cv::Scalar(15, 15, 15),
                    ui_config.status_overlay_alpha);
    }

    drawToast(toast, canvas);

    cv::imshow(window_title, canvas);
    int key = cv::waitKey(1);
    if (key >= 0) {
      switch (key) {
        case 27:  // ESC
        case 'q':
        case 'Q':
          running = false;
          break;
        case ' ': {
          if (processing_active) {
            showToast(toast, "Already processing. Please wait.", std::chrono::seconds(2));
            break;
          }
          if (!detection_good) {
            showToast(toast, "Board not ready for capture.", std::chrono::seconds(2));
            break;
          }
          processing_active = true;
          ++capture_attempts;
          FrameBundle captured = frame;
          processing_future = std::async(std::launch::async, [&pipeline, captured]() {
            return pipeline.ProcessFrame(captured);
          });
          showToast(toast, "Capturing frame...", std::chrono::seconds(1));
          break;
        }
        case 's':
        case 'S': {
          std::string color_path;
          std::string depth_path;
          if (saveDebugFrame(frame, ui_config, color_path, depth_path)) {
            std::string msg = "Saved frame: " + color_path;
            if (!depth_path.empty()) {
              msg += " | " + depth_path;
            }
            showToast(toast, msg, std::chrono::seconds(3));
          } else {
            showToast(toast, "Failed to save frame.", std::chrono::seconds(2));
          }
          break;
        }
        case 'd':
        case 'D': {
          show_overlay = !show_overlay;
          showToast(toast, std::string("Detection overlay ") + (show_overlay ? "ON" : "OFF"),
                    std::chrono::seconds(2));
          break;
        }
        default:
          break;
      }
    }
  }

  if (processing_active && processing_future.valid()) {
    processing_future.get();
  }

  cv::destroyWindow(window_title);
  return 0;
}
