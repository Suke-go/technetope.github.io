#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <librealsense2/rs.hpp>
#include <nlohmann/json.hpp>

#include "locomotion/calibration/CalibrationPipeline.h"
#include "locomotion/calibration/CalibrationSession.h"

using namespace locomotion::calibration;

namespace {

struct QCThresholds {
  // Intrinsics validation
  double min_fx{600.0};
  double max_fx{650.0};
  double min_fy{600.0};
  double max_fy{650.0};
  double min_cx{620.0};
  double max_cx{660.0};
  double min_cy{350.0};
  double max_cy{370.0};

  // Calibration validation
  double max_reprojection_error_id{8.0};
  double max_plane_std_mm{8.0};
  double min_inlier_ratio{0.8};
};

struct QCResult {
  bool overall_pass{false};

  // Device checks
  bool device_connected{false};
  std::string device_serial;
  std::string device_name;
  std::string firmware_version;

  // Intrinsics checks
  bool intrinsics_valid{false};
  CameraIntrinsics intrinsics;
  std::string intrinsics_failure_reason;

  // Calibration checks
  bool calibration_pass{false};
  double reprojection_error_id{0.0};
  double floor_plane_std_mm{0.0};
  double inlier_ratio{0.0};
  int detected_corners{0};
  std::string calibration_failure_reason;

  std::chrono::system_clock::time_point timestamp;
};

bool checkRealSenseConnection(QCResult& result) {
  try {
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();

    if (devices.size() == 0) {
      std::cerr << "[ERROR] No RealSense device detected\n";
      return false;
    }

    rs2::device dev = devices[0];
    result.device_connected = true;
    result.device_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    result.device_name = dev.get_info(RS2_CAMERA_INFO_NAME);
    result.firmware_version = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);

    std::cout << "[INFO] Device detected: " << result.device_name << "\n";
    std::cout << "[INFO] Serial: " << result.device_serial << "\n";
    std::cout << "[INFO] Firmware: " << result.firmware_version << "\n";

    return true;
  } catch (const rs2::error& e) {
    std::cerr << "[ERROR] RealSense error: " << e.what() << "\n";
    return false;
  }
}

bool validateIntrinsics(const CameraIntrinsics& intrinsics, const QCThresholds& thresholds,
                        std::string& failure_reason) {
  std::ostringstream oss;

  if (intrinsics.fx < thresholds.min_fx || intrinsics.fx > thresholds.max_fx) {
    oss << "fx out of range [" << thresholds.min_fx << ", " << thresholds.max_fx
        << "]: " << intrinsics.fx;
    failure_reason = oss.str();
    return false;
  }

  if (intrinsics.fy < thresholds.min_fy || intrinsics.fy > thresholds.max_fy) {
    oss << "fy out of range [" << thresholds.min_fy << ", " << thresholds.max_fy
        << "]: " << intrinsics.fy;
    failure_reason = oss.str();
    return false;
  }

  if (intrinsics.cx < thresholds.min_cx || intrinsics.cx > thresholds.max_cx) {
    oss << "cx out of range [" << thresholds.min_cx << ", " << thresholds.max_cx
        << "]: " << intrinsics.cx;
    failure_reason = oss.str();
    return false;
  }

  if (intrinsics.cy < thresholds.min_cy || intrinsics.cy > thresholds.max_cy) {
    oss << "cy out of range [" << thresholds.min_cy << ", " << thresholds.max_cy
        << "]: " << intrinsics.cy;
    failure_reason = oss.str();
    return false;
  }

  return true;
}

CalibrationConfig loadConfigFromJson(const std::filesystem::path& path) {
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

SessionConfig makeSessionConfig(const nlohmann::json& j, const CalibrationConfig& calib_config) {
  SessionConfig config;

  if (j.contains("session_attempts")) {
    config.attempts = j["session_attempts"].get<int>();
  } else {
    config.attempts = calib_config.session_attempts;
  }

  if (j.contains("max_plane_std_mm")) {
    config.max_plane_std_mm = j["max_plane_std_mm"].get<double>();
  } else {
    config.max_plane_std_mm = calib_config.max_plane_std_mm;
  }

  if (j.contains("min_inlier_ratio")) {
    config.min_inlier_ratio = j["min_inlier_ratio"].get<double>();
  } else {
    config.min_inlier_ratio = calib_config.floor_min_inlier_ratio;
  }

  if (j.contains("save_intermediate_snapshots")) {
    config.save_intermediate_snapshots = j["save_intermediate_snapshots"].get<bool>();
  }

  if (j.contains("snapshot_output_dir")) {
    config.snapshot_output_dir = j["snapshot_output_dir"].get<std::string>();
  }

  return config;
}

QCThresholds loadQCThresholds(const std::filesystem::path& path) {
  QCThresholds thresholds;

  if (!std::filesystem::exists(path)) {
    return thresholds;
  }

  try {
    nlohmann::json j;
    std::ifstream ifs(path);
    ifs >> j;

    if (j.contains("qc_thresholds")) {
      const auto& qc = j["qc_thresholds"];

      if (qc.contains("intrinsics")) {
        const auto& intr = qc["intrinsics"];
        if (intr.contains("min_fx")) thresholds.min_fx = intr["min_fx"].get<double>();
        if (intr.contains("max_fx")) thresholds.max_fx = intr["max_fx"].get<double>();
        if (intr.contains("min_fy")) thresholds.min_fy = intr["min_fy"].get<double>();
        if (intr.contains("max_fy")) thresholds.max_fy = intr["max_fy"].get<double>();
        if (intr.contains("min_cx")) thresholds.min_cx = intr["min_cx"].get<double>();
        if (intr.contains("max_cx")) thresholds.max_cx = intr["max_cx"].get<double>();
        if (intr.contains("min_cy")) thresholds.min_cy = intr["min_cy"].get<double>();
        if (intr.contains("max_cy")) thresholds.max_cy = intr["max_cy"].get<double>();
      }

      if (qc.contains("calibration")) {
        const auto& calib = qc["calibration"];
        if (calib.contains("max_reprojection_error_id")) {
          thresholds.max_reprojection_error_id = calib["max_reprojection_error_id"].get<double>();
        }
        if (calib.contains("max_plane_std_mm")) {
          thresholds.max_plane_std_mm = calib["max_plane_std_mm"].get<double>();
        }
        if (calib.contains("min_inlier_ratio")) {
          thresholds.min_inlier_ratio = calib["min_inlier_ratio"].get<double>();
        }
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "[WARN] Failed to load QC thresholds: " << e.what() << "\n";
  }

  return thresholds;
}

std::string formatTimestamp(const std::chrono::system_clock::time_point& tp) {
  std::time_t time = std::chrono::system_clock::to_time_t(tp);
  std::tm tm_buf{};
#if defined(_WIN32)
  localtime_s(&tm_buf, &time);
#else
  localtime_r(&time, &tm_buf);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
  return oss.str();
}

void generateMarkdownReport(const QCResult& result, const QCThresholds& thresholds,
                            const std::filesystem::path& output_path) {
  std::ofstream out(output_path);
  if (!out) {
    std::cerr << "[ERROR] Failed to create report file: " << output_path << "\n";
    return;
  }

  out << "# Calibration QC Report\n\n";
  out << "**Timestamp:** " << formatTimestamp(result.timestamp) << "\n\n";
  out << "**Overall Status:** " << (result.overall_pass ? "✅ PASS" : "❌ FAIL") << "\n\n";

  out << "---\n\n";
  out << "## Device Information\n\n";
  out << "| Property | Value | Status |\n";
  out << "|----------|-------|--------|\n";
  out << "| Connected | " << (result.device_connected ? "Yes" : "No") << " | "
      << (result.device_connected ? "✅" : "❌") << " |\n";

  if (result.device_connected) {
    out << "| Device Name | " << result.device_name << " | ✅ |\n";
    out << "| Serial Number | " << result.device_serial << " | ✅ |\n";
    out << "| Firmware | " << result.firmware_version << " | ✅ |\n";
  }

  out << "\n---\n\n";
  out << "## Camera Intrinsics\n\n";
  out << "| Parameter | Value | Valid Range | Status |\n";
  out << "|-----------|-------|-------------|--------|\n";

  if (result.intrinsics_valid) {
    auto check_param = [&](const std::string& name, double value, double min_val, double max_val) {
      bool pass = (value >= min_val && value <= max_val);
      out << "| " << name << " | " << std::fixed << std::setprecision(2) << value << " | ["
          << min_val << ", " << max_val << "] | " << (pass ? "✅" : "❌") << " |\n";
    };

    check_param("fx", result.intrinsics.fx, thresholds.min_fx, thresholds.max_fx);
    check_param("fy", result.intrinsics.fy, thresholds.min_fy, thresholds.max_fy);
    check_param("cx", result.intrinsics.cx, thresholds.min_cx, thresholds.max_cx);
    check_param("cy", result.intrinsics.cy, thresholds.min_cy, thresholds.max_cy);

    out << "\n**Distortion Model:** " << result.intrinsics.distortion_model << "\n\n";
    out << "**Distortion Coefficients:** [";
    for (size_t i = 0; i < result.intrinsics.distortion_coeffs.size(); ++i) {
      if (i > 0) out << ", ";
      out << std::fixed << std::setprecision(6) << result.intrinsics.distortion_coeffs[i];
    }
    out << "]\n\n";
  } else {
    out << "| Status | Failed | - | ❌ |\n\n";
    out << "**Failure Reason:** " << result.intrinsics_failure_reason << "\n\n";
  }

  out << "---\n\n";
  out << "## Calibration Results\n\n";

  if (result.calibration_pass) {
    out << "| Metric | Value | Threshold | Status |\n";
    out << "|--------|-------|-----------|--------|\n";

    bool repro_pass = result.reprojection_error_id <= thresholds.max_reprojection_error_id;
    out << "| Reprojection Error (ID units) | " << std::fixed << std::setprecision(3)
        << result.reprojection_error_id << " | ≤ " << thresholds.max_reprojection_error_id
        << " | " << (repro_pass ? "✅" : "❌") << " |\n";

    bool plane_pass = result.floor_plane_std_mm <= thresholds.max_plane_std_mm;
    out << "| Floor Plane Std Dev | " << std::fixed << std::setprecision(3)
        << result.floor_plane_std_mm << " mm | ≤ " << thresholds.max_plane_std_mm << " | "
        << (plane_pass ? "✅" : "❌") << " |\n";

    bool inlier_pass = result.inlier_ratio >= thresholds.min_inlier_ratio;
    out << "| Inlier Ratio | " << std::fixed << std::setprecision(3) << result.inlier_ratio
        << " | ≥ " << thresholds.min_inlier_ratio << " | " << (inlier_pass ? "✅" : "❌")
        << " |\n";

    out << "| Detected Corners | " << result.detected_corners << " | - | ✅ |\n";
  } else {
    out << "**Status:** ❌ FAIL\n\n";
    out << "**Failure Reason:** " << result.calibration_failure_reason << "\n\n";
  }

  out << "\n---\n\n";
  out << "## Recommendations\n\n";

  if (!result.overall_pass) {
    if (!result.device_connected) {
      out << "- ⚠️ Ensure RealSense D415 is properly connected via USB3\n";
      out << "- ⚠️ On Apple Silicon macOS, run: `sudo killall VDCAssistant AppleCameraAssistant`\n";
      out << "- ⚠️ Run the calibration tool with `sudo` privileges\n";
    }

    if (result.device_connected && !result.intrinsics_valid) {
      out << "- ⚠️ Camera intrinsics are out of expected range\n";
      out << "- ⚠️ Verify camera is not damaged or misconfigured\n";
      out << "- ⚠️ Consider factory reset of RealSense device\n";
    }

    if (result.intrinsics_valid && !result.calibration_pass) {
      out << "- ⚠️ ChArUco board detection failed or quality is insufficient\n";
      out << "- ⚠️ Ensure proper lighting conditions (avoid glare and shadows)\n";
      out << "- ⚠️ Verify ChArUco board is printed at correct scale (45mm squares)\n";
      out << "- ⚠️ Position board parallel to floor at 400-1000mm distance\n";
      out << "- ⚠️ Ensure board is fully visible in camera frame\n";
    }
  } else {
    out << "✅ All checks passed. Calibration is ready for production use.\n";
  }

  out << "\n---\n\n";
  out << "*Generated by run_calibration_qc*\n";

  out.close();
  std::cout << "[INFO] QC report saved to " << output_path << "\n";
}

void generateJsonReport(const QCResult& result, const QCThresholds& thresholds,
                        const std::filesystem::path& output_path) {
  nlohmann::json j;

  j["timestamp"] = formatTimestamp(result.timestamp);
  j["overall_pass"] = result.overall_pass;

  nlohmann::json device;
  device["connected"] = result.device_connected;
  if (result.device_connected) {
    device["name"] = result.device_name;
    device["serial"] = result.device_serial;
    device["firmware"] = result.firmware_version;
  }
  j["device"] = device;

  nlohmann::json intrinsics_json;
  intrinsics_json["valid"] = result.intrinsics_valid;
  if (result.intrinsics_valid) {
    intrinsics_json["fx"] = result.intrinsics.fx;
    intrinsics_json["fy"] = result.intrinsics.fy;
    intrinsics_json["cx"] = result.intrinsics.cx;
    intrinsics_json["cy"] = result.intrinsics.cy;
    intrinsics_json["distortion_model"] = result.intrinsics.distortion_model;
    intrinsics_json["distortion_coeffs"] = result.intrinsics.distortion_coeffs;
  } else {
    intrinsics_json["failure_reason"] = result.intrinsics_failure_reason;
  }
  j["intrinsics"] = intrinsics_json;

  nlohmann::json calibration_json;
  calibration_json["pass"] = result.calibration_pass;
  if (result.calibration_pass) {
    calibration_json["reprojection_error_id"] = result.reprojection_error_id;
    calibration_json["floor_plane_std_mm"] = result.floor_plane_std_mm;
    calibration_json["inlier_ratio"] = result.inlier_ratio;
    calibration_json["detected_corners"] = result.detected_corners;

    nlohmann::json checks;
    checks["reprojection_error"] =
        (result.reprojection_error_id <= thresholds.max_reprojection_error_id) ? "PASS" : "FAIL";
    checks["floor_plane_std"] =
        (result.floor_plane_std_mm <= thresholds.max_plane_std_mm) ? "PASS" : "FAIL";
    checks["inlier_ratio"] = (result.inlier_ratio >= thresholds.min_inlier_ratio) ? "PASS" : "FAIL";
    calibration_json["checks"] = checks;
  } else {
    calibration_json["failure_reason"] = result.calibration_failure_reason;
  }
  j["calibration"] = calibration_json;

  std::ofstream out(output_path);
  if (out) {
    out << j.dump(2);
    out.close();
    std::cout << "[INFO] QC JSON report saved to " << output_path << "\n";
  } else {
    std::cerr << "[ERROR] Failed to create JSON report: " << output_path << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path config_path = "calibration_config.json";
  std::filesystem::path report_path = "qc_report.md";
  std::filesystem::path json_report_path = "qc_report.json";
  std::filesystem::path calib_output_path = "qc_calib_result.json";

  if (argc > 1) {
    config_path = argv[1];
  }
  if (argc > 2) {
    report_path = argv[2];
  }
  if (argc > 3) {
    json_report_path = argv[3];
  }

  std::cout << "=== RealSense D415 Calibration QC ===\n\n";

  QCResult qc_result;
  qc_result.timestamp = std::chrono::system_clock::now();

  // Step 1: Check RealSense connection
  std::cout << "[1/4] Checking RealSense device connection...\n";
  if (!checkRealSenseConnection(qc_result)) {
    qc_result.overall_pass = false;
    generateMarkdownReport(qc_result, QCThresholds{}, report_path);
    generateJsonReport(qc_result, QCThresholds{}, json_report_path);
    return 1;
  }

  // Step 2: Load configuration and thresholds
  std::cout << "[2/4] Loading calibration configuration...\n";
  CalibrationConfig calib_config = loadConfigFromJson(config_path);
  QCThresholds thresholds = loadQCThresholds(config_path);

  nlohmann::json config_json;
  if (std::filesystem::exists(config_path)) {
    std::ifstream ifs(config_path);
    ifs >> config_json;
  }
  SessionConfig session_config = makeSessionConfig(config_json, calib_config);

  // Step 3: Run calibration
  std::cout << "[3/4] Running calibration session...\n";
  // Extract config file directory for path resolution
  std::filesystem::path config_dir;
  if (std::filesystem::exists(config_path)) {
    config_dir = std::filesystem::absolute(config_path).parent_path();
  } else {
    config_dir = std::filesystem::current_path();
  }
  
  CalibrationPipeline pipeline(calib_config);
  if (!pipeline.initialize(config_dir.string())) {
    qc_result.overall_pass = false;
    qc_result.calibration_pass = false;
    qc_result.calibration_failure_reason = "Failed to initialize calibration pipeline";
    generateMarkdownReport(qc_result, thresholds, report_path);
    generateJsonReport(qc_result, thresholds, json_report_path);
    return 1;
  }
  CalibrationSession session(std::move(pipeline), session_config);
  auto result = session.Run();

  if (!result) {
    qc_result.overall_pass = false;
    qc_result.calibration_pass = false;
    qc_result.calibration_failure_reason = "Calibration session failed to produce valid result";
    generateMarkdownReport(qc_result, thresholds, report_path);
    generateJsonReport(qc_result, thresholds, json_report_path);
    return 1;
  }

  // Step 4: Validate results
  std::cout << "[4/4] Validating calibration results...\n";

  // Validate intrinsics
  qc_result.intrinsics = result->intrinsics;
  qc_result.intrinsics_valid =
      validateIntrinsics(result->intrinsics, thresholds, qc_result.intrinsics_failure_reason);

  // Validate calibration metrics
  qc_result.reprojection_error_id = result->reprojection_error_toio;
  qc_result.floor_plane_std_mm = result->floor_plane_std_mm;
  qc_result.inlier_ratio = result->inlier_ratio;
  qc_result.detected_corners = result->detected_charuco_corners;

  bool repro_pass = result->reprojection_error_toio <= thresholds.max_reprojection_error_id;
  bool plane_pass = result->floor_plane_std_mm <= thresholds.max_plane_std_mm;
  bool inlier_pass = result->inlier_ratio >= thresholds.min_inlier_ratio;

  qc_result.calibration_pass = repro_pass && plane_pass && inlier_pass;

  if (!qc_result.calibration_pass) {
    std::ostringstream oss;
    if (!repro_pass) {
      oss << "Reprojection error too high (" << result->reprojection_error_toio << " > "
          << thresholds.max_reprojection_error_id << "). ";
    }
    if (!plane_pass) {
      oss << "Floor plane std dev too high (" << result->floor_plane_std_mm << " > "
          << thresholds.max_plane_std_mm << "). ";
    }
    if (!inlier_pass) {
      oss << "Inlier ratio too low (" << result->inlier_ratio << " < "
          << thresholds.min_inlier_ratio << "). ";
    }
    qc_result.calibration_failure_reason = oss.str();
  }

  qc_result.overall_pass =
      qc_result.device_connected && qc_result.intrinsics_valid && qc_result.calibration_pass;

  // Save calibration result
  session.SaveResultJson(*result, calib_output_path.string());
  std::cout << "[INFO] Calibration result saved to " << calib_output_path << "\n";

  // Generate reports
  generateMarkdownReport(qc_result, thresholds, report_path);
  generateJsonReport(qc_result, thresholds, json_report_path);

  std::cout << "\n=== QC Summary ===\n";
  std::cout << "Overall Status: " << (qc_result.overall_pass ? "✅ PASS" : "❌ FAIL") << "\n";
  std::cout << "Device Connected: " << (qc_result.device_connected ? "✅" : "❌") << "\n";
  std::cout << "Intrinsics Valid: " << (qc_result.intrinsics_valid ? "✅" : "❌") << "\n";
  std::cout << "Calibration Pass: " << (qc_result.calibration_pass ? "✅" : "❌") << "\n";

  return qc_result.overall_pass ? 0 : 1;
}
