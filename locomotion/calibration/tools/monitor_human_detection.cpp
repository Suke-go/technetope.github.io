#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "locomotion/calibration/CalibrationPipeline.h"
#include "locomotion/calibration/CalibrationResult.h"
#include "locomotion/calibration/HumanDetector.h"
#include "locomotion/calibration/PlaymatLayout.h"

using namespace locomotion::calibration;

namespace {

// CalibrationResultからCalibrationSnapshotへの変換
CalibrationSnapshot ResultToSnapshot(const CalibrationResult& result) {
  CalibrationSnapshot snapshot;
  snapshot.intrinsics = result.intrinsics;
  snapshot.homography_color_to_floor = result.homography_color_to_floor.clone();
  snapshot.homography_color_to_toio = result.homography_color_to_toio.clone();
  snapshot.toio_transform = result.toio_transform;
  snapshot.floor_plane = result.floor_plane;
  snapshot.reprojection_error_px = result.reprojection_error_px;
  snapshot.reprojection_error_floor_mm = result.reprojection_error_floor_mm;
  snapshot.reprojection_error_toio = result.reprojection_error_toio;
  snapshot.floor_plane_std_mm = result.floor_plane_std_mm;
  snapshot.inlier_ratio = result.inlier_ratio;
  snapshot.floor_inlier_count = result.floor_inlier_count;
  snapshot.camera_height_mm = result.camera_height_mm;
  snapshot.detected_charuco_corners = result.detected_charuco_corners;
  snapshot.timestamp = result.timestamp;
  return snapshot;
}

// HumanDetectionConfigの読み込み
HumanDetectionConfig loadHumanDetectionConfig(const nlohmann::json& j) {
  HumanDetectionConfig config;
  
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
  auto load_bool = [&j](const char* key, bool& dst) {
    if (j.contains(key)) {
      dst = j[key].get<bool>();
    }
  };
  
  // Coarse-to-fine設定
  load_int("coarse_scale_factor", config.coarse_scale_factor);
  load_int("fine_roi_margin_px", config.fine_roi_margin_px);
  
  // 深度フィルタ
  load_double("min_depth_mm", config.min_depth_mm);
  load_double("max_depth_mm", config.max_depth_mm);
  load_double("foot_depth_min_mm", config.foot_depth_min_mm);
  load_double("foot_depth_max_mm", config.foot_depth_max_mm);
  load_double("head_depth_min_mm", config.head_depth_min_mm);
  load_double("head_depth_max_mm", config.head_depth_max_mm);
  
  // サイズフィルタ
  load_double("min_person_height_mm", config.min_person_height_mm);
  load_double("max_person_height_mm", config.max_person_height_mm);
  load_double("min_foot_diameter_mm", config.min_foot_diameter_mm);
  load_double("max_foot_diameter_mm", config.max_foot_diameter_mm);
  load_int("min_foot_area_pixels", config.min_foot_area_pixels);
  
  // 楕円推定
  load_double("ellipse_aspect_ratio", config.ellipse_aspect_ratio);
  load_double("ellipse_vertical_offset", config.ellipse_vertical_offset);
  
  // 動き判定
  load_double("movement_threshold_mm", config.movement_threshold_mm);
  load_int("movement_history_frames", config.movement_history_frames);
  load_double("standing_velocity_threshold_mm_per_s", config.standing_velocity_threshold_mm_per_s);
  
  // 時間重み付き速度計算
  load_double("velocity_alpha", config.velocity_alpha);
  load_double("velocity_half_life_seconds", config.velocity_half_life_seconds);
  load_double("position_smoothing_alpha", config.position_smoothing_alpha);
  
  // ノイズ対策
  load_double("noise_threshold_mm", config.noise_threshold_mm);
  load_double("outlier_threshold_std", config.outlier_threshold_std);
  load_bool("enable_outlier_rejection", config.enable_outlier_rejection);
  
  // トラッキング
  load_double("tracking_max_distance_mm", config.tracking_max_distance_mm);
  load_int("max_tracked_humans", config.max_tracked_humans);
  load_double("tracking_timeout_seconds", config.tracking_timeout_seconds);
  
  // パフォーマンス
  load_bool("enable_coarse_to_fine", config.enable_coarse_to_fine);
  load_int("skip_frames", config.skip_frames);
  
  // モルフォロジー処理
  load_int("morphology_kernel_size", config.morphology_kernel_size);
  load_int("morphology_iterations", config.morphology_iterations);
  
  // フォールバック設定
  load_double("head_to_foot_search_radius_mm", config.head_to_foot_search_radius_mm);
  load_double("average_person_height_mm", config.average_person_height_mm);
  
  return config;
}

// JSONからcv::Matを読み込む（行の配列形式）
cv::Mat jsonToMat(const nlohmann::json& j) {
  if (!j.is_array() || j.empty()) {
    return cv::Mat();
  }
  int rows = static_cast<int>(j.size());
  int cols = static_cast<int>(j[0].size());
  cv::Mat mat(rows, cols, CV_64F);
  for (int r = 0; r < rows; ++r) {
    if (!j[r].is_array() || static_cast<int>(j[r].size()) != cols) {
      return cv::Mat();
    }
    for (int c = 0; c < cols; ++c) {
      mat.at<double>(r, c) = j[r][c].get<double>();
    }
  }
  return mat;
}

// CalibrationResultの読み込み
CalibrationResult loadCalibrationResult(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("Calibration result file not found: " + path.string());
  }
  
  nlohmann::json j;
  std::ifstream ifs(path);
  ifs >> j;
  
  CalibrationResult result;
  
  // intrinsics
  if (j.contains("intrinsics")) {
    const auto& intrinsics = j["intrinsics"];
    if (intrinsics.contains("fx")) result.intrinsics.fx = intrinsics["fx"].get<double>();
    if (intrinsics.contains("fy")) result.intrinsics.fy = intrinsics["fy"].get<double>();
    if (intrinsics.contains("cx")) result.intrinsics.cx = intrinsics["cx"].get<double>();
    if (intrinsics.contains("cy")) result.intrinsics.cy = intrinsics["cy"].get<double>();
    if (intrinsics.contains("distortion_model")) {
      result.intrinsics.distortion_model = intrinsics["distortion_model"].get<std::string>();
    }
    if (intrinsics.contains("distortion_coeffs")) {
      const auto& coeffs = intrinsics["distortion_coeffs"];
      if (coeffs.is_array()) {
        result.intrinsics.distortion_coeffs.clear();
        for (const auto& coeff : coeffs) {
          result.intrinsics.distortion_coeffs.push_back(coeff.get<double>());
        }
      }
    }
  }
  
  // homography matrices
  if (j.contains("homography_color_to_floor")) {
    result.homography_color_to_floor = jsonToMat(j["homography_color_to_floor"]);
  }
  
  // toio_coordinate_transform
  if (j.contains("toio_coordinate_transform")) {
    const auto& transform = j["toio_coordinate_transform"];
    if (transform.contains("transform_color_to_toio")) {
      result.toio_transform.color_to_toio = jsonToMat(transform["transform_color_to_toio"]);
      // homography_color_to_toioも同じ値を使用
      result.homography_color_to_toio = result.toio_transform.color_to_toio.clone();
    }
    if (transform.contains("playmat_id")) {
      result.toio_transform.playmat_id = transform["playmat_id"].get<std::string>();
    }
    if (transform.contains("board_mount_label")) {
      result.toio_transform.mount_label = transform["board_mount_label"].get<std::string>();
    }
    if (transform.contains("coverage_area_toio_id")) {
      const auto& area = transform["coverage_area_toio_id"];
      if (area.contains("min") && area.contains("max")) {
        double min_x = area["min"]["x"].get<double>();
        double min_y = area["min"]["y"].get<double>();
        double max_x = area["max"]["x"].get<double>();
        double max_y = area["max"]["y"].get<double>();
        result.toio_transform.coverage_area = cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
      }
    }
    if (transform.contains("transform_error_id")) {
      if (!transform["transform_error_id"].is_null()) {
        result.toio_transform.transform_error_id = transform["transform_error_id"].get<double>();
      }
    }
  }
  
  // floor_plane
  if (j.contains("floor_plane")) {
    const auto& floor = j["floor_plane"];
    if (floor.contains("coefficients")) {
      const auto& coeffs = floor["coefficients"];
      if (coeffs.is_array() && coeffs.size() == 4) {
        result.floor_plane = cv::Vec4f(
          coeffs[0].get<float>(),
          coeffs[1].get<float>(),
          coeffs[2].get<float>(),
          coeffs[3].get<float>()
        );
      }
    }
    if (floor.contains("std_mm")) {
      result.floor_plane_std_mm = floor["std_mm"].get<double>();
    }
    if (floor.contains("inlier_ratio")) {
      result.inlier_ratio = floor["inlier_ratio"].get<double>();
    }
    if (floor.contains("inlier_count")) {
      result.floor_inlier_count = floor["inlier_count"].get<int>();
    }
    if (floor.contains("camera_height_mm")) {
      result.camera_height_mm = floor["camera_height_mm"].get<double>();
    }
  }
  
  // quality_metrics
  if (j.contains("quality_metrics")) {
    const auto& quality = j["quality_metrics"];
    if (quality.contains("charuco_corners")) {
      result.detected_charuco_corners = quality["charuco_corners"].get<int>();
    }
    if (quality.contains("reprojection_error_px")) {
      result.reprojection_error_px = quality["reprojection_error_px"].get<double>();
    }
    if (quality.contains("reprojection_error_id")) {
      result.reprojection_error_toio = quality["reprojection_error_id"].get<double>();
    }
    if (quality.contains("reprojection_error_floor_mm")) {
      result.reprojection_error_floor_mm = quality["reprojection_error_floor_mm"].get<double>();
    }
    if (quality.contains("capture_count")) {
      result.capture_count = quality["capture_count"].get<int>();
    }
  }
  
  return result;
}

// toio座標範囲の取得
cv::Rect2d getToioCoordinateRange(const CalibrationResult& result) {
  if (result.toio_transform.coverage_area.width > 0.0 && 
      result.toio_transform.coverage_area.height > 0.0) {
    return result.toio_transform.coverage_area;
  }
  // デフォルト範囲（A3 Simple Playmat）
  return cv::Rect2d(34.0, 35.0, 305.0, 215.0);
}

// パネル1: 生画像の描画
cv::Mat drawRawImage(const cv::Mat& color_image) {
  return color_image.clone();
}

// パネル2: 人間検出画像（検出結果オーバーレイ）
cv::Mat drawHumanDetectionOverlay(const cv::Mat& color_image,
                                   const HumanDetectionResult& detection_result) {
  cv::Mat overlay = color_image.clone();
  
  for (const auto& human : detection_result.humans) {
    // 頭位置（緑の円）
    if (human.head_position_px.x > 0 && human.head_position_px.y > 0) {
      cv::circle(overlay, human.head_position_px, 5, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
    
    // 足位置（青の円）
    if (human.foot_center_px.x > 0 && human.foot_center_px.y > 0) {
      cv::circle(overlay, human.foot_center_px, 8, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    }
    
    // 楕円領域（黄色の楕円）
    if (human.ellipse_px.size.width > 0 && human.ellipse_px.size.height > 0) {
      cv::ellipse(overlay, human.ellipse_px, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }
    
    // トラッキングID
    if (human.head_position_px.x > 0 && human.head_position_px.y > 0) {
      std::string id_text = "ID:" + std::to_string(human.id);
      cv::putText(overlay, id_text,
                  cv::Point(static_cast<int>(human.head_position_px.x) + 10,
                           static_cast<int>(human.head_position_px.y) - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    }
    
    // 速度ベクトル（矢印）- スケール調整
    if (human.foot_center_px.x > 0 && human.foot_center_px.y > 0) {
      float velocity_scale = 0.05f;  // 速度表示のスケール（調整可能）
      float velocity_magnitude = std::sqrt(
        human.velocity_mm_per_s.x * human.velocity_mm_per_s.x +
        human.velocity_mm_per_s.y * human.velocity_mm_per_s.y
      );
      if (velocity_magnitude > 1.0f) {
        cv::Point2f velocity_end(
          human.foot_center_px.x + human.velocity_mm_per_s.x * velocity_scale,
          human.foot_center_px.y + human.velocity_mm_per_s.y * velocity_scale
        );
        cv::arrowedLine(overlay, human.foot_center_px, velocity_end,
                       cv::Scalar(255, 0, 255), 2, cv::LINE_AA, 0, 0.3);
      }
    }
    
    // 動き状態
    std::string state_text;
    cv::Scalar state_color;
    switch (human.motion_state) {
      case HumanMotionState::STANDING:
        state_text = "STANDING";
        state_color = cv::Scalar(0, 255, 0);
        break;
      case HumanMotionState::MOVING:
        state_text = "MOVING";
        state_color = cv::Scalar(0, 0, 255);
        break;
      default:
        state_text = "UNKNOWN";
        state_color = cv::Scalar(128, 128, 128);
        break;
    }
    
    if (human.foot_center_px.x > 0 && human.foot_center_px.y > 0) {
      cv::putText(overlay, state_text,
                  cv::Point(static_cast<int>(human.foot_center_px.x) + 10,
                           static_cast<int>(human.foot_center_px.y) + 25),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, state_color, 2, cv::LINE_AA);
    }
    
    // 信頼度
    std::string conf_text = "Conf: " + std::to_string(static_cast<int>(human.confidence * 100)) + "%";
    if (human.foot_center_px.x > 0 && human.foot_center_px.y > 0) {
      cv::putText(overlay, conf_text,
                  cv::Point(static_cast<int>(human.foot_center_px.x) + 10,
                           static_cast<int>(human.foot_center_px.y) + 45),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
  }
  
  return overlay;
}

// パネル3: toio座標上の可視化
cv::Mat drawToioCoordinateView(const HumanDetectionResult& detection_result,
                                const cv::Rect2d& toio_range,
                                int panel_width,
                                int panel_height) {
  cv::Mat panel(panel_height, panel_width, CV_8UC3, cv::Scalar(30, 30, 30));
  
  // スケーリング計算
  double scale_x = static_cast<double>(panel_width - 40) / toio_range.width;
  double scale_y = static_cast<double>(panel_height - 40) / toio_range.height;
  double scale = std::min(scale_x, scale_y);
  
  // オフセット計算（中央配置）
  double offset_x = (panel_width - toio_range.width * scale) / 2.0;
  double offset_y = (panel_height - toio_range.height * scale) / 2.0;
  
  // toio座標を画像座標に変換
  auto toioToImage = [&](cv::Point2f toio_pos) -> cv::Point2f {
    return cv::Point2f(
      static_cast<float>((toio_pos.x - toio_range.x) * scale + offset_x),
      static_cast<float>((toio_pos.y - toio_range.y) * scale + offset_y)
    );
  };
  
  // グリッド描画（オプション）
  const int grid_step = 50;
  for (int x = static_cast<int>(toio_range.x); x <= static_cast<int>(toio_range.x + toio_range.width); x += grid_step) {
    cv::Point2f p1 = toioToImage(cv::Point2f(static_cast<float>(x), static_cast<float>(toio_range.y)));
    cv::Point2f p2 = toioToImage(cv::Point2f(static_cast<float>(x), static_cast<float>(toio_range.y + toio_range.height)));
    cv::line(panel, p1, p2, cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
  }
  for (int y = static_cast<int>(toio_range.y); y <= static_cast<int>(toio_range.y + toio_range.height); y += grid_step) {
    cv::Point2f p1 = toioToImage(cv::Point2f(static_cast<float>(toio_range.x), static_cast<float>(y)));
    cv::Point2f p2 = toioToImage(cv::Point2f(static_cast<float>(toio_range.x + toio_range.width), static_cast<float>(y)));
    cv::line(panel, p1, p2, cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
  }
  
  // 人間の位置を描画
  for (const auto& human : detection_result.humans) {
    cv::Point2f pos_img = toioToImage(human.toio_position);
    
    // 動き状態に応じた色
    cv::Scalar human_color;
    switch (human.motion_state) {
      case HumanMotionState::STANDING:
        human_color = cv::Scalar(0, 255, 0);  // 緑
        break;
      case HumanMotionState::MOVING:
        human_color = cv::Scalar(0, 0, 255);  // 赤
        break;
      default:
        human_color = cv::Scalar(128, 128, 128);  // 灰
        break;
    }
    
    // 人間の位置（円）
    cv::circle(panel, pos_img, 10, human_color, -1, cv::LINE_AA);
    cv::circle(panel, pos_img, 10, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    
    // 楕円領域（toio座標系）
    if (human.ellipse_toio.size.width > 0 && human.ellipse_toio.size.height > 0) {
      cv::Point2f ellipse_center_img = toioToImage(human.ellipse_toio.center);
      cv::Size2f ellipse_size_img(
        static_cast<float>(human.ellipse_toio.size.width * scale),
        static_cast<float>(human.ellipse_toio.size.height * scale)
      );
      cv::RotatedRect ellipse_img(ellipse_center_img, ellipse_size_img, human.ellipse_toio.angle);
      cv::ellipse(panel, ellipse_img, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
    }
    
    // 速度ベクトル（矢印）- スケール調整
    float velocity_magnitude = std::sqrt(
      human.velocity_mm_per_s.x * human.velocity_mm_per_s.x +
      human.velocity_mm_per_s.y * human.velocity_mm_per_s.y
    );
    if (velocity_magnitude > 1.0f) {
      float velocity_scale = 0.1f;  // toio座標系での速度表示スケール
      cv::Point2f velocity_end(
        pos_img.x + human.velocity_mm_per_s.x * scale * velocity_scale * 0.001f,
        pos_img.y + human.velocity_mm_per_s.y * scale * velocity_scale * 0.001f
      );
      cv::arrowedLine(panel, pos_img, velocity_end, cv::Scalar(255, 0, 255), 2, cv::LINE_AA, 0, 0.3);
    }
    
    // トラッキングID
    std::string id_text = std::to_string(human.id);
    cv::putText(panel, id_text,
                cv::Point(static_cast<int>(pos_img.x) + 15, static_cast<int>(pos_img.y) - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
  }
  
  // 統計情報を表示（左上）
  std::vector<std::string> stats;
  stats.push_back("Detections: " + std::to_string(detection_result.humans.size()));
  if (detection_result.processing_time_ms > 0.0) {
    double fps = 1000.0 / detection_result.processing_time_ms;
    stats.push_back("FPS: " + std::to_string(static_cast<int>(fps)));
    stats.push_back("Time: " + std::to_string(static_cast<int>(detection_result.processing_time_ms)) + "ms");
  }
  
  // 背景付きテキストブロック
  int padding = 6;
  int line_height = 18;
  int block_height = static_cast<int>(stats.size()) * line_height + padding * 2;
  int block_width = 180;
  cv::Rect stats_rect(10, 10, block_width, block_height);
  cv::Mat overlay = panel.clone();
  cv::rectangle(overlay, stats_rect, cv::Scalar(0, 0, 0), -1);
  cv::addWeighted(overlay, 0.7, panel, 0.3, 0.0, panel);
  
  int y_offset = 10 + padding + 14;
  for (const auto& stat : stats) {
    cv::putText(panel, stat, cv::Point(10 + padding, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    y_offset += line_height;
  }
  
  // 座標範囲のラベル（左下）
  std::string range_text = "Range: (" + 
    std::to_string(static_cast<int>(toio_range.x)) + "," +
    std::to_string(static_cast<int>(toio_range.y)) + ")-(" +
    std::to_string(static_cast<int>(toio_range.x + toio_range.width)) + "," +
    std::to_string(static_cast<int>(toio_range.y + toio_range.height)) + ")";
  cv::putText(panel, range_text, cv::Point(10, panel_height - 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
  
  return panel;
}

// 3パネルを結合
cv::Mat composeThreePanels(const cv::Mat& panel1, const cv::Mat& panel2, const cv::Mat& panel3) {
  const int gap = 12;
  int panel_width = panel1.cols;
  int panel_height = panel1.rows;
  
  // パネル3のサイズを他のパネルに合わせる
  cv::Mat panel3_resized;
  cv::resize(panel3, panel3_resized, cv::Size(panel_width, panel_height), 0, 0, cv::INTER_LINEAR);
  
  int canvas_width = panel_width * 3 + gap * 2;
  int canvas_height = panel_height;
  cv::Mat canvas(canvas_height, canvas_width, CV_8UC3, cv::Scalar(20, 20, 20));
  
  panel1.copyTo(canvas(cv::Rect(0, 0, panel_width, panel_height)));
  panel2.copyTo(canvas(cv::Rect(panel_width + gap, 0, panel_width, panel_height)));
  panel3_resized.copyTo(canvas(cv::Rect((panel_width + gap) * 2, 0, panel_width, panel_height)));
  
  return canvas;
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path config_path;
  std::filesystem::path calibration_result_path;
  
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <calibration_result.json> [config.json]\n";
    return 1;
  }
  
  calibration_result_path = argv[1];
  if (argc > 2) {
    config_path = argv[2];
  }
  
  try {
    // キャリブレーション結果の読み込み
    CalibrationResult calibration_result = loadCalibrationResult(calibration_result_path);
    CalibrationSnapshot snapshot = ResultToSnapshot(calibration_result);
    
    // 設定ファイルの読み込み
    HumanDetectionConfig detector_config;
    if (!config_path.empty() && std::filesystem::exists(config_path)) {
      nlohmann::json config_json;
      std::ifstream ifs(config_path);
      ifs >> config_json;
      
      if (config_json.contains("human_detection")) {
        detector_config = loadHumanDetectionConfig(config_json["human_detection"]);
      }
    }
    
    // CalibrationPipelineの初期化
    CalibrationConfig pipeline_config;
    CalibrationPipeline pipeline(pipeline_config);
    
    std::filesystem::path config_dir = std::filesystem::absolute(calibration_result_path).parent_path();
    if (!pipeline.initialize(config_dir.string())) {
      std::cerr << "Failed to initialize CalibrationPipeline\n";
      return 1;
    }
    
    // HumanDetectorの初期化
    HumanDetector detector(detector_config);
    
    // toio座標範囲の取得
    cv::Rect2d toio_range = getToioCoordinateRange(calibration_result);
    
    // メインループ
    const std::string window_title = "Human Detection Monitor";
    cv::namedWindow(window_title, cv::WINDOW_AUTOSIZE);
    
    bool running = true;
    bool detecting = true;
    bool show_depth = false;
    
    double fps_estimate = 0.0;
    auto last_frame_tp = std::chrono::steady_clock::now();
    
    HumanDetectionResult last_detection_result;
    
    while (running) {
      FrameBundle frame;
      if (!pipeline.CaptureFrame(frame)) {
        cv::Mat error_img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::putText(error_img, "Camera capture failed. Check RealSense connection.",
                    cv::Point(20, error_img.rows / 2), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        cv::imshow(window_title, error_img);
        int key = cv::waitKey(10);
        if (key == 27 || key == 'q' || key == 'Q') {
          break;
        }
        continue;
      }
      
      auto now = std::chrono::steady_clock::now();
      double delta_s = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_frame_tp).count();
      last_frame_tp = now;
      if (delta_s > 0.0) {
        double inst_fps = 1.0 / delta_s;
        fps_estimate = fps_estimate == 0.0 ? inst_fps : 0.9 * fps_estimate + 0.1 * inst_fps;
      }
      
      // 人間検出の実行
      HumanDetectionResult detection_result;
      if (detecting) {
        detection_result = detector.Detect(frame, snapshot);
        last_detection_result = detection_result;
      } else {
        detection_result = last_detection_result;
      }
      
      // 3つのパネルを準備
      cv::Mat panel1 = drawRawImage(frame.color);
      cv::Mat panel2 = drawHumanDetectionOverlay(frame.color, detection_result);
      cv::Mat panel3 = drawToioCoordinateView(detection_result, toio_range, frame.color.cols, frame.color.rows);
      
      // 3パネルを結合
      cv::Mat canvas = composeThreePanels(panel1, panel2, panel3);
      
      // ステータス情報を表示（パネル1の上にオーバーレイ）
      std::vector<std::string> status_lines;
      status_lines.push_back("Status: " + std::string(detecting ? "DETECTING" : "PAUSED"));
      status_lines.push_back("FPS: " + std::to_string(static_cast<int>(fps_estimate)));
      status_lines.push_back("Humans: " + std::to_string(detection_result.humans.size()));
      if (detection_result.processing_time_ms > 0.0) {
        status_lines.push_back("Processing: " + std::to_string(static_cast<int>(detection_result.processing_time_ms)) + "ms");
      }
      
      // 背景付きテキストブロックを描画
      int padding = 8;
      int line_height = 20;
      int block_height = static_cast<int>(status_lines.size()) * line_height + padding * 2;
      int block_width = 200;
      cv::Rect status_rect(10, 10, block_width, block_height);
      cv::Mat overlay = canvas.clone();
      cv::rectangle(overlay, status_rect, cv::Scalar(0, 0, 0), -1);
      cv::addWeighted(overlay, 0.7, canvas, 0.3, 0.0, canvas);
      
      int y = 10 + padding + 15;
      for (const auto& line : status_lines) {
        cv::putText(canvas, line, cv::Point(10 + padding, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        y += line_height;
      }
      
      // 操作説明（右下に表示）
      std::vector<std::string> instructions;
      instructions.push_back("CONTROLS:");
      instructions.push_back("[SPACE] Start/Stop  [R] Reset  [Q/ESC] Quit");
      
      int inst_y = canvas.rows - 10 - static_cast<int>(instructions.size()) * 18;
      int inst_x = canvas.cols - 250;
      for (const auto& inst : instructions) {
        cv::putText(canvas, inst, cv::Point(inst_x, inst_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        inst_y += 18;
      }
      
      cv::imshow(window_title, canvas);
      
      // キーボード入力処理
      int key = cv::waitKey(1);
      if (key >= 0) {
        switch (key) {
          case 27:  // ESC
          case 'q':
          case 'Q':
            running = false;
            break;
          case ' ':  // SPACE
            detecting = !detecting;
            break;
          case 'r':
          case 'R':
            detector.Reset();
            last_detection_result = HumanDetectionResult();
            break;
          case 'd':
          case 'D':
            show_depth = !show_depth;
            break;
          default:
            break;
        }
      }
    }
    
    cv::destroyWindow(window_title);
    return 0;
    
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}

