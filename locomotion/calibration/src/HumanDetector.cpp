#include "locomotion/calibration/HumanDetector.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include <opencv2/imgproc.hpp>

#include "locomotion/calibration/ToioCoordinateTransform.h"

namespace locomotion::calibration {

namespace {

constexpr double kEpsilon = 1e-6;
constexpr double kPixelsToMmApprox = 2.0;  // 簡易的なピクセル→mm変換係数（概算）
constexpr double kLn2 = 0.6931471805599453;  // ln(2)

}  // namespace

HumanDetector::HumanDetector(HumanDetectionConfig config) : config_(config) {
  // モルフォロジーカーネルの初期化
  morphology_kernel_ = cv::getStructuringElement(
      cv::MORPH_ELLIPSE,
      cv::Size(config_.morphology_kernel_size, config_.morphology_kernel_size));
}

void HumanDetector::SetConfig(HumanDetectionConfig config) {
  config_ = config;
  morphology_kernel_ = cv::getStructuringElement(
      cv::MORPH_ELLIPSE,
      cv::Size(config_.morphology_kernel_size, config_.morphology_kernel_size));
}

const HumanDetectionConfig& HumanDetector::GetConfig() const noexcept {
  return config_;
}

void HumanDetector::Reset() {
  tracked_humans_.clear();
  next_tracking_id_ = 1;
  frame_counter_ = 0;
}

HumanDetectionResult HumanDetector::Detect(const FrameBundle& frame,
                                           const CalibrationSnapshot& calibration) {
  auto start_time = std::chrono::steady_clock::now();
  HumanDetectionResult result;
  result.timestamp = std::chrono::system_clock::now();

  // フレームスキップのチェック
  frame_counter_++;
  if (config_.skip_frames > 0 && (frame_counter_ - 1) % (config_.skip_frames + 1) != 0) {
    // 前回の検出結果を再利用（位置を予測）
    for (auto& human : tracked_humans_) {
      HumanDetection detection;
      detection.id = human.id;
      detection.toio_position = human.last_position;
      detection.motion_state = human.last_motion_state;
      detection.head_position_px = cv::Point2f(0, 0);  // スキップ時は未計算
      detection.foot_center_px = cv::Point2f(0, 0);
      detection.confidence = 0.5;  // 低い信頼度
      detection.timestamp = result.timestamp;
      result.humans.push_back(detection);
    }
    result.has_valid_detections = !result.humans.empty();
    auto end_time = std::chrono::steady_clock::now();
    result.processing_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time).count();
    return result;
  }

  if (frame.color.empty() || frame.depth.empty()) {
    spdlog::warn("HumanDetector: Empty frame received");
    result.has_valid_detections = false;
    return result;
  }

  if (calibration.homography_color_to_toio.empty()) {
    spdlog::warn("HumanDetector: Invalid calibration (empty homography)");
    result.has_valid_detections = false;
    return result;
  }

  std::vector<cv::Rect> coarse_rois;

  // Step 1: Coarse Detection (低解像度)
  if (config_.enable_coarse_to_fine) {
    // 画像を縮小
    cv::Size coarse_size(frame.color.cols / config_.coarse_scale_factor,
                         frame.color.rows / config_.coarse_scale_factor);
    cv::resize(frame.color, color_resized_, coarse_size, cv::INTER_AREA);
    cv::resize(frame.depth, depth_resized_, coarse_size, cv::INTER_NEAREST);

    coarse_rois = CoarseDetect(color_resized_, depth_resized_);

    // ROIを元の解像度にスケールアップ
    for (auto& roi : coarse_rois) {
      roi.x *= config_.coarse_scale_factor;
      roi.y *= config_.coarse_scale_factor;
      roi.width *= config_.coarse_scale_factor;
      roi.height *= config_.coarse_scale_factor;

      // マージンを追加
      roi.x = std::max(0, roi.x - config_.fine_roi_margin_px);
      roi.y = std::max(0, roi.y - config_.fine_roi_margin_px);
      roi.width = std::min(frame.color.cols - roi.x,
                           roi.width + 2 * config_.fine_roi_margin_px);
      roi.height = std::min(frame.color.rows - roi.y,
                            roi.height + 2 * config_.fine_roi_margin_px);
    }
  } else {
    // 全画像を処理
    cv::Rect full_roi(0, 0, frame.color.cols, frame.color.rows);
    coarse_rois.push_back(full_roi);
  }

  // Step 2: Fine Detection (高解像度、ROI内)
  std::vector<HumanDetection> detections =
      FineDetect(frame, coarse_rois, calibration);

  // Step 3: 座標変換と動き判定
  double delta_time_seconds = 1.0 / 30.0;  // デフォルト30 FPS（実際には前回からの経過時間を使用）
  auto current_time = std::chrono::system_clock::now();

  for (auto& detection : detections) {
    // toio座標への変換
    detection.toio_position = ::locomotion::calibration::TransformImagePixelToToio(
        detection.foot_center_px, calibration.homography_color_to_toio);

    // トラッキングIDの割り当て（前回の情報を活用するため、先に割り当てる）
    detection.id = AssignTrackingId(detection.toio_position);

    // トラッキング情報がある場合は、フォールバック処理を試みる
    auto track_it = std::find_if(tracked_humans_.begin(), tracked_humans_.end(),
                                  [detection](const TrackingState& state) {
                                    return state.id == detection.id;
                                  });
    
    // 検出が不完全な場合（信頼度が低い場合）、トラッキング情報を使って改善
    if (track_it != tracked_humans_.end() && detection.confidence < 0.6) {
      // 前回の位置情報がある場合は、それを優先的に使用
      if (track_it->last_foot_position_px.x > 0 && 
          track_it->last_motion_state == HumanMotionState::STANDING) {
        // 静止中の場合は前回の位置を使用
        detection.foot_center_px = track_it->last_foot_position_px;
        detection.head_position_px = track_it->last_head_position_px;
        detection.height_above_floor_mm = track_it->last_height_mm > 0 
                                            ? track_it->last_height_mm 
                                            : detection.height_above_floor_mm;
        // 楕円を再計算
        detection.ellipse_px = EstimateEllipse(detection.head_position_px, 
                                                detection.foot_center_px, 
                                                detection.height_above_floor_mm);
        detection.confidence = std::max(detection.confidence, 0.6f);
        // toio座標を再計算
        detection.toio_position = ::locomotion::calibration::TransformImagePixelToToio(
            detection.foot_center_px, calibration.homography_color_to_toio);
      }
    }

    // 楕円のtoio座標変換（簡易版：中心のみ変換）
    cv::Point2f ellipse_center_toio = ::locomotion::calibration::TransformImagePixelToToio(
        detection.ellipse_px.center, calibration.homography_color_to_toio);
    // スケール変換（簡易版：ピクセル→toio ID単位の変換係数を仮定）
    float scale_factor = 0.7f;  // 概算
    detection.ellipse_toio = cv::RotatedRect(
        ellipse_center_toio,
        cv::Size2f(detection.ellipse_px.size.width * scale_factor,
                   detection.ellipse_px.size.height * scale_factor),
        detection.ellipse_px.angle);

    // トラッキング情報の取得
    auto track_it_after_id = std::find_if(tracked_humans_.begin(), tracked_humans_.end(),
                                          [detection](const TrackingState& state) {
                                            return state.id == detection.id;
                                          });

    // 位置の平滑化（ノイズ低減）
    if (track_it_after_id != tracked_humans_.end()) {
      detection.toio_position = SmoothPosition(
          detection.toio_position,
          track_it_after_id->last_position,
          config_.position_smoothing_alpha);
    }

    // 速度の計算（時間重み付き）
    if (track_it_after_id != tracked_humans_.end()) {
      detection.velocity_mm_per_s = ComputeWeightedVelocity(*track_it_after_id, current_time);
    } else {
      detection.velocity_mm_per_s = cv::Point2f(0.0f, 0.0f);
    }

    // 動きの判定
    detection.motion_state =
        DetermineMotionState(detection.id, detection.toio_position, delta_time_seconds);

    // トラッキング履歴の更新（頭と足の位置、身長も保存）
    UpdateTrackingHistory(detection.id, detection.toio_position, current_time);
    
    // TrackingStateに詳細情報を保存
    auto track_it_final = std::find_if(tracked_humans_.begin(), tracked_humans_.end(),
                                       [detection](const TrackingState& state) {
                                         return state.id == detection.id;
                                       });
    if (track_it_final != tracked_humans_.end()) {
      track_it_final->last_motion_state = detection.motion_state;
      track_it_final->last_head_position_px = detection.head_position_px;
      track_it_final->last_foot_position_px = detection.foot_center_px;
      track_it_final->last_height_mm = detection.height_above_floor_mm;
      track_it_final->last_velocity = detection.velocity_mm_per_s;
      
      // 速度の分散を更新（ノイズ推定用）
      if (track_it_final->position_history.size() >= 5) {
        std::vector<double> velocities;
        for (size_t i = 1; i < track_it_final->position_history.size(); ++i) {
          if (i >= track_it_final->time_history.size()) break;
          double delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(
              track_it_final->time_history[i] - track_it_final->time_history[i - 1]).count() / 1000.0;
          if (delta_time > kEpsilon && delta_time < 1.0) {
            double dist = cv::norm(track_it_final->position_history[i] - track_it_final->position_history[i - 1]);
            velocities.push_back(dist / delta_time);
          }
        }
        if (!velocities.empty()) {
          double mean_vel = 0.0;
          for (double v : velocities) {
            mean_vel += v;
          }
          mean_vel /= static_cast<double>(velocities.size());
          double var = 0.0;
          for (double v : velocities) {
            var += (v - mean_vel) * (v - mean_vel);
          }
          track_it_final->velocity_variance = var / static_cast<double>(velocities.size());
        }
      }
    }

    detection.timestamp = current_time;
  }

  // 古いトラッキングのクリーンアップ
  CleanupOldTracks(current_time);

  result.humans = std::move(detections);
  result.has_valid_detections = !result.humans.empty();

  auto end_time = std::chrono::steady_clock::now();
  result.processing_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time).count();

  return result;
}

std::vector<cv::Rect> HumanDetector::CoarseDetect(const cv::Mat& color_resized,
                                                   const cv::Mat& depth_resized) {
  std::vector<cv::Rect> rois;

  if (color_resized.empty() || depth_resized.empty()) {
    return rois;
  }

  // 深度範囲でフィルタ
  cv::Mat depth_mask;
  cv::inRange(depth_resized,
              cv::Scalar(static_cast<uint16_t>(config_.min_depth_mm)),
              cv::Scalar(static_cast<uint16_t>(config_.max_depth_mm)),
              depth_mask);

  // モルフォロジー処理でノイズ除去
  cv::morphologyEx(depth_mask, depth_mask, cv::MORPH_CLOSE, morphology_kernel_,
                   cv::Point(-1, -1), config_.morphology_iterations);
  cv::morphologyEx(depth_mask, depth_mask, cv::MORPH_OPEN, morphology_kernel_,
                   cv::Point(-1, -1), config_.morphology_iterations);

  // 連結成分解析
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(depth_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 面積フィルタ
  int min_area = (depth_resized.rows * depth_resized.cols) / 100;  // 画像の1%以上
  for (const auto& contour : contours) {
    double area = cv::contourArea(contour);
    if (area < min_area) {
      continue;
    }

    cv::Rect bounding_rect = cv::boundingRect(contour);
    rois.push_back(bounding_rect);
  }

  return rois;
}

std::vector<HumanDetection> HumanDetector::FineDetect(
    const FrameBundle& frame,
    const std::vector<cv::Rect>& coarse_rois,
    const CalibrationSnapshot& calibration) {
  std::vector<HumanDetection> detections;

  for (const auto& roi : coarse_rois) {
    // ROIの範囲チェック
    if (roi.x < 0 || roi.y < 0 || roi.x + roi.width > frame.depth.cols ||
        roi.y + roi.height > frame.depth.rows) {
      continue;
    }

    // ROIを抽出
    cv::Mat depth_roi = frame.depth(roi);

    cv::Point2f head_pos, foot_center;
    double height_mm, confidence;

    // 頭と足の位置推定（トラッキングIDは後で割り当てるため、-1を渡す）
    if (!EstimateHeadAndFoot(depth_roi, roi, calibration, head_pos, foot_center,
                             height_mm, confidence, -1)) {
      continue;
    }

    // 楕円領域の推定
    cv::RotatedRect ellipse = EstimateEllipse(head_pos, foot_center, height_mm);

    HumanDetection detection;
    detection.head_position_px = head_pos;
    detection.foot_center_px = foot_center;
    detection.ellipse_px = ellipse;
    detection.height_above_floor_mm = height_mm;
    detection.confidence = confidence;
    detection.motion_state = HumanMotionState::UNKNOWN;  // 後で更新

    detections.push_back(detection);
  }

  return detections;
}

bool HumanDetector::EstimateHeadAndFoot(const cv::Mat& depth_roi,
                                        const cv::Rect& roi,
                                        const CalibrationSnapshot& calibration,
                                        cv::Point2f& head_pos,
                                        cv::Point2f& foot_center,
                                        double& height_mm,
                                        double& confidence,
                                        int tracking_id) {
  // トラッキング情報を取得（フォールバック用）
  TrackingState* tracked_state = nullptr;
  if (tracking_id >= 0) {
    auto it = std::find_if(tracked_humans_.begin(), tracked_humans_.end(),
                           [tracking_id](const TrackingState& state) {
                             return state.id == tracking_id;
                           });
    if (it != tracked_humans_.end()) {
      tracked_state = &(*it);
    }
  }

  // Step 1: まず頭を検出（頭の方が検出しやすい）
  cv::Mat head_mask;
  cv::inRange(depth_roi,
              cv::Scalar(static_cast<uint16_t>(config_.head_depth_min_mm)),
              cv::Scalar(static_cast<uint16_t>(config_.head_depth_max_mm)),
              head_mask);

  // モルフォロジー処理
  cv::morphologyEx(head_mask, head_mask, cv::MORPH_CLOSE, morphology_kernel_);

  // 頭の検出
  std::vector<std::vector<cv::Point>> head_contours;
  cv::findContours(head_mask, head_contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  bool head_found = false;
  if (!head_contours.empty()) {
    // 最も上（yが小さい）の点を頭とする
    cv::Point2f head_candidate(roi.width / 2.0f + roi.x, roi.height / 2.0f + roi.y);
    float min_y = static_cast<float>(roi.height);
    
    for (const auto& contour : head_contours) {
      for (const auto& pt : contour) {
        if (pt.y < min_y) {
          min_y = static_cast<float>(pt.y);
          head_candidate = cv::Point2f(pt.x + roi.x, pt.y + roi.y);
        }
      }
    }
    head_pos = head_candidate;
    head_found = true;
  } else if (tracked_state && tracked_state->last_head_position_px.x > 0 && 
             tracked_state->last_head_position_px.y > 0) {
    // 頭が見つからないが、トラッキング情報がある場合は前回の位置を使用
    head_pos = tracked_state->last_head_position_px;
    head_found = true;
    confidence = 0.3;  // 低信頼度（頭が見つからないので）
  } else {
    // 頭も見つからず、トラッキング情報もない場合は失敗
    return false;
  }

  // Step 2: 頭中心からの半径15cm以内で足を検出
  bool foot_found = false;
  double foot_confidence = 0.0;
  
  if (head_found) {
    foot_found = DetectFootNearHead(depth_roi, roi, head_pos, calibration,
                                    config_.head_to_foot_search_radius_mm,
                                    foot_center, foot_confidence);
  }

  // Step 3: 足が見つからない場合のフォールバック
  if (!foot_found) {
    if (tracked_state && tracked_state->last_motion_state == HumanMotionState::STANDING) {
      // 頭が動いていなかったら（静止中）、前回の足の位置を使用
      if (tracked_state->last_foot_position_px.x > 0 && 
          tracked_state->last_foot_position_px.y > 0) {
        foot_center = tracked_state->last_foot_position_px;
        foot_found = true;
        foot_confidence = 0.65;  // 中程度の信頼度（静止中なので比較的信頼できる）
      }
    }
    
    if (!foot_found) {
      // 頭から足の位置を推定（平均身長または前回の身長を使用）
      double estimated_height = tracked_state && tracked_state->last_height_mm > 0
                                    ? tracked_state->last_height_mm
                                    : config_.average_person_height_mm;
      
      // 頭の位置から下方向に身長分の距離を追加
      foot_center = cv::Point2f(
          head_pos.x,
          head_pos.y + static_cast<float>(estimated_height / kPixelsToMmApprox));
      foot_found = true;
      foot_confidence = 0.45;  // 低信頼度（推定値なので）
      height_mm = estimated_height;
    }
  }

  // Step 4: 身長の計算
  if (foot_found && head_found) {
    double pixel_height = std::abs(head_pos.y - foot_center.y);
    height_mm = pixel_height * kPixelsToMmApprox;
    
    // 身長が妥当な範囲かチェック
    if (height_mm < config_.min_person_height_mm ||
        height_mm > config_.max_person_height_mm) {
      // 範囲外の場合は平均身長を使用
      height_mm = config_.average_person_height_mm;
      foot_confidence *= 0.8;  // 信頼度を下げる
    }
  }

  // Step 5: 最終的な信頼度の設定
  if (head_found && foot_found) {
    // 頭と足の両方が検出された場合
    confidence = 0.8 * foot_confidence;
    if (head_contours.size() > 1) {
      confidence = std::min(0.95, confidence + 0.1);  // 複数の頭候補がある場合は信頼度を上げる
    }
    // トラッキング情報を使用した場合は信頼度を下げる
    if (tracked_state && tracked_state->last_motion_state == HumanMotionState::STANDING) {
      confidence = std::max(0.5, confidence * 0.9);
    }
    return true;
  }

  return false;
}

bool HumanDetector::DetectFootNearHead(const cv::Mat& depth_roi,
                                       const cv::Rect& roi,
                                       const cv::Point2f& head_pos_px,
                                       const CalibrationSnapshot& calibration,
                                       double search_radius_mm,
                                       cv::Point2f& foot_center,
                                       double& confidence) {
  // 頭位置をROI座標系に変換
  cv::Point2f head_roi(head_pos_px.x - roi.x, head_pos_px.y - roi.y);

  // 検索半径をピクセル単位に変換（簡易版）
  double search_radius_pixels = search_radius_mm / kPixelsToMmApprox;

  // 足の深度範囲でフィルタ
  cv::Mat foot_mask;
  cv::inRange(depth_roi,
              cv::Scalar(static_cast<uint16_t>(config_.foot_depth_min_mm)),
              cv::Scalar(static_cast<uint16_t>(config_.foot_depth_max_mm)),
              foot_mask);

  // モルフォロジー処理
  cv::morphologyEx(foot_mask, foot_mask, cv::MORPH_CLOSE, morphology_kernel_);

  // 頭中心からの円形マスクを作成
  cv::Mat circle_mask = cv::Mat::zeros(depth_roi.size(), CV_8UC1);
  cv::circle(circle_mask, cv::Point(static_cast<int>(head_roi.x),
                                     static_cast<int>(head_roi.y)),
             static_cast<int>(search_radius_pixels), cv::Scalar(255), -1);

  // マスクを適用
  cv::Mat masked_foot;
  cv::bitwise_and(foot_mask, circle_mask, masked_foot);

  // 連結成分解析
  std::vector<std::vector<cv::Point>> foot_contours;
  cv::findContours(masked_foot, foot_contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  if (foot_contours.empty()) {
    return false;
  }

  // 最大の連結成分を足とする
  auto largest_foot = std::max_element(
      foot_contours.begin(), foot_contours.end(),
      [](const auto& a, const auto& b) {
        return cv::contourArea(a) < cv::contourArea(b);
      });

  double foot_area = cv::contourArea(*largest_foot);
  if (foot_area < config_.min_foot_area_pixels) {
    return false;
  }

  // 足の中心を計算
  cv::Moments foot_moments = cv::moments(*largest_foot);
  if (foot_moments.m00 < kEpsilon) {
    return false;
  }

  foot_center = cv::Point2f(
      static_cast<float>(foot_moments.m10 / foot_moments.m00) + roi.x,
      static_cast<float>(foot_moments.m01 / foot_moments.m00) + roi.y);

  // 頭からの距離を計算して信頼度を設定
  double distance_from_head = cv::norm(foot_center - head_pos_px);
  double distance_mm = distance_from_head * kPixelsToMmApprox;
  
  // 距離が近いほど信頼度が高い
  if (distance_mm < search_radius_mm * 0.5) {
    confidence = 0.9;
  } else if (distance_mm < search_radius_mm * 0.8) {
    confidence = 0.7;
  } else {
    confidence = 0.5;
  }

  return true;
}

cv::RotatedRect HumanDetector::EstimateEllipse(const cv::Point2f& head_pos,
                                                const cv::Point2f& foot_center,
                                                double height_mm) {
  // 楕円の中心を計算
  cv::Point2f center = (head_pos + foot_center) * 0.5f;
  center.y = head_pos.y + static_cast<float>(height_mm * config_.ellipse_vertical_offset / kPixelsToMmApprox);

  // 楕円のサイズを計算
  float width = static_cast<float>(height_mm * config_.ellipse_aspect_ratio / kPixelsToMmApprox);
  float height = static_cast<float>(height_mm / kPixelsToMmApprox);

  // 角度を計算（頭から足への方向）
  float angle = std::atan2(foot_center.y - head_pos.y, foot_center.x - head_pos.x) * 180.0f / static_cast<float>(M_PI);
  angle += 90.0f;  // 楕円の長軸が垂直になるように調整

  return cv::RotatedRect(center, cv::Size2f(width, height), angle);
}

HumanMotionState HumanDetector::DetermineMotionState(int human_id,
                                                      const cv::Point2f& current_position,
                                                      double delta_time_seconds) {
  auto it = std::find_if(tracked_humans_.begin(), tracked_humans_.end(),
                         [human_id](const TrackingState& state) {
                           return state.id == human_id;
                         });

  if (it == tracked_humans_.end() || it->position_history.size() < 3) {
    return HumanMotionState::UNKNOWN;
  }

  // 時間重み付き速度を計算（ComputeWeightedVelocityを使用）
  auto current_time = std::chrono::system_clock::now();
  cv::Point2f velocity = ComputeWeightedVelocity(*it, current_time);
  double velocity_magnitude = cv::norm(velocity);

  // 位置の変動を計算（過去Nフレームでの総移動距離）
  // より最近のフレームにより大きな重みを付ける
  double weighted_total_distance = 0.0;
  double total_weight = 0.0;
  
  double alpha = kLn2 / config_.velocity_half_life_seconds;
  auto time_now = current_time;
  
  for (size_t i = 0; i < it->position_history.size() - 1; ++i) {
    if (i >= it->time_history.size()) break;
    
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        time_now - it->time_history[i]).count() / 1000.0;
    
    if (time_diff < 0.0 || time_diff > 10.0) continue;
    
    double distance = cv::norm(it->position_history[i + 1] - it->position_history[i]);
    double weight = std::exp(-alpha * time_diff);
    
    weighted_total_distance += distance * weight;
    total_weight += weight;
  }
  
  double avg_weighted_distance = total_weight > kEpsilon 
                                    ? weighted_total_distance / total_weight 
                                    : 0.0;

  // 静止判定: 速度が閾値以下 かつ 変動距離が閾値以下
  // ノイズを考慮して、より厳しい条件を設定
  bool is_standing = (velocity_magnitude < config_.standing_velocity_threshold_mm_per_s) &&
                     (avg_weighted_distance < config_.movement_threshold_mm);
  
  // さらに、最近のフレームで連続して静止しているかを確認
  if (is_standing && it->position_history.size() >= 10) {
    // 過去10フレームのうち、8フレーム以上で静止しているか
    int standing_count = 0;
    for (size_t i = std::max(0, static_cast<int>(it->position_history.size()) - 10); 
         i < it->position_history.size() - 1; ++i) {
      double dist = cv::norm(it->position_history[i + 1] - it->position_history[i]);
      if (dist < config_.movement_threshold_mm) {
        standing_count++;
      }
    }
    is_standing = (standing_count >= 8);
  }

  return is_standing ? HumanMotionState::STANDING : HumanMotionState::MOVING;
}

int HumanDetector::AssignTrackingId(const cv::Point2f& position) {
  // 既存のトラッキングとマッチング（最近傍法）
  int best_id = -1;
  double best_distance = config_.tracking_max_distance_mm;

  for (auto& tracked : tracked_humans_) {
    double distance = cv::norm(position - tracked.last_position);
    if (distance < best_distance) {
      best_distance = distance;
      best_id = tracked.id;
    }
  }

  if (best_id >= 0) {
    return best_id;
  }

  // 新しいIDを割り当て
  int new_id = next_tracking_id_++;
  if (static_cast<int>(tracked_humans_.size()) >= config_.max_tracked_humans) {
    // 最大人数に達した場合、最も古いトラッキングを削除
    tracked_humans_.erase(
        std::min_element(tracked_humans_.begin(), tracked_humans_.end(),
                         [](const TrackingState& a, const TrackingState& b) {
                           return a.last_seen < b.last_seen;
                         }));
  }

  TrackingState new_track;
  new_track.id = new_id;
  new_track.last_position = position;
  new_track.last_velocity = cv::Point2f(0.0f, 0.0f);
  new_track.last_seen = std::chrono::system_clock::now();
  new_track.position_history.push_back(position);
  new_track.time_history.push_back(std::chrono::system_clock::now());
  new_track.last_head_position_px = cv::Point2f(0, 0);  // 初期化
  new_track.last_foot_position_px = cv::Point2f(0, 0);  // 初期化
  new_track.last_height_mm = 0.0;  // 初期化
  new_track.last_motion_state = HumanMotionState::UNKNOWN;
  new_track.velocity_variance = 0.0;
  tracked_humans_.push_back(new_track);

  return new_id;
}

void HumanDetector::UpdateTrackingHistory(int id,
                                          const cv::Point2f& position,
                                          std::chrono::system_clock::time_point timestamp) {
  auto it = std::find_if(tracked_humans_.begin(), tracked_humans_.end(),
                         [id](const TrackingState& state) {
                           return state.id == id;
                         });

  if (it != tracked_humans_.end()) {
    // 外れ値チェック
    if (config_.enable_outlier_rejection && IsOutlier(position, *it)) {
      // 外れ値の場合は前回の位置を使用（平滑化のため）
      it->last_position = SmoothPosition(it->last_position, it->last_position, 0.1);
      return;
    }
    
    // 位置の平滑化（外れ値チェック後）
    cv::Point2f smoothed_position = SmoothPosition(position, it->last_position, config_.position_smoothing_alpha);
    it->last_position = smoothed_position;
    it->last_seen = timestamp;
    
    // 履歴に追加（平滑化済み位置を使用）
    it->position_history.push_back(smoothed_position);
    it->time_history.push_back(timestamp);

    // 履歴のサイズを制限（最大60フレーム）
    const int max_history = config_.movement_history_frames;
    if (static_cast<int>(it->position_history.size()) > max_history) {
      it->position_history.erase(it->position_history.begin(),
                                 it->position_history.end() - max_history);
      it->time_history.erase(it->time_history.begin(),
                             it->time_history.end() - max_history);
    }
  }
}

void HumanDetector::CleanupOldTracks(std::chrono::system_clock::time_point current_time) {
  auto timeout_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(config_.tracking_timeout_seconds));

  tracked_humans_.erase(
      std::remove_if(tracked_humans_.begin(), tracked_humans_.end(),
                     [current_time, timeout_duration](const TrackingState& track) {
                       auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           current_time - track.last_seen);
                       return elapsed > timeout_duration;
                     }),
      tracked_humans_.end());
}

cv::Point3f HumanDetector::DepthTo3D(const cv::Point2f& pixel,
                                     uint16_t depth_value,
                                     const CameraIntrinsics& intrinsics,
                                     double depth_scale_m) const {
  if (intrinsics.fx == 0.0 || intrinsics.fy == 0.0) {
    return cv::Point3f(0, 0, 0);
  }

  double z_mm = static_cast<double>(depth_value) * depth_scale_m * 1000.0;
  double x_mm = ((pixel.x - intrinsics.cx) / intrinsics.fx) * z_mm;
  double y_mm = ((pixel.y - intrinsics.cy) / intrinsics.fy) * z_mm;

  return cv::Point3f(static_cast<float>(x_mm), static_cast<float>(y_mm),
                     static_cast<float>(z_mm));
}

cv::Point2f HumanDetector::ComputeWeightedVelocity(const TrackingState& track,
                                                    std::chrono::system_clock::time_point current_time) const {
  if (track.position_history.size() < 2 || track.time_history.size() < 2) {
    return cv::Point2f(0.0f, 0.0f);
  }

  const auto& positions = track.position_history;
  const auto& times = track.time_history;
  size_t n = std::min(positions.size(), times.size());
  
  if (n < 2) {
    return cv::Point2f(0.0f, 0.0f);
  }

  // 指数減衰重みを使用した時間重み付き速度計算
  // 重み: w_i = exp(-α * (t_current - t_i))
  // より最近のフレームにより大きな重みを付ける
  
  cv::Point2f weighted_velocity(0.0f, 0.0f);
  double total_weight = 0.0;
  
  // 現在時刻からの経過時間を計算
  auto current_time_point = current_time;
  if (times.size() < n) {
    current_time_point = times.back();
  }

  // 半減期から減衰係数を計算
  double alpha = kLn2 / config_.velocity_half_life_seconds;  // 半減期に基づく減衰係数
  
  for (size_t i = 1; i < n; ++i) {
    // 経過時間（秒）を計算
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time_point - times[i]).count() / 1000.0;
    
    if (time_diff < 0.0 || time_diff > 10.0) {
      continue;  // 異常な時間差はスキップ
    }
    
    // 速度を計算（フレーム間の変位）
    cv::Point2f displacement = positions[i] - positions[i - 1];
    double delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        times[i] - times[i - 1]).count() / 1000.0;
    
    if (delta_time < kEpsilon || delta_time > 1.0) {
      continue;  // 異常な時間間隔はスキップ
    }
    
    // ノイズフィルタ（小さな動きを無視）
    if (IsNoise(displacement, delta_time)) {
      continue;
    }
    
    cv::Point2f velocity = displacement * (1.0f / static_cast<float>(delta_time));
    
    // 指数減衰重みを計算
    double weight = std::exp(-alpha * time_diff);
    
    // 重み付き平均に加算
    weighted_velocity += velocity * static_cast<float>(weight);
    total_weight += weight;
  }
  
  if (total_weight > kEpsilon) {
    weighted_velocity *= (1.0f / static_cast<float>(total_weight));
  } else {
    // 重みが0の場合は前回の速度を使用
    return track.last_velocity;
  }
  
  // 指数平滑化で前回の速度と統合（さらに平滑化）
  if (track.last_velocity.x != 0.0f || track.last_velocity.y != 0.0f) {
    double smoothing = std::exp(-kLn2 * (1.0 / 30.0) / config_.velocity_half_life_seconds);
    weighted_velocity = weighted_velocity * (1.0f - static_cast<float>(smoothing)) + 
                        track.last_velocity * static_cast<float>(smoothing);
  }
  
  return weighted_velocity;
}

cv::Point2f HumanDetector::SmoothPosition(const cv::Point2f& current_position,
                                          const cv::Point2f& previous_smoothed,
                                          double alpha) const {
  // 指数平滑化: smoothed = α × current + (1 - α) × previous
  return current_position * static_cast<float>(alpha) + 
         previous_smoothed * static_cast<float>(1.0 - alpha);
}

bool HumanDetector::IsOutlier(const cv::Point2f& position,
                               const TrackingState& track) const {
  if (track.position_history.size() < 3) {
    return false;  // 履歴が少ない場合は判定しない
  }
  
  // 前回の位置からの変位
  cv::Point2f displacement = position - track.last_position;
  double distance = cv::norm(displacement);
  
  // 過去の変位の統計を計算
  std::vector<double> distances;
  for (size_t i = 1; i < track.position_history.size(); ++i) {
    double dist = cv::norm(track.position_history[i] - track.position_history[i - 1]);
    distances.push_back(dist);
  }
  
  if (distances.empty()) {
    return false;
  }
  
  // 平均と標準偏差を計算
  double mean = 0.0;
  for (double d : distances) {
    mean += d;
  }
  mean /= static_cast<double>(distances.size());
  
  double variance = 0.0;
  for (double d : distances) {
    variance += (d - mean) * (d - mean);
  }
  variance /= static_cast<double>(distances.size());
  double std_dev = std::sqrt(variance);
  
  // 外れ値判定: 平均から標準偏差のN倍以上離れている
  if (std_dev > kEpsilon) {
    double z_score = std::abs(distance - mean) / std_dev;
    return z_score > config_.outlier_threshold_std;
  }
  
  return false;
}

bool HumanDetector::IsNoise(const cv::Point2f& displacement, double delta_time_seconds) const {
  if (delta_time_seconds < kEpsilon) {
    return true;
  }
  
  double distance = cv::norm(displacement);
  double velocity = distance / delta_time_seconds;
  
  // ノイズ閾値以下の動きは無視
  return distance < config_.noise_threshold_mm;
}

}  // namespace locomotion::calibration

