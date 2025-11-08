#include "locomotion/calibration/HumanDetectionConverter.h"

#include <algorithm>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace locomotion {
namespace calibration {

std::vector<locomotion::robot::DynamicObstacle> ConvertToDynamicObstacles(
    const HumanDetectionResult& detection_result,
    float prediction_horizon_seconds,
    int num_samples) {
  std::vector<locomotion::robot::DynamicObstacle> obstacles;

  for (const auto& human : detection_result.humans) {
    locomotion::robot::DynamicObstacle obstacle;
    obstacle.label = "human_" + std::to_string(human.id);

    // 速度を使用（HumanDetectorから計算済み）
    locomotion::robot::Vector2f velocity(
        human.velocity_mm_per_s.x,
        human.velocity_mm_per_s.y);

    // 現在位置
    locomotion::robot::Vector2f current_pos(human.toio_position.x, human.toio_position.y);

    // 未来位置をサンプル
    for (int i = 0; i < num_samples; ++i) {
      float time_ahead = (i + 1) * prediction_horizon_seconds / static_cast<float>(num_samples);
      
      locomotion::robot::ObstacleSample sample;
      sample.timeAhead = time_ahead;
      
      // 位置 = 現在位置 + 速度 * 時間
      sample.position = locomotion::robot::Vector2f(
          current_pos.x + velocity.x * time_ahead,
          current_pos.y + velocity.y * time_ahead);
      
      // 確信度：時間が経つほど減少
      sample.certainty = 1.0f - (time_ahead / prediction_horizon_seconds) * 0.3f;
      sample.certainty = std::max(0.0f, std::min(1.0f, sample.certainty));
      
      // 移動中の場合は確信度を上げる
      if (human.motion_state == locomotion::calibration::HumanMotionState::MOVING) {
        sample.certainty = std::min(1.0f, sample.certainty + 0.2f);
      }
      
      obstacle.samples.push_back(sample);
    }

    obstacles.push_back(obstacle);
  }

  return obstacles;
}

}  // namespace calibration
}  // namespace locomotion

