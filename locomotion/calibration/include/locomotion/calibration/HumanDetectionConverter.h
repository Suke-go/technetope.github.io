#pragma once

#include "locomotion/calibration/HumanDetector.h"
#include "locomotion/robot/MotionPlanner.h"

#include <vector>

namespace locomotion {
namespace calibration {

// HumanDetectionResult を MotionPlanner の DynamicObstacle に変換
std::vector<locomotion::robot::DynamicObstacle> ConvertToDynamicObstacles(
    const HumanDetectionResult& detection_result,
    float prediction_horizon_seconds = 0.8f,
    int num_samples = 5);

}  // namespace calibration
}  // namespace locomotion

