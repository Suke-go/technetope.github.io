#include "goal_tracker.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kRadToDeg = 180.0f / 3.14159265f;

float Clamp(float value, float min_value, float max_value) {
  return std::max(min_value, std::min(max_value, value));
}

float WrapAngle(float angle_deg) {
  while (angle_deg > 180.0f) {
    angle_deg -= 360.0f;
  }
  while (angle_deg < -180.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}
}  // namespace

void GoalTracker::setTuning(float vmax, float wmax, float k_r, float k_a,
                            float reverse_threshold_deg,
                            float reverse_hysteresis_deg) {
  vmax_ = vmax;
  wmax_ = wmax;
  k_r_ = k_r;
  k_a_ = k_a;
  reverse_threshold_deg_ = reverse_threshold_deg;
  reverse_hysteresis_deg_ = reverse_hysteresis_deg;
}

void GoalTracker::setGoal(float x, float y, float stop_distance) {
  goal_.active = true;
  goal_.x = x;
  goal_.y = y;
  goal_.stop_distance = stop_distance;
  reverse_mode_ = false;
}

void GoalTracker::clearGoal() {
  goal_.active = false;
  reverse_mode_ = false;
}

bool GoalTracker::computeCommand(const CubePose& pose, int8_t* left_speed,
                                 int8_t* right_speed) {
  if (!goal_.active || !left_speed || !right_speed) {
    return false;
  }

  if (!pose.on_mat) {
    *left_speed = 0;
    *right_speed = 0;
    return true;
  }

  const float dx = goal_.x - static_cast<float>(pose.x);
  const float dy = goal_.y - static_cast<float>(pose.y);
  const float dist = std::sqrt(dx * dx + dy * dy);
  if (dist < goal_.stop_distance) {
    *left_speed = 0;
    *right_speed = 0;
    goal_.active = false;
    return true;
  }

  const float target_heading = std::atan2(dy, dx) * kRadToDeg;
  const float heading_error =
      -WrapAngle(target_heading - static_cast<float>(pose.angle));
  const float abs_error = std::fabs(heading_error);

  const float enter_reverse =
      reverse_threshold_deg_ + reverse_hysteresis_deg_;
  const float exit_reverse =
      std::max(0.0f, reverse_threshold_deg_ - reverse_hysteresis_deg_);

  if (!reverse_mode_) {
    reverse_mode_ = (abs_error > enter_reverse);
  } else {
    reverse_mode_ = !(abs_error < exit_reverse);
  }

  float heading_correction = heading_error;
  float direction_scale = 1.0f;
  if (reverse_mode_) {
    direction_scale = -1.0f;
    heading_correction =
        (heading_error > 0.0f) ? heading_error - 180.0f
                               : heading_error + 180.0f;
  }

  float v = Clamp(k_r_ * dist * direction_scale, -vmax_, vmax_);
  float w = Clamp(k_a_ * heading_correction, -wmax_, wmax_);

  const float left = Clamp(v - 0.5f * w, -100.0f, 100.0f);
  const float right = Clamp(v + 0.5f * w, -100.0f, 100.0f);

  const auto encode = [](float value) -> int8_t {
    return static_cast<int8_t>(Clamp(value, -100.0f, 100.0f));
  };

  *left_speed = encode(left);
  *right_speed = encode(right);
  return true;
}
