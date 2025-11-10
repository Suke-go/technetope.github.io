#include "goal_tracker.h"

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

void GoalTracker::setTuning(float vmax, float wmax, float k_r, float k_a) {
  vmax_ = vmax;
  wmax_ = wmax;
  k_r_ = k_r;
  k_a_ = k_a;
}

void GoalTracker::setGoal(float x, float y, float stop_distance) {
  goal_.active = true;
  goal_.x = x;
  goal_.y = y;
  goal_.stop_distance = stop_distance;
}

void GoalTracker::clearGoal() {
  goal_.active = false;
}

bool GoalTracker::computeCommand(const CubePose& pose, bool* left_dir,
                                 uint8_t* left_speed, bool* right_dir,
                                 uint8_t* right_speed) {
  if (!goal_.active || !left_dir || !left_speed || !right_dir ||
      !right_speed) {
    return false;
  }

  if (!pose.on_mat) {
    *left_dir = true;
    *right_dir = true;
    *left_speed = 0;
    *right_speed = 0;
    return true;
  }

  const float dx = goal_.x - static_cast<float>(pose.x);
  const float dy = goal_.y - static_cast<float>(pose.y);
  const float dist = std::sqrt(dx * dx + dy * dy);
  if (dist < goal_.stop_distance) {
    *left_dir = true;
    *right_dir = true;
    *left_speed = 0;
    *right_speed = 0;
    goal_.active = false;
    return true;
  }

  const float target_heading = std::atan2(dy, dx) * kRadToDeg;
  const float heading_error =
      -WrapAngle(target_heading - static_cast<float>(pose.angle));

  float v = Clamp(k_r_ * dist, -vmax_, vmax_);
  float w = Clamp(k_a_ * heading_error, -wmax_, wmax_);

  const float left = Clamp(v - 0.5f * w, -100.0f, 100.0f);
  const float right = Clamp(v + 0.5f * w, -100.0f, 100.0f);

  const auto encode = [](float value, bool* dir, uint8_t* speed) {
    *dir = value >= 0.0f;
    *speed = static_cast<uint8_t>(Clamp(std::fabs(value), 0.0f, 100.0f));
  };

  encode(left, left_dir, left_speed);
  encode(right, right_dir, right_speed);
  return true;
}
