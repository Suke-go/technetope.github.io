#include "controller/toio_controller.h"

#include <cmath>

ToioController::InitStatus ToioController::scanTargets(
    const std::string& target_fragment, uint32_t scan_duration_sec,
    ToioCore** out_target) {
  scan_duration_sec_ = scan_duration_sec;

  auto cores = scan(scan_duration_sec_);
  if (cores.empty()) {
    if (out_target) {
      *out_target = nullptr;
    }
    return InitStatus::kNoCubeFound;
  }

  ToioCore* target = pickTarget(cores, target_fragment);
  if (!target) {
    if (out_target) {
      *out_target = nullptr;
    }
    return InitStatus::kTargetNotFound;
  }

  if (out_target) {
    *out_target = target;
  }
  return InitStatus::kReady;
}

ToioController::InitStatus ToioController::connectAndConfigure(
    ToioCore* target_core) {
  if (!target_core) {
    return InitStatus::kInvalidArgument;
  }

  InitStatus status = connectCore(target_core);
  if (status != InitStatus::kReady) {
    return status;
  }

  configureCore(target_core);
  return InitStatus::kReady;
}

void ToioController::loop() {
  toio_.loop();
  updateGoalTracking();
}

bool ToioController::setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  if (!active_core_) {
    return false;
  }
  active_core_->turnOnLed(r, g, b);
  led_color_.r = r;
  led_color_.g = g;
  led_color_.b = b;
  return true;
}

namespace {
uint8_t ClampSpeed(int value) {
  if (value < 0) {
    value = -value;
  }
  if (value > 100) {
    value = 100;
  }
  return static_cast<uint8_t>(value);
}
}  // namespace

bool ToioController::driveMotor(int8_t left_speed, int8_t right_speed) {
  if (!active_core_) {
    return false;
  }
  const bool left_dir = left_speed >= 0;
  const bool right_dir = right_speed >= 0;
  const uint8_t left_mag = ClampSpeed(left_speed);
  const uint8_t right_mag = ClampSpeed(right_speed);
  active_core_->controlMotor(left_dir, left_mag, right_dir, right_mag);
  motor_state_.left_speed = left_speed; // clamp していない生の値を保存
  motor_state_.right_speed = right_speed; // clamp していない生の値を保存
  return true;
}

void ToioController::setGoal(float x, float y, float stop_distance) {
  goal_tracker_.setGoal(x, y, stop_distance);
}

void ToioController::clearGoal() {
  goal_tracker_.clearGoal();
  driveMotor(0, 0);
}

void ToioController::setGoalTuning(float vmax, float wmax, float k_r,
                                   float k_a, float reverse_threshold_deg,
                                   float reverse_hysteresis_deg) {
  goal_tracker_.setTuning(vmax, wmax, k_r, k_a, reverse_threshold_deg,
                          reverse_hysteresis_deg);
}

std::vector<ToioCore*> ToioController::scan(uint32_t duration_sec) {
  last_scan_results_ = toio_.scan(duration_sec);
  return last_scan_results_;
}

ToioCore* ToioController::pickTarget(
    const std::vector<ToioCore*>& cores, const std::string& fragment) const {
  if (cores.empty()) {
    return nullptr;
  }
  if (fragment.empty()) {
    return cores.front();
  }
  for (auto* core : cores) {
    const std::string& name = core->getName();
    if (name.find(fragment) != std::string::npos) {
      return core;
    }
  }
  return nullptr;
}

ToioController::InitStatus ToioController::connectCore(ToioCore* core) {
  if (!core) {
    return InitStatus::kInvalidArgument;
  }
  if (!core->connect()) {
    return InitStatus::kConnectionFailed;
  }
  return InitStatus::kReady;
}

void ToioController::configureCore(ToioCore* core) {
  core->setIDnotificationSettings(/*minimum_interval=*/5, /*condition=*/0x01);
  core->setIDmissedNotificationSettings(/*sensitivity=*/10);
  core->onIDReaderData([this](ToioCoreIDData data) { handleIdData(data); });
  core->onBattery([this](uint8_t level) { handleBatteryLevel(level); });

  active_core_ = core;
  handleBatteryLevel(core->getBatteryLevel());
  handleIdData(core->getIDReaderData());
}

void ToioController::handleIdData(const ToioCoreIDData& data) {
  if (data.type == ToioCoreIDTypePosition) {
    pose_.x = data.position.cubePosX;
    pose_.y = data.position.cubePosY;
    pose_.angle = data.position.cubeAngleDegree;
    pose_.on_mat = true;
    has_pose_ = true;
  } else if (data.type == ToioCoreIDTypeNone) {
    pose_.on_mat = false;
    has_pose_ = true;
  } else {
    has_pose_ = false;
  }
  pose_dirty_ = true;
  pose_updated_ms_ = millis();
}

void ToioController::handleBatteryLevel(uint8_t level) {
  battery_level_ = level;
  has_battery_ = true;
  battery_dirty_ = true;
  battery_updated_ms_ = millis();
}

void ToioController::updateGoalTracking() {
  if (!goal_tracker_.hasGoal() || !active_core_ || !has_pose_) {
    return;
  }

  int8_t left_speed = 0;
  int8_t right_speed = 0;
  if (goal_tracker_.computeCommand(pose_, &left_speed, &right_speed)) {
    driveMotor(left_speed, right_speed);
  }
}
