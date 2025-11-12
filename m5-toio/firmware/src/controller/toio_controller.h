#pragma once

#include <Arduino.h>
#include <Toio.h>

#include <string>
#include <vector>

#include "../goal_tracker/cube_pose.h"
#include "../goal_tracker/goal_tracker.h"

struct ToioLedColor {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

struct ToioMotorState {
  int8_t left_speed = 0;
  int8_t right_speed = 0;
};

class ToioController {
 public:
  enum class InitStatus {
    kReady,
    kNoCubeFound,
    kTargetNotFound,
    kConnectionFailed,
    kInvalidArgument,
  };

  InitStatus scanTargets(const std::string& target_fragment,
                         uint32_t scan_duration_sec, ToioCore** out_target);
  InitStatus connectAndConfigure(ToioCore* target_core);
  void loop();

  bool hasActiveCore() const { return active_core_ != nullptr; }
  bool hasGoal() const { return goal_tracker_.hasGoal(); }

  bool hasPose() const { return has_pose_; }
  const CubePose& pose() const { return pose_; }
  bool poseDirty() const { return pose_dirty_; }
  void clearPoseDirty() { pose_dirty_ = false; }

  bool hasBatteryLevel() const { return has_battery_; }
  uint8_t batteryLevel() const { return battery_level_; }
  bool batteryDirty() const { return battery_dirty_; }
  void clearBatteryDirty() { battery_dirty_ = false; }

  ToioLedColor ledColor() const { return led_color_; }
  ToioMotorState motorState() const { return motor_state_; }

  bool setLedColor(uint8_t r, uint8_t g, uint8_t b);
  bool driveMotor(int8_t left_speed, int8_t right_speed);

  void setGoal(float x, float y, float stop_distance = 20.0f);
  void clearGoal();
  void setGoalTuning(float vmax, float wmax, float k_r, float k_a,
                     float reverse_threshold_deg = 90.0f,
                     float reverse_hysteresis_deg = 10.0f);

 private:
  std::vector<ToioCore*> scan(uint32_t duration_sec);
  ToioCore* pickTarget(const std::vector<ToioCore*>& cores,
                       const std::string& fragment) const;
  InitStatus connectCore(ToioCore* core);
  void configureCore(ToioCore* core);
  void handleIdData(const ToioCoreIDData& data);
  void handleBatteryLevel(uint8_t level);
  void updateGoalTracking();

  Toio toio_;
  ToioCore* active_core_ = nullptr;
  std::vector<ToioCore*> last_scan_results_;

  CubePose pose_{};
  bool has_pose_ = false;
  bool pose_dirty_ = false;
  uint32_t pose_updated_ms_ = 0;

  uint8_t battery_level_ = 0;
  bool has_battery_ = false;
  bool battery_dirty_ = false;
  uint32_t battery_updated_ms_ = 0;

  ToioLedColor led_color_{};
  ToioMotorState motor_state_{};

  uint32_t scan_duration_sec_ = 0;

  GoalTracker goal_tracker_;
};
