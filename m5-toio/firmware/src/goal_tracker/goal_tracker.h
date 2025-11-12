#pragma once

#include <Arduino.h>

#include "cube_pose.h"

class GoalTracker {
 public:
  void setTuning(float vmax, float wmax, float k_r, float k_a,
                 float reverse_threshold_deg = 90.0f,
                 float reverse_hysteresis_deg = 10.0f);

  void setGoal(float x, float y, float stop_distance = 20.0f);
  void clearGoal();

  bool hasGoal() const { return goal_.active; }

  // 戻り値: 指令が生成されたら true
  bool computeCommand(const CubePose& pose, int8_t* left_speed,
                      int8_t* right_speed);

 private:
  struct GoalState {
    bool active = false;
    float x = 0.0f;
    float y = 0.0f;
    float stop_distance = 20.0f;
  } goal_;

  float vmax_ = 70.0f;
  float wmax_ = 60.0f;
  float k_r_ = 0.5f;
  float k_a_ = 1.2f;
  float reverse_threshold_deg_ = 90.0f;
  float reverse_hysteresis_deg_ = 10.0f;
  bool reverse_mode_ = false;
};
