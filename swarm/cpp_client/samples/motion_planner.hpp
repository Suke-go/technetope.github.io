#pragma once

#include "toio/middleware/cube_state.hpp"

#include <chrono>
#include <random>
#include <vector>

namespace swarm::samples {

struct TargetPoint {
  double x = 0.0;
  double y = 0.0;
};

struct MotionPlannerParameters {
  double field_min_x = 34.0;
  double field_min_y = 35.0;
  double field_max_x = 949.0;
  double field_max_y = 898.0;
  double safety_margin = 50.0;

  double random_theta = 0.8;
  double random_sigma = 120.0;
  double random_speed_limit = 150.0;
  double random_bias_x = 0.0;
  double random_bias_y = 0.0;
  double boundary_reflect_margin = 60.0;
  double boundary_damping = 0.5;

  double safe_distance = 120.0;
  double repulsion_gain = 2600.0;
  double boundary_repulsion_gain = 3200.0;
  double max_speed = 180.0;
  double collision_stop_distance = 90.0;
  double collision_stop_min_scale = 0.05;
  double lookahead_time = 0.35;
};

class MotionPlanner {
public:
  explicit MotionPlanner(MotionPlannerParameters params = {});

  std::vector<TargetPoint> initial_targets(std::size_t cube_count) const;
  std::vector<TargetPoint>
  next_targets(const std::vector<toio::middleware::Position> &positions);

private:
  struct RobotState {
    double vx = 0.0;
    double vy = 0.0;
  };

  void ensure_robot_states(std::size_t count);
  void update_random_velocity(std::size_t index, double dt,
                              const toio::middleware::Position &position);
  void apply_boundary_reflection(std::size_t index,
                                 const toio::middleware::Position &position);
  void apply_repulsion_forces(
      const std::vector<toio::middleware::Position> &positions, double dt);
  void enforce_collision_brake(
      const std::vector<toio::middleware::Position> &positions);
  TargetPoint make_target(const toio::middleware::Position &position,
                          std::size_t index) const;
  double min_x() const;
  double max_x() const;
  double min_y() const;
  double max_y() const;

  MotionPlannerParameters params_;
  std::chrono::steady_clock::time_point last_time_;
  std::vector<RobotState> robot_states_;
  std::mt19937 rng_;
  std::normal_distribution<double> normal_dist_;
};

} // namespace swarm::samples
