#include "motion_planner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace swarm::samples {
namespace {

constexpr double kEpsilon = 1e-6;

double clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(max_value, value));
}

} // namespace

MotionPlanner::MotionPlanner(MotionPlannerParameters params)
    : params_(std::move(params)),
      last_time_(std::chrono::steady_clock::now()),
      rng_(std::random_device{}()),
      normal_dist_(0.0, 1.0) {}

std::vector<TargetPoint>
MotionPlanner::initial_targets(std::size_t cube_count) const {
  std::vector<TargetPoint> targets;
  if (cube_count == 0) {
    return targets;
  }
  targets.reserve(cube_count);
  const double span_x = std::max(1.0, max_x() - min_x());
  const double mid_y = (min_y() + max_y()) * 0.5;
  for (std::size_t i = 0; i < cube_count; ++i) {
    const double ratio = (static_cast<double>(i) + 0.5) /
                         static_cast<double>(cube_count);
    TargetPoint point;
    point.x = min_x() + span_x * ratio;
    point.y = mid_y;
    targets.push_back(point);
  }
  return targets;
}

std::vector<TargetPoint> MotionPlanner::next_targets(
    const std::vector<toio::middleware::Position> &positions) {
  std::vector<TargetPoint> targets;
  if (positions.empty()) {
    return targets;
  }

  const auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_time_).count();
  last_time_ = now;
  if (dt <= 0.0) {
    dt = 1e-3;
  }

  ensure_robot_states(positions.size());

  for (std::size_t i = 0; i < positions.size(); ++i) {
    update_random_velocity(i, dt, positions[i]);
    apply_boundary_reflection(i, positions[i]);
  }

  apply_repulsion_forces(positions, dt);
  enforce_collision_brake(positions);

  targets.reserve(positions.size());
  for (std::size_t i = 0; i < positions.size(); ++i) {
    targets.push_back(make_target(positions[i], i));
  }
  return targets;
}

void MotionPlanner::ensure_robot_states(std::size_t count) {
  robot_states_.resize(count);
}

void MotionPlanner::update_random_velocity(
    std::size_t index, double dt,
    const toio::middleware::Position & /*position*/) {
  if (index >= robot_states_.size()) {
    return;
  }
  auto &state = robot_states_[index];
  const double theta = params_.random_theta;
  const double sigma = params_.random_sigma;
  const double sqrt_dt = std::sqrt(std::max(0.0, dt));
  state.vx += theta * (params_.random_bias_x - state.vx) * dt +
              sigma * sqrt_dt * normal_dist_(rng_);
  state.vy += theta * (params_.random_bias_y - state.vy) * dt +
              sigma * sqrt_dt * normal_dist_(rng_);

  const double speed = std::hypot(state.vx, state.vy);
  if (speed > params_.random_speed_limit && params_.random_speed_limit > 0.0) {
    const double scale = params_.random_speed_limit / speed;
    state.vx *= scale;
    state.vy *= scale;
  }
}

void MotionPlanner::apply_boundary_reflection(
    std::size_t index, const toio::middleware::Position &position) {
  if (index >= robot_states_.size()) {
    return;
  }
  auto &state = robot_states_[index];
  const double margin = params_.boundary_reflect_margin;
  const double damping = params_.boundary_damping;
  if (margin <= 0.0) {
    return;
  }

  const double left = min_x();
  const double right = max_x();
  const double top = min_y();
  const double bottom = max_y();

  if ((position.x <= left + margin && state.vx < 0.0) ||
      (position.x >= right - margin && state.vx > 0.0)) {
    state.vx = -state.vx * std::clamp(damping, 0.0, 1.0);
  }

  if ((position.y <= top + margin && state.vy < 0.0) ||
      (position.y >= bottom - margin && state.vy > 0.0)) {
    state.vy = -state.vy * std::clamp(damping, 0.0, 1.0);
  }
}

void MotionPlanner::apply_repulsion_forces(
    const std::vector<toio::middleware::Position> &positions, double dt) {
  if (robot_states_.size() != positions.size() || dt <= 0.0) {
    return;
  }

  const double safe_distance =
      std::max(params_.safe_distance, params_.safety_margin);
  const double repulsion_gain = params_.repulsion_gain;
  const double boundary_gain = params_.boundary_repulsion_gain;

  std::vector<std::pair<double, double>> accelerations(positions.size(),
                                                       {0.0, 0.0});

  if (repulsion_gain > 0.0 && safe_distance > 0.0) {
    for (std::size_t i = 0; i < positions.size(); ++i) {
      for (std::size_t j = i + 1; j < positions.size(); ++j) {
        const double dx = positions[i].x - positions[j].x;
        const double dy = positions[i].y - positions[j].y;
        const double dist = std::hypot(dx, dy);
        if (dist < safe_distance && dist > kEpsilon) {
          const double strength =
              repulsion_gain * (1.0 / dist - 1.0 / safe_distance);
          const double fx = strength * (dx / dist);
          const double fy = strength * (dy / dist);
          accelerations[i].first += fx;
          accelerations[i].second += fy;
          accelerations[j].first -= fx;
          accelerations[j].second -= fy;
        }
      }
    }
  }

  if (boundary_gain > 0.0 && safe_distance > 0.0) {
    const double left = min_x();
    const double right = max_x();
    const double top = min_y();
    const double bottom = max_y();
    for (std::size_t i = 0; i < positions.size(); ++i) {
      const double dist_left = positions[i].x - left;
      const double dist_right = right - positions[i].x;
      const double dist_top = positions[i].y - top;
      const double dist_bottom = bottom - positions[i].y;

      if (dist_left < safe_distance && dist_left > kEpsilon) {
        const double force =
            boundary_gain * (1.0 / dist_left - 1.0 / safe_distance);
        accelerations[i].first += force;
      }
      if (dist_right < safe_distance && dist_right > kEpsilon) {
        const double force =
            boundary_gain * (1.0 / dist_right - 1.0 / safe_distance);
        accelerations[i].first -= force;
      }
      if (dist_top < safe_distance && dist_top > kEpsilon) {
        const double force =
            boundary_gain * (1.0 / dist_top - 1.0 / safe_distance);
        accelerations[i].second += force;
      }
      if (dist_bottom < safe_distance && dist_bottom > kEpsilon) {
        const double force =
            boundary_gain * (1.0 / dist_bottom - 1.0 / safe_distance);
        accelerations[i].second -= force;
      }
    }
  }

  for (std::size_t i = 0; i < robot_states_.size(); ++i) {
    robot_states_[i].vx += accelerations[i].first * dt;
    robot_states_[i].vy += accelerations[i].second * dt;
    const double speed = std::hypot(robot_states_[i].vx, robot_states_[i].vy);
    if (speed > params_.max_speed && params_.max_speed > 0.0) {
      const double scale = params_.max_speed / speed;
      robot_states_[i].vx *= scale;
      robot_states_[i].vy *= scale;
    }
  }
}

void MotionPlanner::enforce_collision_brake(
    const std::vector<toio::middleware::Position> &positions) {
  if (robot_states_.size() != positions.size()) {
    return;
  }
  const double stop_distance = params_.collision_stop_distance;
  if (stop_distance <= kEpsilon) {
    return;
  }
  const double min_scale =
      clamp(params_.collision_stop_min_scale, 0.0, 1.0);
  std::vector<double> scales(positions.size(), 1.0);
  for (std::size_t i = 0; i < positions.size(); ++i) {
    for (std::size_t j = i + 1; j < positions.size(); ++j) {
      const double dx = positions[i].x - positions[j].x;
      const double dy = positions[i].y - positions[j].y;
      const double dist = std::hypot(dx, dy);
      if (dist < stop_distance) {
        const double ratio = dist / stop_distance;
        const double factor = clamp(ratio, min_scale, 1.0);
        scales[i] = std::min(scales[i], factor);
        scales[j] = std::min(scales[j], factor);
      }
    }
  }

  for (std::size_t i = 0; i < robot_states_.size(); ++i) {
    robot_states_[i].vx *= scales[i];
    robot_states_[i].vy *= scales[i];
  }
}

TargetPoint MotionPlanner::make_target(
    const toio::middleware::Position &position, std::size_t index) const {
  TargetPoint target;
  if (index >= robot_states_.size()) {
    target.x = clamp(position.x, min_x(), max_x());
    target.y = clamp(position.y, min_y(), max_y());
    return target;
  }
  const auto &state = robot_states_[index];
  const double lookahead = std::max(0.0, params_.lookahead_time);
  target.x = position.x + state.vx * lookahead;
  target.y = position.y + state.vy * lookahead;
  target.x = clamp(target.x, min_x(), max_x());
  target.y = clamp(target.y, min_y(), max_y());
  return target;
}

double MotionPlanner::min_x() const {
  const double left = params_.field_min_x + params_.safety_margin;
  const double right = params_.field_max_x - params_.safety_margin;
  return std::min(left, right);
}

double MotionPlanner::max_x() const {
  const double left = params_.field_min_x + params_.safety_margin;
  const double right = params_.field_max_x - params_.safety_margin;
  return std::max(left, right);
}

double MotionPlanner::min_y() const {
  const double top = params_.field_min_y + params_.safety_margin;
  const double bottom = params_.field_max_y - params_.safety_margin;
  return std::min(top, bottom);
}

double MotionPlanner::max_y() const {
  const double top = params_.field_min_y + params_.safety_margin;
  const double bottom = params_.field_max_y - params_.safety_margin;
  return std::max(top, bottom);
}

} // namespace swarm::samples
