#include "toio/control/goal_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

namespace toio::control {

namespace {

using toio::middleware::CubeState;
using toio::middleware::FleetManager;
using toio::middleware::Position;

double wrap_deg180(double angle) {
  double wrapped = std::fmod(angle + 180.0, 360.0);
  if (wrapped < 0) {
    wrapped += 360.0;
  }
  return wrapped - 180.0;
}

std::optional<std::pair<int, int>>
compute_goal_move(const Position &current,
                  const GoalOptions &params,
                  double &direction_state) {
  const double dx = static_cast<double>(params.goal_x - current.x);
  const double dy = static_cast<double>(params.goal_y - current.y);
  const double dist = std::hypot(dx, dy);
  if (dist < params.stop_dist) {
    return std::nullopt;
  }

  constexpr double rad_to_deg = 180.0 / 3.14159265358979323846;
  const double target_heading = std::atan2(dy, dx) * rad_to_deg;
  const double heading_error =
      -wrap_deg180(target_heading - static_cast<double>(current.angle));
  const double abs_error = std::abs(heading_error);

  const double enter_reverse =
      params.reverse_threshold_deg + params.reverse_hysteresis_deg;
  const double exit_reverse =
      std::max(0.0, params.reverse_threshold_deg - params.reverse_hysteresis_deg);

  if (direction_state >= 0.0) {
    if (abs_error > enter_reverse) {
      direction_state = -1.0;
    } else {
      direction_state = 1.0;
    }
  } else {
    if (abs_error < exit_reverse) {
      direction_state = 1.0;
    } else {
      direction_state = -1.0;
    }
  }

  double heading_correction = heading_error;
  if (direction_state < 0.0) {
    heading_correction =
        heading_error > 0 ? heading_error - 180.0 : heading_error + 180.0;
  }

  double v = params.k_r * dist * direction_state;
  double w = params.k_a * heading_correction;

  v = std::clamp(v, -params.vmax, params.vmax);
  w = std::clamp(w, -params.wmax, params.wmax);

  const double left = std::clamp(v - 0.5 * w, -100.0, 100.0);
  const double right = std::clamp(v + 0.5 * w, -100.0, 100.0);
  return std::pair<int, int>{static_cast<int>(left), static_cast<int>(right)};
}

} // namespace

GoalController::GoalController(FleetManager &manager)
    : manager_(manager),
      logger_([](const std::string &key, const std::string &message) {
        std::cout << "[goal " << key << "] " << message << std::endl;
      }) {}

GoalController::~GoalController() {
  stop_all();
}

void GoalController::set_logger(Logger logger) {
  std::lock_guard<std::mutex> lock(log_mutex_);
  logger_ = std::move(logger);
}

std::string GoalController::make_key(const std::string &server_id,
                                     const std::string &cube_id) {
  return server_id + ":" + cube_id;
}

void GoalController::log(const std::string &key,
                         const std::string &message) const {
  Logger logger_copy;
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    logger_copy = logger_;
  }
  if (logger_copy) {
    logger_copy(key, message);
  }
}

std::optional<CubeState>
GoalController::find_cube_state(const std::string &server_id,
                                const std::string &cube_id) const {
  auto snapshots = manager_.snapshot();
  for (const auto &snapshot : snapshots) {
    if (snapshot.state.server_id == server_id &&
        snapshot.state.cube_id == cube_id) {
      return snapshot.state;
    }
  }
  return std::nullopt;
}

void GoalController::run_goal_task(
    const std::string &server_id,
    const std::string &cube_id,
    std::shared_ptr<SharedGoal> shared_goal,
    std::shared_ptr<std::atomic<bool>> cancel_flag) {
  const std::string key = make_key(server_id, cube_id);

  auto copy_goal = [&shared_goal]() {
    std::lock_guard<std::mutex> lock(shared_goal->mutex);
    return shared_goal->options;
  };

  auto options = copy_goal();

  auto wait_for_connection = [&]() {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!cancel_flag->load() &&
           std::chrono::steady_clock::now() < deadline) {
      auto state = find_cube_state(server_id, cube_id);
      if (state && state->connected) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    auto state = find_cube_state(server_id, cube_id);
    return state && state->connected;
  };

  try {
    log(key,
        "started toward (" + std::to_string(options.goal_x) + ", " +
            std::to_string(options.goal_y) + ")");

    auto initial_state = find_cube_state(server_id, cube_id);
    if (!initial_state) {
      log(key, "cube state not found, aborting");
      return;
    }
    if (!initial_state->connected) {
      if (!manager_.connect(server_id, cube_id, true)) {
        log(key, "failed to send connect command");
        return;
      }
      if (!wait_for_connection()) {
        log(key, "failed to confirm connection");
        return;
      }
    }

    manager_.query_position(server_id, cube_id, false);
    bool reached_goal = false;
    double direction_state = 1.0;

    while (!cancel_flag->load()) {
      options = copy_goal();
      auto state = find_cube_state(server_id, cube_id);
      if (!state) {
        log(key, "cube disappeared from manager state");
        break;
      }
      if (!state->position) {
        manager_.query_position(server_id, cube_id, false);
        std::this_thread::sleep_for(options.poll_interval);
        continue;
      }
      auto speeds =
          compute_goal_move(*state->position, options, direction_state);
      if (!speeds) {
        if (shared_goal->auto_stop_on_goal.load()) {
          reached_goal = true;
          break;
        }
        manager_.move(server_id, cube_id, 0, 0, false);
        manager_.query_position(server_id, cube_id, false);
        std::this_thread::sleep_for(options.poll_interval);
        continue;
      }
      manager_.move(server_id, cube_id, speeds->first, speeds->second, false);
      manager_.query_position(server_id, cube_id, false);
      std::this_thread::sleep_for(options.poll_interval);
    }

    manager_.move(server_id, cube_id, 0, 0, false);
    if (reached_goal) {
      log(key, "goal reached");
    } else if (cancel_flag->load()) {
      log(key, "goal task cancelled");
    }
  } catch (const std::exception &ex) {
    log(key, std::string("error: ") + ex.what());
  } catch (...) {
    log(key, "error: unknown exception");
  }
}

bool GoalController::start_goal(const std::string &server_id,
                                const std::string &cube_id,
                                GoalOptions options) {
  const std::string key = make_key(server_id, cube_id);
  stop_goal(server_id, cube_id);

  auto shared_goal = std::make_shared<SharedGoal>();
  {
    std::lock_guard<std::mutex> lock(shared_goal->mutex);
    shared_goal->options = std::move(options);
    shared_goal->auto_stop_on_goal.store(true);
  }
  auto cancel_flag = std::make_shared<std::atomic<bool>>(false);
  auto worker = std::async(std::launch::async,
                           [this,
                            server_id,
                            cube_id,
                            shared_goal,
                            cancel_flag]() {
                             run_goal_task(server_id,
                                           cube_id,
                                           shared_goal,
                                           cancel_flag);
                           });

  std::lock_guard<std::mutex> lock(tasks_mutex_);
  tasks_.emplace(key,
                 GoalTask{std::move(shared_goal),
                          std::move(cancel_flag),
                          std::move(worker)});
  return true;
}

bool GoalController::update_goal(const std::string &server_id,
                                 const std::string &cube_id,
                                 GoalOptions options) {
  const std::string key = make_key(server_id, cube_id);
  std::shared_ptr<SharedGoal> shared_goal;
  {
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    auto it = tasks_.find(key);
    if (it == tasks_.end()) {
      return false;
    }
    shared_goal = it->second.shared_goal;
  }
  {
    std::lock_guard<std::mutex> goal_lock(shared_goal->mutex);
    shared_goal->options = options;
  }
  shared_goal->auto_stop_on_goal.store(false);
  log(key,
      "goal updated to (" + std::to_string(options.goal_x) + ", " +
          std::to_string(options.goal_y) + ")");
  return true;
}

bool GoalController::stop_goal(const std::string &server_id,
                               const std::string &cube_id) {
  const std::string key = make_key(server_id, cube_id);
  std::future<void> worker;
  {
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    auto it = tasks_.find(key);
    if (it == tasks_.end()) {
      return false;
    }
    it->second.cancel_flag->store(true);
    worker = std::move(it->second.worker);
    tasks_.erase(it);
  }
  if (worker.valid()) {
    worker.wait();
  }
  return true;
}

std::size_t GoalController::stop_all() {
  std::vector<std::future<void>> workers;
  {
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    workers.reserve(tasks_.size());
    for (auto &[_, task] : tasks_) {
      task.cancel_flag->store(true);
      workers.push_back(std::move(task.worker));
    }
    tasks_.clear();
  }
  for (auto &worker : workers) {
    if (worker.valid()) {
      worker.wait();
    }
  }
  return workers.size();
}

bool GoalController::has_goal(const std::string &server_id,
                              const std::string &cube_id) const {
  const std::string key = make_key(server_id, cube_id);
  std::lock_guard<std::mutex> lock(tasks_mutex_);
  return tasks_.count(key) > 0;
}

} // namespace toio::control
