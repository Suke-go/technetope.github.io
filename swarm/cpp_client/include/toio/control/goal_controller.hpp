#pragma once

#include "toio/middleware/fleet_manager.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace toio::control {

struct GoalOptions {
  int goal_x = 0;
  int goal_y = 0;
  double vmax = 90.0;
  double wmax = 70.0;
  double k_r = 0.8;
  double k_a = 0.4;
  double stop_dist = 10.0;
  double reverse_threshold_deg = 100.0;
  double reverse_hysteresis_deg = 15.0;
  std::chrono::milliseconds poll_interval{100};
};

class GoalController {
public:
  using Logger =
      std::function<void(const std::string &key, const std::string &message)>;

  explicit GoalController(toio::middleware::FleetManager &manager);
  ~GoalController();

  GoalController(const GoalController &) = delete;
  GoalController &operator=(const GoalController &) = delete;

  void set_logger(Logger logger);

  bool start_goal(const std::string &server_id,
                  const std::string &cube_id,
                  GoalOptions options = {});
  bool update_goal(const std::string &server_id,
                   const std::string &cube_id,
                   GoalOptions options);

  bool stop_goal(const std::string &server_id, const std::string &cube_id);
  std::size_t stop_all();
  bool has_goal(const std::string &server_id,
                const std::string &cube_id) const;

private:
  struct SharedGoal {
    GoalOptions options;
    mutable std::mutex mutex;
    std::atomic<bool> auto_stop_on_goal{true};
  };

  struct GoalTask {
    std::shared_ptr<SharedGoal> shared_goal;
    std::shared_ptr<std::atomic<bool>> cancel_flag;
    std::future<void> worker;
  };

  toio::middleware::FleetManager &manager_;
  Logger logger_;
  mutable std::mutex log_mutex_;

  mutable std::mutex tasks_mutex_;
  std::unordered_map<std::string, GoalTask> tasks_;

  static std::string make_key(const std::string &server_id,
                              const std::string &cube_id);

  void log(const std::string &key, const std::string &message) const;

  std::optional<toio::middleware::CubeState>
  find_cube_state(const std::string &server_id,
                  const std::string &cube_id) const;

  void run_goal_task(const std::string &server_id,
                     const std::string &cube_id,
                     std::shared_ptr<SharedGoal> shared_goal,
                     std::shared_ptr<std::atomic<bool>> cancel_flag);
};

} // namespace toio::control
