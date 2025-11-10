#pragma once

#include "toio/control/goal_controller.hpp"
#include "toio/middleware/fleet_manager.hpp"

#include <future>
#include <chrono>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace toio::api {

struct CubeHandle {
  std::string server_id;
  std::string cube_id;
};

class FleetControl {
public:
  explicit FleetControl(std::vector<middleware::ServerConfig> configs);
  ~FleetControl();

  FleetControl(const FleetControl &) = delete;
  FleetControl &operator=(const FleetControl &) = delete;
  FleetControl(FleetControl &&) = delete;
  FleetControl &operator=(FleetControl &&) = delete;

  void start();
  void stop();
  bool started() const noexcept;

  void set_state_callback(middleware::ServerSession::StateCallback callback);
  void set_message_callback(middleware::ServerSession::MessageCallback callback);
  void set_goal_logger(control::GoalController::Logger logger);

  std::vector<CubeHandle> cubes() const;
  std::vector<middleware::CubeSnapshot> snapshot() const;

  CubeHandle resolve_cube(const std::string &cube_id) const;

  bool connect(const std::string &cube_id,
               std::optional<bool> require_result = std::nullopt,
               std::chrono::milliseconds timeout =
                   std::chrono::seconds(30));
  bool connect(const CubeHandle &handle,
               std::optional<bool> require_result = std::nullopt,
               std::chrono::milliseconds timeout =
                   std::chrono::seconds(30));
  bool disconnect(const std::string &cube_id,
                  std::optional<bool> require_result = std::nullopt,
                  std::chrono::milliseconds timeout =
                      std::chrono::seconds(15));
  bool disconnect(const CubeHandle &handle,
                  std::optional<bool> require_result = std::nullopt,
                  std::chrono::milliseconds timeout =
                      std::chrono::seconds(15));

  bool set_led(const std::string &cube_id,
               const middleware::LedColor &color,
               std::optional<bool> require_result = std::nullopt);
  bool set_led(const CubeHandle &handle,
               const middleware::LedColor &color,
               std::optional<bool> require_result = std::nullopt);

  bool move(const std::string &cube_id,
            int left_speed,
            int right_speed,
            std::optional<bool> require_result = std::nullopt);
  bool move(const CubeHandle &handle,
            int left_speed,
            int right_speed,
            std::optional<bool> require_result = std::nullopt);

  bool start_goal(const std::string &cube_id,
                  control::GoalOptions options = {});
  bool start_goal(const CubeHandle &handle,
                  control::GoalOptions options = {});
  bool update_goal(const std::string &cube_id,
                   control::GoalOptions options);
  bool update_goal(const CubeHandle &handle,
                   control::GoalOptions options);

  bool stop_goal(const std::string &cube_id);
  bool stop_goal(const CubeHandle &handle);
  std::size_t stop_all_goals();

private:
  struct CommandResult {
    bool success = false;
    std::string message;
  };

  void ensure_started();
  void rebuild_cube_index();
  void handle_message(const std::string &server_id,
                      const nlohmann::json &json);
  std::string command_key(const std::string &server_id,
                          const std::string &cube_id,
                          const std::string &cmd) const;
  std::future<struct CommandResult>
  register_pending_command(const std::string &key);
  bool complete_pending_command(const std::string &key,
                                const struct CommandResult &result);
  std::optional<struct CommandResult>
  wait_for_command_result(const std::string &key,
                          std::future<struct CommandResult> &future,
                          std::chrono::milliseconds timeout);
  void cancel_all_pending(const std::string &reason);

  middleware::FleetManager manager_;
  control::GoalController goal_controller_;
  std::unordered_map<std::string, std::string> cube_index_;
  std::unordered_map<std::string,
                     std::shared_ptr<std::promise<struct CommandResult>>>
      pending_commands_;
  std::mutex pending_mutex_;
  mutable std::shared_mutex message_callback_mutex_;
  middleware::ServerSession::MessageCallback user_message_callback_;
  bool started_ = false;
};

} // namespace toio::api
