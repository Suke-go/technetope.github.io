#include "toio/api/fleet_control.hpp"

#include <chrono>
#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace toio::api {

namespace {

CubeHandle make_handle(const std::string &server_id,
                       const std::string &cube_id) {
  return CubeHandle{server_id, cube_id};
}

std::string read_string_field(const nlohmann::json &obj, const char *key) {
  auto it = obj.find(key);
  if (it != obj.end() && it->is_string()) {
    return it->get<std::string>();
  }
  return {};
}

} // namespace

FleetControl::FleetControl(std::vector<middleware::ServerConfig> configs)
    : manager_(),
      goal_controller_(manager_) {
  manager_.apply_config(std::move(configs));
  manager_.set_message_callback(
      [this](const std::string &server_id, const nlohmann::json &json) {
        handle_message(server_id, json);
      });
  rebuild_cube_index();
}

FleetControl::~FleetControl() {
  try {
    stop();
  } catch (...) {
  }
}

void FleetControl::start() {
  if (started_) {
    return;
  }
  manager_.start();
  started_ = true;
}

void FleetControl::stop() {
  goal_controller_.stop_all();
  cancel_all_pending("stopped");
  if (!started_) {
    return;
  }
  manager_.stop();
  started_ = false;
}

bool FleetControl::started() const noexcept {
  return started_;
}

void FleetControl::set_state_callback(
    middleware::ServerSession::StateCallback callback) {
  manager_.set_state_callback(std::move(callback));
}

void FleetControl::set_message_callback(
    middleware::ServerSession::MessageCallback callback) {
  std::unique_lock lock(message_callback_mutex_);
  user_message_callback_ = std::move(callback);
}

void FleetControl::set_goal_logger(control::GoalController::Logger logger) {
  goal_controller_.set_logger(std::move(logger));
}

std::vector<CubeHandle> FleetControl::cubes() const {
  std::vector<CubeHandle> result;
  result.reserve(cube_index_.size());
  for (const auto &[cube_id, server_id] : cube_index_) {
    result.push_back(make_handle(server_id, cube_id));
  }
  return result;
}

std::vector<middleware::CubeSnapshot> FleetControl::snapshot() const {
  return manager_.snapshot();
}

CubeHandle FleetControl::resolve_cube(const std::string &cube_id) const {
  auto it = cube_index_.find(cube_id);
  if (it == cube_index_.end()) {
    throw std::runtime_error("Unknown cube id: " + cube_id);
  }
  return make_handle(it->second, it->first);
}

bool FleetControl::connect(const std::string &cube_id,
                           std::optional<bool> require_result,
                           std::chrono::milliseconds timeout) {
  return connect(resolve_cube(cube_id), require_result, timeout);
}

bool FleetControl::connect(const CubeHandle &handle,
                           std::optional<bool> require_result,
                           std::chrono::milliseconds timeout) {
  ensure_started();
  if (require_result.has_value() && *require_result) {
    const auto key = command_key(handle.server_id, handle.cube_id, "connect");
    auto future = register_pending_command(key);
    if (!manager_.connect(handle.server_id, handle.cube_id, true)) {
      complete_pending_command(
          key, CommandResult{false, "failed to dispatch connect command"});
      return false;
    }
    auto result = wait_for_command_result(key, future, timeout);
    if (!result.has_value()) {
      complete_pending_command(
          key, CommandResult{false, "connect timed out"});
      result = wait_for_command_result(key, future, std::chrono::milliseconds(
                                                   0));
    }
    if (!result.has_value()) {
      return false;
    }
    if (!result->success && !result->message.empty()) {
      std::cerr << "[FleetControl] connect failed for " << handle.server_id
                << ":" << handle.cube_id << " - " << result->message
                << std::endl;
    }
    return result->success;
  }
  return manager_.connect(handle.server_id, handle.cube_id, require_result);
}

bool FleetControl::disconnect(const std::string &cube_id,
                              std::optional<bool> require_result,
                              std::chrono::milliseconds timeout) {
  return disconnect(resolve_cube(cube_id), require_result, timeout);
}

bool FleetControl::disconnect(const CubeHandle &handle,
                              std::optional<bool> require_result,
                              std::chrono::milliseconds timeout) {
  ensure_started();
  if (require_result.has_value() && *require_result) {
    const auto key =
        command_key(handle.server_id, handle.cube_id, "disconnect");
    auto future = register_pending_command(key);
    if (!manager_.disconnect(handle.server_id, handle.cube_id, true)) {
      complete_pending_command(key, CommandResult{
                                         false,
                                         "failed to dispatch disconnect"});
      return false;
    }
    auto result = wait_for_command_result(key, future, timeout);
    if (!result.has_value()) {
      complete_pending_command(
          key, CommandResult{false, "disconnect timed out"});
      result = wait_for_command_result(key, future,
                                       std::chrono::milliseconds(0));
    }
    if (!result.has_value()) {
      return false;
    }
    if (!result->success && !result->message.empty()) {
      std::cerr << "[FleetControl] disconnect failed for " << handle.server_id
                << ":" << handle.cube_id << " - " << result->message
                << std::endl;
    }
    return result->success;
  }
  return manager_.disconnect(handle.server_id,
                             handle.cube_id,
                             require_result);
}

bool FleetControl::set_led(const std::string &cube_id,
                           const middleware::LedColor &color,
                           std::optional<bool> require_result) {
  return set_led(resolve_cube(cube_id), color, require_result);
}

bool FleetControl::set_led(const CubeHandle &handle,
                           const middleware::LedColor &color,
                           std::optional<bool> require_result) {
  ensure_started();
  return manager_.set_led(handle.server_id,
                          handle.cube_id,
                          color,
                          require_result);
}

bool FleetControl::move(const std::string &cube_id,
                        int left_speed,
                        int right_speed,
                        std::optional<bool> require_result) {
  return move(resolve_cube(cube_id), left_speed, right_speed, require_result);
}

bool FleetControl::move(const CubeHandle &handle,
                        int left_speed,
                        int right_speed,
                        std::optional<bool> require_result) {
  ensure_started();
  return manager_.move(handle.server_id,
                       handle.cube_id,
                       left_speed,
                       right_speed,
                       require_result);
}

bool FleetControl::start_goal(const std::string &cube_id,
                              control::GoalOptions options) {
  return start_goal(resolve_cube(cube_id), std::move(options));
}

bool FleetControl::start_goal(const CubeHandle &handle,
                              control::GoalOptions options) {
  ensure_started();
  return goal_controller_.start_goal(handle.server_id,
                                     handle.cube_id,
                                     std::move(options));
}

bool FleetControl::update_goal(const std::string &cube_id,
                               control::GoalOptions options) {
  return update_goal(resolve_cube(cube_id), std::move(options));
}

bool FleetControl::update_goal(const CubeHandle &handle,
                               control::GoalOptions options) {
  ensure_started();
  return goal_controller_.update_goal(handle.server_id,
                                      handle.cube_id,
                                      std::move(options));
}

bool FleetControl::stop_goal(const std::string &cube_id) {
  return stop_goal(resolve_cube(cube_id));
}

bool FleetControl::stop_goal(const CubeHandle &handle) {
  return goal_controller_.stop_goal(handle.server_id, handle.cube_id);
}

std::size_t FleetControl::stop_all_goals() {
  return goal_controller_.stop_all();
}

void FleetControl::handle_message(const std::string &server_id,
                                  const nlohmann::json &json) {
  bool handled = false;
  auto type_it = json.find("type");
  if (type_it != json.end() && type_it->is_string() &&
      *type_it == "result") {
    auto payload_it = json.find("payload");
    if (payload_it != json.end() && payload_it->is_object()) {
      const auto &payload = *payload_it;
      const std::string cmd = read_string_field(payload, "cmd");
      const std::string target = read_string_field(payload, "target");
      if (!cmd.empty() && !target.empty()) {
        CommandResult result;
        result.success = payload.value("status", "") == "success";
        if (!result.success) {
          result.message = read_string_field(payload, "message");
          if (result.message.empty()) {
            result.message = read_string_field(payload, "reason");
          }
        }
        const auto key = command_key(server_id, target, cmd);
        handled = complete_pending_command(key, result);
      }
    }
  }
  middleware::ServerSession::MessageCallback callback;
  {
    std::shared_lock lock(message_callback_mutex_);
    callback = user_message_callback_;
  }
  if (callback) {
    callback(server_id, json);
  }
  if (!handled && type_it != json.end() && *type_it == "result") {
    // Allow subclasses to observe results even if no callback is set.
  }
}

std::string FleetControl::command_key(const std::string &server_id,
                                      const std::string &cube_id,
                                      const std::string &cmd) const {
  return server_id + ":" + cube_id + ":" + cmd;
}

std::future<FleetControl::CommandResult>
FleetControl::register_pending_command(const std::string &key) {
  auto promise = std::make_shared<std::promise<CommandResult>>();
  auto future = promise->get_future();
  {
    std::lock_guard lock(pending_mutex_);
    auto existing = pending_commands_.find(key);
    if (existing != pending_commands_.end()) {
      try {
        existing->second->set_value(
            CommandResult{false, "command superseded"});
      } catch (...) {
      }
      pending_commands_.erase(existing);
    }
    pending_commands_[key] = promise;
  }
  return future;
}

bool FleetControl::complete_pending_command(
    const std::string &key,
    const CommandResult &result) {
  std::shared_ptr<std::promise<CommandResult>> promise;
  {
    std::lock_guard lock(pending_mutex_);
    auto it = pending_commands_.find(key);
    if (it == pending_commands_.end()) {
      return false;
    }
    promise = std::move(it->second);
    pending_commands_.erase(it);
  }
  try {
    promise->set_value(result);
  } catch (...) {
  }
  return true;
}

std::optional<FleetControl::CommandResult>
FleetControl::wait_for_command_result(const std::string &key,
                                      std::future<CommandResult> &future,
                                      std::chrono::milliseconds timeout) {
  if (future.valid()) {
    if (future.wait_for(timeout) == std::future_status::ready) {
      return future.get();
    }
    // Timed out, caller will decide whether to synthesize a failure.
  }
  return std::nullopt;
}

void FleetControl::cancel_all_pending(const std::string &reason) {
  std::vector<std::shared_ptr<std::promise<CommandResult>>> promises;
  {
    std::lock_guard lock(pending_mutex_);
    for (auto &[_, promise] : pending_commands_) {
      promises.push_back(promise);
    }
    pending_commands_.clear();
  }
  for (auto &promise : promises) {
    try {
      promise->set_value(CommandResult{false, reason});
    } catch (...) {
    }
  }
}

void FleetControl::ensure_started() {
  if (!started_) {
    start();
  }
}

void FleetControl::rebuild_cube_index() {
  cube_index_.clear();
  for (const auto &[server_id, cube_id] : manager_.enumerate_cubes()) {
    auto [_, inserted] = cube_index_.emplace(cube_id, server_id);
    if (!inserted) {
      throw std::runtime_error("Duplicate cube id detected: " + cube_id);
    }
  }
}

} // namespace toio::api
