#include "toio/middleware/fleet_manager.hpp"

#include <iterator>
#include <stdexcept>
#include <utility>

namespace toio::middleware {

FleetManager::FleetManager() = default;

FleetManager::FleetManager(std::vector<ServerConfig> configs) {
  apply_config(std::move(configs));
}

FleetManager::~FleetManager() {
  try {
    stop();
  } catch (...) {
  }
}

void FleetManager::apply_config(std::vector<ServerConfig> configs) {
  stop();
  sessions_.clear();
  for (auto &config : configs) {
    auto session = std::make_unique<ServerSession>(std::move(config));
    if (state_callback_) {
      session->set_state_callback(state_callback_);
    }
    if (message_callback_) {
      session->set_message_callback(message_callback_);
    }
    sessions_.emplace(session->id(), std::move(session));
  }
}

void FleetManager::start() {
  for (auto &[_, session] : sessions_) {
    session->start();
  }
}

void FleetManager::stop() {
  for (auto &[_, session] : sessions_) {
    session->stop();
  }
}

std::vector<std::string> FleetManager::server_ids() const {
  std::vector<std::string> ids;
  ids.reserve(sessions_.size());
  for (const auto &[id, _] : sessions_) {
    ids.push_back(id);
  }
  return ids;
}

bool FleetManager::has_server(const std::string &server_id) const {
  return sessions_.count(server_id) > 0;
}

bool FleetManager::has_cube(const std::string &server_id,
                            const std::string &cube_id) const {
  const auto *session = find_session(server_id);
  return session && session->has_cube(cube_id);
}

std::vector<std::pair<std::string, std::string>>
FleetManager::enumerate_cubes() const {
  std::vector<std::pair<std::string, std::string>> list;
  for (const auto &[server_id, session] : sessions_) {
    for (const auto &cube_id : session->cube_ids()) {
      list.emplace_back(server_id, cube_id);
    }
  }
  return list;
}

bool FleetManager::use(const std::string &server_id,
                       const std::string &cube_id) {
  if (!has_cube(server_id, cube_id)) {
    return false;
  }
  active_target_ = std::make_pair(server_id, cube_id);
  return true;
}

std::optional<std::pair<std::string, std::string>>
FleetManager::active_target() const {
  return active_target_;
}

bool FleetManager::connect(const std::string &server_id,
                           const std::string &cube_id,
                           std::optional<bool> require_result) {
  auto *session = find_session(server_id);
  if (!session) {
    return false;
  }
  session->connect_cube(cube_id, require_result);
  return true;
}

bool FleetManager::disconnect(const std::string &server_id,
                              const std::string &cube_id,
                              std::optional<bool> require_result) {
  auto *session = find_session(server_id);
  if (!session) {
    return false;
  }
  session->disconnect_cube(cube_id, require_result);
  return true;
}

bool FleetManager::move(const std::string &server_id,
                        const std::string &cube_id,
                        int left_speed,
                        int right_speed,
                        std::optional<bool> require_result) {
  auto *session = find_session(server_id);
  if (!session) {
    return false;
  }
  session->send_move(cube_id, left_speed, right_speed, require_result);
  return true;
}

std::size_t FleetManager::move_all(int left_speed,
                                   int right_speed,
                                   std::optional<bool> require_result) {
  return for_each_cube([&](ServerSession &session,
                           const std::string &,
                           const std::string &cube_id) {
    session.send_move(cube_id, left_speed, right_speed, require_result);
  });
}

bool FleetManager::set_led(const std::string &server_id,
                           const std::string &cube_id,
                           const LedColor &color,
                           std::optional<bool> require_result) {
  auto *session = find_session(server_id);
  if (!session) {
    return false;
  }
  session->set_led(cube_id, color, require_result);
  return true;
}

std::size_t FleetManager::set_led_all(const LedColor &color,
                                      std::optional<bool> require_result) {
  return for_each_cube([&](ServerSession &session,
                           const std::string &,
                           const std::string &cube_id) {
    session.set_led(cube_id, color, require_result);
  });
}

bool FleetManager::query_battery(const std::string &server_id,
                                 const std::string &cube_id) {
  auto *session = find_session(server_id);
  if (!session) {
    return false;
  }
  session->query_battery(cube_id);
  return true;
}

std::size_t FleetManager::query_battery_all() {
  return for_each_cube([&](ServerSession &session,
                           const std::string &,
                           const std::string &cube_id) {
    session.query_battery(cube_id);
  });
}

bool FleetManager::query_position(const std::string &server_id,
                                  const std::string &cube_id,
                                  std::optional<bool> notify) {
  auto *session = find_session(server_id);
  if (!session) {
    return false;
  }
  session->query_position(cube_id, notify);
  return true;
}

std::size_t FleetManager::query_position_all(std::optional<bool> notify) {
  return for_each_cube([&](ServerSession &session,
                           const std::string &,
                           const std::string &cube_id) {
    session.query_position(cube_id, notify);
  });
}

bool FleetManager::toggle_subscription(const std::string &server_id,
                                       const std::string &cube_id,
                                       bool enable) {
  return query_position(server_id, cube_id, enable);
}

std::size_t FleetManager::toggle_subscription_all(bool enable) {
  return query_position_all(enable);
}

std::vector<CubeSnapshot> FleetManager::snapshot() const {
  std::vector<CubeSnapshot> result;
  for (const auto &[_, session] : sessions_) {
    auto session_snapshot = session->snapshot();
    result.insert(result.end(),
                  std::make_move_iterator(session_snapshot.begin()),
                  std::make_move_iterator(session_snapshot.end()));
  }
  return result;
}

void FleetManager::set_state_callback(ServerSession::StateCallback callback) {
  state_callback_ = std::move(callback);
  for (auto &[_, session] : sessions_) {
    session->set_state_callback(state_callback_);
  }
}

void FleetManager::set_message_callback(
    ServerSession::MessageCallback callback) {
  message_callback_ = std::move(callback);
  for (auto &[_, session] : sessions_) {
    session->set_message_callback(message_callback_);
  }
}

ServerSession *FleetManager::find_session(const std::string &server_id) {
  auto it = sessions_.find(server_id);
  if (it == sessions_.end()) {
    return nullptr;
  }
  return it->second.get();
}

const ServerSession *
FleetManager::find_session(const std::string &server_id) const {
  auto it = sessions_.find(server_id);
  if (it == sessions_.end()) {
    return nullptr;
  }
  return it->second.get();
}

} // namespace toio::middleware
