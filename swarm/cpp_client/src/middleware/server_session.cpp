#include "toio/middleware/server_session.hpp"

#include "toio/client/toio_client.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <shared_mutex>
#include <utility>

namespace toio::middleware {

namespace {

std::optional<bool>
effective_require(std::optional<bool> require_result, bool fallback) {
  if (require_result.has_value()) {
    return require_result;
  }
  return fallback;
}

Position parse_position(const nlohmann::json &position_json) {
  Position pos{};
  pos.x = position_json.value("x", 0);
  pos.y = position_json.value("y", 0);
  pos.angle = position_json.value("angle", 0);
  pos.on_mat = position_json.value("on_mat", false);
  if (position_json.contains("timestamp_ms")) {
    pos.timestamp_ms = position_json["timestamp_ms"].get<std::uint64_t>();
  } else if (position_json.contains("timestamp")) {
    pos.timestamp_ms = position_json["timestamp"].get<std::uint64_t>();
  }
  return pos;
}

} // namespace

ServerSession::ServerSession(ServerConfig config)
    : config_(std::move(config)),
      client_(std::make_unique<client::ToioClient>(
          config_.host, config_.port, config_.endpoint)) {
  client_->set_message_handler(
      [this](const nlohmann::json &json) { handle_message(json); });

  for (const auto &cube : config_.cubes) {
    CubeState state;
    state.server_id = config_.id;
    state.cube_id = cube.id;
    states_.emplace(cube.id, std::move(state));
  }
}

ServerSession::~ServerSession() {
  try {
    stop();
  } catch (...) {
  }
}

const std::string &ServerSession::id() const {
  return config_.id;
}

void ServerSession::start() {
  client_->connect();

  for (const auto &cube : config_.cubes) {
    if (cube.auto_connect) {
      try {
        connect_cube(cube.id, config_.default_require_result);
        if (cube.initial_led.has_value()) {
          set_led(cube.id, *cube.initial_led, config_.default_require_result);
        }
      } catch (const std::exception &ex) {
        std::cerr << "[ServerSession] auto_connect error (" << cube.id
                  << "): " << ex.what() << std::endl;
      }
    }
    if (cube.auto_subscribe) {
      try {
        query_position(cube.id, true);
      } catch (const std::exception &ex) {
        std::cerr << "[ServerSession] auto_subscribe error (" << cube.id
                  << "): " << ex.what() << std::endl;
      }
    }
  }
}

void ServerSession::stop() {
  if (client_) {
    client_->close();
  }
}

void ServerSession::connect_cube(const std::string &cube_id,
                                 std::optional<bool> require_result) {
  client_->connect_cube(cube_id,
                        effective_require(require_result,
                                          config_.default_require_result));
}

void ServerSession::disconnect_cube(const std::string &cube_id,
                                    std::optional<bool> require_result) {
  client_->disconnect_cube(cube_id,
                           effective_require(require_result,
                                             config_.default_require_result));
}

void ServerSession::send_move(const std::string &cube_id,
                              int left_speed,
                              int right_speed,
                              std::optional<bool> require_result) {
  client_->send_move(cube_id,
                     left_speed,
                     right_speed,
                     effective_require(require_result,
                                       config_.default_require_result));
}

void ServerSession::set_led(const std::string &cube_id,
                            const LedColor &color,
                            std::optional<bool> require_result) {
  client_->set_led(cube_id,
                   color.r,
                   color.g,
                   color.b,
                   effective_require(require_result,
                                     config_.default_require_result));
  update_state(cube_id, [&](CubeState &state) {
    state.led = color;
  });
}

void ServerSession::query_battery(const std::string &cube_id) {
  client_->query_battery(cube_id);
}

void ServerSession::query_position(const std::string &cube_id,
                                   std::optional<bool> notify) {
  client_->query_position(cube_id, notify);
}

bool ServerSession::has_cube(const std::string &cube_id) const {
  std::shared_lock lock(state_mutex_);
  return states_.count(cube_id) > 0;
}

CubeState ServerSession::get_state(const std::string &cube_id) const {
  std::shared_lock lock(state_mutex_);
  auto it = states_.find(cube_id);
  if (it == states_.end()) {
    throw std::out_of_range("Cube not found: " + cube_id);
  }
  return it->second;
}

std::vector<CubeSnapshot> ServerSession::snapshot() const {
  std::vector<CubeSnapshot> result;
  std::shared_lock lock(state_mutex_);
  result.reserve(states_.size());
  for (const auto &[cube_id, state] : states_) {
    CubeSnapshot snap;
    snap.state = state;
    result.push_back(std::move(snap));
  }
  return result;
}

void ServerSession::set_state_callback(StateCallback callback) {
  state_callback_ = std::move(callback);
}

void ServerSession::set_message_callback(MessageCallback callback) {
  message_callback_ = std::move(callback);
}

void ServerSession::handle_message(const nlohmann::json &json) {
  if (message_callback_) {
    message_callback_(config_.id, json);
  }
  const auto type_it = json.find("type");
  if (type_it == json.end() || !type_it->is_string()) {
    return;
  }
  const auto payload_it = json.find("payload");
  if (payload_it == json.end() || !payload_it->is_object()) {
    return;
  }
  const std::string type = type_it->get<std::string>();
  const auto &payload = *payload_it;

  auto target_it = payload.find("target");
  std::string target;
  if (target_it != payload.end() && target_it->is_string()) {
    target = target_it->get<std::string>();
  }

  if (type == "result") {
    const std::string cmd = payload.value("cmd", "");
    const std::string status = payload.value("status", "");
    if (target.empty() || status != "success") {
      return;
    }
    if (cmd == "connect") {
      update_state(target, [](CubeState &state) { state.connected = true; });
    } else if (cmd == "disconnect") {
      update_state(target, [](CubeState &state) { state.connected = false; });
    }
  } else if (type == "response") {
    const std::string info = payload.value("info", "");
    if (target.empty() || info.empty()) {
      return;
    }
    if (info == "battery" && payload.contains("battery_level")) {
      int level = payload["battery_level"].get<int>();
      update_state(target, [level](CubeState &state) {
        state.battery_percent = level;
      });
    } else if (info == "position" && payload.contains("position")) {
      Position pos = parse_position(payload["position"]);
      update_state(target, [pos](CubeState &state) { state.position = pos; });
    }
  }
}

void ServerSession::update_state(
    const std::string &cube_id,
    const std::function<void(CubeState &)> &mutator) {
  CubeState snapshot;
  {
    std::unique_lock lock(state_mutex_);
    auto &state = states_[cube_id];
    if (state.cube_id.empty()) {
      state.cube_id = cube_id;
      state.server_id = config_.id;
    }
    mutator(state);
    state.last_update = std::chrono::steady_clock::now();
    snapshot = state;
  }
  if (state_callback_) {
    state_callback_(snapshot);
  }
}

} // namespace toio::middleware
