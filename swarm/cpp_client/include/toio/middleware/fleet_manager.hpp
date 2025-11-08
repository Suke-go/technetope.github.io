#pragma once

#include "toio/middleware/server_session.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace toio::middleware {

class FleetManager {
public:
  FleetManager();
  explicit FleetManager(std::vector<ServerConfig> configs);
  ~FleetManager();

  FleetManager(const FleetManager &) = delete;
  FleetManager &operator=(const FleetManager &) = delete;

  void apply_config(std::vector<ServerConfig> configs);
  void start();
  void stop();

  std::vector<std::string> server_ids() const;
  bool has_server(const std::string &server_id) const;
  bool has_cube(const std::string &server_id, const std::string &cube_id) const;
  std::vector<std::pair<std::string, std::string>> enumerate_cubes() const;

  bool use(const std::string &server_id, const std::string &cube_id);
  std::optional<std::pair<std::string, std::string>>
  active_target() const;

  bool connect(const std::string &server_id,
               const std::string &cube_id,
               std::optional<bool> require_result = std::nullopt);
  bool disconnect(const std::string &server_id,
                  const std::string &cube_id,
                  std::optional<bool> require_result = std::nullopt);
  bool move(const std::string &server_id,
            const std::string &cube_id,
            int left_speed,
            int right_speed,
            std::optional<bool> require_result = std::nullopt);
  std::size_t move_all(int left_speed,
                       int right_speed,
                       std::optional<bool> require_result = std::nullopt);
  bool set_led(const std::string &server_id,
               const std::string &cube_id,
               const LedColor &color,
               std::optional<bool> require_result = std::nullopt);
  std::size_t set_led_all(const LedColor &color,
                          std::optional<bool> require_result = std::nullopt);
  bool query_battery(const std::string &server_id,
                     const std::string &cube_id);
  std::size_t query_battery_all();
  bool query_position(const std::string &server_id,
                      const std::string &cube_id,
                      std::optional<bool> notify);
  std::size_t query_position_all(std::optional<bool> notify);
  bool toggle_subscription(const std::string &server_id,
                           const std::string &cube_id,
                           bool enable);
  std::size_t toggle_subscription_all(bool enable);

  std::vector<CubeSnapshot> snapshot() const;

  void set_state_callback(ServerSession::StateCallback callback);
  void set_message_callback(ServerSession::MessageCallback callback);

private:
  ServerSession *find_session(const std::string &server_id);
  const ServerSession *find_session(const std::string &server_id) const;

  std::unordered_map<std::string, std::unique_ptr<ServerSession>> sessions_;
  std::optional<std::pair<std::string, std::string>> active_target_;
  ServerSession::StateCallback state_callback_;
  ServerSession::MessageCallback message_callback_;

  template <typename Func>
  std::size_t for_each_cube(Func &&func) {
    std::size_t count = 0;
    for (auto &[server_id, session] : sessions_) {
      for (const auto &cube_id : session->cube_ids()) {
        func(*session, server_id, cube_id);
        ++count;
      }
    }
    return count;
  }
};

} // namespace toio::middleware
