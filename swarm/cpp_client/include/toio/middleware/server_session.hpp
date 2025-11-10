#pragma once

#include "toio/middleware/cube_state.hpp"

#include <functional>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

namespace toio::transport {
class ToioClient;
}

namespace toio::middleware {

struct CubeConfig {
  std::string id;
  bool auto_connect = false;
  bool auto_subscribe = false;
  std::optional<LedColor> initial_led;
};

struct ServerConfig {
  std::string id;
  std::string host;
  std::string port;
  std::string endpoint = "/ws";
  bool default_require_result = false;
  std::vector<CubeConfig> cubes;
};

class ServerSession {
public:
  using StateCallback = std::function<void(const CubeState &)>;
  using MessageCallback =
      std::function<void(const std::string &, const nlohmann::json &)>;

  explicit ServerSession(ServerConfig config);
  ~ServerSession();

  ServerSession(const ServerSession &) = delete;
  ServerSession &operator=(const ServerSession &) = delete;

  const std::string &id() const;
  void start();
  void stop();

  void connect_cube(const std::string &cube_id,
                    std::optional<bool> require_result = std::nullopt);
  void disconnect_cube(const std::string &cube_id,
                       std::optional<bool> require_result = std::nullopt);
  void send_move(const std::string &cube_id,
                 int left_speed,
                 int right_speed,
                 std::optional<bool> require_result = std::nullopt);
  void set_led(const std::string &cube_id,
               const LedColor &color,
               std::optional<bool> require_result = std::nullopt);
  void query_battery(const std::string &cube_id);
  void query_position(const std::string &cube_id,
                      std::optional<bool> notify);

  bool has_cube(const std::string &cube_id) const;
  CubeState get_state(const std::string &cube_id) const;
  std::vector<std::string> cube_ids() const;
  std::vector<CubeSnapshot> snapshot() const;

  void set_state_callback(StateCallback callback);
  void set_message_callback(MessageCallback callback);

private:
  void handle_message(const nlohmann::json &json);
  void update_state(const std::string &cube_id,
                    const std::function<void(CubeState &)> &mutator);

  ServerConfig config_;
  std::unique_ptr<transport::ToioClient> client_;
  StateCallback state_callback_;
  MessageCallback message_callback_;

  mutable std::shared_mutex state_mutex_;
  std::unordered_map<std::string, CubeState> states_;
};

} // namespace toio::middleware
