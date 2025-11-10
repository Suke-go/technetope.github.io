#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>

namespace toio::transport {

class ToioClient {
public:
  using Json = nlohmann::json;
  using MessageHandler = std::function<void(const Json &)>;
  using LogHandler = std::function<void(const std::string &)>;

  ToioClient(std::string host, std::string port, std::string endpoint = "/ws");
  ~ToioClient();

  ToioClient(const ToioClient &) = delete;
  ToioClient &operator=(const ToioClient &) = delete;

  void set_message_handler(MessageHandler handler);
  void set_log_handler(LogHandler handler);

  void connect();
  void close();

  void send_command(const std::string &cmd,
                    const std::string &target,
                    const Json &params = Json::object(),
                    std::optional<bool> require_result = std::nullopt);

  void send_query(const std::string &info,
                  const std::string &target,
                  std::optional<bool> notify = std::nullopt);

  void connect_cube(const std::string &target,
                    std::optional<bool> require_result = true);
  void disconnect_cube(const std::string &target,
                       std::optional<bool> require_result = true);
  void send_move(const std::string &target,
                 int left_speed,
                 int right_speed,
                 std::optional<bool> require_result = std::nullopt);
  void set_led(const std::string &target,
               int r,
               int g,
               int b,
               std::optional<bool> require_result = std::nullopt);
  void query_battery(const std::string &target);
  void query_position(const std::string &target,
                      std::optional<bool> notify);

private:
  using websocket_t =
      boost::beast::websocket::stream<boost::asio::ip::tcp::socket>;

  void ensure_connected() const;
  void reader_loop();
  void dispatch_message(const std::string &payload_text);
  void send_json(const Json &message);
  void log(const std::string &message) const;

  std::string host_;
  std::string port_;
  std::string endpoint_;

  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::resolver resolver_;
  websocket_t websocket_;

  std::atomic<bool> connected_{false};
  std::atomic<bool> running_{false};
  std::thread reader_thread_;
  mutable std::mutex write_mutex_;

  MessageHandler message_handler_;
  LogHandler log_handler_;
};

} // namespace toio::transport
