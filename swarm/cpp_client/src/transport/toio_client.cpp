#include "toio/transport/toio_client.hpp"

#include <iostream>

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>

namespace toio::transport {

namespace {
namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;
} // namespace

ToioClient::ToioClient(std::string host,
                       std::string port,
                       std::string endpoint)
    : host_(std::move(host)),
      port_(std::move(port)),
      endpoint_(std::move(endpoint)),
      resolver_(io_context_),
      websocket_(io_context_) {}

ToioClient::~ToioClient() {
  try {
    close();
  } catch (...) {
  }
}

void ToioClient::set_message_handler(MessageHandler handler) {
  message_handler_ = std::move(handler);
}

void ToioClient::set_log_handler(LogHandler handler) {
  log_handler_ = std::move(handler);
}

void ToioClient::connect() {
  if (connected_) {
    return;
  }

  auto const results = resolver_.resolve(host_, port_);
  auto const endpoint = asio::connect(websocket_.next_layer(), results);
  std::string host_header = host_ + ":" + std::to_string(endpoint.port());

  websocket_.set_option(
      websocket::stream_base::timeout::suggested(beast::role_type::client));
  websocket_.set_option(websocket::stream_base::decorator(
      [](websocket::request_type &req) {
        req.set(beast::http::field::user_agent, "toio-cpp-client/0.1");
      }));
  websocket_.handshake(host_header, endpoint_);

  connected_ = true;
  running_ = true;
  reader_thread_ = std::thread([this] { reader_loop(); });

  log("WebSocket connected to " + host_header + endpoint_);
}

void ToioClient::close() {
  if (!connected_) {
    return;
  }

  running_ = false;
  beast::error_code ec;
  websocket_.close(websocket::close_code::normal, ec);
  if (ec) {
    log("WebSocket close error: " + ec.message());
  }

  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }

  connected_ = false;
  log("WebSocket closed");
}

void ToioClient::ensure_connected() const {
  if (!connected_) {
    throw std::runtime_error("WebSocket is not connected");
  }
}

void ToioClient::send_command(const std::string &cmd,
                              const std::string &target,
                              const Json &params,
                              std::optional<bool> require_result) {
  ensure_connected();
  Json payload = {
      {"cmd", cmd},
      {"target", target},
      {"params", params.is_null() ? Json::object() : params},
  };
  if (require_result.has_value()) {
    payload["require_result"] = *require_result;
  }

  Json message = {
      {"type", "command"},
      {"payload", std::move(payload)},
  };
  send_json(message);
}

void ToioClient::send_query(const std::string &info,
                            const std::string &target,
                            std::optional<bool> notify) {
  ensure_connected();
  Json payload = {
      {"info", info},
      {"target", target},
  };
  if (notify.has_value()) {
    payload["notify"] = *notify;
  }
  Json message = {
      {"type", "query"},
      {"payload", std::move(payload)},
  };
  send_json(message);
}

void ToioClient::connect_cube(const std::string &target,
                              std::optional<bool> require_result) {
  send_command("connect", target, Json::object(), require_result);
}

void ToioClient::disconnect_cube(const std::string &target,
                                 std::optional<bool> require_result) {
  send_command("disconnect", target, Json::object(), require_result);
}

void ToioClient::send_move(const std::string &target,
                           int left_speed,
                           int right_speed,
                           std::optional<bool> require_result) {
  Json params = {
      {"left_speed", left_speed},
      {"right_speed", right_speed},
  };
  send_command("move", target, params, require_result);
}

void ToioClient::set_led(const std::string &target,
                         int r,
                         int g,
                         int b,
                         std::optional<bool> require_result) {
  Json params = {
      {"r", r},
      {"g", g},
      {"b", b},
  };
  send_command("led", target, params, require_result);
}

void ToioClient::query_battery(const std::string &target) {
  send_query("battery", target, std::nullopt);
}

void ToioClient::query_position(const std::string &target,
                                std::optional<bool> notify) {
  send_query("position", target, notify);
}

void ToioClient::reader_loop() {
  beast::flat_buffer buffer;
  while (running_) {
    beast::error_code ec;
    websocket_.read(buffer, ec);
    if (ec == websocket::error::closed) {
      break;
    }
    if (ec) {
      log("WebSocket read error: " + ec.message());
      break;
    }

    auto payload_text = beast::buffers_to_string(buffer.data());
    buffer.consume(buffer.size());
    dispatch_message(payload_text);
  }

  running_ = false;
  connected_ = false;
}

void ToioClient::dispatch_message(const std::string &payload_text) {
  if (!message_handler_) {
    log("Received message: " + payload_text);
    return;
  }

  try {
    auto json = Json::parse(payload_text);
    message_handler_(json);
  } catch (const std::exception &ex) {
    log(std::string("Failed to parse JSON: ") + ex.what());
  }
}

void ToioClient::send_json(const Json &message) {
  const std::string serialized = message.dump();
  std::lock_guard<std::mutex> lock(write_mutex_);
  beast::error_code ec;
  websocket_.write(boost::asio::buffer(serialized), ec);
  if (ec) {
    throw beast::system_error(ec);
  }
}

void ToioClient::log(const std::string &message) const {
  if (log_handler_) {
    log_handler_(message);
  } else {
    std::cout << "[toio-client] " << message << std::endl;
  }
}

} // namespace toio::transport
