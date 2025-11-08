#include "toio/cli/config_loader.hpp"
#include "toio/control/goal_controller.hpp"
#include "toio/middleware/fleet_manager.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

using Json = nlohmann::json;
using toio::cli::FleetPlan;
using toio::cli::Options;
using toio::cli::build_fleet_plan;
using toio::cli::parse_options;
using toio::cli::print_usage;
using toio::control::GoalController;
using toio::control::GoalOptions;
using toio::middleware::CubeSnapshot;
using toio::middleware::FleetManager;
using toio::middleware::LedColor;

namespace {

struct FleetGuard {
  FleetManager &manager;
  bool active = true;
  ~FleetGuard() {
    if (active) {
      try {
        manager.stop();
      } catch (...) {
      }
    }
  }
};

std::vector<std::string> tokenize(const std::string &line) { // tokenの配列(std::vector<std::string>)を返す
  std::istringstream iss(line);
  std::vector<std::string> tokens;
  std::string token;
  while (iss >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

int to_int(const std::string &value) {
  return std::stoi(value);
}

std::pair<std::string, std::string>
resolve_target(const std::string &token,
               const std::unordered_map<std::string, std::string> &cube_index) {
  auto delim = token.find(':');
  if (delim == std::string::npos) {
    auto it = cube_index.find(token);
    if (it == cube_index.end()) {
      throw std::runtime_error("Unknown cube id: " + token);
    }
    return {it->second, it->first};
  }
  std::string server_id = token.substr(0, delim);
  std::string cube_id = token.substr(delim + 1);
  auto it = cube_index.find(cube_id);
  if (it == cube_index.end() || it->second != server_id) {
    throw std::runtime_error("Unknown cube reference: " + token);
  }
  return {server_id, cube_id};
}

void print_status(const std::vector<CubeSnapshot> &snapshots) {
  if (snapshots.empty()) {
    std::cout << "No cube state available yet.\n";
    return;
  }
  std::cout << std::left << std::setw(15) << "Server" << std::setw(12)
            << "Cube" << std::setw(12) << "Connected" << std::setw(10)
            << "Battery" << std::setw(18) << "Position"
            << "LED\n";
  for (const auto &snapshot : snapshots) {
    const auto &state = snapshot.state;
    std::string battery =
        state.battery_percent ? std::to_string(*state.battery_percent) + "%"
                              : "-";
    std::string position = "-";
    if (state.position) {
      position = std::to_string(state.position->x) + "," +
                 std::to_string(state.position->y) + "," +
                 std::to_string(state.position->angle) +
                 (state.position->on_mat ? " (mat)" : " (off)");
    }
    std::ostringstream led;
    led << static_cast<int>(state.led.r) << "," << static_cast<int>(state.led.g)
        << "," << static_cast<int>(state.led.b);
    std::cout << std::left << std::setw(15) << state.server_id << std::setw(12)
              << state.cube_id << std::setw(12)
              << (state.connected ? "yes" : "no") << std::setw(10) << battery
              << std::setw(18) << position << led.str() << "\n";
  }
}

void print_help() {
  std::cout << "Commands:\n"
            << "  help                      Show this message\n"
            << "  status                    Show latest state snapshot\n"
            << "  use <cube-id>|<srv:cube>  Switch active cube\n"
            << "  connect                   Connect active cube\n"
            << "  disconnect                Disconnect active cube\n"
            << "  move <L> <R> [require]    Send move (-100..100). "
               "require=0 to skip result\n"
            << "  moveall <L> <R> [require] Broadcast move to all cubes\n"
            << "  stop                      Shortcut for move 0 0\n"
            << "  goal <X> <Y> [stop]       Drive active cube toward goal (mm)\n"
            << "  goalstop [cube]           Stop goal task for active/target cube\n"
            << "  goalstopall               Stop all goal tasks\n"
            << "  led <R> <G> <B>           Set LED color (0-255)\n"
            << "  ledall <R> <G> <B>        Broadcast LED color\n"
            << "  battery                   Query battery once\n"
            << "  batteryall                Query battery for all cubes\n"
            << "  pos                       Query position once\n"
            << "  posall                    Query position for all cubes\n"
            << "  subscribe                 Enable position notify\n"
            << "  subscribeall              Enable notify for all cubes\n"
            << "  unsubscribe               Disable position notify\n"
            << "  unsubscribeall            Disable notify for all cubes\n"
            << "  exit / quit               Disconnect all cubes and exit\n";
}

class ActiveCube {
public:
  void set(std::string server_id, std::string cube_id) {
    server_id_ = std::move(server_id);
    cube_id_ = std::move(cube_id);
  }

  bool has_value() const {
    return server_id_.has_value() && cube_id_.has_value();
  }

  std::pair<std::string, std::string> get() const {
    if (!has_value()) {
      throw std::runtime_error("No active cube selected");
    }
    return {server_id_.value(), cube_id_.value()};
  }

private:
  std::optional<std::string> server_id_;
  std::optional<std::string> cube_id_;
};


void print_received(const std::string &server_id, const Json &json) {
  auto extract_target = [](const Json &obj) -> std::string {
    if (obj.contains("target") && obj["target"].is_string()) {
      return obj["target"].get<std::string>();
    }
    return {};
  };

  std::string target = extract_target(json);
  if (target.empty()) {
    auto payload_it = json.find("payload");
    if (payload_it != json.end() && payload_it->is_object()) {
      target = extract_target(*payload_it);
    }
  }

  if (!target.empty()) {
    std::cout << "[RECV][" << server_id << ":" << target
              << "] " << json.dump() << std::endl;
  } else {
    std::cout << "[RECV][" << server_id << "] " << json.dump() << std::endl;
  }
}

} // namespace

int main(int argc, char **argv) {
  try {
    const Options options = parse_options(argc, argv);
    FleetPlan plan = build_fleet_plan(options);

    FleetManager manager(plan.configs);
    manager.set_message_callback(
        [](const std::string &server_id, const Json &json) {
          print_received(server_id, json);
        });
    manager.start();
    FleetGuard guard{manager};
    GoalController goal_controller{manager};

    std::unordered_map<std::string, std::string> cube_index;
    for (const auto &[server_id, cube_id] : plan.cube_sequence) {
      if (!cube_index.emplace(cube_id, server_id).second) {
        throw std::runtime_error("Duplicate cube id detected: " + cube_id);
      }
    }

    std::unordered_map<std::string, bool> subscriptions;
    for (const auto &server : plan.configs) {
      for (const auto &cube : server.cubes) {
        subscriptions.emplace(cube.id, cube.auto_subscribe);
      }
    }

    ActiveCube active;
    if (!plan.cube_sequence.empty()) {
      const auto &[server_id, cube_id] = plan.cube_sequence.front();
      if (!manager.use(server_id, cube_id)) {
        throw std::runtime_error("Failed to select initial active cube");
      }
      active.set(server_id, cube_id);
      std::cout << "Active cube set to " << cube_id << " (server " << server_id
                << ")\n";
    }

    print_help();
    std::string line;
    while (std::cout << "> " && std::getline(std::cin, line)) {
      auto tokens = tokenize(line);
      if (tokens.empty()) {
        continue;
      }
      const std::string &cmd = tokens.front();
      try {
        if (cmd == "help") {
          print_help();
        } else if (cmd == "status") {
          print_status(manager.snapshot());
        } else if (cmd == "use") {
          if (tokens.size() < 2) {
            std::cout << "Usage: use <cube-id> or use <server>:<cube>\n";
            continue;
          }
          auto target = resolve_target(tokens[1], cube_index);
          if (!manager.use(target.first, target.second)) {
            throw std::runtime_error("Failed to set active cube");
          }
          active.set(target.first, target.second);
          std::cout << "Active cube set to " << target.second << " (server "
                    << target.first << ")\n";
        } else if (cmd == "connect") {
          auto target = active.get();
          manager.connect(target.first, target.second, true);
        } else if (cmd == "disconnect") {
          auto target = active.get();
          manager.disconnect(target.first, target.second, true);
        } else if (cmd == "move" && tokens.size() >= 3) {
          auto target = active.get();
          int left = to_int(tokens[1]);
          int right = to_int(tokens[2]);
          std::optional<bool> req = true;
          if (tokens.size() >= 4) {
            req = tokens[3] != "0";
          }
          manager.move(target.first, target.second, left, right, req);
        } else if (cmd == "moveall" && tokens.size() >= 3) {
          int left = to_int(tokens[1]);
          int right = to_int(tokens[2]);
          std::optional<bool> req = true;
          if (tokens.size() >= 4) {
            req = tokens[3] != "0";
          }
          std::size_t success = manager.move_all(left, right, req);
          std::cout << "Broadcast move command to " << success << " cubes.\n";
        } else if (cmd == "stop") {
          auto target = active.get();
          manager.move(target.first, target.second, 0, 0, false);
        } else if (cmd == "goal" && tokens.size() >= 3) {
          auto target = active.get();
          GoalOptions options;
          options.goal_x = to_int(tokens[1]);
          options.goal_y = to_int(tokens[2]);
          if (tokens.size() >= 4) {
            options.stop_dist =
                std::max(1.0, static_cast<double>(to_int(tokens[3])));
          }
          goal_controller.start_goal(
              target.first, target.second, options);
          std::cout << "Goal task started for " << target.second << " -> ("
                    << options.goal_x << "," << options.goal_y << ")\n";
        } else if (cmd == "goalstop") {
          std::pair<std::string, std::string> resolved;
          if (tokens.size() >= 2) {
            resolved = resolve_target(tokens[1], cube_index);
          } else {
            resolved = active.get();
          }
          if (goal_controller.stop_goal(resolved.first, resolved.second)) {
            std::cout << "Stopped goal task for " << resolved.second << "\n";
          } else {
            std::cout << "No goal task running for " << resolved.second
                      << "\n";
          }
        } else if (cmd == "goalstopall") {
          std::size_t count = goal_controller.stop_all();
          std::cout << "Stopped " << count << " goal task(s).\n";
        } else if (cmd == "led" && tokens.size() >= 4) {
          auto target = active.get();
          LedColor color{static_cast<std::uint8_t>(to_int(tokens[1])),
                         static_cast<std::uint8_t>(to_int(tokens[2])),
                         static_cast<std::uint8_t>(to_int(tokens[3]))};
          manager.set_led(target.first, target.second, color, false);
        } else if (cmd == "ledall" && tokens.size() >= 4) {
          LedColor color{static_cast<std::uint8_t>(to_int(tokens[1])),
                         static_cast<std::uint8_t>(to_int(tokens[2])),
                         static_cast<std::uint8_t>(to_int(tokens[3]))};
          std::size_t success = manager.set_led_all(color, false);
          std::cout << "Broadcast LED command to " << success << " cubes.\n";
        } else if (cmd == "battery") {
          auto target = active.get();
          manager.query_battery(target.first, target.second);
        } else if (cmd == "batteryall") {
          std::size_t count = manager.query_battery_all();
          std::cout << "Requested battery from " << count << " cubes.\n";
        } else if (cmd == "pos") {
          auto target = active.get();
          manager.query_position(target.first, target.second, false);
        } else if (cmd == "posall") {
          std::size_t count = manager.query_position_all(false);
          std::cout << "Requested position from " << count << " cubes.\n";
        } else if (cmd == "subscribe") {
          auto target = active.get();
          if (subscriptions[target.second]) {
            std::cout << "Already subscribed to " << target.second
                      << std::endl;
          } else {
            manager.toggle_subscription(target.first, target.second, true);
            subscriptions[target.second] = true;
            std::cout << "Subscribed to " << target.second << std::endl;
          }
        } else if (cmd == "subscribeall") {
          std::size_t count = manager.toggle_subscription_all(true);
          for (auto &[cube_id, flag] : subscriptions) {
            (void)cube_id;
            flag = true;
          }
          std::cout << "Subscribed to " << count << " cubes.\n";
        } else if (cmd == "unsubscribe") {
          auto target = active.get();
          if (!subscriptions[target.second]) {
            std::cout << "Not subscribed to " << target.second << std::endl;
          } else {
            manager.toggle_subscription(target.first, target.second, false);
            subscriptions[target.second] = false;
            std::cout << "Unsubscribed from " << target.second << std::endl;
          }
        } else if (cmd == "unsubscribeall") {
          std::size_t count = manager.toggle_subscription_all(false);
          for (auto &[cube_id, flag] : subscriptions) {
            (void)cube_id;
            flag = false;
          }
          std::cout << "Unsubscribed from " << count << " cubes.\n";
        } else if (cmd == "exit" || cmd == "quit") {
          break;
        } else {
          std::cout << "Unknown command. Type 'help' for options.\n";
        }
      } catch (const std::exception &ex) {
        std::cout << "Command error: " << ex.what() << std::endl;
      }
    }
  } catch (const std::exception &ex) {
    std::cerr << "Fatal error: " << ex.what() << std::endl;
    print_usage(argv[0]);
    return 1;
  }

  return 0;
}
