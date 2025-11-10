#include "motion_planner.hpp"

#include "toio/api/fleet_control.hpp"
#include "toio/cli/config_loader.hpp"
#include "toio/middleware/cube_state.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

namespace {

std::vector<toio::middleware::Position>
extract_positions(const std::vector<toio::api::CubeHandle> &cubes,
                  const std::vector<toio::middleware::CubeSnapshot> &snapshots) {
  std::vector<toio::middleware::Position> positions;
  positions.reserve(cubes.size());
  for (const auto &cube : cubes) {
    toio::middleware::Position position{};
    bool found = false;
    for (const auto &snapshot : snapshots) {
      if (snapshot.state.server_id == cube.server_id &&
          snapshot.state.cube_id == cube.cube_id &&
          snapshot.state.position.has_value()) {
        position = *snapshot.state.position;
        found = true;
        break;
      }
    }
    if (!found) {
      position.x = 0;
      position.y = 0;
      position.angle = 0;
    }
    positions.push_back(position);
  }
  return positions;
}

constexpr std::chrono::milliseconds kUpdateInterval{120};
constexpr std::chrono::seconds kDemoDuration{30};
constexpr std::chrono::seconds kConnectionTimeout{30};
constexpr std::chrono::milliseconds kConnectionPoll{200};
std::atomic<bool> g_interrupted{false};

void handle_sigint(int) {
  g_interrupted.store(true);
}

struct PendingConnect {
  PendingConnect() : future(promise.get_future()) {}
  std::promise<bool> promise;
  std::future<bool> future;
  std::mutex mutex;
  std::string message;
};

bool wait_for_snapshot_connection(toio::api::FleetControl &control,
                                  const toio::api::CubeHandle &cube) {
  const auto deadline =
      std::chrono::steady_clock::now() + kConnectionTimeout;

  while (std::chrono::steady_clock::now() < deadline &&
         !g_interrupted.load()) {
    auto snapshots = control.snapshot();
    for (const auto &snap : snapshots) {
      if (snap.state.server_id == cube.server_id &&
          snap.state.cube_id == cube.cube_id && snap.state.connected) {
        return true;
      }
    }
    std::this_thread::sleep_for(kConnectionPoll);
  }

  return false;
}

} // namespace

int main(int argc, char **argv) {
  try {
    std::signal(SIGINT, handle_sigint);
    const auto options = toio::cli::parse_options(argc, argv);
    const auto plan = toio::cli::build_fleet_plan(options);
    toio::api::FleetControl control(plan.configs);

    control.set_goal_logger([](const std::string &key,
                               const std::string &message) {
      std::cout << "[goal " << key << "] " << message << std::endl;
    });

    std::mutex connect_mutex;
    std::unordered_map<std::string, std::shared_ptr<PendingConnect>>
        pending_connects;

    control.set_message_callback(
        [&](const std::string &server_id, const nlohmann::json &json) {
          auto type_it = json.find("type");
          if (type_it == json.end() || !type_it->is_string() ||
              *type_it != "result") {
            return;
          }
          auto payload_it = json.find("payload");
          if (payload_it == json.end() || !payload_it->is_object()) {
            return;
          }
          const auto &payload = *payload_it;
          if (payload.value("cmd", "") != "connect") {
            return;
          }
          const std::string target = payload.value("target", "");
          if (target.empty()) {
            return;
          }
          const std::string key = server_id + ":" + target;
          std::shared_ptr<PendingConnect> pending;
          {
            std::lock_guard<std::mutex> lock(connect_mutex);
            auto it = pending_connects.find(key);
            if (it == pending_connects.end()) {
              return;
            }
            pending = it->second;
            pending_connects.erase(it);
          }
          std::string message;
          if (auto message_it = payload.find("message");
              message_it != payload.end() && message_it->is_string()) {
            message = message_it->get<std::string>();
          } else if (auto reason_it = payload.find("reason");
                     reason_it != payload.end() && reason_it->is_string()) {
            message = reason_it->get<std::string>();
          }
          {
            std::lock_guard<std::mutex> guard(pending->mutex);
            pending->message = std::move(message);
          }
          const bool success = payload.value("status", "") == "success";
          pending->promise.set_value(success);
        });

    control.start();

    const auto cubes = control.cubes();
    if (cubes.empty()) {
      std::cerr << "No cubes available in configuration.\n";
      return 1;
    }

    std::vector<toio::api::CubeHandle> active_cubes;
    active_cubes.reserve(cubes.size());
    for (const auto &cube : cubes) {
      if (g_interrupted.load()) {
        break;
      }
      const std::string key = cube.server_id + ":" + cube.cube_id;
      auto pending = std::make_shared<PendingConnect>();
      {
        std::lock_guard<std::mutex> lock(connect_mutex);
        pending_connects[key] = pending;
      }

      std::cout << "Connecting to " << cube.server_id << ":" << cube.cube_id
                << " ... " << std::flush;
      if (!control.connect(cube, true)) {
        std::lock_guard<std::mutex> lock(connect_mutex);
        pending_connects.erase(key);
        std::cout << "failed (command dispatch)\n";
        continue;
      }

      bool command_success = false;
      const auto result_status =
          pending->future.wait_for(kConnectionTimeout);
      if (result_status == std::future_status::ready) {
        command_success = pending->future.get();
      } else {
        {
          std::lock_guard<std::mutex> lock(connect_mutex);
          pending_connects.erase(key);
        }
        std::cout << "failed (no response)\n";
        continue;
      }

      if (!command_success) {
        std::string message;
        {
          std::lock_guard<std::mutex> guard(pending->mutex);
          message = pending->message;
        }
        if (!message.empty()) {
          std::cout << "failed: " << message << "\n";
        } else {
          std::cout << "failed\n";
        }
        continue;
      }

      if (wait_for_snapshot_connection(control, cube)) {
        std::cout << "connected\n";
        active_cubes.push_back(cube);
      } else {
        std::cout << "failed (state update timeout)\n";
      }
    }

    if (g_interrupted.load()) {
      std::cout << "\nInterrupted. Stopping demo.\n";
      control.stop_all_goals();
      control.stop();
      return 0;
    }

    if (active_cubes.empty()) {
      std::cerr << "No cubes connected. Aborting demo.\n";
      return 1;
    }

    std::cout << "Driving " << active_cubes.size()
              << " cube(s) using the motion planner demo.\n";

    swarm::samples::MotionPlanner planner;
    auto goal_for_position = [](double x, double y) {
      toio::control::GoalOptions goal;
      goal.goal_x = static_cast<int>(std::lround(x));
      goal.goal_y = static_cast<int>(std::lround(y));
      goal.stop_dist = 5.0;
      goal.poll_interval = std::chrono::milliseconds(120);
      goal.vmax = 80.0;
      goal.wmax = 80.0;
      return goal;
    };

    
    auto initial_targets = planner.initial_targets(active_cubes.size());
    for (std::size_t i = 0; i < active_cubes.size(); ++i) {
      if (i >= initial_targets.size()) {
        break;
      }
      const auto &target = initial_targets[i];
      control.start_goal(active_cubes[i], goal_for_position(target.x, target.y));
    }

    const auto start_time = std::chrono::steady_clock::now();
    const auto end_time = start_time + kDemoDuration;

    while (std::chrono::steady_clock::now() < end_time &&
           !g_interrupted.load()) {
      auto snapshots = control.snapshot();
      auto positions = extract_positions(active_cubes, snapshots);
      auto targets = planner.next_targets(positions);
      if (targets.size() != active_cubes.size()) {
        std::cerr << "Planner output size mismatch, skipping update cycle.\n";
        std::this_thread::sleep_for(kUpdateInterval);
        continue;
      }

      for (std::size_t i = 0; i < active_cubes.size(); ++i) {
        const auto &target = targets[i];
        if (!control.update_goal(active_cubes[i],
                                 goal_for_position(target.x, target.y))) {
          std::cerr << "Failed to update goal for cube "
                    << active_cubes[i].cube_id
                    << "\n";
        }
      }
      std::this_thread::sleep_for(kUpdateInterval);
    }

    control.stop_all_goals();
    control.stop();
  } catch (const std::exception &ex) {
    std::cerr << "Circle sample failed: " << ex.what() << std::endl;
    toio::cli::print_usage(argv[0]);
    return 1;
  }
  return 0;
}
