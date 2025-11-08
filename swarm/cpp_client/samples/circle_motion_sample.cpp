#include "toio/api/fleet_control.hpp"
#include "toio/cli/config_loader.hpp"

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

constexpr double kCenterX = 250.0;
constexpr double kCenterY = 250.0;
constexpr double kBaseRadius = 120.0;
constexpr double kRadiusAmplitude = 30.0;
constexpr double kAngularSpeed = 0.45;            // rad/s
constexpr double kRadiusOscillationPeriod = 14.0; // seconds
constexpr double kDirectionFlipInterval = 6.0;    // seconds
constexpr std::chrono::milliseconds kUpdateInterval{120};
constexpr std::chrono::seconds kDemoDuration{30};
constexpr std::chrono::seconds kConnectionTimeout{30};
constexpr std::chrono::milliseconds kConnectionPoll{200};

std::atomic<bool> g_interrupted{false};

void handle_sigint(int) {
  g_interrupted.store(true);
}

double oscillating_radius(double elapsed_seconds) {
  if (kRadiusOscillationPeriod <= 0.0 || kRadiusAmplitude <= 0.0) {
    return kBaseRadius;
  }
  constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
  double phase = kTwoPi * elapsed_seconds / kRadiusOscillationPeriod;
  double radius = kBaseRadius + kRadiusAmplitude * std::sin(phase);
  return std::max(30.0, radius);
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
              << " cube(s) around a dynamic circle centered at (" << kCenterX
              << ", " << kCenterY << ").\n";

    const double delta = 2.0 * 3.14159265358979323846 /
                         static_cast<double>(active_cubes.size());
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
    double initial_radius = oscillating_radius(0.0);
    for (std::size_t i = 0; i < active_cubes.size(); ++i) {
      const double angle = delta * static_cast<double>(i);
      const double x = kCenterX + initial_radius * std::cos(angle);
      const double y = kCenterY + initial_radius * std::sin(angle);
      control.start_goal(active_cubes[i], goal_for_position(x, y));
    }

    const auto start_time = std::chrono::steady_clock::now();
    auto last_time = start_time;
    const auto end_time = start_time + kDemoDuration;
    double base_angle = 0.0;
    double direction = 1.0;
    double direction_timer = 0.0;

    while (std::chrono::steady_clock::now() < end_time &&
           !g_interrupted.load()) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed =
          std::chrono::duration<double>(now - start_time).count();
      const double dt =
          std::chrono::duration<double>(now - last_time).count();
      last_time = now;

      direction_timer += dt;
      if (direction_timer >= kDirectionFlipInterval) {
        direction_timer -= kDirectionFlipInterval;
        direction = -direction;
      }

      base_angle += direction * kAngularSpeed * dt;
      const double radius = oscillating_radius(elapsed);

      for (std::size_t i = 0; i < active_cubes.size(); ++i) {
        const double angle = base_angle + delta * static_cast<double>(i);
        const double x = kCenterX + radius * std::cos(angle);
        const double y = kCenterY + radius * std::sin(angle);
        if (!control.update_goal(active_cubes[i], goal_for_position(x, y))) {
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
