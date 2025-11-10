#include "toio/api/fleet_control.hpp"
#include "toio/cli/config_loader.hpp"
#include "toio/middleware/cube_state.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {

void print_snapshot(const toio::middleware::CubeSnapshot &snapshot) {
  const auto &state = snapshot.state;
  std::cout << std::left << std::setw(15) << state.server_id << std::setw(12)
            << state.cube_id << std::setw(12)
            << (state.connected ? "yes" : "no") << std::setw(10);

  if (state.battery_percent) {
    std::cout << (*state.battery_percent) << "%";
  } else {
    std::cout << "-";
  }

  std::cout << std::setw(18);
  if (state.position) {
    std::cout << state.position->x << "," << state.position->y << ","
              << state.position->angle;
  } else {
    std::cout << "-";
  }

  std::cout << static_cast<int>(state.led.r) << ","
            << static_cast<int>(state.led.g) << ","
            << static_cast<int>(state.led.b) << "\n";
}

} // namespace

int main(int argc, char **argv) {
  try {
    auto options = toio::cli::parse_options(argc, argv);
    auto plan = toio::cli::build_fleet_plan(options);
    toio::api::FleetControl control(plan.configs);

    control.set_goal_logger([](const std::string &key,
                               const std::string &message) {
      std::cout << "[goal " << key << "] " << message << std::endl;
    });

    control.start();

    const auto cubes = control.cubes();
    if (cubes.empty()) {
      std::cerr << "No cubes available in configuration.\n";
      return 1;
    }

    const auto target = cubes.front();
    std::cout << "Using cube " << target.cube_id << " on server "
              << target.server_id << "\n";

    toio::middleware::LedColor color{0, 32, 255};
    control.set_led(target, color);

    toio::control::GoalOptions goal;
    goal.goal_x = 150;
    goal.goal_y = 150;
    control.start_goal(target, goal);

    std::cout << std::left << std::setw(15) << "Server" << std::setw(12)
              << "Cube" << std::setw(12) << "Connected" << std::setw(10)
              << "Battery" << std::setw(18) << "Position"
              << "LED\n";

    for (int i = 0; i < 10; ++i) {
      for (const auto &snapshot : control.snapshot()) {
        print_snapshot(snapshot);
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    control.stop_goal(target);
    control.stop();
  } catch (const std::exception &ex) {
    std::cerr << "Sample failed: " << ex.what() << std::endl;
    toio::cli::print_usage(argv[0]);
    return 1;
  }
  return 0;
}

