#include "toio/cli/config_loader.hpp"

#include <cstdlib>
#include <iostream>
#include <random>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace toio::cli {

using toio::middleware::CubeConfig;
using toio::middleware::LedColor;
using toio::middleware::ServerConfig;

namespace {

LedColor random_led_color() {
  static std::mt19937 rng(std::random_device{}());
  static std::uniform_int_distribution<int> dist(0, 255);
  LedColor color;
  color.r = static_cast<std::uint8_t>(dist(rng));
  color.g = static_cast<std::uint8_t>(dist(rng));
  color.b = static_cast<std::uint8_t>(dist(rng));
  return color;
}

template <typename T>
T scalar_or_throw(const YAML::Node &node, const std::string &field) {
  if (!node || !node.IsScalar()) {
    throw std::runtime_error("Field '" + field + "' must be a scalar");
  }
  return node.as<T>();
}

std::vector<ServerConfig> load_fleet_config(const std::string &path) {
  YAML::Node root = YAML::LoadFile(path);
  auto servers_node = root["servers"];
  if (!servers_node || !servers_node.IsSequence()) {
    throw std::runtime_error("fleet.yaml must contain 'servers' sequence");
  }

  std::vector<ServerConfig> configs;
  for (const auto &server_node : servers_node) {
    ServerConfig config;
    config.id = scalar_or_throw<std::string>(server_node["id"], "servers[].id");
    config.host =
        scalar_or_throw<std::string>(server_node["host"], "servers[].host");
    config.port =
        scalar_or_throw<std::string>(server_node["port"], "servers[].port");
    if (server_node["endpoint"]) {
      config.endpoint = server_node["endpoint"].as<std::string>();
    }
    if (server_node["default_require_result"]) {
      config.default_require_result =
          server_node["default_require_result"].as<bool>();
    }
    if (auto cubes_node = server_node["cubes"]; cubes_node && cubes_node.IsSequence()) {
      for (const auto &cube_node : cubes_node) {
        CubeConfig cube;
        cube.id =
            scalar_or_throw<std::string>(cube_node["id"], "servers[].cubes[].id");
        cube.auto_connect = cube_node["auto_connect"].as<bool>(true);
        cube.auto_subscribe = cube_node["auto_subscribe"].as<bool>(false);
        cube.initial_led = random_led_color();
        if (auto led_node = cube_node["initial_led"]; led_node) {
          if (!led_node.IsSequence() || led_node.size() != 3) {
            throw std::runtime_error("initial_led must be an array of 3 ints");
          }
          LedColor color;
          color.r = static_cast<std::uint8_t>(led_node[0].as<int>(0));
          color.g = static_cast<std::uint8_t>(led_node[1].as<int>(0));
          color.b = static_cast<std::uint8_t>(led_node[2].as<int>(0));
          cube.initial_led = color;
        }
        config.cubes.push_back(std::move(cube));
      }
    }
    configs.push_back(std::move(config));
  }
  return configs;
}

} // namespace

void print_usage(const char *argv0) {
  std::cout << "Usage: " << argv0 << " --fleet-config <fleet.yaml>\n";
}

Options parse_options(int argc, char **argv) {
  Options opt;
  bool has_config = false;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--fleet-config" && i + 1 < argc) {
      opt.fleet_config_path = argv[++i];
      has_config = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (!has_config) {
    throw std::runtime_error("--fleet-config is required");
  }

  return opt;
}

FleetPlan build_fleet_plan(const Options &options) {
  FleetPlan plan;
  plan.configs = load_fleet_config(options.fleet_config_path);

  for (const auto &server : plan.configs) {
    for (const auto &cube : server.cubes) {
      plan.cube_sequence.emplace_back(server.id, cube.id);
    }
  }

  if (plan.cube_sequence.empty()) {
    throw std::runtime_error("No cubes defined in configuration");
  }
  return plan;
}

} // namespace toio::cli
