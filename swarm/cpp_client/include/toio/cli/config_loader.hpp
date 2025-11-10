#pragma once

#include "toio/middleware/server_session.hpp"

#include <string>
#include <utility>
#include <vector>

namespace toio::cli {

struct Options {
  std::string fleet_config_path;
};

struct FleetPlan {
  std::vector<middleware::ServerConfig> configs;
  std::vector<std::pair<std::string, std::string>> cube_sequence;
};

Options parse_options(int argc, char **argv);
FleetPlan build_fleet_plan(const Options &options);
void print_usage(const char *argv0);

} // namespace toio::cli

