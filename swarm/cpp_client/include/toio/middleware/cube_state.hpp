#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace toio::middleware {

struct Position {
  int x = 0;
  int y = 0;
  int angle = 0;
  std::uint64_t timestamp_ms = 0;
  bool on_mat = false;
};

struct LedColor {
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
};

struct CubeState {
  std::string server_id;
  std::string cube_id;
  bool connected = false;
  std::optional<Position> position;
  std::optional<int> battery_percent;
  LedColor led{};
  std::chrono::steady_clock::time_point last_update{};
};

struct CubeSnapshot {
  CubeState state;
  std::chrono::system_clock::time_point exported_at =
      std::chrono::system_clock::now();
};

} // namespace toio::middleware
