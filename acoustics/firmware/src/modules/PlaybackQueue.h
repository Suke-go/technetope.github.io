#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <deque>
#include <optional>
#include <string>

namespace firmware {

struct PlaybackItem {
  std::string preset_id;
  uint64_t start_time_us = 0;
  float gain = 1.0f;
  bool loop = false;
};

class PlaybackQueue {
 public:
  PlaybackQueue();

  bool push(const PlaybackItem& item);
  std::optional<PlaybackItem> peek() const;
  std::optional<PlaybackItem> pop();
  std::optional<PlaybackItem> popDue(uint64_t now_us);
  size_t size() const;
  void clear();

 private:
  mutable portMUX_TYPE mux_;
  std::deque<PlaybackItem> queue_;
};

}  // namespace firmware
