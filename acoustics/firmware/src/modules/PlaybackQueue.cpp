#include "PlaybackQueue.h"

#include <algorithm>

namespace firmware {

PlaybackQueue::PlaybackQueue() : mux_(portMUX_INITIALIZER_UNLOCKED) {}

bool PlaybackQueue::push(const PlaybackItem& item) {
  portENTER_CRITICAL(&mux_);
  queue_.push_back(item);
  std::sort(queue_.begin(), queue_.end(),
            [](const PlaybackItem& lhs, const PlaybackItem& rhs) {
              return lhs.start_time_us < rhs.start_time_us;
            });
  portEXIT_CRITICAL(&mux_);
  return true;
}

std::optional<PlaybackItem> PlaybackQueue::peek() const {
  portENTER_CRITICAL(&mux_);
  std::optional<PlaybackItem> result;
  if (!queue_.empty()) {
    result = queue_.front();
  }
  portEXIT_CRITICAL(&mux_);
  return result;
}

std::optional<PlaybackItem> PlaybackQueue::pop() {
  portENTER_CRITICAL(&mux_);
  std::optional<PlaybackItem> result;
  if (!queue_.empty()) {
    result = queue_.front();
    queue_.pop_front();
  }
  portEXIT_CRITICAL(&mux_);
  return result;
}

std::optional<PlaybackItem> PlaybackQueue::popDue(uint64_t now_us) {
  portENTER_CRITICAL(&mux_);
  std::optional<PlaybackItem> result;
  if (!queue_.empty() && queue_.front().start_time_us <= now_us) {
    result = queue_.front();
    queue_.pop_front();
  }
  portEXIT_CRITICAL(&mux_);
  return result;
}

size_t PlaybackQueue::size() const {
  portENTER_CRITICAL(&mux_);
  size_t result = queue_.size();
  portEXIT_CRITICAL(&mux_);
  return result;
}

void PlaybackQueue::clear() {
  portENTER_CRITICAL(&mux_);
  queue_.clear();
  portEXIT_CRITICAL(&mux_);
}

}  // namespace firmware
