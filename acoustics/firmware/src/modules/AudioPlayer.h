#pragma once

#include <SPIFFS.h>

#include <memory>
#include <optional>
#include <string>

#include "PresetStore.h"

namespace firmware {

class AudioPlayer {
 public:
  AudioPlayer();

  void begin();
  void loop();

  bool play(const PresetDescriptor& preset, float gain_override = 1.0f);
  void stop();

  bool isPlaying() const;
  std::optional<std::string> currentPreset() const;

 private:
  bool initialized_;
  std::optional<std::string> current_preset_id_;

  struct BufferDeleter {
    void operator()(uint8_t* ptr) const;
  };

  std::unique_ptr<uint8_t, BufferDeleter> wav_buffer_;
  size_t wav_size_ = 0;
};

}  // namespace firmware
