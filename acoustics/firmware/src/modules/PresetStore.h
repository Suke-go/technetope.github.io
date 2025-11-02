#pragma once

#include <ArduinoJson.h>
#include <FS.h>

#include <optional>
#include <string>
#include <vector>

namespace firmware {

struct PresetDescriptor {
  std::string id;
  std::string file_path;
  uint32_t sample_rate_hz = 44100;
  float gain = 1.0f;
};

class PresetStore {
 public:
  bool load(fs::FS& fs, const char* manifest_path);
  std::optional<PresetDescriptor> findById(const std::string& id) const;
  const std::vector<PresetDescriptor>& presets() const { return presets_; }

 private:
  std::vector<PresetDescriptor> presets_;
};

}  // namespace firmware
