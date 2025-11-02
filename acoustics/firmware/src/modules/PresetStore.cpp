#include "PresetStore.h"

#include <Arduino.h>

namespace firmware {

bool PresetStore::load(fs::FS& fs, const char* manifest_path) {
  presets_.clear();

  File file = fs.open(manifest_path, "r");
  if (!file) {
    Serial.printf("[PresetStore] Failed to open manifest: %s\n", manifest_path);
    return false;
  }

  const size_t size = file.size();
  if (size == 0) {
    Serial.println("[PresetStore] Manifest file is empty.");
    return false;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, file);
  if (err) {
    Serial.printf("[PresetStore] Failed to parse manifest: %s\n", err.c_str());
    return false;
  }

  JsonArray presets = doc["presets"].as<JsonArray>();
  if (presets.isNull()) {
    Serial.println("[PresetStore] Manifest missing 'presets' array.");
    return false;
  }

  for (JsonObject entry : presets) {
    PresetDescriptor preset;
    if (entry["id"].is<const char*>()) {
      preset.id = entry["id"].as<std::string>();
    } else if (entry["sample_id"].is<int>()) {
      preset.id = std::to_string(entry["sample_id"].as<int>());
    } else {
      Serial.println("[PresetStore] Entry missing id.");
      continue;
    }

    if (entry["file"].is<const char*>()) {
      preset.file_path = entry["file"].as<std::string>();
    } else if (entry["filename"].is<const char*>()) {
      preset.file_path = entry["filename"].as<std::string>();
    } else {
      Serial.println("[PresetStore] Entry missing file path.");
      continue;
    }

    preset.sample_rate_hz = entry["sample_rate"] | 44100;
    preset.gain = entry["gain"] | 1.0f;
    presets_.push_back(preset);
  }

  Serial.printf("[PresetStore] Loaded %u presets.\n",
                static_cast<unsigned>(presets_.size()));
  return !presets_.empty();
}

std::optional<PresetDescriptor> PresetStore::findById(
    const std::string& id) const {
  for (const auto& preset : presets_) {
    if (preset.id == id) {
      return preset;
    }
  }
  return std::nullopt;
}

}  // namespace firmware
