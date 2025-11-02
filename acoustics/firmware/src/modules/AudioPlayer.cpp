#include "AudioPlayer.h"

#include <Arduino.h>
#include <M5StickCPlus2.h>
#include <esp32-hal-psram.h>

#include <algorithm>

namespace {

uint8_t* allocateAudioBuffer(size_t size) {
  uint8_t* ptr = static_cast<uint8_t*>(ps_malloc(size));
  if (ptr) {
    return ptr;
  }
  return static_cast<uint8_t*>(malloc(size));
}

uint8_t gainToVolume(float gain) {
  float clamped = std::clamp(gain, 0.0f, 1.0f);
  return static_cast<uint8_t>(clamped * 255.0f);
}

}  // namespace

namespace firmware {

void AudioPlayer::BufferDeleter::operator()(uint8_t* ptr) const {
  if (ptr != nullptr) {
    free(ptr);
  }
}

AudioPlayer::AudioPlayer() : initialized_(false) {}

void AudioPlayer::begin() {
  StickCP2.Speaker.stop();
  StickCP2.Speaker.setVolume(200);
  initialized_ = true;
}

void AudioPlayer::loop() {
  if (!initialized_) {
    return;
  }
  if (!StickCP2.Speaker.isPlaying() && current_preset_id_) {
    wav_buffer_.reset();
    wav_size_ = 0;
    current_preset_id_.reset();
  }
}

bool AudioPlayer::play(const PresetDescriptor& preset, float gain_override) {
  if (!initialized_) {
    Serial.println("[AudioPlayer] begin() not called.");
    return false;
  }

  File file = SPIFFS.open(preset.file_path.c_str(), "r");
  if (!file) {
    Serial.printf("[AudioPlayer] Missing file: %s\n", preset.file_path.c_str());
    return false;
  }

  const size_t size = file.size();
  if (size == 0) {
    Serial.printf("[AudioPlayer] File empty: %s\n", preset.file_path.c_str());
    return false;
  }

  auto buffer = allocateAudioBuffer(size);
  if (!buffer) {
    Serial.println("[AudioPlayer] Failed to allocate audio buffer.");
    return false;
  }

  const size_t bytes_read = file.read(buffer, size);
  file.close();
  if (bytes_read != size) {
    Serial.printf("[AudioPlayer] Failed to read file (%u/%u)\n",
                  static_cast<unsigned>(bytes_read),
                  static_cast<unsigned>(size));
    free(buffer);
    return false;
  }

  const float resolved_gain = preset.gain * gain_override;
  StickCP2.Speaker.setVolume(gainToVolume(resolved_gain));

  if (!StickCP2.Speaker.playWav(buffer, size, 1, 0, true)) {
    Serial.println("[AudioPlayer] playWav rejected.");
    free(buffer);
    return false;
  }

  wav_buffer_.reset(buffer);
  wav_size_ = size;
  current_preset_id_ = preset.id;
  return true;
}

void AudioPlayer::stop() {
  StickCP2.Speaker.stop();
  wav_buffer_.reset();
  wav_size_ = 0;
  current_preset_id_.reset();
}

bool AudioPlayer::isPlaying() const {
  return StickCP2.Speaker.isPlaying();
}

std::optional<std::string> AudioPlayer::currentPreset() const {
  return current_preset_id_;
}

}  // namespace firmware
