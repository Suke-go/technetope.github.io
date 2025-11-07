#pragma once

#include <WiFiUdp.h>
#include <mbedtls/aes.h>

#include <array>
#include <cstdint>
#include <vector>

#include "NtpClient.h"
#include "PlaybackQueue.h"
#include "PresetStore.h"

namespace firmware {

class OscReceiver {
 public:
  OscReceiver();

  void configure(uint16_t listen_port);
  void setCryptoKey(const std::array<uint8_t, 32>& key,
                    const std::array<uint8_t, 16>& iv);
  void begin();
  void loop(const NtpClient& ntp, PlaybackQueue& queue,
            const PresetStore& presets);

 private:
  WiFiUDP udp_;
  uint16_t listen_port_ = 0;
  bool crypto_enabled_ = false;
  std::array<uint8_t, 32> key_{};
  std::array<uint8_t, 16> iv_{};

  bool decryptInPlace(std::vector<uint8_t>& buffer);
  std::array<uint8_t, 16> deriveIv(uint64_t counter) const;
  void handlePacket(const uint8_t* data, size_t length, const NtpClient& ntp,
                    PlaybackQueue& queue, const PresetStore& presets);
};

}  // namespace firmware
