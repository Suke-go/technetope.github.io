#include "OscReceiver.h"

#include <Arduino.h>
#include <OSCBundle.h>
#include <OSCMessage.h>
#include <cstddef>
#include <cstring>

namespace firmware {

namespace {

constexpr uint64_t kNtpUnixOffsetSeconds = 2208988800ULL;
constexpr uint64_t kSecondsToMicros = 1000000ULL;

class ScopedAesContext {
 public:
  ScopedAesContext() { mbedtls_aes_init(&ctx_); }
  ~ScopedAesContext() { mbedtls_aes_free(&ctx_); }
  ScopedAesContext(const ScopedAesContext&) = delete;
  ScopedAesContext& operator=(const ScopedAesContext&) = delete;

  mbedtls_aes_context* get() { return &ctx_; }

 private:
  mbedtls_aes_context ctx_{};
};

uint64_t oscTimeToMicros(const osctime_t& tt) {
  const uint64_t seconds = tt.seconds;
  const double fraction =
      static_cast<double>(tt.fractionofseconds) / 4294967296.0;
  return seconds * kSecondsToMicros +
         static_cast<uint64_t>(fraction * static_cast<double>(kSecondsToMicros));
}

uint64_t oscTimeToUnixMicros(const osctime_t& tt) {
  const uint64_t micros = oscTimeToMicros(tt);
  const uint64_t offset = kNtpUnixOffsetSeconds * kSecondsToMicros;
  if (micros >= offset) {
    return micros - offset;
  }
  return 0ULL;
}

uint64_t bigEndianToUint32(const uint8_t* data) {
  return (static_cast<uint64_t>(data[0]) << 24) |
         (static_cast<uint64_t>(data[1]) << 16) |
         (static_cast<uint64_t>(data[2]) << 8) |
         static_cast<uint64_t>(data[3]);
}

bool decodeBundleTimetag(const uint8_t* data, size_t length,
                         osctime_t& outTimetag) {
  static const char header[] = "#bundle";
  if (length < 16) {
    return false;
  }
  if (std::memcmp(data, header, sizeof(header) - 1) != 0) {
    return false;
  }
  outTimetag.seconds = static_cast<uint32_t>(bigEndianToUint32(data + 8));
  outTimetag.fractionofseconds =
      static_cast<uint32_t>(bigEndianToUint32(data + 12));
  return true;
}

}  // namespace

OscReceiver::OscReceiver() = default;

void OscReceiver::configure(uint16_t listen_port) { listen_port_ = listen_port; }

void OscReceiver::setCryptoKey(const std::array<uint8_t, 32>& key,
                               const std::array<uint8_t, 16>& iv) {
  key_ = key;
  iv_ = iv;
  crypto_enabled_ = true;
}

void OscReceiver::begin() { udp_.begin(listen_port_); }

void OscReceiver::loop(const NtpClient& ntp, PlaybackQueue& queue,
                       const PresetStore& presets) {
  int packet_size = udp_.parsePacket();
  while (packet_size > 0) {
    std::vector<uint8_t> buffer(packet_size);
    udp_.read(buffer.data(), packet_size);
    if (!decryptInPlace(buffer)) {
      Serial.println("[OSC] Failed to decrypt packet.");
      packet_size = udp_.parsePacket();
      continue;
    }
    handlePacket(buffer.data(), buffer.size(), ntp, queue, presets);
    packet_size = udp_.parsePacket();
  }
}

bool OscReceiver::decryptInPlace(std::vector<uint8_t>& buffer) {
  if (!crypto_enabled_) {
    return true;
  }
  if (buffer.size() < sizeof(uint64_t)) {
    Serial.println("[OSC] Encrypted packet too short (missing counter)");
    return false;
  }
  uint64_t counter = 0;
  for (size_t i = 0; i < sizeof(uint64_t); ++i) {
    counter = (counter << 8) | static_cast<uint64_t>(buffer[i]);
  }
  if (counter == 0) {
    Serial.println("[OSC] Invalid encryption counter value");
    return false;
  }
  auto derivedIv = deriveIv(counter);
  buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(sizeof(uint64_t)));
  ScopedAesContext ctx;
  int ret = mbedtls_aes_setkey_enc(ctx.get(), key_.data(), 256);
  if (ret != 0) {
    Serial.printf("[OSC] AES key setup failed: %d\n", ret);
    return false;
  }

  size_t nc_off = 0;
  std::array<uint8_t, 16> nonce = derivedIv;
  uint8_t stream_block[16] = {0};
  ret = mbedtls_aes_crypt_ctr(ctx.get(), buffer.size(), &nc_off, nonce.data(),
                              stream_block, buffer.data(), buffer.data());
  if (ret != 0) {
    Serial.printf("[OSC] AES decrypt failed: %d\n", ret);
    return false;
  }
  return true;
}

std::array<uint8_t, 16> OscReceiver::deriveIv(uint64_t counter) const {
  std::array<uint8_t, 16> derived = iv_;
  uint64_t high = 0;
  uint64_t low = 0;
  for (int i = 0; i < 8; ++i) {
    high = (high << 8) | static_cast<uint64_t>(derived[static_cast<size_t>(i)]);
  }
  for (int i = 8; i < 16; ++i) {
    low = (low << 8) | static_cast<uint64_t>(derived[static_cast<size_t>(i)]);
  }
  const uint64_t previousLow = low;
  low += counter;
  if (low < previousLow) {
    ++high;
  }
  for (int i = 15; i >= 8; --i) {
    derived[static_cast<size_t>(i)] =
        static_cast<uint8_t>(low & 0xFFu);
    low >>= 8U;
  }
  for (int i = 7; i >= 0; --i) {
    derived[static_cast<size_t>(i)] =
        static_cast<uint8_t>(high & 0xFFu);
    high >>= 8U;
  }
  return derived;
}

void OscReceiver::handlePacket(const uint8_t* data, size_t length,
                               const NtpClient& ntp, PlaybackQueue& queue,
                               const PresetStore& presets) {
  OSCBundle bundle;
  osctime_t bundleTimetag{0, 1};
  bool hasBundleTimetag = decodeBundleTimetag(data, length, bundleTimetag);
  if (hasBundleTimetag &&
      bundleTimetag.seconds == 0 &&
      bundleTimetag.fractionofseconds == 1) {
    hasBundleTimetag = false;
  }
  bundle.fill(data, length);
  if (bundle.hasError()) {
    Serial.printf("[OSC] Bundle parse error: %d\n", bundle.getError());
    return;
  }

  const uint64_t now_us = ntp.nowMicros();
  const uint64_t bundle_time_us =
      hasBundleTimetag ? oscTimeToUnixMicros(bundleTimetag) : 0ULL;

  auto computeScheduledTime = [&](OSCMessage* message, bool hasTimetag,
                                  uint64_t timetagUs) -> uint64_t {
    uint64_t scheduled_time_us = hasTimetag ? timetagUs : now_us + 500000ULL;
    if (!hasTimetag && message != nullptr) {
      if (message->isTime(1)) {
        scheduled_time_us = oscTimeToUnixMicros(message->getTime(1));
      } else if (message->isInt64(1)) {
        scheduled_time_us = static_cast<uint64_t>(message->getInt64(1));
      } else if (message->isInt(1)) {
        scheduled_time_us =
            static_cast<uint64_t>(message->getInt(1)) * 1000ULL + now_us;
      }
    }
    if (scheduled_time_us < now_us) {
      scheduled_time_us = now_us;
    }
    return scheduled_time_us;
  };

  for (int i = 0; i < bundle.size(); ++i) {
    OSCMessage* msg = bundle.getOSCMessage(i);
    if (msg == nullptr) {
      continue;
    }

    if (msg->fullMatch("/acoustics/play")) {
      char preset_id[64] = {0};
      if (msg->isString(0)) {
        msg->getString(0, preset_id, sizeof(preset_id));
      } else {
        Serial.println("[OSC] Missing preset id string.");
        continue;
      }

      uint64_t scheduled_time_us =
          computeScheduledTime(msg, hasBundleTimetag, bundle_time_us);

      float gain = 1.0f;
      if (msg->isFloat(2)) {
        gain = msg->getFloat(2);
      }

      bool loop = false;
      if (msg->isInt(3)) {
        loop = msg->getInt(3) != 0;
      }

      const auto preset = presets.findById(preset_id);
      if (!preset) {
        Serial.printf("[OSC] Unknown preset requested: %s\n", preset_id);
        continue;
      }

      PlaybackItem item;
      item.preset_id = preset->id;
      item.start_time_us = scheduled_time_us;
      item.gain = gain;
      item.loop = loop;

      queue.push(item);
      Serial.printf("[OSC] Queued preset %s for %llu us (gain=%.2f loop=%d)\n",
                    preset_id,
                    static_cast<unsigned long long>(scheduled_time_us),
                    gain, loop);

    } else if (msg->fullMatch("/acoustics/stop")) {
      uint64_t scheduled_time_us =
          computeScheduledTime(msg, hasBundleTimetag, bundle_time_us);

      PlaybackItem item;
      item.is_stop = true;
      item.start_time_us = scheduled_time_us;
      queue.push(item);
      if (scheduled_time_us > now_us) {
        Serial.printf("[OSC] Stop scheduled for %llu us\n",
                      static_cast<unsigned long long>(scheduled_time_us));
      } else {
        Serial.println("[OSC] Stop requested.");
      }
    }
  }
}

}  // namespace firmware
