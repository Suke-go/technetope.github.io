#include "OscReceiver.h"

#include <Arduino.h>
#include <OSCBundle.h>
#include <OSCMessage.h>

namespace firmware {

namespace {

uint64_t oscTimeToMicros(const osctime_t& tt) {
  const uint64_t seconds = tt.seconds;
  const double fraction =
      static_cast<double>(tt.fractionofseconds) / 4294967296.0;
  return seconds * 1000000ULL + static_cast<uint64_t>(fraction * 1000000.0);
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
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  int ret = mbedtls_aes_setkey_enc(&ctx, key_.data(), 256);
  if (ret != 0) {
    Serial.printf("[OSC] AES key setup failed: %d\n", ret);
    mbedtls_aes_free(&ctx);
    return false;
  }

  size_t nc_off = 0;
  std::array<uint8_t, 16> nonce = iv_;
  uint8_t stream_block[16] = {0};
  ret = mbedtls_aes_crypt_ctr(&ctx, buffer.size(), &nc_off, nonce.data(),
                              stream_block, buffer.data(), buffer.data());
  mbedtls_aes_free(&ctx);
  if (ret != 0) {
    Serial.printf("[OSC] AES decrypt failed: %d\n", ret);
    return false;
  }
  return true;
}

void OscReceiver::handlePacket(const uint8_t* data, size_t length,
                               const NtpClient& ntp, PlaybackQueue& queue,
                               const PresetStore& presets) {
  OSCBundle bundle;
  bundle.fill(data, length);
  if (bundle.hasError()) {
    Serial.printf("[OSC] Bundle parse error: %d\n", bundle.getError());
    return;
  }

  const uint64_t now_us = ntp.nowMicros();

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

      uint64_t scheduled_time_us = now_us + 500000ULL;
      if (msg->isTime(1)) {
        scheduled_time_us = oscTimeToMicros(msg->getTime(1));
      } else if (msg->isInt64(1)) {
        scheduled_time_us = static_cast<uint64_t>(msg->getInt64(1));
      } else if (msg->isInt(1)) {
        scheduled_time_us =
            static_cast<uint64_t>(msg->getInt(1)) * 1000ULL + now_us;
      }

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
      queue.clear();
      Serial.println("[OSC] Stop requested.");
    }
  }
}

}  // namespace firmware
