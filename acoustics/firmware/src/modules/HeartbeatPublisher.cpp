#include "HeartbeatPublisher.h"

#include <Arduino.h>
#include <cstring>

namespace firmware {

HeartbeatPublisher::HeartbeatPublisher() = default;

void HeartbeatPublisher::configure(const std::string& host, uint16_t port,
                                   std::string device_id,
                                   std::string firmware_version) {
  remote_host_ = host;
  remote_port_ = port;
  device_id_ = std::move(device_id);
  firmware_version_ = std::move(firmware_version);
}

void HeartbeatPublisher::begin() { udp_.begin(0); }

void HeartbeatPublisher::loop(const WiFiManager& wifi, const NtpClient& ntp,
                              const PlaybackQueue& queue,
                              const AudioPlayer& player) {
  const unsigned long now = millis();
  if (now - last_send_ms_ < 1000) {
    return;
  }
  last_send_ms_ = now;

  if (!wifi.isConnected() || remote_host_.empty() || remote_port_ == 0) {
    announced_ = false;
    sequence_ = 0;
    return;
  }

  if (!announced_) {
    sendAnnounce(wifi);
    announced_ = true;
  }

  if (!ntp.isSynced()) {
    // Skip heartbeat until NTP has provided a valid epoch time.
    return;
  }

  sendHeartbeat(ntp, queue, player);
}

void HeartbeatPublisher::sendMessage(OSCMessage& message) {
  udp_.beginPacket(remote_host_.c_str(), remote_port_);
  message.send(udp_);
  udp_.endPacket();
  message.empty();
}

void HeartbeatPublisher::sendAnnounce(const WiFiManager& wifi) {
  OSCMessage msg("/announce");
  msg.add(device_id_.c_str());
  const std::string mac = wifi.mac();
  msg.add(mac.c_str());
  msg.add(firmware_version_.c_str());
  sendMessage(msg);
  Serial.printf("[Heartbeat] announce sent id=%s mac=%s\n",
                device_id_.c_str(), mac.c_str());
}

void HeartbeatPublisher::sendHeartbeat(const NtpClient& ntp,
                                       const PlaybackQueue& queue,
                                       const AudioPlayer& player) {
  OSCMessage msg("/heartbeat");
  msg.add(device_id_.c_str());
  msg.add(static_cast<int32_t>(sequence_++));
  const uint64_t micros = ntp.nowMicros();
  const uint32_t seconds = static_cast<uint32_t>(micros / 1'000'000ULL);
  const uint32_t microsPart = static_cast<uint32_t>(micros % 1'000'000ULL);
  msg.add(static_cast<int32_t>(seconds));
  msg.add(static_cast<int32_t>(microsPart));
  msg.add(static_cast<int32_t>(queue.size()));
  msg.add(player.isPlaying() ? 1 : 0);
  sendMessage(msg);
  Serial.printf("[Heartbeat] sent seq=%lu queue=%u playing=%d\n",
                static_cast<unsigned long>(sequence_ - 1),
                static_cast<unsigned>(queue.size()),
                player.isPlaying() ? 1 : 0);
}

}  // namespace firmware
