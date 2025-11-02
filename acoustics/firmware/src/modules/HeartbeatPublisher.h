#pragma once

#include <WiFiUdp.h>

#include <string>

#include <OSCMessage.h>

#include "AudioPlayer.h"
#include "NtpClient.h"
#include "PlaybackQueue.h"
#include "WiFiManager.h"

namespace firmware {

class HeartbeatPublisher {
 public:
  HeartbeatPublisher();

  void configure(const std::string& host, uint16_t port,
                 std::string device_id,
                 std::string firmware_version);
  void begin();
  void loop(const WiFiManager& wifi, const NtpClient& ntp,
            const PlaybackQueue& queue, const AudioPlayer& player);

 private:
  void sendMessage(OSCMessage& message);
  void sendAnnounce(const WiFiManager& wifi);
  void sendHeartbeat(const NtpClient& ntp,
                     const PlaybackQueue& queue,
                     const AudioPlayer& player);

  WiFiUDP udp_;
  std::string remote_host_;
  uint16_t remote_port_ = 0;
  unsigned long last_send_ms_ = 0;
  bool announced_ = false;
  uint32_t sequence_ = 0;
  std::string device_id_;
  std::string firmware_version_;
};

}  // namespace firmware
