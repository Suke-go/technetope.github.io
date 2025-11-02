#pragma once

#include <NTPClient.h>
#include <WiFiUdp.h>

#include <cstdint>

namespace firmware {

class NtpClient {
 public:
  explicit NtpClient(const char* server = "pool.ntp.org",
                     long time_offset_sec = 0,
                     unsigned long update_interval_ms = 60000);

  void begin();
  bool forceSync(uint32_t timeout_ms);
  void loop();

  void seed(uint32_t epoch_seconds);

  bool isSynced() const;
  uint64_t nowMicros() const;
  uint32_t lastSyncEpoch() const;

 private:
  WiFiUDP udp_;
  NTPClient client_;
  bool synced_ = false;
  uint32_t last_sync_ = 0;
  uint32_t last_sync_millis_ = 0;
};

}  // namespace firmware
