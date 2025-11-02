#pragma once

#include <IPAddress.h>
#include <WiFi.h>

#include <array>
#include <optional>
#include <string>

namespace firmware {

struct WiFiCredentials {
  std::string ssid;
  std::string password;
};

class WiFiManager {
 public:
  void configure(const WiFiCredentials& primary,
                 const std::optional<WiFiCredentials>& secondary);

  void begin();
  void loop();
  bool ensureConnected(uint32_t timeout_ms);
  void disconnect();

  bool isConnected() const;
  IPAddress ip() const;
  int32_t rssi() const;
  std::string mac() const;

 private:
  WiFiCredentials primary_{};
  std::optional<WiFiCredentials> secondary_{};
  bool configured_ = false;
  unsigned long last_attempt_ms_ = 0;
  uint8_t attempt_count_ = 0;
  bool last_connected_ = false;

  void connect(const WiFiCredentials& credentials);
};

}  // namespace firmware
