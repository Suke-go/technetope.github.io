// #pragma once

// #include <array>

// namespace secrets {

// // Replace with the Wi-Fi credentials used at the deployment site.
// constexpr const char WIFI_PRIMARY_SSID[] = "YOURSSID";
// constexpr const char WIFI_PRIMARY_PASS[] = "YOURPASSWORD";

// // Optional backup network.
// constexpr const char WIFI_SECONDARY_SSID[] = "";
// constexpr const char WIFI_SECONDARY_PASS[] = "";

// // OSC endpoints
// constexpr uint16_t OSC_LISTEN_PORT = 9000;
// constexpr uint16_t HEARTBEAT_REMOTE_PORT = 9100;
// constexpr const char HEARTBEAT_REMOTE_HOST[] = "192.168.4.2";

// // NTP settings (override in Secrets.h)
// constexpr const char NTP_SERVER[] = "pool.ntp.org";
// constexpr long NTP_TIME_OFFSET_SEC = 0;       // UTC by default
// constexpr unsigned long NTP_UPDATE_INTERVAL_MS = 30000;  // 30 second refresh

// // AES-CTR 256-bit key/IV for OSC payload decryption.
// // Populate with production values and keep the actual file out of version control.
// constexpr std::array<uint8_t, 32> OSC_AES_KEY = {
//     0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
//     0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
//     0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
//     0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};

// constexpr std::array<uint8_t, 16> OSC_AES_IV = {
//     0xA0, 0xA1, 0xA2, 0xA3,
//     0xA4, 0xA5, 0xA6, 0xA7,
//     0xA8, 0xA9, 0xAA, 0xAB,
//     0xAC, 0xAD, 0xAE, 0xAF};

// // NTP settings
// constexpr const char NTP_SERVER[] = "pool.ntp.org";
// constexpr long NTP_TIME_OFFSET_SEC = 0;
// constexpr unsigned long NTP_UPDATE_INTERVAL_MS = 60'000;

// }  // namespace secrets
