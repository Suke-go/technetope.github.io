#include <M5StickCPlus2.h>
#include <SPIFFS.h>

#include <optional>
#include <cstring>
#include <cctype>
#include <ctime>
#include <string>

#include "modules/AudioPlayer.h"
#include "modules/HeartbeatPublisher.h"
#include "modules/NtpClient.h"
#include "modules/OscReceiver.h"
#include "modules/PlaybackQueue.h"
#include "modules/PresetStore.h"
#include "modules/WiFiManager.h"

#if __has_include("Secrets.h")
#include "Secrets.h"
#else
#include "Secrets.example.h"
#endif

namespace firmware {

WiFiManager wifi_manager;
NtpClient ntp_client(secrets::NTP_SERVER,
                     secrets::NTP_TIME_OFFSET_SEC,
                     secrets::NTP_UPDATE_INTERVAL_MS);
PresetStore preset_store;
PlaybackQueue playback_queue;
AudioPlayer audio_player;
OscReceiver osc_receiver;
HeartbeatPublisher heartbeat;

std::string device_id;

TaskHandle_t wifi_task_handle = nullptr;
TaskHandle_t ntp_task_handle = nullptr;
TaskHandle_t osc_task_handle = nullptr;
TaskHandle_t playback_task_handle = nullptr;
TaskHandle_t heartbeat_task_handle = nullptr;

void refreshStatusDisplay();
std::string makeDeviceIdFromMac(const std::string& mac);

constexpr const char kFirmwareVersion[] = "0.1.0-dev";

std::optional<uint32_t> rtcEpochSeconds();
uint32_t tmToEpochSeconds(const tm& tmStruct);
void updateRtcFromEpoch(uint32_t epochSeconds);

void wifiTask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    wifi_manager.loop();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void ntpTask(void* pvParameters) {
  (void)pvParameters;
  bool initial_sync_done = false;
  for (;;) {
    if (!wifi_manager.isConnected()) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    if (!initial_sync_done) {
      if (ntp_client.forceSync(5000)) {
        initial_sync_done = true;
        updateRtcFromEpoch(ntp_client.lastSyncEpoch());
      } else {
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    } else {
      auto before = ntp_client.lastSyncEpoch();
      ntp_client.loop();
      if (ntp_client.isSynced() && ntp_client.lastSyncEpoch() != before) {
        updateRtcFromEpoch(ntp_client.lastSyncEpoch());
      }
      vTaskDelay(pdMS_TO_TICKS(250));
    }
  }
}

void oscTask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    osc_receiver.loop(ntp_client, playback_queue, preset_store);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void playbackTask(void* pvParameters) {
  (void)pvParameters;
  std::optional<PlaybackItem> active_item;

  for (;;) {
    const uint64_t now_us = ntp_client.nowMicros();

    if (!audio_player.isPlaying()) {
      if (active_item && active_item->loop) {
        auto preset = preset_store.findById(active_item->preset_id);
        if (preset) {
          audio_player.play(*preset, active_item->gain);
        }
      } else {
        active_item.reset();
      }
    }

    auto due = playback_queue.popDue(now_us);
    if (due) {
      const auto preset = preset_store.findById(due->preset_id);
      if (!preset) {
        Serial.printf("[Playback] Missing preset for id %s\n",
                      due->preset_id.c_str());
      } else if (audio_player.play(*preset, due->gain)) {
        active_item = due;
        Serial.printf("[Playback] Started preset %s\n",
                      due->preset_id.c_str());
      }
    }

    audio_player.loop();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void heartbeatTask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    heartbeat.loop(wifi_manager, ntp_client, playback_queue, audio_player);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void refreshStatusDisplay() {
  static uint32_t last_update_ms = 0;
  const uint32_t now = millis();
  if (now - last_update_ms < 500) {
    return;
  }
  last_update_ms = now;

  auto& display = StickCP2.Display;
  display.startWrite();
  display.fillRect(0, 0, display.width(), display.height(), BLACK);
  display.setCursor(4, 16);
  display.setTextColor(WHITE, BLACK);
  display.setTextSize(1);

  const bool wifi_ok = wifi_manager.isConnected();
  display.printf("WiFi : %s\n", wifi_ok ? "OK" : "----");
  if (wifi_ok) {
    String ip = wifi_manager.ip().toString();
    display.printf("IP   : %s\n", ip.c_str());
    display.printf("RSSI : %d dBm\n", wifi_manager.rssi());
  } else {
    display.println("IP   : ---.---.---.---");
    display.println("RSSI : ---");
  }

  display.printf("NTP  : %s\n", ntp_client.isSynced() ? "SYNC" : "----");
  display.printf("Queue: %u\n",
                 static_cast<unsigned>(playback_queue.size()));

  const auto current = audio_player.currentPreset();
  if (current) {
    display.printf("Play : %s\n", current->c_str());
  } else {
    display.println("Play : -");
  }

  if (!device_id.empty()) {
    display.printf("ID   : %s\n", device_id.c_str());
  }

  display.printf("HB -> %s:%u\n",
                 secrets::HEARTBEAT_REMOTE_HOST,
                 static_cast<unsigned>(secrets::HEARTBEAT_REMOTE_PORT));

  display.endWrite();
}

std::string makeDeviceIdFromMac(const std::string& mac) {
  std::string normalized;
  normalized.reserve(mac.size());
  for (char c : mac) {
    if (c == ':' || c == '-') {
      continue;
    }
    normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return "dev-" + normalized;
}

uint32_t tmToEpochSeconds(const tm& tmStruct) {
  tm copy = tmStruct;
#if defined(_WIN32)
  return static_cast<uint32_t>(_mkgmtime(&copy));
#else
  return static_cast<uint32_t>(timegm(&copy));
#endif
}

std::optional<uint32_t> rtcEpochSeconds() {
  if (StickCP2.Rtc.getVoltLow()) {
    return std::nullopt;
  }
  rtc_datetime_t datetime = StickCP2.Rtc.getDateTime();
  tm tmUtc = datetime.get_tm();
  auto epoch = tmToEpochSeconds(tmUtc);
  if (epoch == 0) {
    return std::nullopt;
  }
  return epoch;
}

void updateRtcFromEpoch(uint32_t epochSeconds) {
  time_t raw = static_cast<time_t>(epochSeconds);
  tm tmUtc{};
#if defined(_WIN32)
  gmtime_s(&tmUtc, &raw);
#else
  gmtime_r(&raw, &tmUtc);
#endif
  StickCP2.Rtc.setDateTime(&tmUtc);
  Serial.printf("[RTC] Updated (epoch=%lu)\n",
                static_cast<unsigned long>(epochSeconds));
}

}  // namespace firmware

void setup() {
  using namespace firmware;

  Serial.begin(115200);
  delay(100);
  Serial.println("[Boot] setup begin");

  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  cfg.external_speaker_value = 0;
  cfg.internal_spk = false;            // use external HAT speaker
  cfg.internal_mic = false;
  cfg.external_speaker.hat_spk2 = 1;   // enable SPK2 hat routing
  cfg.external_speaker.hat_spk = 0;
  cfg.external_speaker.atomic_spk = 0;
  cfg.output_power = true;

  StickCP2.begin(cfg);
  StickCP2.Power.setLed(0);
  StickCP2.Display.setRotation(3);
  StickCP2.Display.fillScreen(BLACK);
  StickCP2.Display.setCursor(10, 20);
  StickCP2.Display.setTextColor(WHITE, BLACK);
  StickCP2.Display.setTextSize(1);
  StickCP2.Display.println("Booting...");
  Serial.println("[Boot] StickCP2 initialized");

  if (!SPIFFS.begin(true)) {
    Serial.println("[Boot] Failed to mount SPIFFS.");
  } else {
    Serial.println("[Boot] SPIFFS mounted");
  }

  wifi_manager.configure(
      {secrets::WIFI_PRIMARY_SSID, secrets::WIFI_PRIMARY_PASS},
      (strlen(secrets::WIFI_SECONDARY_SSID) > 0)
          ? std::optional<WiFiCredentials>(
                WiFiCredentials{secrets::WIFI_SECONDARY_SSID,
                                secrets::WIFI_SECONDARY_PASS})
          : std::nullopt);
  wifi_manager.begin();
  Serial.println("[Boot] Wi-Fi manager started");

  const std::string mac = wifi_manager.mac();
  device_id = makeDeviceIdFromMac(mac);
  Serial.printf("[Boot] Device ID %s (MAC %s)\n",
                device_id.c_str(), mac.c_str());

  if (auto rtcSeed = rtcEpochSeconds()) {
    ntp_client.seed(*rtcSeed);
  } else {
    Serial.println("[RTC] Seed skipped (invalid or power loss)");
  }

  audio_player.begin();
  Serial.println("[Boot] Audio player ready");

  ntp_client.begin();
  Serial.println("[Boot] NTP client started");

  if (!preset_store.load(SPIFFS, "/manifest.json")) {
    Serial.println("[Boot] Preset manifest not loaded.");
  } else {
    Serial.println("[Boot] Preset manifest loaded");
  }

  osc_receiver.configure(secrets::OSC_LISTEN_PORT);
  osc_receiver.setCryptoKey(secrets::OSC_AES_KEY, secrets::OSC_AES_IV);
  osc_receiver.begin();
  Serial.printf("[Boot] OSC receiver listening on %u\n",
                static_cast<unsigned>(secrets::OSC_LISTEN_PORT));

  heartbeat.configure(secrets::HEARTBEAT_REMOTE_HOST,
                      secrets::HEARTBEAT_REMOTE_PORT,
                      device_id,
                      kFirmwareVersion);
  heartbeat.begin();
  Serial.printf("[Boot] Heartbeat target %s:%u\n",
                secrets::HEARTBEAT_REMOTE_HOST,
                static_cast<unsigned>(secrets::HEARTBEAT_REMOTE_PORT));

  xTaskCreatePinnedToCore(wifiTask, "wifiTask", 4096, nullptr, 2,
                          &wifi_task_handle, 0);
  xTaskCreatePinnedToCore(ntpTask, "ntpTask", 4096, nullptr, 3,
                          &ntp_task_handle, 0);
  xTaskCreatePinnedToCore(oscTask, "oscTask", 6144, nullptr, 4,
                          &osc_task_handle, 1);
  xTaskCreatePinnedToCore(playbackTask, "playbackTask", 8192, nullptr, 5,
                          &playback_task_handle, 1);
  xTaskCreatePinnedToCore(heartbeatTask, "heartbeatTask", 4096, nullptr, 1,
                          &heartbeat_task_handle, 1);
  Serial.println("[Boot] Tasks launched");

  // --- Temporary test: play sample_test preset on boot ---
  if (auto preset = preset_store.findById("sample_test")) {
    audio_player.play(*preset);
    Serial.println("[Boot] sample_test preset auto-play triggered");
  } else {
    Serial.println("[Boot] sample_test preset not found.");
  }
  // --- End of temporary test code ---

  Serial.println("[Boot] setup complete");
}

void loop() {
  StickCP2.update();
  firmware::refreshStatusDisplay();
  delay(50);
}
