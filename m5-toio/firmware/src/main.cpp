#include <M5Unified.h>
#include <string>

#include "controller/toio_controller.h"

namespace {
constexpr uint32_t kScanDurationSec = 3;
constexpr uint32_t kRefreshIntervalMs = 1000;
constexpr uint32_t kStatusAreaY = 40;

ToioController g_toio;
uint32_t g_lastDisplay = 0;

void DrawHeader(const char* message) {
  auto& display = M5.Display;
  display.fillScreen(BLACK);
  display.setTextDatum(MC_DATUM);
  display.setTextColor(WHITE, BLACK);
  display.setTextSize(2);
  display.drawString("toio position monitor", display.width() / 2, 14);
  display.setTextSize(1);
  display.drawString(message, display.width() / 2, 30);
  display.setTextDatum(TL_DATUM);
}

void ShowPositionData(const CubePose& pose, bool hasPose, bool hasBattery,
                      uint8_t batteryLevel) {
  auto& display = M5.Display;
  display.fillRect(0, kStatusAreaY, display.width(),
                   display.height() - kStatusAreaY, BLACK);
  display.setCursor(6, kStatusAreaY + 4);

  const uint32_t now_ms = millis();
  display.printf("t:%08lu ms\n", static_cast<unsigned long>(now_ms));
  M5.Log.printf("[%08lu ms][display] ", static_cast<unsigned long>(now_ms));

  if (hasPose) {
    display.printf("Cube  X:%4u  Y:%4u \n Angle:%3u, on_mat:%s\n", pose.x,
                   pose.y, pose.angle, pose.on_mat ? "yes" : "no");
    M5.Log.printf("x=%u y=%u angle=%u on_mat=%s ", pose.x, pose.y, pose.angle,
                  pose.on_mat ? "yes" : "no");
  } else {
    display.println("No position (off mat)");
    M5.Log.print("no position ");
  }

  if (hasBattery) {
    display.printf("Battery: %3u%%", batteryLevel);
    M5.Log.printf("battery=%u%%", batteryLevel);
  }
  M5.Log.println();
}

void InitializeM5Hardware() {
  auto cfg = M5.config();
  cfg.clear_display = true;
  cfg.output_power = true;
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);

  M5.Display.setRotation(3);
  DrawHeader("Scanning...");
}

void ShowInitResult(ToioController::InitStatus status) {
  const char* message = nullptr;
  switch (status) {
    case ToioController::InitStatus::kReady:
      message = "Connected";
      break;
    case ToioController::InitStatus::kNoCubeFound:
      message = "No cube found.";
      break;
    case ToioController::InitStatus::kTargetNotFound:
      message = "Target cube not found.";
      break;
    case ToioController::InitStatus::kConnectionFailed:
      message = "Connection failed.";
      break;
    case ToioController::InitStatus::kInvalidArgument:
    default:
      message = "Invalid request.";
      break;
  }
  DrawHeader(message);
  M5.Log.println(message);
}

void PerformStartupTest() {
  constexpr uint8_t kLedR = 0x00;
  constexpr uint8_t kLedG = 0xff;
  constexpr uint8_t kLedB = 0x80;
  constexpr uint8_t kTestSpeed = 30;

  if (g_toio.setLedColor(kLedR, kLedG, kLedB)) {
    M5.Log.println("LED test applied.");
  }
  if (g_toio.driveMotor(true, kTestSpeed, true, kTestSpeed)) {
    delay(1000);
    g_toio.driveMotor(true, 0, true, 0);
  }
}

void InitGoalFollowing() {
  float g_goalX = 350.0f;
  float g_goalY = 200.0f;
  
  g_toio.setGoalTuning(/*vmax=*/90.0f, /*wmax=*/80.0f, /*k_r=*/1.0f,
                       /*k_a=*/0.8f);
  g_toio.setGoal(g_goalX, g_goalY, /*stop_distance=*/20.0f);
}
}  // namespace

void setup() {
  InitializeM5Hardware();

  std::string TargetFragment = "m7d";

  ToioCore* target = nullptr;
  auto scan_status =
      g_toio.scanTargets(TargetFragment, kScanDurationSec, &target);
  if (scan_status != ToioController::InitStatus::kReady) {
    ShowInitResult(scan_status);
    return;
  }

  auto connect_status = g_toio.connectAndConfigure(target);
  if (connect_status != ToioController::InitStatus::kReady) {
    ShowInitResult(connect_status);
    return;
  }

  ShowInitResult(ToioController::InitStatus::kReady);
  ShowPositionData(g_toio.pose(), g_toio.hasPose(), g_toio.hasBatteryLevel(),
                   g_toio.batteryLevel());
  g_toio.clearPoseDirty();
  g_toio.clearBatteryDirty();
  g_lastDisplay = millis();

  PerformStartupTest();
  InitGoalFollowing();
}

void loop() {
  InitGoalFollowing(); // testing
  M5.update();
  g_toio.loop();

  if (!g_toio.hasActiveCore()) {
    delay(100);
    return;
  }

  const bool pose_dirty = g_toio.poseDirty();
  if (pose_dirty) {
    g_toio.clearPoseDirty();
  }
  const bool battery_dirty = g_toio.batteryDirty();
  if (battery_dirty) {
    g_toio.clearBatteryDirty();
  }
  const bool needs_update = pose_dirty || battery_dirty ||
                            (millis() - g_lastDisplay >= kRefreshIntervalMs);

  if (needs_update) {
    ShowPositionData(g_toio.pose(), g_toio.hasPose(),
                     g_toio.hasBatteryLevel(), g_toio.batteryLevel());
    g_lastDisplay = millis();
  }

  delay(10);
}
