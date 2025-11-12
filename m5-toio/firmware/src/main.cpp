#include <M5Unified.h>
#include <string>

#include "controller/toio_controller.h"
#include "ui/ui_helpers.h"

namespace {
constexpr uint32_t kScanDurationSec = 3;
constexpr uint32_t kRefreshIntervalMs = 1000;

ToioController g_toio;
UiHelpers g_ui;

void InitializeM5Hardware() {
  auto cfg = M5.config();
  cfg.clear_display = true;
  cfg.output_power = true;
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);

  M5.Display.setRotation(3);
  g_ui.Begin();
  g_ui.DrawHeader("Scanning...");
}

void PerformStartupTest() {
  constexpr uint8_t kLedR = 0x00;
  constexpr uint8_t kLedG = 0xff;
  constexpr uint8_t kLedB = 0x80;
  constexpr uint8_t kTestSpeed = 30;

  if (g_toio.setLedColor(kLedR, kLedG, kLedB)) {
    M5.Log.println("LED test applied.");
  }
  if (g_toio.driveMotor(kTestSpeed, kTestSpeed)) {
    delay(1000);
    g_toio.driveMotor(0, 0);
  }
}

void InitGoalFollowing() {
  float g_goalX = 300.0f;
  float g_goalY = 200.0f;
  
  g_toio.setGoalTuning(/*vmax=*/80.0f, /*wmax=*/70.0f, /*k_r=*/1.0f,
                       /*k_a=*/0.8f, /*reverse_threshold_deg=*/90.0f,
                       /*reverse_hysteresis_deg=*/10.0f);
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
    g_ui.ShowInitResult(scan_status);
    return;
  }

  auto connect_status = g_toio.connectAndConfigure(target);
  if (connect_status != ToioController::InitStatus::kReady) {
    g_ui.ShowInitResult(connect_status);
    return;
  }

  g_ui.ShowInitResult(ToioController::InitStatus::kReady);
  g_ui.UpdateStatus(g_toio.pose(), g_toio.hasPose(), g_toio.batteryLevel(),
                    g_toio.hasBatteryLevel(), g_toio.ledColor(),
                    g_toio.motorState(),
                    /*pose_dirty=*/true, /*battery_dirty=*/true,
                    kRefreshIntervalMs);
  g_toio.clearPoseDirty();
  g_toio.clearBatteryDirty();

  PerformStartupTest();
  InitGoalFollowing();
}

void loop() {
  // InitGoalFollowing(); // testing
  g_toio.driveMotor(-30, -30);

  M5.update();
  g_toio.loop();

  if (!g_toio.hasActiveCore()) {
    delay(100);
    return;
  }

  const bool pose_dirty = g_toio.poseDirty();
  const bool battery_dirty = g_toio.batteryDirty();

  g_ui.UpdateStatus(g_toio.pose(), g_toio.hasPose(), g_toio.batteryLevel(),
                    g_toio.hasBatteryLevel(), g_toio.ledColor(),
                    g_toio.motorState(), pose_dirty, battery_dirty,
                    kRefreshIntervalMs);

  if (pose_dirty) {
    g_toio.clearPoseDirty();
  }
  if (battery_dirty) {
    g_toio.clearBatteryDirty();
  }

  delay(10);
}
