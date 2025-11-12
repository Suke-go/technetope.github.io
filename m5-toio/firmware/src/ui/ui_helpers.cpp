#include "ui/ui_helpers.h"

#include <M5Unified.h>

namespace {
constexpr uint32_t kHeaderTitleY = 14;
constexpr uint32_t kHeaderMessageY = 30;
constexpr uint32_t kStatusAreaY = 40;
}  // namespace

void UiHelpers::Begin() {
  auto& display = M5.Display;
  display.fillScreen(BLACK);
  display.setTextDatum(TL_DATUM);
  display.setTextColor(WHITE, BLACK);
  last_display_ms_ = millis();
}

void UiHelpers::DrawHeader(const char* message) {
  auto& display = M5.Display;
  display.fillScreen(BLACK);
  display.setTextDatum(MC_DATUM);
  display.setTextColor(WHITE, BLACK);
  display.setTextSize(2);
  display.drawString("toio position monitor", display.width() / 2,
                     kHeaderTitleY);
  display.setTextSize(1);
  display.drawString(message ? message : "", display.width() / 2,
                     kHeaderMessageY);
  display.setTextDatum(TL_DATUM);
}

void UiHelpers::ShowInitResult(ToioController::InitStatus status) {
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

void UiHelpers::UpdateStatus(const CubePose& pose, bool has_pose,
                             uint8_t battery_level, bool has_battery,
                             const ToioLedColor& led,
                             const ToioMotorState& motor, bool pose_dirty,
                             bool battery_dirty,
                             uint32_t refresh_interval_ms) {
  status_.pose = pose;
  status_.has_pose = has_pose;
  status_.battery_level = battery_level;
  status_.has_battery = has_battery;
  status_.led = led;
  status_.motor = motor;

  const uint32_t now_ms = millis();
  const bool needs_update = pose_dirty || battery_dirty ||
                            (now_ms - last_display_ms_ >= refresh_interval_ms);
  if (!needs_update) {
    return;
  }

  ShowStatus(now_ms);
  last_display_ms_ = now_ms;
}

void UiHelpers::ShowStatus(uint32_t now_ms) {
  auto& display = M5.Display;
  display.fillRect(0, kStatusAreaY, display.width(),
                   display.height() - kStatusAreaY, BLACK);
  display.setCursor(6, kStatusAreaY + 4);

  display.printf("t:%08lu ms\n", static_cast<unsigned long>(now_ms));
  M5.Log.printf("[%08lu ms][display] ", static_cast<unsigned long>(now_ms));

  if (status_.has_pose) {
    display.printf("Cube  X:%4u  Y:%4u \n Angle:%3u, on_mat:%s\n", status_.pose.x,
                   status_.pose.y, status_.pose.angle,
                   status_.pose.on_mat ? "yes" : "no");
    M5.Log.printf("x=%u y=%u angle=%u on_mat=%s ", status_.pose.x,
                  status_.pose.y, status_.pose.angle,
                  status_.pose.on_mat ? "yes" : "no");
  } else {
    display.println("No position (off mat)");
    M5.Log.print("no position ");
  }

  if (status_.has_battery) {
    display.printf("Battery: %3u%%", status_.battery_level);
    M5.Log.printf("battery=%u%%", status_.battery_level);
  }
  display.printf("LED RGB:(%3u,%3u,%3u)\n", status_.led.r, status_.led.g,
                 status_.led.b);
  display.printf("Motor L:%4d R:%4d\n", status_.motor.left_speed,
                 status_.motor.right_speed);
  M5.Log.printf(" LED RGB:(%u,%u,%u) Motor L:%d R:%d", status_.led.r,
                status_.led.g, status_.led.b, status_.motor.left_speed,
                status_.motor.right_speed);
  M5.Log.println();
}
