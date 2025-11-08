#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "locomotion/calibration/CalibrationPipeline.h"
#include "locomotion/calibration/CalibrationResult.h"

namespace locomotion::calibration {

struct SessionConfig {
  int attempts{5};
  double max_plane_std_mm{8.0};
  double min_inlier_ratio{0.7};
  bool save_intermediate_snapshots{false};
  std::string snapshot_output_dir;
};

class CalibrationSession {
 public:
  CalibrationSession(CalibrationPipeline pipeline, SessionConfig session_config);

  std::optional<CalibrationResult> Run();
  bool SaveResultJson(const CalibrationResult& result, const std::string& path) const;
  static CalibrationResult SnapshotToResult(const CalibrationSnapshot& snapshot);
  static bool SaveResultJson(const CalibrationResult& result,
                             const std::string& path,
                             const CalibrationConfig& pipeline_config,
                             const SessionConfig& session_config,
                             const std::string& camera_model,
                             const std::string& camera_serial,
                             double depth_scale_m);

 private:
  CalibrationPipeline pipeline_;
  SessionConfig session_config_;
};

}  // namespace locomotion::calibration
