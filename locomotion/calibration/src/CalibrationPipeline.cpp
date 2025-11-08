#include "locomotion/calibration/CalibrationPipeline.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "locomotion/calibration/ToioCoordinateTransform.h"

namespace locomotion::calibration {

namespace {

rs2::config buildRealSenseConfig(const CalibrationConfig& cfg) {
  rs2::config rs_cfg;
  rs_cfg.enable_stream(RS2_STREAM_COLOR, cfg.color_width, cfg.color_height,
                       RS2_FORMAT_BGR8, cfg.fps);
  rs_cfg.enable_stream(RS2_STREAM_DEPTH, cfg.depth_width, cfg.depth_height,
                       RS2_FORMAT_Z16, cfg.fps);
  return rs_cfg;
}

cv::aruco::PredefinedDictionaryType parseDictionary(const std::string& name) {
  using PD = cv::aruco::PredefinedDictionaryType;
  static const std::map<std::string, PD> kMap = {
      {"DICT_4X4_50", PD::DICT_4X4_50},
      {"DICT_4X4_100", PD::DICT_4X4_100},
      {"DICT_5X5_50", PD::DICT_5X5_50},
      {"DICT_5X5_100", PD::DICT_5X5_100},
      {"DICT_6X6_50", PD::DICT_6X6_50},
      {"DICT_6X6_100", PD::DICT_6X6_100},
      {"DICT_APRILTAG_16h5", PD::DICT_APRILTAG_16h5},
      {"DICT_APRILTAG_25h9", PD::DICT_APRILTAG_25h9},
      {"DICT_APRILTAG_36h11", PD::DICT_APRILTAG_36h11}};
  auto it = kMap.find(name);
  if (it == kMap.end()) {
    spdlog::warn("Unknown ArUco dictionary '{}', fallback to DICT_4X4_50", name);
    return PD::DICT_4X4_50;
  }
  return it->second;
}

cv::Ptr<cv::aruco::Dictionary> makeDictionary(const CalibrationConfig& config) {
  auto dict_id = parseDictionary(config.aruco_dictionary);
  return cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(dict_id));
}

spdlog::level::level_enum parseLogLevel(const std::string& name) {
  static const std::map<std::string, spdlog::level::level_enum> kLevels = {
      {"trace", spdlog::level::trace}, {"debug", spdlog::level::debug},
      {"info", spdlog::level::info},   {"warn", spdlog::level::warn},
      {"error", spdlog::level::err},   {"critical", spdlog::level::critical}};
  auto it = kLevels.find(name);
  if (it == kLevels.end()) {
    spdlog::warn("Unknown log level '{}', fallback to 'info'", name);
    return spdlog::level::info;
  }
  return it->second;
}

std::string distortionModelToString(rs2_distortion model) {
  switch (model) {
    case RS2_DISTORTION_NONE:
      return "none";
    case RS2_DISTORTION_MODIFIED_BROWN_CONRADY:
      return "modified_brown_conrady";
    case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
      return "inverse_brown_conrady";
    case RS2_DISTORTION_FTHETA:
      return "f_theta";
    case RS2_DISTORTION_BROWN_CONRADY:
      return "brown_conrady";
    case RS2_DISTORTION_KANNALA_BRANDT4:
      return "kannala_brandt4";
    default:
      return "unknown";
  }
}

cv::Mat matFromMatx(const cv::Matx33d& matx) {
  cv::Mat mat(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      mat.at<double>(r, c) = matx(r, c);
    }
  }
  return mat;
}

double computeRmsError(const std::vector<cv::Point2f>& expected,
                       const std::vector<cv::Point2f>& observed) {
  if (expected.size() != observed.size() || expected.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double accum = 0.0;
  for (size_t i = 0; i < expected.size(); ++i) {
    const cv::Point2f diff = observed[i] - expected[i];
    accum += diff.dot(diff);
  }
  return std::sqrt(accum / static_cast<double>(expected.size()));
}

std::vector<cv::Point2f> toFloorPoints(const std::vector<cv::Point3f>& board_points) {
  std::vector<cv::Point2f> floor_points;
  floor_points.reserve(board_points.size());
  for (const auto& pt : board_points) {
    floor_points.emplace_back(pt.x, pt.y);
  }
  return floor_points;
}

}  // namespace

CalibrationPipeline::CalibrationPipeline(CalibrationConfig config)
    : config_(std::move(config)) {}

CalibrationPipeline::~CalibrationPipeline() {
  try {
    pipeline_.stop();
  } catch (const rs2::error& err) {
    spdlog::warn("RealSense pipeline stop failed: {}", err.what());
  }
}

bool CalibrationPipeline::initialize() {
  return initialize("");
}

bool CalibrationPipeline::initialize(const std::string& config_file_dir) {
  if (is_initialized_) {
    spdlog::warn("CalibrationPipeline already initialized, skipping re-initialization");
    return true;
  }

  spdlog::set_level(parseLogLevel(config_.log_level));
  dictionary_ = makeDictionary(config_);
  board_ = cv::makePtr<cv::aruco::CharucoBoard>(
      cv::Size(config_.charuco_squares_x, config_.charuco_squares_y),
      config_.charuco_square_length_mm, config_.charuco_marker_length_mm, *dictionary_);

  charuco_config_.min_corners = config_.min_charuco_corners;
  charuco_config_.enable_subpixel_refine = config_.charuco_enable_subpixel_refine;
  charuco_config_.subpixel_window = cv::Size(config_.charuco_subpixel_window,
                                             config_.charuco_subpixel_window);
  charuco_config_.subpixel_max_iterations = config_.charuco_subpixel_max_iterations;
  charuco_config_.subpixel_epsilon = config_.charuco_subpixel_epsilon;
  charuco_detector_ =
      std::make_unique<CharucoDetector>(dictionary_, board_, charuco_config_);
  spatial_filter_enabled_ = config_.enable_spatial_filter;

  if (config_.enable_floor_plane_fit) {
    FloorPlaneEstimatorConfig floor_config;
    floor_config.inlier_threshold_mm = config_.floor_inlier_threshold_mm;
    floor_config.ransac_iterations = config_.floor_ransac_iterations;
    floor_config.min_inlier_ratio = config_.floor_min_inlier_ratio;
    floor_config.z_min_mm = config_.floor_z_min_mm;
    floor_config.z_max_mm = config_.floor_z_max_mm;
    floor_config.downsample_grid = config_.floor_downsample_grid;
    floor_config.min_valid_points = config_.floor_min_valid_points;
    floor_config.random_seed = config_.random_seed;
    floor_estimator_ = std::make_unique<FloorPlaneEstimator>(floor_config);
  } else {
    floor_estimator_.reset();
  }

  try {
    playmat_layout_ = PlaymatLayout::LoadFromFile(config_.playmat_layout_path, config_file_dir);
    has_playmat_layout_ = true;
    spdlog::info("Loaded playmat layout from '{}' (base dir: '{}')", 
                 config_.playmat_layout_path, 
                 config_file_dir.empty() ? "current directory" : config_file_dir);
  } catch (const std::exception& ex) {
    has_playmat_layout_ = false;
    spdlog::warn("Failed to load playmat layout '{}': {}", config_.playmat_layout_path, ex.what());
  }

  try {
    profile_ = pipeline_.start(buildRealSenseConfig(config_));
    spdlog::info("RealSense pipeline started for calibration");
  } catch (const rs2::error& err) {
    spdlog::error("Failed to start RealSense pipeline: {}", err.what());
    return false;
  }

  try {
    rs2::video_stream_profile color_profile =
        profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intr = color_profile.get_intrinsics();

    camera_intrinsics_.fx = intr.fx;
    camera_intrinsics_.fy = intr.fy;
    camera_intrinsics_.cx = intr.ppx;
    camera_intrinsics_.cy = intr.ppy;
    camera_intrinsics_.distortion_model = distortionModelToString(intr.model);

    camera_matrix_ = (cv::Mat_<double>(3, 3) << intr.fx, 0.0, intr.ppx, 0.0, intr.fy,
                      intr.ppy, 0.0, 0.0, 1.0);
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    for (size_t i = 0; i < camera_intrinsics_.distortion_coeffs.size(); ++i) {
      double coeff = (i < 5) ? static_cast<double>(intr.coeffs[i]) : 0.0;
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = coeff;
      camera_intrinsics_.distortion_coeffs[i] = coeff;
    }
    intrinsics_loaded_ = true;
    spdlog::info("Loaded color intrinsics fx={:.2f}, fy={:.2f}, cx={:.2f}, cy={:.2f}",
                 camera_intrinsics_.fx, camera_intrinsics_.fy, camera_intrinsics_.cx,
                 camera_intrinsics_.cy);
  } catch (const rs2::error& err) {
    intrinsics_loaded_ = false;
    spdlog::warn("Failed to query color intrinsics: {}", err.what());
  }

  try {
    rs2::device device = profile_.get_device();
    if (device.supports(RS2_CAMERA_INFO_NAME)) {
      camera_model_ = device.get_info(RS2_CAMERA_INFO_NAME);
    }
    if (device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
      camera_serial_ = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    }

    if (config_.enable_color_auto_exposure) {
      for (rs2::sensor sensor : device.query_sensors()) {
        if (sensor.is<rs2::color_sensor>() && sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
          try {
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
            spdlog::info("Enabled auto-exposure on color sensor");
          } catch (const rs2::error& err) {
            spdlog::debug("Failed to enable color auto-exposure: {}", err.what());
          }
        }
      }
    }

    rs2::depth_sensor depth_sensor = device.first<rs2::depth_sensor>();
    depth_scale_m_ = depth_sensor.get_depth_scale();
    spdlog::info("Depth scale: {:.6f} meters per unit", depth_scale_m_);
    if (std::abs(depth_scale_m_ - config_.expected_depth_scale_m) > 1e-4) {
      spdlog::warn("Depth scale differs from expected {:.6f} (device reported {:.6f}).",
                   config_.expected_depth_scale_m, depth_scale_m_);
    }

    auto maybeSetDepthOption = [&](rs2_option opt, float value) {
      if (!depth_sensor.supports(opt)) {
        return;
      }
      try {
        depth_sensor.set_option(opt, value);
      } catch (const rs2::error& err) {
        spdlog::debug("Failed to set depth option {}: {}", static_cast<int>(opt), err.what());
      }
    };

    maybeSetDepthOption(RS2_OPTION_MIN_DISTANCE,
                        static_cast<float>(config_.depth_min_distance_mm / 1000.0));
    maybeSetDepthOption(RS2_OPTION_MAX_DISTANCE,
                        static_cast<float>(config_.depth_max_distance_mm / 1000.0));
  } catch (const rs2::error& err) {
    spdlog::warn("Failed to configure RealSense sensors: {}", err.what());
  }
  
  is_initialized_ = true;
  return true;
}

std::optional<CalibrationSnapshot> CalibrationPipeline::runOnce() {
  FrameBundle bundle;
  if (!CaptureFrame(bundle)) {
    spdlog::warn("Failed to capture frame for calibration");
    return std::nullopt;
  }

  return ProcessFrame(bundle);
}

bool CalibrationPipeline::CaptureFrame(FrameBundle& bundle) {
  return captureAlignedFrame(bundle);
}

std::optional<CalibrationSnapshot> CalibrationPipeline::ProcessFrame(
    const FrameBundle& bundle) const {
  auto detection =
      (charuco_detector_ && !bundle.color.empty())
          ? charuco_detector_->Detect(bundle.color)
          : std::nullopt;
  if (!detection) {
    spdlog::info("ChArUco board not detected in current frame");
    return std::nullopt;
  }

  CalibrationSnapshot snapshot;
  snapshot.intrinsics = camera_intrinsics_;
  snapshot.detected_charuco_corners = detection->detected_charuco_corners;
  if (!computeHomographies(*detection, snapshot.homography_color_to_floor,
                           snapshot.homography_color_to_toio, snapshot.toio_transform,
                           snapshot.reprojection_error_floor_mm,
                           snapshot.reprojection_error_toio,
                           snapshot.reprojection_error_px)) {
    spdlog::warn("Homography solve failed or reprojection error too large");
    return std::nullopt;
  }

  if (config_.enable_floor_plane_fit) {
    if (!estimateFloorPlane(bundle, snapshot.floor_plane, snapshot.floor_plane_std_mm,
                            snapshot.inlier_ratio, snapshot.floor_inlier_count,
                            snapshot.camera_height_mm)) {
      spdlog::warn("Floor plane estimation failed");
      snapshot.floor_plane = {0.0F, 0.0F, 1.0F, 0.0F};
      snapshot.floor_plane_std_mm = 0.0;
      snapshot.inlier_ratio = 0.0;
      snapshot.floor_inlier_count = 0;
      snapshot.camera_height_mm = 0.0;
    } else {
      warnOnceCameraHeight(snapshot.camera_height_mm);
    }
  }

  snapshot.timestamp = std::chrono::system_clock::now();
  return snapshot;
}

bool CalibrationPipeline::captureAlignedFrame(FrameBundle& bundle) {
  try {
    rs2::frameset frames = pipeline_.wait_for_frames();
    frames = align_to_color_.process(frames);

    rs2::video_frame color = frames.get_color_frame();
    rs2::depth_frame depth = frames.get_depth_frame();
    if (spatial_filter_enabled_) {
      rs2::frame filtered = spatial_filter_.process(depth);
      depth = filtered.as<rs2::depth_frame>();
    }

    bundle.timestamp_ms = color.get_timestamp();
    bundle.color = cv::Mat(cv::Size(color.get_width(), color.get_height()),
                           CV_8UC3, const_cast<void*>(color.get_data()),
                           cv::Mat::AUTO_STEP)
                       .clone();
    if (intrinsics_loaded_ && !camera_matrix_.empty() && !dist_coeffs_.empty()) {
      cv::Mat undistorted;
      cv::undistort(bundle.color, undistorted, camera_matrix_, dist_coeffs_);
      bundle.color = std::move(undistorted);
    }
    bundle.depth = cv::Mat(cv::Size(depth.get_width(), depth.get_height()),
                           CV_16UC1, const_cast<void*>(depth.get_data()),
                           cv::Mat::AUTO_STEP)
                       .clone();
    return true;
  } catch (const rs2::error& err) {
    spdlog::error("RealSense capture error: {}", err.what());
    return false;
  }
}

bool CalibrationPipeline::computeHomographies(
    const CharucoDetectionResult& detection,
    cv::Mat& homography_color_to_floor,
    cv::Mat& homography_color_to_toio,
    ToioCoordinateTransform& toio_transform,
    double& reprojection_error_floor_mm,
    double& reprojection_error_toio,
    double& reprojection_error_px) const {
  if (detection.image_points.size() < 4) {
    spdlog::warn("Need at least 4 correspondences for homography (got {}).",
                 detection.image_points.size());
    return false;
  }

  std::vector<cv::Point2f> floor_points = toFloorPoints(detection.board_points);
  cv::Mat inliers;
  homography_color_to_floor =
      cv::findHomography(detection.image_points, floor_points, cv::RANSAC,
                         config_.homography_ransac_thresh_px, inliers);
  if (homography_color_to_floor.empty()) {
    return false;
  }

  // Compute reprojection error in pixel coordinates (image space)
  cv::Mat homography_inv = homography_color_to_floor.inv();
  std::vector<cv::Point2f> projected_image_points;
  cv::perspectiveTransform(floor_points, projected_image_points, homography_inv);
  reprojection_error_px = computeRmsError(detection.image_points, projected_image_points);

  std::vector<cv::Point2f> projected_floor;
  cv::perspectiveTransform(detection.image_points, projected_floor,
                           homography_color_to_floor);
  reprojection_error_floor_mm = computeRmsError(floor_points, projected_floor);

  const BoardMount* mount = nullptr;
  std::vector<cv::Point2f> toio_points =
      boardPointsToPositionId(detection.board_points, &mount);

  cv::Matx33d board_to_toio_matx =
      mount ? mount->affine_mm_to_position : cv::Matx33d::eye();
  cv::Mat board_to_toio = matFromMatx(board_to_toio_matx);
  homography_color_to_toio = board_to_toio * homography_color_to_floor;
  toio_transform.board_to_toio = board_to_toio_matx;
  toio_transform.color_to_toio = homography_color_to_toio.clone();
  toio_transform.mount_label =
      mount ? mount->label : config_.board_mount_label;
  toio_transform.playmat_id = mount ? mount->playmat_id : std::string();

  if (mount && mount->board_points_mm.size() == mount->position_id_points.size() &&
      !mount->board_points_mm.empty()) {
    double accum = 0.0;
    for (size_t i = 0; i < mount->board_points_mm.size(); ++i) {
      cv::Vec3d src(mount->board_points_mm[i].x, mount->board_points_mm[i].y, 1.0);
      cv::Vec3d dst = mount->affine_mm_to_position * src;
      cv::Point2d diff(dst[0], dst[1]);
      diff -= mount->position_id_points[i];
      accum += diff.dot(diff);
    }
    toio_transform.transform_error_id =
        std::sqrt(accum / static_cast<double>(mount->board_points_mm.size()));
  } else {
    toio_transform.transform_error_id = std::numeric_limits<double>::quiet_NaN();
  }

  if (mount) {
    if (const PlaymatInfo* playmat = playmat_layout_.GetPlaymat(mount->playmat_id);
        playmat) {
      double width = playmat->extent.max.x - playmat->extent.min.x;
      double height = playmat->extent.max.y - playmat->extent.min.y;
      toio_transform.coverage_area =
          cv::Rect2d(playmat->extent.min.x, playmat->extent.min.y, width, height);
    }
  }

  std::vector<cv::Point2f> projected_toio;
  cv::perspectiveTransform(detection.image_points, projected_toio,
                           homography_color_to_toio);
  reprojection_error_toio = computeRmsError(toio_points, projected_toio);

  return reprojection_error_toio <= config_.max_reprojection_error_id;
}

bool CalibrationPipeline::estimateFloorPlane(const FrameBundle& bundle,
                                             cv::Vec4f& plane,
                                             double& plane_std_mm,
                                             double& inlier_ratio,
                                             int& inlier_count,
                                             double& camera_height_mm) const {
  if (!floor_estimator_) {
    spdlog::warn("FloorPlaneEstimator is not initialized.");
    return false;
  }

  if (!intrinsics_loaded_ || depth_scale_m_ <= 0.0) {
    spdlog::warn("Skipping floor estimation: intrinsics or depth scale not initialized.");
    return false;
  }
  if (bundle.depth.empty()) {
    spdlog::warn("Floor estimation skipped: depth image is empty.");
    return false;
  }

  auto estimate =
      floor_estimator_->Estimate(bundle.depth, camera_intrinsics_, depth_scale_m_);
  if (!estimate) {
    spdlog::warn("Floor plane estimation returned no result.");
    return false;
  }

  plane = estimate->plane;
  plane_std_mm = estimate->plane_std_mm;
  inlier_ratio = estimate->inlier_ratio;
  inlier_count = estimate->inlier_count;
  camera_height_mm = estimate->camera_height_mm;
  return true;
}

std::vector<cv::Point2f> CalibrationPipeline::boardPointsToPositionId(
    const std::vector<cv::Point3f>& object_points,
    const BoardMount** mount_out) const {
  std::vector<cv::Point2f> result;
  result.reserve(object_points.size());

  const BoardMount* mount = nullptr;
  if (!has_playmat_layout_) {
    if (!warned_layout_not_loaded_) {
      spdlog::warn("Playmat layout not loaded. Returning raw Charuco coordinates.");
      warned_layout_not_loaded_ = true;
    }
  } else {
    mount = playmat_layout_.GetBoardMount(config_.board_mount_label);
    if (!mount && !warned_mount_missing_) {
      spdlog::warn("Board mount '{}' not found in layout. Using raw board coordinates.",
                   config_.board_mount_label);
      warned_mount_missing_ = true;
    }
  }

  if (!mount) {
    for (const auto& pt : object_points) {
      result.emplace_back(pt.x, pt.y);
    }
  } else {
    for (const auto& pt : object_points) {
      cv::Point2f mapped = playmat_layout_.TransformBoardPoint(mount->label, pt);
      result.emplace_back(mapped);
    }
  }

  if (mount_out) {
    *mount_out = mount;
  }
  return result;
}

bool CalibrationPipeline::warnOnceCameraHeight(double camera_height_mm) const {
  if (camera_height_mm <= 0.0) {
    return false;
  }
  if (camera_height_mm >= config_.camera_height_warn_min_mm &&
      camera_height_mm <= config_.camera_height_warn_max_mm) {
    return false;
  }
  if (!warned_camera_height_) {
    spdlog::warn(
        "Estimated camera height {:.1f} mm outside expected [{:.0f}, {:.0f}] mm. "
        "Please verify mount height.",
        camera_height_mm, config_.camera_height_warn_min_mm,
        config_.camera_height_warn_max_mm);
    warned_camera_height_ = true;
  }
  return true;
}

cv::Point2f CalibrationPipeline::TransformPixelToToio(
    cv::Point2f pixel, const CalibrationSnapshot& snapshot) const {
  return TransformImagePixelToToio(pixel, snapshot.homography_color_to_toio);
}

cv::Point2f CalibrationPipeline::TransformToioToPixel(
    cv::Point2f toio_pos, const CalibrationSnapshot& snapshot) const {
  if (snapshot.homography_color_to_toio.empty()) {
    return cv::Point2f(0.0f, 0.0f);
  }
  
  // Compute inverse homography
  cv::Mat homography_toio_to_color = snapshot.homography_color_to_toio.inv();
  return TransformToioToImagePixel(toio_pos, homography_toio_to_color);
}

bool CalibrationPipeline::IsToioPositionInCoverage(
    cv::Point2f toio_pos, const CalibrationSnapshot& snapshot) const {
  // Use the free function from ToioCoordinateTransform namespace
  return ::locomotion::calibration::IsToioPositionInCoverage(toio_pos, snapshot.toio_transform);
}

CalibrationValidation CalibrationPipeline::ValidateCalibration(
    const CalibrationSnapshot& snapshot) const {
  CalibrationValidation validation;

  // 1. Check transformation error at correspondence points
  // This is already computed in computeHomographies and stored in transform_error_id
  validation.transform_error_at_correspondences = snapshot.toio_transform.transform_error_id;
  
  if (!std::isfinite(validation.transform_error_at_correspondences)) {
    validation.warnings.push_back("Transform error at correspondences is not available");
  } else if (validation.transform_error_at_correspondences > 2.0) {
    validation.warnings.push_back(
        "Transform error at correspondences (" +
        std::to_string(validation.transform_error_at_correspondences) +
        " ID units) exceeds threshold (2.0 ID units)");
  }

  // 2. Check coverage area validity
  if (snapshot.toio_transform.coverage_area.width > 0.0 &&
      snapshot.toio_transform.coverage_area.height > 0.0) {
    validation.coverage_area_valid = true;
    
    // Validate coverage area matches expected toio playmat extent
    if (has_playmat_layout_) {
      const PlaymatInfo* playmat = playmat_layout_.GetPlaymat(snapshot.toio_transform.playmat_id);
      if (playmat) {
        const double tolerance = 1.0;  // 1 ID unit tolerance
        bool min_x_match = std::abs(snapshot.toio_transform.coverage_area.x - playmat->extent.min.x) < tolerance;
        bool min_y_match = std::abs(snapshot.toio_transform.coverage_area.y - playmat->extent.min.y) < tolerance;
        double coverage_max_x = snapshot.toio_transform.coverage_area.x + snapshot.toio_transform.coverage_area.width;
        double coverage_max_y = snapshot.toio_transform.coverage_area.y + snapshot.toio_transform.coverage_area.height;
        bool max_x_match = std::abs(coverage_max_x - playmat->extent.max.x) < tolerance;
        bool max_y_match = std::abs(coverage_max_y - playmat->extent.max.y) < tolerance;
        
        if (!min_x_match || !min_y_match || !max_x_match || !max_y_match) {
          validation.warnings.push_back(
              "Coverage area does not match expected playmat extent. "
              "Expected: (" + std::to_string(playmat->extent.min.x) + ", " +
              std::to_string(playmat->extent.min.y) + ") to (" +
              std::to_string(playmat->extent.max.x) + ", " +
              std::to_string(playmat->extent.max.y) + "), "
              "Got: (" + std::to_string(snapshot.toio_transform.coverage_area.x) + ", " +
              std::to_string(snapshot.toio_transform.coverage_area.y) + ") to (" +
              std::to_string(coverage_max_x) + ", " +
              std::to_string(coverage_max_y) + ")");
        }
      }
    }
  } else {
    validation.coverage_area_valid = false;
    validation.warnings.push_back("Coverage area is not valid (width or height is zero)");
  }

  // 3. Optional: Check roundtrip error (pixel → toio → pixel)
  // Sample a few test points from the image center and corners
  std::vector<cv::Point2f> test_pixels = {
    cv::Point2f(config_.color_width / 2.0f, config_.color_height / 2.0f),  // Center
    cv::Point2f(config_.color_width * 0.1f, config_.color_height * 0.1f),  // Top-left
    cv::Point2f(config_.color_width * 0.9f, config_.color_height * 0.1f),  // Top-right
    cv::Point2f(config_.color_width * 0.1f, config_.color_height * 0.9f),  // Bottom-left
    cv::Point2f(config_.color_width * 0.9f, config_.color_height * 0.9f)   // Bottom-right
  };

  if (!snapshot.homography_color_to_toio.empty()) {
    double roundtrip_error_sum = 0.0;
    int valid_roundtrips = 0;
    
    for (const auto& pixel : test_pixels) {
      cv::Point2f toio_pos = TransformPixelToToio(pixel, snapshot);
      
      // Check if toio position is valid (not zero and within coverage)
      if (toio_pos.x != 0.0f || toio_pos.y != 0.0f) {
        if (IsToioPositionInCoverage(toio_pos, snapshot)) {
          cv::Point2f back_pixel = TransformToioToPixel(toio_pos, snapshot);
          double error = cv::norm(pixel - back_pixel);
          roundtrip_error_sum += error;
          valid_roundtrips++;
        }
      }
    }
    
    if (valid_roundtrips > 0) {
      validation.roundtrip_error_px = roundtrip_error_sum / valid_roundtrips;
      if (validation.roundtrip_error_px > 2.0) {
        validation.warnings.push_back(
            "Roundtrip error (" + std::to_string(validation.roundtrip_error_px) +
            " pixels) exceeds threshold (2.0 pixels)");
      }
    }
  }

  return validation;
}

}  // namespace locomotion::calibration
