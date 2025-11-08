# RealSense D415 Floor Calibration System - Requirements Specification v2.0

**Project**: locomotion/calibration
**Target Camera**: Intel RealSense D415
**Target Platform**: Apple Silicon macOS (primary), Linux (secondary)
**Date**: 2025-11-09
**Status**: Draft for Review

---

## 1. System Overview

### 1.1 Purpose
Calibrate overhead-mounted RealSense D415 camera to:
1. Detect floor plane equation in 3D space
2. Compute homography from camera image to floor coordinates
3. Enable transformation from camera pixels to toio Position ID coordinate system
4. Detect and track human feet positions (150-300mm above floor)

### 1.2 Physical Setup
```
                    [RealSense D415]
                          ↓ (2600mm)
    ════════════════════════════════  ← Detection Zone (150-300mm above floor)
                          ↓ (2300-2450mm)
                    [Human Feet]
                          ↓
    ════════════════════════════════  ← Floor (0mm)
           [toio Playmat #01]
         [ChArUco Board 5x7]
```

**Constraints:**
- Camera mount height: **2600mm ± 100mm** (fixed installation)
- Camera orientation: **Facing downward**, angle TBD (approximately perpendicular to floor)
- Field of view: Must cover at least **420mm × 297mm** (A3 playmat size)
- Detection target height: **150-300mm above floor** (human feet)
- Walking zone height: **800-1200mm above floor** (human body, for future use)

---

## 2. Coordinate Systems

### 2.1 Camera Coordinate System
- **Origin**: Camera optical center
- **Z-axis**: Camera optical axis (positive = away from camera)
- **X-axis**: Right (image width direction)
- **Y-axis**: Down (image height direction)
- **Units**: millimeters (mm)

### 2.2 Floor Coordinate System
- **Origin**: Arbitrary point on floor plane
- **Z-axis**: Floor normal vector (positive = up from floor)
- **X, Y-axes**: Parallel to floor, orientation TBD
- **Units**: millimeters (mm)

### 2.3 ChArUco Board Coordinate System
- **Origin**: Top-left inner corner of ChArUco board
- **X-axis**: Right (along top edge)
- **Y-axis**: Down (along left edge)
- **Z-axis**: Normal to board surface (positive = toward camera)
- **Physical dimensions**:
  - Width: `(charuco_squares_x - 1) × charuco_square_length_mm` = 180mm
  - Height: `(charuco_squares_y - 1) × charuco_square_length_mm` = 270mm
- **Units**: millimeters (mm)

### 2.4 toio Position ID Coordinate System
- **Origin**: System-defined, varies by playmat
- **A3 Simple Playmat (#01)** (from sequential mat TMD01SS):
  - Start (top-left): **(x=34, y=35)**
  - End (bottom-right): **(x=339, y=250)**
  - Physical size: **420mm × 297mm**
  - Coordinate range: **305 units (x) × 215 units (y)**
- **X-axis**: Right (increases →)
- **Y-axis**: Down (increases ↓)
- **Units**: toio Position ID units (approx. 0.73 units/mm)
- **Conversion factor**:
  - X: `0.7261904762` ID units per mm (305 ID range / 420mm)
  - Y: `0.7239057239` ID units per mm (215 ID range / 297mm)
- **Note**: This corresponds to face #01 (top-left) of the 3×4 grid layout in the sequential mat. The full mat spans x: 34-949, y: 35-898 across 12 faces.

---

## 3. Functional Requirements

### FR-1: Camera Configuration
**Priority**: P0
**Description**: Configure RealSense D415 with optimal settings for overhead floor detection.

**Acceptance Criteria:**
- ✅ Color stream: **640×480 @ 15 FPS** (RGB8 format)
- ✅ Depth stream: **640×480 @ 15 FPS** (Z16 format)
- ✅ Depth scale: **0.001** (1 unit = 1mm)
- ✅ Depth range: **2300-2800mm** from camera
- ✅ Auto-exposure enabled for color stream
- ✅ Depth preprocessing: spatial filter enabled

**Configuration Parameters:**
```json
{
  "color_width": 640,
  "color_height": 480,
  "depth_width": 640,
  "depth_height": 480,
  "fps": 15,
  "depth_scale": 0.001,
  "depth_min_distance_mm": 2300.0,
  "depth_max_distance_mm": 2800.0
}
```

---

### FR-2: ChArUco Board Detection
**Priority**: P0
**Description**: Detect ChArUco board corners in color image for calibration.

**Board Specification:**
- Dictionary: **DICT_4X4_50** (OpenCV ArUco)
- Grid size: **5×7 squares**
- Square length: **45.0mm**
- Marker length: **33.0mm** (73.3% of square)
- Total physical size: **180mm × 270mm** (4×6 inner corners)

**Detection Requirements:**
- ✅ Minimum detectable corners: **8** (relaxed for preview)
- ✅ Minimum for calibration: **12** (recommended: 16+)
- ✅ Subpixel refinement: **Enabled** (window size: 5×5, max iterations: 30)
- ✅ Corner detection tolerance: **±0.5 pixels** after refinement
- ✅ Homography RANSAC threshold: **3.0 pixels**
- ✅ Maximum reprojection error: **8.0 pixels** (average across all corners)

**Output:**
```cpp
struct CharucoDetectionResult {
  std::vector<cv::Point2f> image_points;      // Detected corners in image (pixels)
  std::vector<cv::Point3f> board_points;      // Corresponding 3D points on board (mm)
  std::vector<int> ids;                       // Corner IDs
  int detected_markers;                       // Number of detected ArUco markers
  int detected_charuco_corners;               // Number of detected ChArUco corners
};
```

---

### FR-3: Floor Plane Estimation
**Priority**: P0
**Description**: Estimate floor plane equation from depth image using RANSAC.

**Algorithm:**
1. Convert depth image to 3D point cloud (camera coordinates)
2. Filter points by Z-range: **2400-2800mm** (floor region)
3. Downsample point cloud: **4×4 grid** (reduce computational cost)
4. Apply RANSAC to fit plane: `ax + by + cz + d = 0`
5. Compute inliers and standard deviation

**Parameters:**
```json
{
  "floor_inlier_threshold_mm": 8.0,
  "floor_ransac_iterations": 500,
  "floor_min_inlier_ratio": 0.7,
  "floor_z_min_mm": 2400.0,
  "floor_z_max_mm": 2800.0,
  "floor_downsample_grid": 4,
  "random_seed": 42
}
```

**Quality Metrics:**
- ✅ Plane standard deviation: **≤ 8.0mm**
- ✅ Inlier ratio: **≥ 0.7** (70% of points within threshold)
- ✅ Minimum valid points: **≥ 100** after downsampling

**Output:**
```cpp
struct FloorPlaneResult {
  cv::Vec4f plane;              // Plane equation [a, b, c, d]
  double plane_std_mm;          // Standard deviation of inliers (mm)
  double inlier_ratio;          // Ratio of inliers to total points
  int inlier_count;             // Number of inlier points
  double camera_height_mm;      // Estimated camera height above floor
};
```

**Validation:**
- Camera height must be in range: **2500-2700mm**
- If outside range, display warning but allow continuation

---

### FR-4: Homography Computation
**Priority**: P0
**Description**: Compute homography matrix from camera image plane to floor plane.

**Method:**
1. Use ChArUco corner correspondences:
   - Image points: `image_points` (2D, pixels)
   - Board points: `board_points` (3D, mm) projected onto floor plane
2. Apply `cv::findHomography()` with RANSAC
3. Validate reprojection error

**Parameters:**
```json
{
  "homography_ransac_thresh_px": 3.0,
  "max_reprojection_error_id": 8.0
}
```

**Output:**
- Homography matrix **H** (3×3, double precision)
- Transformation: `[x_floor, y_floor, 1]^T = H × [u, v, 1]^T`
  - Input: Image pixel `(u, v)`
  - Output: Floor position `(x_floor, y_floor)` in mm

**Validation:**
- Average reprojection error: **≤ 8.0 pixels**
- Per-corner error: **≤ 15.0 pixels** (outlier threshold)

---

### FR-5: toio Coordinate Transform
**Priority**: P1
**Description**: Transform floor coordinates to toio Position ID coordinate system.

**Input Data Source:**
- File: `config/toio_playmat.json`
- Playmat: `a3_simple` (A3 Simple Playmat #01)
- Mount: `center_mount_nominal` (ChArUco board mounted at center)

**Transformation Pipeline:**
```
Image pixel (u, v)
  ↓ [Homography H_cam_to_floor]
Floor position (x_floor, y_floor) in mm
  ↓ [Offset & Scale]
ChArUco board position (x_board, y_board) in mm
  ↓ [Affine transform from correspondences]
toio Position ID (x_toio, y_toio) in ID units
```

**ChArUco Board to toio Position ID Correspondences:**
(from `toio_playmat.json`, center mount for face #01)
```json
{
  "board_point_mm": {"x": 0.0, "y": 0.0},
  "position_id": {"x": 121.143, "y": 44.773}
},
{
  "board_point_mm": {"x": 180.0, "y": 0.0},
  "position_id": {"x": 251.857, "y": 44.773}
},
{
  "board_point_mm": {"x": 0.0, "y": 270.0},
  "position_id": {"x": 121.143, "y": 240.227}
}
```
**Note**: These correspondences assume the board is centered on the playmat. Actual mount positions should be measured and updated in the configuration file.

**Computation:**
1. Compute affine transformation `A` (3×3) from 3 correspondences
2. Verify transformation accuracy with known points
3. Output combined transformation: `T_toio = A × H_cam_to_floor`

**Acceptance Criteria:**
- ✅ Transformation error at correspondence points: **≤ 2.0 ID units** (≈ 1.5mm)
- ✅ Coverage area: toio range **(34, 35) to (339, 250)** for face #01
- ✅ Position accuracy for foot detection: **≤ 5.0 ID units** (≈ 3.5mm)

**Output:**
```cpp
struct ToioCoordinateTransform {
  cv::Mat transform_matrix;        // 3×3 combined transformation
  cv::Rect2d toio_coverage_area;   // Coverage in toio ID units
  double transform_error_id;       // RMS error at correspondences
  std::string playmat_id;          // "a3_simple"
  std::string mount_label;         // "center_mount_nominal"
};
```

---

### FR-6: Interactive Calibration GUI
**Priority**: P0
**Description**: Provide real-time visual feedback during calibration process.

**Display Layout:**
```
┌─────────────────────────────────────────────────┐
│ [Color Preview 640×480]  │ [Depth Heatmap]      │
│                          │                      │
│ ● Detected corners: 18   │ Height: 150-300mm    │
│ ○ Status: Ready (GREEN)  │ (Blue→Red gradient)  │
│                          │                      │
│ Captures: 3/5            │                      │
│ Last: RMS 5.2px, 6.8mm   │ INSTRUCTIONS:        │
│ Height: 2.59m            │ • Place board flat   │
│                          │ • Distance: 40-80cm  │
│ [SPACE] Capture          │ • Press SPACE (green)│
│ [D] Overlay [Q] Quit     │ • [ESC] to quit      │
└─────────────────────────────────────────────────┘
```

**Visual Feedback:**
- ✅ **ChArUco overlay**: Green circles at detected corners, yellow IDs
- ✅ **Status indicator**: Color-coded circle (Red/Yellow/Green)
  - 🔴 Red: < 8 corners detected
  - 🟡 Yellow: 8-11 corners detected
  - 🟢 Green: ≥ 12 corners detected (ready to capture)
- ✅ **Depth heatmap**: Height above floor (0-500mm range)
  - Blue: 0-150mm (below detection zone)
  - Green: 150-300mm (foot detection zone) ← **TARGET**
  - Yellow: 300-450mm (above feet)
  - Red: 450mm+ (out of range)
- ✅ **Real-time FPS**: Display preview frame rate (target: 10-15 FPS)
- ✅ **Progress bar**: Visual capture progress (3/5 completed)

**User Controls:**
| Key | Action | Condition |
|-----|--------|-----------|
| SPACE | Capture frame | Status = Green |
| S | Save debug frame | Always |
| D | Toggle overlay | Always |
| Q / ESC | Quit | Always |

**Status Messages (Toast notifications):**
- "Capturing frame..." (1s)
- "Calibration saved to {path}" (3s)
- "Board not ready for capture" (2s)
- "Already processing. Please wait." (2s)
- "Saved frame: {path}" (3s)

---

### FR-7: Multi-Capture Session
**Priority**: P0
**Description**: Acquire multiple calibration samples to improve robustness.

**Session Parameters:**
```json
{
  "session_attempts": 5,           // Target number of captures
  "max_plane_std_mm": 8.0,         // Quality threshold for plane fit
  "min_inlier_ratio": 0.7,         // Quality threshold for inliers
  "save_intermediate_snapshots": true,
  "snapshot_output_dir": "debug_snapshots"
}
```

**Workflow:**
1. User captures frames when board is well-detected (green status)
2. Each capture triggers background processing:
   - ChArUco detection
   - Floor plane estimation
   - Homography computation
   - Quality validation
3. Valid captures increment success counter: `N/5`
4. After 5 successful captures, final result is saved
5. User can continue capturing or quit

**Quality Checks per Capture:**
- ✅ ChArUco corners: ≥ 12 detected
- ✅ Reprojection error: ≤ 8.0 pixels
- ✅ Plane std: ≤ 8.0 mm
- ✅ Inlier ratio: ≥ 0.7
- ✅ Camera height: 2500-2700 mm (warning if outside)

**Result Selection:**
- Use **most recent valid capture** as final result
- (Alternative: Use capture with lowest reprojection error)

---

### FR-8: Calibration Result Output
**Priority**: P0
**Description**: Save calibration result to JSON file for downstream use.

**Output File:** `calib_result.json`

**JSON Schema:**
```json
{
  "schema_version": "2.0",
  "timestamp": "2025-11-09T15:30:45Z",

  "camera": {
    "model": "Intel RealSense D415",
    "serial_number": "038522062737",
    "mount_height_mm": 2587.3,
    "color_resolution": {"width": 640, "height": 480},
    "depth_resolution": {"width": 640, "height": 480},
    "fps": 15
  },

  "intrinsics": {
    "fx": 613.01,
    "fy": 612.75,
    "cx": 326.75,
    "cy": 246.01,
    "distortion_model": "brown_conrady",
    "distortion_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0]
  },

  "floor_plane": {
    "coefficients": [0.002, -0.001, 0.9999, -2587.3],
    "normal_vector": [0.002, -0.001, 0.9999],
    "distance_from_origin_mm": 2587.3,
    "std_mm": 6.8,
    "inlier_ratio": 0.92,
    "inlier_count": 1523
  },

  "homography_color_to_floor": [
    [1.234, -0.056, 123.4],
    [0.034, 1.456, 234.5],
    [0.0001, -0.0002, 1.0]
  ],

  "toio_coordinate_transform": {
    "playmat_id": "a3_simple",
    "board_mount_label": "center_mount_nominal",
    "transform_color_to_toio": [
      [0.0082, -0.0003, 180.5],
      [0.0002, 0.0097, 148.2],
      [0.00001, -0.00001, 1.0]
    ],
    "coverage_area_toio_id": {
      "min": {"x": 34.0, "y": 35.0},
      "max": {"x": 339.0, "y": 250.0}
    },
    "transform_error_id": 1.8
  },

  "quality_metrics": {
    "charuco_corners": 20,
    "reprojection_error_px": 5.2,
    "capture_count": 5,
    "timestamp_first": "2025-11-09T15:28:10Z",
    "timestamp_last": "2025-11-09T15:30:45Z"
  },

  "validation": {
    "passed": true,
    "checks": {
      "reprojection_error": "PASS",
      "floor_plane_std": "PASS",
      "floor_inlier_ratio": "PASS",
      "camera_height": "PASS",
      "toio_transform": "PASS"
    },
    "warnings": []
  },

  "config": {
    "calibration_config": { /* full calibration config */ },
    "session_config": { /* full session config */ }
  }
}
```

---

### FR-9: Foot Position Detection (Future)
**Priority**: P2
**Description**: Real-time detection of human feet positions using calibrated camera.

**Detection Pipeline:**
1. Capture depth frame
2. Filter depth by range: **2300-2450mm** (floor + 150-300mm)
3. Segment connected components (potential feet)
4. Filter by size: **50-200mm diameter**
5. Compute centroid for each component
6. Transform centroid to toio coordinates

**Output:**
```cpp
struct FootPosition {
  cv::Point2f image_pos;        // Position in image (pixels)
  cv::Point3f floor_pos_mm;     // Position on floor (mm)
  cv::Point2f toio_pos_id;      // Position in toio ID units
  double height_above_floor_mm; // Height above floor
  double confidence;            // Detection confidence (0-1)
  int foot_id;                  // Tracking ID
};
```

**Performance Requirements:**
- ✅ Latency: **< 50ms** (capture → toio position)
- ✅ Frame rate: **≥ 15 FPS**
- ✅ Position accuracy: **≤ 10mm** (in floor coordinates)
- ✅ toio accuracy: **≤ 7 ID units** (≈ 5mm)

---

## 4. Non-Functional Requirements

### NFR-1: Performance
- **Calibration time**: ≤ 2 minutes for 5 captures (interactive)
- **Frame processing**: ≤ 100ms per frame (background async)
- **Preview frame rate**: ≥ 10 FPS (interactive GUI)
- **Memory usage**: ≤ 500MB RAM

### NFR-2: Reliability
- **ChArUco detection rate**: ≥ 80% when board is visible
- **Floor plane estimation success rate**: ≥ 90% with valid depth data
- **Calibration reproducibility**: RMS error variance ≤ 1.0px across 5 runs

### NFR-3: Usability
- **Setup time**: ≤ 5 minutes (including camera mounting and board placement)
- **User guidance**: On-screen English instructions, color-coded status
- **Error messages**: Clear, actionable (e.g., "Move board closer" vs "Detection failed")
- **Recovery**: Allow retry without restart

### NFR-4: Maintainability
- **Code structure**: Modular (separate classes for detection, estimation, calibration)
- **Configuration**: JSON-based, hot-reloadable
- **Logging**: Structured logging with spdlog (debug/info/warn/error levels)
- **Testing**: Unit tests for each module, integration test for full pipeline

### NFR-5: Portability
- **Primary platform**: Apple Silicon macOS (M1/M2/M3)
- **Secondary platform**: Linux (x86_64, ARM64)
- **Dependencies**: RealSense SDK, OpenCV 4.x, spdlog, nlohmann_json
- **Build system**: CMake 3.20+, C++20

---

## 5. Constraints & Assumptions

### 5.1 Hardware Constraints
- **Camera**: Intel RealSense D415 (firmware ≥ 5.16.0.1)
- **USB**: USB 3.0 required for full resolution/framerate
- **Mount**: Rigid mount (no vibration/movement during calibration)
- **Lighting**: Ambient indoor lighting (avoid direct sunlight, harsh shadows)

### 5.2 Software Constraints
- **OS permissions**: macOS requires `sudo` for camera access (or USB permission rules)
- **Camera exclusivity**: Kill conflicting processes (VDCAssistant, AppleCameraAssistant)
- **OpenCV version**: 4.5.0+ (ArUco module included)

### 5.3 Physical Setup Assumptions
- ChArUco board is **printed on flat rigid surface** (foam board, acrylic)
- Board is placed **parallel to floor** (tilt ≤ 5°)
- Board is **within camera FOV** (40-80cm from camera projection)
- Floor is **flat and opaque** (no mirrors, glass, water)
- toio playmat is **securely attached to floor** (no wrinkles, movement)

### 5.4 Coordinate System Assumptions
- ChArUco board origin (top-left corner) is **aligned with toio playmat**
- `toio_playmat.json` correspondences are **measured accurately** (±2mm)
- Floor plane is **approximately horizontal** (normal vector ≈ [0, 0, 1])

---

## 6. Testing Requirements

### 6.1 Unit Tests
- ✅ `CharucoDetector`: Synthetic board images with known corners
- ✅ `FloorPlaneEstimator`: Synthetic depth data with known plane
- ✅ `CalibrationPipeline`: Mock camera data with known transformation
- ✅ Config loading: Validate JSON parsing and defaults

### 6.2 Integration Tests
- ✅ End-to-end calibration with mock data
- ✅ Coordinate transformation accuracy (board → toio)
- ✅ GUI rendering without camera (mock frames)

### 6.3 Hardware Tests (RealSense Required)
- ✅ Camera initialization and frame capture
- ✅ ChArUco detection with printed board
- ✅ Floor plane estimation with real depth data
- ✅ Full calibration session (5 captures)
- ✅ Result validation against known ground truth

### 6.4 Acceptance Tests
1. **Calibration Accuracy Test**:
   - Place markers at known toio positions
   - Capture image and transform to toio coordinates
   - Measure error: **target ≤ 5 ID units**

2. **Reproducibility Test**:
   - Run calibration 5 times without moving camera/board
   - Compute variance in homography matrix
   - Target: RMS reprojection error std dev ≤ 1.0px

3. **Height Accuracy Test**:
   - Measure actual camera-to-floor distance with tape measure
   - Compare to calibrated `camera_height_mm`
   - Target: ≤ 10mm error

4. **Performance Test**:
   - Measure time from launch to first valid capture
   - Target: ≤ 30 seconds

---

## 7. Deliverables

### 7.1 Code Modules
- ✅ `CalibrationPipeline` - Core calibration logic
- ✅ `CharucoDetector` - ChArUco board detection
- ✅ `FloorPlaneEstimator` - RANSAC plane fitting
- ✅ `CalibrationSession` - Multi-capture session management
- ✅ `capture_calibration_interactive` - Interactive GUI tool
- ⏸️ `ToioCoordinateTransform` - Coordinate transformation utilities (P1)

### 7.2 Configuration Files
- ✅ `calibration_config.json` - Full resolution config
- ✅ `calibration_config_low_res.json` - Low resolution config (640×480@15fps)
- ✅ `config/toio_playmat.json` - Playmat and board mount definitions

### 7.3 Documentation
- ✅ `README.md` - Overview and quick start
- ✅ `implementation_requirements.md` - Implementation details (v1)
- ✅ `REQUIREMENTS_CALIBRATION_V2.md` - This document (v2)
- ⏸️ `SETUP_GUIDE.md` - Hardware setup and calibration procedure (P1)
- ⏸️ `COORDINATE_SYSTEMS.md` - Detailed coordinate system reference (P1)

### 7.4 Test Artifacts
- ✅ Unit tests: `test_floor_plane_estimator.cpp`, `test_calibration_pipeline_integration.cpp`
- ⏸️ Hardware test data: Sample captures with ground truth (P1)
- ⏸️ Validation report: Accuracy measurements (P1)

---

## 8. Open Questions & Future Work

### 8.1 Open Questions (Need Decision)
1. **ChArUco board placement**:
   - Should board be permanently mounted on playmat?
   - Or placed temporarily during calibration only?

2. **Multiple playmat support**:
   - Do we need to support stitching multiple toio playmats (#01-#12)?
   - If yes, how to handle discontinuous Position ID space?

3. **Camera angle optimization**:
   - Current assumption: Camera faces downward (≈90° from floor)
   - Should we support angled mounting (e.g., 60-80° for wider FOV)?

4. **Recalibration frequency**:
   - How often should system be recalibrated?
   - Trigger conditions: camera moved, playmat moved, accuracy degradation?

5. **UI layout preference**:
   - Side-by-side (color + depth) vs. stacked layout?
   - Should depth heatmap show height above floor or raw distance?

### 8.2 Future Enhancements (P2)
1. **Auto-capture mode**: Automatically capture when board is well-detected
2. **Multi-camera support**: Stitch calibrations from multiple cameras
3. **Dynamic recalibration**: Update calibration during runtime if drift detected
4. **3D visualization**: Show camera FOV, floor plane, and toio coverage in 3D viewer
5. **Foot tracking**: Implement real-time foot detection and tracking (FR-9)
6. **Occlusion handling**: Detect and filter occluded regions (e.g., furniture legs)
7. **Historical data**: Log calibration history and detect parameter drift over time

---

## 9. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-11-08 | Initial | Original implementation requirements |
| 2.0 | 2025-11-09 | Revision | Detailed spec with toio coordinates, height constraints, acceptance criteria |

---

## 10. Sign-off

**Prepared by**: Claude (AI Assistant)
**Review Status**: **DRAFT - AWAITING USER REVIEW**
**Action Required**:
- Review all sections for accuracy
- Answer open questions (Section 8.1)
- Approve or request revisions
- Proceed to implementation upon approval

---

**END OF REQUIREMENTS SPECIFICATION v2.0**
