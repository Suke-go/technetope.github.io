# Interactive Calibration Capture Tool — Requirements Specification

**Target binary:** `locomotion/calibration/tools/capture_calibration_interactive`  
**Purpose:** Provide a visual, user-driven capture experience for ChArUco-based calibration on RealSense D415, reducing miscaptures and giving immediate feedback before `CalibrationPipeline` consumes a frame.  
**Related docs:**  
- [workingflow.md](../../workingflow.md) — implementation priorities / QC flow  
- [implementation_requirements.md](implementation_requirements.md) — core pipeline + detector contract  

---

## 1. Overview
- **Tool name:** `capture_calibration_interactive` (alias `capture_calibration_gui`)  
- **Users:** On-site operators who currently run `capture_calibration` CLI and need higher confidence before capturing.  
- **Goal:** Stream live camera data, visualize ChArUco detection, gate captures until the board is in a valid pose, and hand off frames to `CalibrationPipeline` for computation.  
- **Execution:** `sudo ./build/capture_calibration_interactive <config.json>` (sudo required, same as other RealSense utilities).  

---

## 2. Scope & Assumptions
- **Hardware:** Intel RealSense D415 connected over USB3, Apple Silicon macOS host.  
- **Inputs:** RealSense color (BGR8) + depth (Z16) frames as configured by existing `CalibrationPipeline`.  
- **Outputs:**  
  - Saved calibration JSON (schema v2.0) identical to CLI tool output.  
  - Optional captured frame dumps for debugging (`.png`/`.npy`).  
  - On-screen success/failure summary with reprojection metrics.  
- **Dependencies:** librealsense2, OpenCV (imgproc, highgui, aruco), spdlog, existing `CalibrationPipeline` & `CharucoDetector`.  
- **Out-of-scope:** Camera intrinsic recalibration, BLE/toio control, non-RealSense cameras.  

---

## 3. Functional Requirements

### 3.1 Camera Live Preview
| ID | Requirement | Notes |
| --- | --- | --- |
| FR-1 | Display live RealSense color stream in an OpenCV window | Use `cv::imshow` with double buffering. |
| FR-2 (P2) | Show depth map side-by-side or allow toggling | Default layout: color left, depth right; toggle via `D`. |
| FR-3 | Refresh preview at camera frame rate (15–30 FPS) | Limit processing overhead to keep latency <100 ms (see NFR). |
| FR-4 (P2) | Include resolution + FPS in title/overlay | E.g., window title `D415 640×480 @15FPS`. |

### 3.2 ChArUco Detection Visualization
| ID | Requirement | Notes |
| --- | --- | --- |
| FR-5 | Overlay detected ChArUco corners on the color preview | Reuse `CharucoDetector` output; draw with `cv::aruco::drawDetectedCornersCharuco`. |
| FR-6 | Show detection status indicator (Green/Yellow/Red) | Status derived from `interpolated_corners` vs `min_charuco_corners`. |
| FR-7 (P1) | Display detected/expected corner count text | Example `Detected: 18/24`. |
| FR-8 (P1) | Draw board outline + corner IDs | Use `cv::aruco::drawDetectedMarkers` and text overlays. |

### 3.3 User Instructions Display
| ID | Requirement | Notes |
| --- | --- | --- |
| FR-9 | Show placement & lighting guidelines in English | Static text panel; keep within window bounds. |
| FR-10 | Display keyboard shortcuts (`SPACE`, `ESC/Q`, `S`, `D`) | Overlay panel or status footer. |

### 3.4 Manual Capture Control
| ID | Requirement | Notes |
| --- | --- | --- |
| FR-11 | Capture triggers on `SPACE` | Use non-blocking `cv::waitKey` polling. |
| FR-12 (P2) | Only allow capture when `interpolated_corners ≥ min_charuco_corners` | Otherwise flash red warning. |
| FR-13 (P1) | Show visual feedback during capture (`Capturing…`, flash) | Short overlay + optional border invert. |
| FR-14 (P1) | Display capture count (current/target) | Target count pulled from config (`session_capture_goal`). |

### 3.5 Calibration Processing
| ID | Requirement | Notes |
| --- | --- | --- |
| FR-15 | Run calibration pipeline asynchronously after capture | Use worker thread; preview continues. |
| FR-16 | Show progress indicator while pipeline runs | Spinner/progress text in status area. |
| FR-17 (P1) | Display summary metrics on completion | Include reprojection RMS, plane std, inlier ratio. |
| FR-18 (P2) | Allow retry if result fails quality thresholds | Buttonless workflow: status instructs user to recapture. |

### 3.6 Results Export
| ID | Requirement | Notes |
| --- | --- | --- |
| FR-19 | Auto-save calibration JSON on success | Same schema path as CLI (config-provided output dir). |
| FR-20 (P2) | Save captured frames to debug directory when requested (`S`) | Include overlays + raw frames for offline review. |
| FR-21 | Display output file paths once written | Show absolute path in status panel. |

### 3.7 Error Handling & Resilience
- Gracefully handle: camera disconnect, detection failure, poor lighting, calibration failure.  
- Provide actionable message (e.g., “Camera stream lost — reconnect D415 and press SPACE to retry”).  
- Auto-retry RealSense pipeline init a limited number of times before aborting.  

---

## 4. Non-Functional Requirements
| ID | Requirement | Measurement |
| --- | --- | --- |
| NFR-1 | Window resizable while keeping preview aspect ratio | Use `cv::resizeWindow` + letterboxing. |
| NFR-2 | Readable overlays | Minimum font scale 0.6 on 1080p display, high-contrast colors. |
| NFR-3 | Instructions fit without scrolling | Max 5 bullet lines; wrap text if needed. |
| NFR-4 | Preview latency < 100 ms | Frame-to-display pipeline budget. |
| NFR-5 | Capture response < 500 ms | From SPACE press to capture acknowledgement. |
| NFR-6 | Calibration processing < 5 s | For typical single-frame session. |
| NFR-7 | Compatible with Apple Silicon macOS + RealSense D415 | Tested under Ventura/Sonoma. |
| NFR-8 | Requires sudo (USB access) | Documented in usage instructions. |
| NFR-9 | Must reuse `CalibrationPipeline` + `CharucoDetector` | No duplicate logic. |
| NFR-10 | Graceful error handling for camera/detection issues | No hard crashes; status message required. |

---

## 5. User Interface Blueprint

```
┌────────────────────────────────────────────────────────────────────┐
│ RealSense D415 Calibration – 640×480 @ 15 FPS                  [×] │
├────────────────────────────────────────────────────────────────────┤
│ ┌──────────────────────┐  ┌──────────────────────┐                 │
│ │  Color Feed +        │  │  Depth Map (optional)│                 │
│ │  ChArUco Overlay     │  │  Floor cues          │                 │
│ └──────────────────────┘  └──────────────────────┘                 │
│ Status: 🟢 Board Detected (18/24 corners)   Captures: 0/5          │
│ Progress: idle | capturing | processing...                         │
│ ┌──────────────────────────────────────────────────────────────┐   │
│ │ INSTRUCTIONS                                                 │   │
│ │  • Place board 40–80 cm from camera, parallel to floor       │   │
│ │  • Avoid glare / hard shadows                                │   │
│ │  • Wait for green status, then press SPACE                   │   │
│ │ CONTROLS: [SPACE] Capture  [S] Save Frame  [D] Toggle Overlay│   │
│ │           [ESC/Q] Quit                                       │   │
│ └──────────────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────────┘
```

---

## 6. Architecture & Components

### 6.1 High-level Flow
1. Initialize RealSense pipeline (color + depth) using current config (resolution, FPS, align-to-color).  
2. Enter preview loop (~30 FPS):  
   - Fetch frames, align, undistort color via `CalibrationPipeline` helper.  
   - Run `CharucoDetector::Detect` to obtain detection result + metrics.  
   - Render overlays + status UI.  
   - Poll keyboard input for controls.  
3. On `SPACE` (when detection healthy) enqueue captured frame bundle to processing thread.  
4. Worker thread invokes `CalibrationPipeline`/`CalibrationSession` to compute calibration result.  
5. UI thread receives progress + final status via thread-safe queue and updates overlays.  

### 6.2 Key Classes
| Component | Responsibility | Notes |
| --- | --- | --- |
| `InteractiveCalibrationTool` | Owns lifecycle; orchestrates preview loop, input, rendering. | Singleton per process. |
| `CalibrationPipeline` (existing) | Performs calibration math + JSON serialization. | Called asynchronously. |
| `CharucoDetector` (existing) | Provides detection data for overlays + validation gating. | Reuse to avoid logic drift. |
| `FrameRecorder` (new helper) | Saves optional debug frames on demand. | Controlled via `S` shortcut/config. |
| `StatusPanelRenderer` (helper) | Renders text blocks, status icons, progress indicators using OpenCV primitives. | Keeps UI code isolated. |

### 6.3 Threading Model
- **UI/Preview thread:** RealSense frame grab + detection + rendering.  
- **Worker thread:** Processes capture queue (size=1) to avoid concurrent calibrations.  
- Synchronize via `std::atomic<State>` for gating capture availability.  

### 6.4 Integration Touchpoints
- Shares `CalibrationConfig` JSON loader with CLI tool.  
- Uses `CalibrationPipeline::CaptureSnapshot` (or equivalent) to package color/depth frames.  
- Emits metrics through existing `CalibrationResult` struct for downstream logging/QC.  

---

## 7. Configuration

All existing calibration parameters remain valid. Additional GUI-specific keys (extend current JSON schema under `"interactive_ui"` namespace):
| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `preview_scale` | float | 1.0 | Display scale multiplier for window content. |
| `show_depth` | bool | true | Enable side-by-side depth preview. |
| `show_instructions` | bool | true | Toggle instructions panel. |
| `min_charuco_corners` | int | 18 | Threshold for green status / capture enable. |
| `auto_capture_mode` | bool | false | Optional mode: auto-trigger after board stable for `auto_capture_delay_s`. |
| `capture_goal` | int | 5 | Target number of captures to show in counter. |
| `debug_frame_dir` | path | `logs/calibration_debug` | Save path when user presses `S`. |
| `status_overlay_alpha` | float | 0.85 | Background opacity for text panels. |

Config loader falls back to defaults when fields are absent to remain backward compatible.

---

## 8. User Workflow
1. Operator runs `sudo ./build/capture_calibration_interactive configs/calibration.json`.  
2. Tool performs RealSense init + streaming; window appears with live feeds.  
3. Operator positions ChArUco board 40–80 cm away, adjusts until indicator turns green.  
4. Press `SPACE` to capture; UI flashes `Capturing…`, counter increments.  
5. Background processing runs; UI shows progress and eventually success/failure metrics.  
6. On success, tool prints/saves JSON path and optionally lets the operator exit or capture again (multi-shot).  
7. On failure, UI displays reason (insufficient corners, plane std too high, etc.) with recommended actions.  

---

## 9. Error Handling & Messaging
- **Camera not found:** show blocking dialog text, retry connection every 2 s, log via spdlog.  
- **Board not detected:** keep Yellow/Red status, overlay tip “Ensure board fills ≥30% of view, reduce glare.”  
- **Poor lighting:** detect via low contrast/failed corners; prompt to adjust lighting.  
- **Calibration quality insufficient:** display metrics vs thresholds and suggest recapturing.  
- **Depth stream loss:** automatically disable depth preview and continue color-only mode with warning.  

All errors must avoid crashing the UI and must be user-visible within two frames.

---

## 10. Future Enhancements (Backlog)
1. **Auto-capture mode:** start capture automatically after board stable for N seconds.  
2. **Multi-capture session scoring:** gather 3–5 captures and choose best result.  
3. **Calibration history panel:** list previous runs + timestamps.  
4. **Board generator utility:** quick access to ChArUco template printing.  
5. **Real-time quality score:** predict capture quality before pressing SPACE.  

---

## 11. Acceptance Criteria
- ✅ Live preview works with RealSense D415 on Apple Silicon.  
- ✅ ChArUco detection overlays update in real time (≥15 FPS).  
- ✅ Capture triggers via SPACE and is blocked until board detection passes threshold.  
- ✅ Calibration completes using existing pipeline and writes JSON automatically.  
- ✅ UI displays instructions + keyboard shortcuts in English.  
- ✅ Errors (camera missing, board missing, lighting, poor quality) are explained in status panel.  
- ✅ End-to-end run succeeds under sudo with the same config files as CLI tool.  

Meeting all P0 requirements (FR-1, FR-3, FR-5, FR-6, FR-9, FR-10, FR-11, FR-15, FR-19) is mandatory for shipping; P1/P2 items are tracked but can defer if blocked.  

---

## 12. Open Questions
1. Should capture goal remain configurable per session or inferred from config (e.g., `session.num_snapshots`)?  
2. Is multi-language support required for instructions (JP/EN), or is English sufficient for initial release?  
3. Can we leverage ImGui for richer UI, or must we stay with pure OpenCV windows for portability?  
4. How should we persist debug frames (PNG vs lossless video) to balance speed and disk usage?  

Clarifying the above before implementation will de-risk the first milestone.  
