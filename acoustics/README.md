# Acoustics Build Guide (Beginner Friendly)

Welcome! This document walks you through setting up the `acoustics` workspace so you can build and test both the device firmware and the PC-side tools with confidence.

## Project Layout
- `firmware/` – PlatformIO project for the M5StickC Plus2 device.
- `pc_tools/` – C++17 utilities (scheduler, monitor, shared OSC library).
- `sound_assets/` – Reference WAV files and the shared sound manifest.
- `agents/` – Documentation for Agent A/B behaviours and protocols.
- `tests/` – Manual and automated checklists (see `audio_smoke_test.md`).
- `docs/` – High level design notes that apply across the acoustics stack.

## Tooling Checklist
### Common setup
- Git, a recent shell (PowerShell, bash, zsh), and Python 3.10+.
- CMake ≥ 3.16 and a build tool (Ninja or Make) for the C++ projects.
- A C++17-capable compiler (GCC 11+, Clang 13+, or Visual Studio 2022).

### Firmware (M5StickC Plus2)
- Install PlatformIO CLI  
  ```sh
  pip install --user platformio
  ```
  or use the VS Code PlatformIO extension.
- USB data cable plus the CH9102 serial driver (for Windows/macOS).
- Optional: `pio settings set force_verbose yes` to see detailed logs.

### PC tools (scheduler / monitor)
- Recommended package manager: `conan` or `vcpkg` (future dependency management).  
  For now all third-party headers live under `pc_tools/third_party/`.
- On Windows, enable the “Desktop development with C++” workload.
- On Linux/macOS, ensure `build-essential` (Debian) or `xcode-select --install` (macOS) is available.

## Firmware Workflow
1. **Create secrets**
   ```sh
   cp acoustics/firmware/include/Secrets.example.h acoustics/firmware/include/Secrets.h
   ```
   Edit `Secrets.h` and set Wi-Fi SSID/password plus OSC keys.
2. **Prepare sound assets**
   - Place WAV files under `acoustics/firmware/data/presets/`.
   - Update `acoustics/firmware/data/manifest.json` with the filenames and gain values.  
     The `acoustics/sound_assets/` folder contains example material such as `presets/sample.wav`.
3. **Build and upload**
   ```sh
   cd acoustics/firmware
   pio run                      # compile the firmware
   pio run -t upload            # flash to the M5StickC Plus2
   pio run -t uploadfs          # (optional) upload SPIFFS sound assets
   ```
4. **Monitor the device**
   ```sh
   pio device monitor -b 115200
   ```
   You should see logs for Wi-Fi connection, NTP sync, OSC events, and heartbeat messages.  
   If you just want to validate audio output, follow the checklist in `acoustics/tests/audio_smoke_test.md`.

## PC Tools Workflow
1. **Configure a build directory**
   ```sh
   cmake -S acoustics/pc_tools -B build/acoustics -DCMAKE_BUILD_TYPE=Release
   ```
   Add `-G Ninja` if you prefer Ninja over Makefiles.
2. **Compile the tools**
   ```sh
   cmake --build build/acoustics
   ```
   This produces:
   - `build/acoustics/scheduler/agent_a_scheduler`
   - `build/acoustics/monitor/agent_a_monitor`
3. **Run the scheduler (example)**
   ```sh
   ./build/acoustics/scheduler/agent_a_scheduler \
     acoustics/pc_tools/scheduler/examples/basic_timeline.json \
     --host 255.255.255.255 --port 9000 --spacing 0.02
   ```
   Use `--dry-run` to print bundles without transmitting. Set `--base-time 2024-05-01T21:00:00Z` to schedule relative to a fixed UTC time.
4. **Run the monitor (example)**
   ```sh
   ./build/acoustics/monitor/agent_a_monitor --port 9100 --csv logs/heartbeat.csv
   ```
   Stop with `Ctrl+C`; a metrics summary prints before the program exits.

## Helpful References
- `acoustics/firmware/README.md` – Module deep dive, FreeRTOS task overview.
- `acoustics/pc_tools/README.md` – Planned dependency strategy and CLI tips.
- `acoustics/tests/audio_smoke_test.md` – First-boot validation sequence.
- `docs/masterdocs.md` – Shared background knowledge for the wider project.

Stuck or unsure? Open an issue in `acoustics/process/` documents or leave TODO comments near the code you are working on. Happy building!
