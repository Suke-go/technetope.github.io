# PC Sequencer Workflow (Draft)

1. **Start NTP service**
   - Ensure the local NTP server is running and reachable by all M5Stick devices.
2. **Load SoundTimeline**
   - Import or author the playback schedule (JSON/CSV/UI).
3. **Preview**
   - Simulate output locally to validate timing and parameter changes.
4. **Launch GUI**
   - Open the control surface in `pc_tools/gui/` to visualise device list, states, and live metrics.
   - Edit timeline entries or tweak gain/pan parameters in real time; changes should propagate to the scheduler backend.
5. **Broadcast Commands**
   - Generate OSC bundles with future timetags and send via UDP broadcast/multicast.
   - Use the GUI to trigger single/multi-device tests by selecting device IDs from the list.
6. **Monitor**
   - Collect heartbeats, delay metrics, and logs from each device (GUI and CLI双方から確認)。
7. **Adjust**
   - If jitter exceeds threshold, tweak lead time or network settingsとともにGUIから即時再試行し、結果を文書化。

Document CLI commands, GUI screenshots, and automation scripts as they become available.
