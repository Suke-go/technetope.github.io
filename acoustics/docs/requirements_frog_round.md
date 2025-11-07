# Acoustic Round Control Requirements

This document consolidates the functional and non-functional requirements for operating up to 30 M5StickC Plus2 devices in a coordinated “frog song” round using the existing acoustics stack. It captures asset preparation, firmware behaviour, scheduler logic, monitoring, time synchronisation, networking, and validation expectations. Each requirement is phrased to avoid overlap and to be directly testable.

## 1. Scope & Goals
- **Primary objective**: achieve precise, repeatable playback of a four-voice round (and variations) across up to 30 devices, using OSC transport with ≥3 s scheduling lead time.
- **Command origin**: the master PC is the sole authority for issuing playback instructions; all tooling (CLI, APIs, GUI) run there and relay OSC to the devices.
- **Operator interface**: while new GUI work is deferred, the system must provide programmatic control hooks (library/API) so a GUI or automation layer can drive playback without diverging code paths.
- **Stakeholders**: firmware developers, PC tooling developers, test operators.

## 2. Assets & Preset Management
1. WAV assets used by the firmware shall be generated via `tools/sound_gen/generate_missing_f0.py` in 16 kHz, mono, 16-bit PCM format to limit SPIFFS usage.
2. Each tone shall be declared in both `acoustics/firmware/data/manifest.json` and `acoustics/sound_assets/manifest.json`, using matching preset IDs and file paths.
3. Preset gain values must be defined per tone to equalise perceived loudness (default 0.8 unless calibrated differently).
4. A conversion README or script instructions shall be maintained alongside the generator detailing how to extend or regenerate the preset set.
5. The maximum cumulative size of `/presets` data shall remain ≤6 MB to leave headroom for SPIFFS metadata and future assets.

## 3. Firmware Behaviour
1. Firmware shall load the preset manifest at boot and log any missing entries; boot must fail fast on manifest parse errors.
2. OSC playback commands must honour timetag execution with absolute timestamps. The firmware shall drop or warn if the scheduled time is already in the past.
3. Looping semantics (`loop` flag) must persist across playback completion only when explicitly set; otherwise the queue clears automatically.
4. Immediate “test” playback hooks (e.g., `sample_test` auto-play) must be disabled for production runs to avoid unintended sound.
5. Heartbeat payload must include: device ID, sequence, queue size, playing state, last NTP sync epoch, and real playback start timestamp (µs) for the most recent trigger.
6. On OSC `/acoustics/stop`, playback queue and active loops shall halt within 100 ms.
7. Firmware shall reject OSC commands whose preset IDs are not present in the manifest and log the occurrence.

## 4. Time Synchronisation
1. Each device shall invoke a forced NTP sync at boot and subsequently every 10 minutes, regardless of event traffic.
2. Firmware must seed the RTC from NTP on successful sync and re-seed whenever connectivity is restored after a dropout.
3. Heartbeat messages shall include the epoch (seconds) of the last successful NTP sync for monitoring drift.
4. The master PC shall operate an NTP client (chrony or ntpd) synchronised to a stable reference; setup steps must be documented in `docs/pc_workflow.md`.
5. Operational checklists shall include a chrony status command (e.g., `chronyc tracking`) to be executed before any scheduled performance.

## 5. Scheduler Requirements
1. The scheduler CLI shall accept an extended timeline schema where each event may specify a `targets` list of device IDs. If omitted, the event applies to the default target group (all devices).
2. For events with shared execution time and distinct `targets`, the scheduler shall emit separate OSC messages per target, preserving the common timetag.
3. Timeline files must declare `default_lead_time` ≥ 3 s; CLI overrides shall enforce the same minimum.
4. CLI options shall allow loading an external CSV/JSON mapping of logical voices to device IDs to support the round arrangement without rewriting the timeline.
5. Dry-run mode must display derived timetags, targets, and preset IDs so operators can verify bundle contents before transmission.
6. Scheduler must throttle bundle emission by at least 10 ms spacing to avoid AP overload when dispatching large batches.
7. Scheduler shall always enable AES-256-CTR encryption by loading the shared key/IV from `osc_config.json` (via `--osc-config`, defaulting to `acoustics/secrets/osc_config.json`), ensuring the firmware and CLI consume the same secrets without duplicating material in version control.

## 6. Control & Integration Requirements
1. Scheduler functionality shall be exposed through a reusable C++ library that can be linked by the CLI, automated scripts, and any GUI layer without duplicating logic.
2. The master PC shall host a long-running control service (e.g., WebSocket, gRPC, or named-pipe IPC) that wraps the shared library and accepts playback jobs from GUI/automation clients.
3. Control API messages shall allow both template-driven arrangements (generate voices automatically) and explicit per-device instructions covering up to 30 unique IDs per job.
4. Each job definition must capture: preset or timeline identifiers, device targets, execution window (lead/base times), and optional repetitions; the schema shall be documented with examples.
5. The control service shall maintain a dequeue/queue mechanism with job identifiers, enabling clients to query status, cancel pending jobs, or rerun a completed job without resubmitting the payload.
6. Audit logs shall capture which client (CLI, API, GUI) originated each job along with timestamp and targeted devices.

## 7. Calibration & Device Identification
1. A calibration command shall be provided on the master PC that sequentially plays a short test tone on selected devices and records success/failure in the registry.
2. Operators shall be able to trigger a “ping” round-trip check that confirms OSC reachability and heartbeat reception for each device prior to a performance.
3. Device registry workflows must support assigning and persisting human-readable aliases, mapping them to MAC addresses and firmware IDs automatically on first contact.
4. The system shall provide a utility to prompt audible identification (e.g., play a spoken ID or beep pattern) so operators can match physical devices to logical aliases during setup.
5. Calibration scripts shall produce a summary report (CSV/console) detailing connectivity, audio confirmation, and current NTP sync age for all devices inspected.
6. Documentation shall include a pre-show checklist referencing the calibration utilities and expected acceptance criteria.

## 8. Monitor & Observability
1. The monitor must persist device announces and heartbeats to `state/devices.json`, keeping alias information intact.
2. For each heartbeat, monitor shall compute:
   - Network latency (arrival time minus heartbeat timestamp).
   - Playback onset delay (arrival time minus reported playback start) when provided.
   These metrics shall be appended to CSV logs when enabled.
3. Monitor shall raise a warning (console + optional log entry) when:
   - Mean latency exceeds 50 ms over the last 10 samples.
   - No heartbeat has arrived within 3 s (warning) or 10 s (critical).
   - Last NTP sync age exceeds 15 minutes.
4. Summary reports printed at shutdown must include per-device mean latency, standard deviation, last-seen time, and last NTP sync age.
5. Monitor shall expose an option to stop automatically after capturing N heartbeats per device, aiding repeatable test runs.

## 9. Network & Deployment Constraints
1. UDP broadcast/multicast usage must comply with the AP configuration; operational procedures must include enabling broadcast on the chosen SSID and pinning to a 5 GHz channel when possible.
2. Scheduler shall detect and report socket send failures; repeated failures (>3 within 1 s) must be surfaced as a critical error.
3. For high-density tests (≥20 devices), operators must stagger firmware boots to prevent simultaneous DHCP storms.
4. Network provisioning documents shall include firewall rule templates (nftables/iptables) ensuring UDP 9000/19100 accessibility.
5. Control services shall retain persistent mappings between MAC addresses, firmware IDs, and aliases, enabling operators to address devices even before the next announce heartbeat.
6. Upon heartbeat recovery after a dropout, the master PC shall automatically verify time synchronisation (force NTP sync if stale) and provide an option to requeue any missed playback jobs for the affected device set.
7. Operational documentation shall describe the recovery workflow for network disruptions, including manual overrides and how to confirm the device has rejoined the active round.

## 10. Testing & Validation
1. A repeatable sound-check procedure shall be maintained in `acoustics/tests/osc_sync_results.md`, covering:
   - Two-device synchronisation sanity test.
   - Four-voice round rehearsal (multiple iterations).
   - 30-device endurance test (≥15 minutes) verifying drift and packet loss.
2. Scheduler and OSC transport libraries shall have unit tests validating timetag generation, target filtering, and bundle encoding.
3. Firmware integration tests shall verify queue handling and heartbeat payload structure on sample hardware before production flashing.
4. Test runs must capture monitor CSV output and archive them under `acoustics/tests/logs/` with timestamped filenames.
5. Acceptance of new presets or timeline changes requires re-running the four-voice round test and documenting results.

## 11. Operations & Maintenance
1. SPIFFS refresh procedure shall be scripted: generate WAVs → update manifest → `pio run -t uploadfs` → `pio run -t upload`.
2. Device alias management remains via the monitor/registry; alias changes must be saved immediately and reflected in test reports.
3. Any firmware or scheduler release shall include a changelog entry describing impacts on synchronization or network behaviour.
4. Backup strategy: maintain versioned copies of presets and timelines under `sound_assets/` to allow quick rollback.

## 12. Open Issues & Follow-up Actions
1. **GUI integration**: intentionally postponed; future requirements will revisit ImGui features once core coordination is stable.
2. **Playback delay reporting**: firmware modifications to include actual playback onset are pending implementation; until delivered, monitor shall mark the field as unavailable.
3. **Scale beyond 30 devices**: not a current objective, but documents should note capacity assumptions (AP throughput, OSC bundle size) to inform future scaling discussions.

## 13. Implementation Phases
- **Phase 1 – Foundations**: Finalise the audio asset pipeline (generator, manifests, SPIFFS upload), document PC-side NTP setup/checklists in `docs/pc_workflow.md`, and remove temporary firmware auto-play hooks to confirm baseline builds.
- **Phase 2 – Firmware Enhancements**: Add manifest validation, queue safeguards, `/acoustics/stop` responsiveness, heartbeat enrichment (playback onset + sync epoch), and 10-minute NTP resync cadence; validate on hardware.
- **Phase 3 – Scheduler Refactor**: Extract scheduling/bundle logic into a shared library, extend timeline schema for `targets` and mapping files, enforce ≥3 s lead time with informative dry-run output, and throttle bundle emission with proper error surfacing.
- **Phase 4 – Control Service & Calibration Toolkit**: Deploy a master-side control service (IPC/WebSocket/gRPC) around the shared library, implement job queue/audit logging, and deliver calibration utilities (tone sweep, OSC ping, audible ID) that persist alias↔MAC mappings and generate reports.
- **Phase 5 – Monitoring & Recovery**: Enhance monitor analytics (latency/onset metrics, warnings, auto-stop), document AP/firewall/recovery procedures, and automate post-dropout sync validation with job replay options.
- **Phase 6 – Validation & Hardening**: Execute the standard sync/round/endurance test suite, archive CSV logs, address regressions, log changes, and freeze the control API surface for future GUI reintegration.

---

**Critical Review Notes**
- Requirements avoid duplication by segregating concerns (assets, firmware, scheduler, monitoring, operations). Cross references are deliberate where components interact.
- Pending firmware work (heartbeat payload enrichment) is explicitly tracked in Section 12; cannot be marked complete until implemented.
- Time sync and network procedures are now testable via documentation updates; ensure `docs/pc_workflow.md` is revised to satisfy Section 4.4.
- Scheduler target mapping and API exposure require schema, CLI, and control-service updates; work items should be raised to modify `SoundTimeline`, CLI option parsing, and the new integration service accordingly.
- Monitoring warnings cover latency, heartbeat absence, and sync age, providing operators actionable feedback during rehearsals.
