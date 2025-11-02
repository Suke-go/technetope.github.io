# OSC Contract (Draft)

Use this document to define the OSC message schema shared between the PC sequencer and the M5StickC Plus2 firmware.

Sections to complete:
1. **Address space** – e.g., `/play`, `/stop`, `/heartbeat`.
2. **Argument list** – type, units, valid range, default value.
3. **Timetag usage** – lead time, allowable jitter, timeout behaviour.
4. **Security** – encryption mode, key rotation schedule, replay protection.
5. **Error handling** – required acknowledgements, retry policies.

Keep the table synchronised with `acoustics/tests/osc_sync_results.md` so measurements always reference the current spec.
