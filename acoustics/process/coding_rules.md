# Coding Rules (Quick Reference)

This file mirrors the guidelines in `acoustics/docs/masterdocs.md` but is split for quick onboarding.

1. Separate responsibilities (OSC intake, rendering, UI) into dedicated classes/modules.
2. Avoid dangerous casts (`const_cast`, C-style, `reinterpret_cast`). Rethink the design instead.
3. Manage ownership with RAII (`std::unique_ptr`, `std::shared_ptr`); never leak resources.
4. Profile before optimising. Record timings with `ofGetElapsedTimeMicros()` or equivalent.
5. Read documentation/header files before relying on APIs; cite references in issues/PRs.
6. Ensure every resource has a clear shutdown path (Wi-Fi, audio buffers, files).

Update this document if the rules evolve so that onboarding engineers always have the latest reference.
