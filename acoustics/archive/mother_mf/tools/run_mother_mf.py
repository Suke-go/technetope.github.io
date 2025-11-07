#!/usr/bin/env python3
"""Fire the MOTHER Earth MF timeline through the scheduler with sane defaults."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Sequence

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_TIMELINE = (
    REPO_ROOT / "acoustics" / "pc_tools" / "scheduler" / "examples" / "mother_mf_a_section.json"
)
DEFAULT_TARGETS = REPO_ROOT / "acoustics" / "tests" / "test01" / "targets_mother.json"
DEFAULT_BIN = REPO_ROOT / "build" / "scheduler" / "agent_a_scheduler"
DEFAULT_OSC_CONFIG: Sequence[Path] = (
    REPO_ROOT / "acoustics" / "secrets" / "osc_config.json",
    REPO_ROOT / "acoustics" / "secrets" / "osc_config.example.json",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send the MOTHER Earth Missing Fundamental timeline to the devices."
    )
    parser.add_argument("--timeline", type=Path, default=DEFAULT_TIMELINE)
    parser.add_argument("--target-map", type=Path, default=DEFAULT_TARGETS)
    parser.add_argument("--scheduler-bin", type=Path, default=DEFAULT_BIN)
    parser.add_argument("--host", default="255.255.255.255")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--lead", type=float, default=4.0, help="Lead time (seconds)")
    parser.add_argument("--spacing", type=float, default=0.05, help="Bundle spacing (seconds)")
    parser.add_argument("--dry-run", action="store_true", default=False)
    parser.add_argument(
        "--osc-config",
        type=Path,
        help="Path to osc_config.json (defaults to acoustics/secrets/osc_config.json or the example).",
    )
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("extra", nargs=argparse.REMAINDER, help="Additional args passed verbatim to the scheduler.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not args.scheduler_bin.exists():
        print(f"Scheduler binary not found: {args.scheduler_bin}", file=sys.stderr)
        print("Build it first (e.g. cmake --build build/scheduler)", file=sys.stderr)
        return 1
    if not args.timeline.exists():
        print(f"Timeline JSON not found: {args.timeline}", file=sys.stderr)
        return 1
    if not args.target_map.exists():
        print(f"Target map not found: {args.target_map}", file=sys.stderr)
        return 1

    osc_config = args.osc_config
    if osc_config is None:
        for candidate in DEFAULT_OSC_CONFIG:
            if candidate.exists():
                osc_config = candidate
                break
    if osc_config is None or not osc_config.exists():
        print(
            "OSC config not found. Pass --osc-config pointing to acoustics/secrets/osc_config.json",
            file=sys.stderr,
        )
        return 1

    cmd = [
        str(args.scheduler_bin),
        "--target-map",
        str(args.target_map),
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--lead-time",
        f"{args.lead}",
        "--bundle-spacing",
        f"{args.spacing}",
    ]
    if args.dry_run:
        cmd.append("--dry-run")
    cmd.extend(["--osc-config", str(osc_config)])
    cmd.extend(args.extra)
    cmd.append(str(args.timeline))

    if args.verbose:
        print("Executing:", " ".join(cmd))

    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
