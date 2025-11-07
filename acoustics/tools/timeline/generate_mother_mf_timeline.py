#!/usr/bin/env python3
"""
Generate a SoundTimeline JSON for the MOTHER Earth (Missing Fundamental ver.)
playback sequence.  The resulting timeline can be consumed by the scheduler
tool under `acoustics/pc_tools/scheduler`.

Tempo を落としてゆっくり鳴らしたい場合は、CLI オプション `--tempo-scale`
（デフォルト 1.0）を指定すると `offset_ms` が一括で拡大・縮小される。例:
`--tempo-scale 4.2` で 240ms → 約 1000ms になり、1 音 / 秒ペースのタイムラインを
コード変更無しで生成できる。
DRY_RUN=0 HOST=172.20.10.15 \
SCHED_BIN=$PWD/build/acoustics/scheduler/agent_a_scheduler \
./acoustics/archive/mother_mf/tools/run_mother_mf.sh

The script encodes the A-section instructions shared by the music team:

    - Tempo: 125 BPM (quarter note = 480 ms, eight note = 240 ms)
    - Right hand melody: stop the active note before triggering the next one
    - Left hand bass: sustain pedal behaviour (no stop between notes)

By default the script emits two A-section iterations and writes the timeline
JSON to stdout.  Use `--output` to save directly to a file.
python3 acoustics/tools/timeline/generate_mother_mf_timeline.py \
  --loops 1 \
  --melody-gain 1.0 \
  --bass-gain 1.0 \
  --output acoustics/pc_tools/scheduler/examples/mother_mf_a_section.json
"""

from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path
from typing import Iterable, List, Sequence


@dataclasses.dataclass(frozen=True)
class TimelineStep:
    """Represents one musical moment within the A-section sequence."""

    offset_ms: int
    melody_preset: str
    bass_preset: str


# fmt: off
A_SECTION_STEPS: Sequence[TimelineStep] = [
    TimelineStep(0,    "A4_mf",      "A2_mf"),
    TimelineStep(0,    "A4_mf",      "A2_mf"),
    TimelineStep(240,  "Csharp5_mf", "Csharp3_mf"),
    TimelineStep(480,  "E5_mf",      "E3_mf"),
    TimelineStep(720,  "Fsharp5_mf", "Csharp3_mf"),
    TimelineStep(960,  "D5_mf",      "A2_mf"),
    TimelineStep(1200, "E5_mf",      "Csharp3_mf"),
    TimelineStep(1440, "Csharp5_mf", "E3_mf"),
    TimelineStep(1920, "A4_mf",      "A2_mf"),
    TimelineStep(2160, "Csharp5_mf", "Csharp3_mf"),
    TimelineStep(2400, "E5_mf",      "E3_mf"),
    TimelineStep(2640, "Fsharp5_mf", "Csharp3_mf"),
    TimelineStep(2880, "D5_mf",      "A2_mf"),
    TimelineStep(3120, "B4_mf",      "D3_mf"),
    TimelineStep(3360, "G4_mf",      "B2_mf"),
    TimelineStep(3840, "A4_mf",      "A2_mf"),
]
# fmt: on


def _format_offset(offset_ms: int) -> float:
    """Return the offset in seconds suitable for the timeline JSON."""
    return round(offset_ms / 1000.0, 6)


def _scaled_steps(scale: float) -> Sequence[TimelineStep]:
    """Return new steps list scaled by tempo."""
    if scale <= 0.0:
        raise ValueError("tempo-scale must be > 0")
    scaled: List[TimelineStep] = []
    for step in A_SECTION_STEPS:
        scaled.append(
            TimelineStep(
                offset_ms=int(round(step.offset_ms * scale)),
                melody_preset=step.melody_preset,
                bass_preset=step.bass_preset,
            )
        )
    return scaled


def _cycle_duration_ms(steps: Sequence[TimelineStep]) -> int:
    """Duration of one full A-section iteration in milliseconds."""
    return steps[-1].offset_ms


def build_events(
    loops: int,
    melody_gain: float,
    bass_gain: float,
    final_stop_delay_ms: int,
    steps: Sequence[TimelineStep],
) -> List[dict]:
    """Construct the ordered event list for the timeline JSON."""
    if loops < 1:
        raise ValueError("loop count must be >= 1")

    cycle_ms = _cycle_duration_ms(steps)
    events: List[dict] = []

    for loop_index in range(loops):
        for step_index, step in enumerate(steps):
            # The first step of subsequent loops is already covered by the
            # last step of the previous loop (which retriggers the A4 chord).
            if loop_index > 0 and step_index == 0:
                continue

            offset_ms = loop_index * cycle_ms + step.offset_ms
            offset_sec = _format_offset(offset_ms)

            # Stop the previous melody note before triggering a new one.
            if loop_index > 0 or step_index > 0:
                events.append(
                    {
                        "offset": offset_sec,
                        "address": "/acoustics/stop",
                        "targets": ["mother_melody"],
                        "args": [],
                    }
                )

            events.append(
                {
                    "offset": offset_sec,
                    "address": "/acoustics/play",
                    "targets": ["mother_melody"],
                    "args": [step.melody_preset, 0, melody_gain, 0],
                }
            )
            events.append(
                {
                    "offset": offset_sec,
                    "address": "/acoustics/play",
                    "targets": ["mother_bass"],
                    "args": [step.bass_preset, 0, bass_gain, 0],
                }
            )

    if final_stop_delay_ms > 0:
        final_offset_ms = loops * cycle_ms + final_stop_delay_ms
        final_offset_sec = _format_offset(final_offset_ms)
        # Stop both hands to guarantee silence after the sequence.
        for target in ("mother_melody", "mother_bass"):
            events.append(
                {
                    "offset": final_offset_sec,
                    "address": "/acoustics/stop",
                    "targets": [target],
                    "args": [],
                }
            )

    return events


def build_timeline(
    loops: int,
    melody_gain: float,
    bass_gain: float,
    final_stop_delay_ms: int,
    tempo_scale: float,
) -> dict:
    """Return the full timeline document."""
    steps = _scaled_steps(tempo_scale)
    events = build_events(
        loops=loops,
        melody_gain=melody_gain,
        bass_gain=bass_gain,
        final_stop_delay_ms=final_stop_delay_ms,
        steps=steps,
    )
    return {
        "version": "1.2",
        "default_lead_time": 4.0,
        "metadata": {
            "title": "MOTHER Earth (Missing Fundamental ver.) — A Section",
            "loops": loops,
            "tempo_bpm": 125,
            "description": (
                "Autogenerated by generate_mother_mf_timeline.py. "
                "Targets 'mother_melody' and 'mother_bass' must be "
                "mapped to the respective M5Stick devices."
            ),
        },
        "events": events,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate timeline JSON for the MOTHER Earth (MF) sequence."
    )
    parser.add_argument(
        "--loops",
        type=int,
        default=2,
        help="Number of A-section iterations to encode (default: 2).",
    )
    parser.add_argument(
        "--melody-gain",
        type=float,
        default=1.0,
        help="Gain multiplier for melody presets (default: 1.0).",
    )
    parser.add_argument(
        "--bass-gain",
        type=float,
        default=1.0,
        help="Gain multiplier for bass presets (default: 1.0).",
    )
    parser.add_argument(
        "--final-stop-delay",
        type=int,
        default=960,
        help=(
            "Extra delay in milliseconds before issuing final stop commands "
            "(default: 960 ms, i.e., two beats)."
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional path to write the resulting JSON. Stdout if omitted.",
    )
    parser.add_argument(
        "--tempo-scale",
        type=float,
        default=1.0,
        help="Multiply all offsets by this factor (e.g., 4.0 makes ~1s per note).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    timeline = build_timeline(
        loops=args.loops,
        melody_gain=args.melody_gain,
        bass_gain=args.bass_gain,
        final_stop_delay_ms=args.final_stop_delay,
        tempo_scale=args.tempo_scale,
    )
    json_payload = json.dumps(timeline, indent=2)
    if args.output:
        args.output.write_text(json_payload + "\n", encoding="utf-8")
    else:
        print(json_payload)


if __name__ == "__main__":
    main()
