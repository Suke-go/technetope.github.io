#!/usr/bin/env python3
"""
Utility to generate “missing fundamental” tone presets for the M5StickC Plus2.

- Sample rate defaults to 16 kHz mono so the resulting WAVs fit comfortably in SPIFFS.
- The fundamental itself is omitted; instead we add upper partials (2*f0, 3*f0, …)
  to recreate the pitch perceptually while keeping energy compact.
- Output files are normalised to 16-bit PCM and lightly windowed to avoid clicks.

Example:
    python generate_missing_f0.py \\
        --output-dir ../../sound_assets/f0 \\
        tone_do=392 tone_re=440 tone_mi=494 tone_fa=523.25
"""

from __future__ import annotations

import argparse
import math
import wave
from array import array
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = SCRIPT_DIR.parent.parent / "sound_assets" / "f0"


@dataclass(frozen=True)
class ToneSpec:
    name: str
    frequency_hz: float


def parse_tone_specs(raw: Iterable[str]) -> List[ToneSpec]:
    """Parse CLI tone assignments (id=freq)."""
    specs: List[ToneSpec] = []
    for entry in raw:
        if "=" not in entry:
            raise ValueError(f"Tone spec must look like id=freq_hz (got: {entry})")
        name, raw_freq = entry.split("=", maxsplit=1)
        name = name.strip()
        if not name:
            raise ValueError(f"Tone name cannot be empty (input: {entry})")
        try:
            freq = float(raw_freq)
        except ValueError as exc:
            raise ValueError(f"Tone frequency must be numeric (input: {entry})") from exc
        if freq <= 0.0:
            raise ValueError(f"Tone frequency must be positive (input: {entry})")
        specs.append(ToneSpec(name=name, frequency_hz=freq))
    return specs


def hann_envelope(length: int, fade_samples: int) -> List[float]:
    """Create a simple fade-in/out envelope using half Hann windows."""
    if fade_samples <= 0 or fade_samples * 2 >= length:
        return [1.0] * length

    env = [1.0] * length
    for i in range(fade_samples):
        coeff = 0.5 * (1.0 - math.cos(math.pi * (i + 1) / (fade_samples + 1)))
        env[i] = coeff
        env[length - 1 - i] = coeff
    return env


def synth_missing_f0(
    spec: ToneSpec,
    sample_rate: int,
    duration_sec: float,
    harmonics: int,
    amplitude: float,
    fade_ms: float,
) -> List[int]:
    """
    Generate PCM samples for a missing-fundamental tone.

    Args:
        spec: tone definition (id + target perceptual f0).
        sample_rate: samples per second, e.g. 16000.
        duration_sec: tone length (seconds).
        harmonics: number of partials (starting at 2*f0).
        amplitude: target peak (0.0 - 1.0) after normalisation.
        fade_ms: fade-in/out length (milliseconds) to suppress clicks.
    """
    total_samples = int(round(duration_sec * sample_rate))
    if total_samples <= 0:
        raise ValueError("duration_sec yields zero samples")
    if harmonics < 1:
        raise ValueError("harmonics must be >= 1")

    fade_samples = int(round(fade_ms / 1000.0 * sample_rate))
    envelope = hann_envelope(total_samples, fade_samples)

    raw: List[float] = []
    for n in range(total_samples):
        t = n / sample_rate
        acc = 0.0
        for h in range(2, 2 + harmonics):
            partial_freq = spec.frequency_hz * h
            # simple inverse weighting keeps upper partials under control
            acc += (1.0 / h) * math.sin(2.0 * math.pi * partial_freq * t)
        raw.append(acc * envelope[n])

    peak = max(abs(v) for v in raw) if raw else 1.0
    normalise = amplitude / peak if peak > 0.0 else 0.0

    pcm = [int(max(-32768, min(32767, round(v * normalise * 32767.0)))) for v in raw]
    return pcm


def write_wav(path: Path, samples: List[int], sample_rate: int) -> None:
    """Persist 16-bit mono PCM samples."""
    data = array("h", samples)
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(sample_rate)
        wf.writeframes(data.tobytes())


def generate_tones(
    tones: Iterable[ToneSpec],
    output_dir: Path,
    sample_rate: int,
    duration_sec: float,
    harmonics: int,
    amplitude: float,
    fade_ms: float,
) -> Dict[str, Path]:
    """Create WAV files for all requested tones."""
    output_dir.mkdir(parents=True, exist_ok=True)
    written: Dict[str, Path] = {}
    for spec in tones:
        pcm = synth_missing_f0(
            spec=spec,
            sample_rate=sample_rate,
            duration_sec=duration_sec,
            harmonics=harmonics,
            amplitude=amplitude,
            fade_ms=fade_ms,
        )
        output_path = output_dir / f"{spec.name}.wav"
        write_wav(output_path, pcm, sample_rate)
        written[spec.name] = output_path
    return written


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate missing-fundamental tone WAV files for firmware presets."
    )
    parser.add_argument(
        "tones",
        nargs="*",
        default=(),
        help="Tone definition as id=freq_hz (e.g., tone_do=392). "
        "If omitted, a default C-major tetrachord is generated.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Destination directory for WAV files (default: {DEFAULT_OUTPUT_DIR}).",
    )
    parser.add_argument(
        "--sample-rate",
        type=int,
        default=16_000,
        help="Sample rate (Hz), default 16000.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="Tone duration in seconds (default 1.0).",
    )
    parser.add_argument(
        "--harmonics",
        type=int,
        default=6,
        help="Number of partials to sum (starting at 2*f0), default 6.",
    )
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.8,
        help="Peak amplitude scaling after normalisation (0.0-1.0).",
    )
    parser.add_argument(
        "--fade-ms",
        type=float,
        default=8.0,
        help="Fade-in/out length in milliseconds to suppress clicks (default 8).",
    )
    return parser


def default_tones() -> List[ToneSpec]:
    """Fallback tone set resembling a C-major tetrachord."""
    return [
        ToneSpec("tone_do", 392.0),    # perceive ~196 Hz fundamental
        ToneSpec("tone_re", 440.0),
        ToneSpec("tone_mi", 494.0),
        ToneSpec("tone_fa", 523.25),
    ]


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    try:
        tone_specs = (
            parse_tone_specs(args.tones) if args.tones else default_tones()
        )
    except ValueError as exc:
        parser.error(str(exc))
        return

    output_dir = args.output_dir.expanduser()

    written = generate_tones(
        tones=tone_specs,
        output_dir=output_dir,
        sample_rate=args.sample_rate,
        duration_sec=args.duration,
        harmonics=args.harmonics,
        amplitude=args.amplitude,
        fade_ms=args.fade_ms,
    )

    for name, path in written.items():
        print(f"{name}: {path}")


if __name__ == "__main__":
    main()
