#!/usr/bin/env python3
"""Print OSC AES key/IV from osc_config.json (or legacy Secrets headers)."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Iterable, Optional, Tuple


DEFAULT_SEARCH_PATHS: Iterable[Path] = (
    Path("acoustics/secrets/osc_config.json"),
    Path("acoustics/secrets/osc_config.example.json"),
    Path("acoustics/firmware/include/Secrets.h"),
    Path("acoustics/firmware/include/Secrets.myhome.h"),
    Path("acoustics/firmware/include/Secrets.example.h"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "secrets",
        nargs="*",
        type=Path,
        help="Optional explicit secrets paths to inspect (defaults to common search list).",
    )
    return parser.parse_args()


def sanitize_hex(value: str, expected_bytes: int) -> Optional[str]:
    digits = "".join(ch.lower() for ch in value if ch.lower() in "0123456789abcdef")
    if len(digits) != expected_bytes * 2:
        return None
    return digits


def extract_hex(lines: str, symbol: str, expected_bytes: int) -> Optional[str]:
    pattern = rf"{symbol}\s*=\s*\{{([^}}]+)\}}"
    match = re.search(pattern, lines, flags=re.MULTILINE | re.DOTALL)
    if not match:
        return None
    raw = match.group(1)
    values = re.findall(r"0x([0-9a-fA-F]{2})", raw)
    if len(values) != expected_bytes:
        return None
    return "".join(v.lower() for v in values)


def read_json_key_iv(path: Path) -> Optional[Tuple[str, str]]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        print(f"{path}: failed to read ({exc})", file=sys.stderr)
        return None
    except json.JSONDecodeError as exc:
        print(f"{path}: invalid json ({exc})", file=sys.stderr)
        return None

    osc = data.get("osc", {})
    key_hex = osc.get("key_hex")
    iv_hex = osc.get("iv_hex")
    if not isinstance(key_hex, str) or not isinstance(iv_hex, str):
        return None
    key_norm = sanitize_hex(key_hex, 32)
    iv_norm = sanitize_hex(iv_hex, 16)
    if key_norm and iv_norm:
        return key_norm, iv_norm
    return None


def read_header_key_iv(path: Path) -> Optional[Tuple[str, str]]:
    try:
        contents = path.read_text(encoding="utf-8")
    except OSError as exc:
        print(f"{path}: failed to read ({exc})", file=sys.stderr)
        return None

    key = extract_hex(contents, r"OSC_AES_KEY", 32)
    iv = extract_hex(contents, r"OSC_AES_IV", 16)
    if key and iv:
        return key, iv
    return None


def main() -> int:
    args = parse_args()
    search_paths = args.secrets or list(DEFAULT_SEARCH_PATHS)

    found = False
    for candidate in search_paths:
        candidate = candidate.resolve()
        if not candidate.exists():
            continue
        if candidate.suffix == ".json":
            result = read_json_key_iv(candidate)
        else:
            result = read_header_key_iv(candidate)
        if result:
            key, iv = result
            print(candidate)
            print(f"  OSC_AES_KEY: {key}")
            print(f"  OSC_AES_IV : {iv}")
            found = True
    if not found:
        print("No Secrets file with OSC_AES_KEY/OSC_AES_IV found.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
