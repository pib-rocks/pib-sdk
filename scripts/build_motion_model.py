#!/usr/bin/env python3
"""Build or verify src/pib_sdk/data/motion_model.json from the bundled URDF."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from pib_sdk.motion_model import (  # noqa: E402
    MOTION_MODEL_PATH,
    build_motion_model,
    serialize_motion_model,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="exit 1 if the output file does not match a fresh build",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=MOTION_MODEL_PATH,
        help=f"path to write or verify (default: {MOTION_MODEL_PATH})",
    )
    args = parser.parse_args()
    output_path = args.output.resolve()
    built = serialize_motion_model(build_motion_model())

    if args.check:
        if not output_path.is_file():
            print(f"motion_model drift: missing {output_path}", file=sys.stderr)
            return 1
        on_disk = output_path.read_text(encoding="utf-8")
        if on_disk == built:
            return 0
        print(f"motion_model drift: {output_path} differs from generator output", file=sys.stderr)
        if len(on_disk) != len(built):
            print(f"  byte length: on_disk={len(on_disk)} built={len(built)}", file=sys.stderr)
        return 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(built, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
