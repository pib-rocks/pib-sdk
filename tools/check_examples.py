#!/usr/bin/env python3
"""Compile every example script without importing it or creating pyc files."""

from __future__ import annotations

import py_compile
import re
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    examples = sorted((ROOT / "examples").rglob("*.py"))
    failures: list[str] = []
    with tempfile.TemporaryDirectory() as directory:
        compiled = Path(directory)
        for index, path in enumerate(examples):
            try:
                py_compile.compile(
                    str(path),
                    cfile=str(compiled / f"{index}.pyc"),
                    doraise=True,
                )
            except py_compile.PyCompileError as error:
                failures.append(f"{path.relative_to(ROOT)}: {error.msg}")
    index = (ROOT / "docs" / "EXAMPLES.md").read_text(encoding="utf-8")
    indexed = {
        (ROOT / "docs" / target).resolve()
        for target in re.findall(r"\]\((\.\./examples/[^)#?]+\.py)\)", index)
    }
    unlisted = [path for path in examples if path.resolve() not in indexed]
    stale = [path for path in indexed if path not in {item.resolve() for item in examples}]
    failures.extend(f"{path.relative_to(ROOT)}: not listed in docs/EXAMPLES.md" for path in unlisted)
    failures.extend(
        f"{path.relative_to(ROOT)}: indexed but not compiled" for path in stale if path.exists()
    )
    print(f"Example scripts compiled: {len(examples)}")
    if failures:
        print(f"Syntax failures: {len(failures)}")
        for failure in failures:
            print(f"  {failure}")
        return 1
    print("Syntax failures: 0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
