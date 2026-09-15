#!/usr/bin/env python3
"""Fail when a public Python API name is absent from project documentation."""

from __future__ import annotations

import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src" / "pib_sdk"
REFERENCE = ROOT / "docs" / "REFERENCE.md"
DEFINITION_NODES = (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)


def public_api() -> list[tuple[str, str, str]]:
    items: list[tuple[str, str, str]] = []
    for path in sorted(SOURCE.rglob("*.py")):
        if path.name == "__init__.py":
            continue
        module = ".".join(path.with_suffix("").relative_to(ROOT / "src").parts)
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in tree.body:
            if not isinstance(node, DEFINITION_NODES) or node.name.startswith("_"):
                continue
            qualified = f"{module}.{node.name}"
            items.append((qualified, module, node.name))
            if isinstance(node, ast.ClassDef):
                for member in node.body:
                    if isinstance(member, DEFINITION_NODES) and not member.name.startswith("_"):
                        items.append((f"{qualified}.{member.name}", module, member.name))
    return items


def reference_sections() -> dict[str, str]:
    text = REFERENCE.read_text(encoding="utf-8")
    headings = list(re.finditer(r"^## `([^`]+)`\s*$", text, flags=re.MULTILINE))
    return {
        match.group(1): text[match.end() : headings[index + 1].start()]
        if index + 1 < len(headings)
        else text[match.end() :]
        for index, match in enumerate(headings)
    }


def main() -> int:
    api = public_api()
    sections = reference_sections()
    missing = [
        qualified
        for qualified, module, name in api
        if module not in sections
        or not re.search(
            rf"(?<![A-Za-z0-9_]){re.escape(name)}(?![A-Za-z0-9_])",
            sections[module],
        )
    ]
    print(f"Documentation coverage: {len(api) - len(missing)}/{len(api)} public API elements")
    if missing:
        print(f"Undocumented: {len(missing)}")
        for qualified in missing:
            print(f"  {qualified}")
        return 1
    print("Undocumented: 0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
