#!/usr/bin/env python3
"""Validate local Markdown links and heading anchors in README/docs."""

from __future__ import annotations

import re
from collections import Counter
from pathlib import Path
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
MARKDOWN = [ROOT / "README.md", *sorted((ROOT / "docs").glob("*.md"))]
LINK = re.compile(r"(?<!!)\[[^\]]+\]\(([^)\s]+)(?:\s+['\"][^)]*['\"])?\)")
HEADING = re.compile(r"^#{1,6}\s+(.+?)\s*#*\s*$", re.MULTILINE)


def github_slug(text: str) -> str:
    text = re.sub(r"<[^>]+>", "", text)
    text = text.replace("`", "").strip().lower()
    text = re.sub(r"[^\w\-\s]", "", text, flags=re.UNICODE)
    return re.sub(r"\s", "-", text)


def anchors(path: Path) -> set[str]:
    seen: Counter[str] = Counter()
    result: set[str] = set()
    for heading in HEADING.findall(path.read_text(encoding="utf-8")):
        base = github_slug(heading)
        count = seen[base]
        seen[base] += 1
        result.add(base if count == 0 else f"{base}-{count}")
    return result


def main() -> int:
    failures: list[str] = []
    checked = 0
    anchor_cache: dict[Path, set[str]] = {}
    for source in MARKDOWN:
        for raw_target in LINK.findall(source.read_text(encoding="utf-8")):
            if re.match(r"^[a-z][a-z0-9+.-]*:", raw_target) or raw_target.startswith("//"):
                continue
            checked += 1
            path_text, separator, fragment = raw_target.partition("#")
            target = (source.parent / unquote(path_text)).resolve() if path_text else source
            try:
                target.relative_to(ROOT)
            except ValueError:
                failures.append(f"{source.relative_to(ROOT)}: link escapes repository: {raw_target}")
                continue
            if not target.exists():
                failures.append(f"{source.relative_to(ROOT)}: missing target: {raw_target}")
                continue
            if separator:
                if target.suffix.lower() != ".md":
                    failures.append(
                        f"{source.relative_to(ROOT)}: anchor on non-Markdown target: {raw_target}"
                    )
                    continue
                available = anchor_cache.setdefault(target, anchors(target))
                if unquote(fragment).lower() not in available:
                    failures.append(f"{source.relative_to(ROOT)}: missing anchor: {raw_target}")
    print(f"Internal documentation links checked: {checked}")
    if failures:
        print(f"Broken links/anchors: {len(failures)}")
        for failure in failures:
            print(f"  {failure}")
        return 1
    print("Broken links/anchors: 0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
