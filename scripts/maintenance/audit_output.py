"""Publish a fresh local verification report without changing historical receipts."""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def write_report(output: Path | None, report: dict) -> None:
    if output is None:
        return
    output = output.resolve()
    if not output.is_relative_to(ROOT) or output.is_relative_to(ROOT / "docs/migrations"):
        raise ValueError("Use a new local output inside this repository, outside frozen docs/migrations")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
