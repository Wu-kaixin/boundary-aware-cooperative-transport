"""Reconstruct deleted a priori research files from agent transcript Write/StrReplace ops."""
from __future__ import annotations

import hashlib
import json
from collections import defaultdict
from pathlib import Path

ROOT = Path(r"D:\boundary-aware-cooperative-transport-apriori")
TRANSCRIPT_DIR = Path(
    r"C:\Users\kevin\.cursor\projects\d-boundary-aware-cooperative-transport-apriori\agent-transcripts"
)
OUT = ROOT / "_recovery"
OUT.mkdir(exist_ok=True)

TARGETS = (
    "apriori_centroid_bound.py",
    "run_apriori_centroid_bound.py",
    "test_apriori_centroid_bound.py",
    "apriori_centroid_bound_derivation.md",
    "BRANCH_NOTE_feat_apriori_centroid_bound.md",
    "finalize_advisor_delivery.py",
)


def interesting(path: str) -> bool:
    p = path.replace("\\", "/")
    return any(t in p for t in TARGETS) or (
        "artifacts/apriori_centroid_bound_2026-09-15/" in p
        and p.endswith((".md", ".json"))
    )


def apply_str_replace(text: str, old: str, new: str, replace_all: bool = False) -> tuple[str, bool]:
    if old not in text:
        return text, False
    if replace_all:
        return text.replace(old, new), True
    return text.replace(old, new, 1), True


def main() -> None:
    files: dict[str, str] = {}
    ops: dict[tuple[str, str], int] = defaultdict(int)
    misses: list[str] = []

    transcripts = sorted(TRANSCRIPT_DIR.rglob("*.jsonl"), key=lambda p: p.stat().st_mtime)
    print(f"transcripts={len(transcripts)}")
    for tp in transcripts:
        print(f"  {tp.parent.name} size={tp.stat().st_size}")

    for tp in transcripts:
        for line in tp.open(encoding="utf-8"):
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            content = (obj.get("message") or {}).get("content")
            if not isinstance(content, list):
                continue
            for block in content:
                if not isinstance(block, dict) or block.get("type") != "tool_use":
                    continue
                name = block.get("name")
                inp = block.get("input") or {}
                path = str(inp.get("path") or "")
                if not interesting(path):
                    continue
                ops[(str(name), Path(path).name)] += 1
                if name == "Write" and "contents" in inp:
                    files[path] = str(inp["contents"])
                elif name == "StrReplace" and path in files:
                    old = inp.get("old_string")
                    new = inp.get("new_string")
                    if old is None or new is None:
                        continue
                    files[path], ok = apply_str_replace(
                        files[path], str(old), str(new), bool(inp.get("replace_all"))
                    )
                    if not ok:
                        misses.append(f"{Path(path).name}: {repr(str(old)[:80])}")

    print("--- ops ---")
    for k, v in sorted(ops.items()):
        print(k, v)
    print(f"misses={len(misses)}")
    for m in misses[:20]:
        print("  MISS", m)

    manifest: dict[str, dict] = {}
    for path, content in files.items():
        safe = path.replace(":", "").replace("\\", "/").replace("/", "__")
        dest = OUT / safe
        dest.write_text(content, encoding="utf-8")
        digest = hashlib.sha256(content.encode("utf-8")).hexdigest()
        manifest[path] = {
            "chars": len(content),
            "lines": content.count("\n") + 1,
            "sha256": digest,
            "recovery_file": str(dest),
        }
        print(f"recovered {path} lines={manifest[path]['lines']} sha={digest[:12]}")

    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"wrote {OUT / 'manifest.json'}")


if __name__ == "__main__":
    main()
