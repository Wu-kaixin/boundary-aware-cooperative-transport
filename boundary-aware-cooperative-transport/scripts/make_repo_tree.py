from __future__ import annotations

from pathlib import Path

IGNORE = {".git", ".venv", "__pycache__", ".pytest_cache", "runs"}


def tree(path: Path, prefix: str = "") -> None:
    entries = [p for p in sorted(path.iterdir(), key=lambda x: (x.is_file(), x.name)) if p.name not in IGNORE]
    for i, p in enumerate(entries):
        branch = "└── " if i == len(entries) - 1 else "├── "
        print(prefix + branch + p.name)
        if p.is_dir():
            tree(p, prefix + ("    " if i == len(entries) - 1 else "│   "))


if __name__ == "__main__":
    print(Path.cwd().name + "/")
    tree(Path.cwd())
