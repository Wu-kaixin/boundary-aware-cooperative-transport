from __future__ import annotations

from pathlib import Path

IGNORE = {
    ".git",
    ".pytest_cache",
    ".venv",
    "__pycache__",
    "data",
    "logs",
    "outputs",
    "runs",
}


def tree(path: Path, prefix: str = "") -> None:
    entries = [
        p
        for p in sorted(path.iterdir(), key=lambda x: (x.is_file(), x.name))
        if p.name not in IGNORE
    ]
    for i, p in enumerate(entries):
        is_last = i == len(entries) - 1
        branch = "`-- " if is_last else "|-- "
        print(prefix + branch + p.name)
        if p.is_dir():
            tree(p, prefix + ("    " if is_last else "|   "))


if __name__ == "__main__":
    print(Path.cwd().name + "/")
    tree(Path.cwd())
