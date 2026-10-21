"""Resolve cases before creating outputs or processes."""
from __future__ import annotations

import copy
import re
from pathlib import Path

import yaml

from dbact.deployment.variants import METHODS
from dbact_sim.experiments.deployment import config_for


def load_protocol(path: Path, split: str, root: Path) -> tuple[dict, list[dict]]:
    protocol = yaml.safe_load(path.read_text(encoding="utf-8-sig"))
    if not isinstance(protocol, dict) or not isinstance(protocol.get("cases"), list):
        raise ValueError("Protocol requires a cases list")
    cases, identifiers = [], set()
    for raw in protocol["cases"]:
        if not isinstance(raw, dict):
            raise ValueError("Each case must be a mapping")
        identifier = raw.get("case_id", "")
        if not isinstance(identifier, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", identifier):
            raise ValueError(f"Invalid case_id: {identifier!r}; use a single portable directory name")
        reserved = {"CON", "PRN", "AUX", "NUL", *(f"COM{i}" for i in range(1, 10)), *(f"LPT{i}" for i in range(1, 10))}
        if identifier.upper() in reserved:
            raise ValueError(f"Invalid case_id: {identifier!r}; reserved Windows device name")
        if identifier in identifiers:
            raise ValueError(f"Duplicate case_id: {identifier}")
        identifiers.add(identifier)
        if split != "all" and raw.get("split") != split:
            continue
        case = copy.deepcopy(raw)
        for name in ("seed", "frames", "gate"):
            if type(case.get(name)) is not int:
                raise ValueError(f"{identifier}: {name} must be an integer")
        if case["gate"] not in (5, 6) or case["frames"] <= 0 or case["seed"] < 0:
            raise ValueError(f"{identifier}: gate must be 5 or 6, frames > 0, seed >= 0")
        variants = ("oracle", "local") if case["gate"] == 5 else METHODS
        if case.get("variant") not in variants:
            raise ValueError(f"{identifier}: unsupported variant {case.get('variant')!r}")
        if case.get("shape") not in ("l_shape", "rectangle", "c_shape"):
            raise ValueError(f"{identifier}: unsupported shape")
        if "config" in case and "config_ref" in case:
            raise ValueError(f"{identifier}: choose config or config_ref")
        if "config_ref" in case:
            reference = (root / case.pop("config_ref")).resolve()
            if not reference.is_relative_to(root.resolve()):
                raise ValueError(f"{identifier}: config_ref must stay inside the repository")
            case["config"] = yaml.safe_load(reference.read_text(encoding="utf-8-sig"))
        elif "config" not in case:
            case["config"] = config_for(case["gate"], case["shape"])
        if not isinstance(case["config"], dict):
            raise ValueError(f"{identifier}: config must be a mapping")
        cases.append(case)
    if not cases:
        raise ValueError(f"Empty split: {split}")
    return protocol, cases
