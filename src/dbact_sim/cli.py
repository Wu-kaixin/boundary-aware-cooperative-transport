"""Single command entry point; load only the selected workflow."""
from __future__ import annotations

import argparse
import importlib
import sys

COMMANDS = {
    "simulate": ("dbact_sim.run_sim", "Run a simulation and optionally render it"),
    "deployment": ("dbact_sim.experiments.deployment", "Run matched static or transport pairs"),
    "study": ("dbact_sim.experiments.study", "Validate or execute an experiment protocol"),
    "evidence": ("dbact_sim.experiments.evidence", "Build tables and figures from recorded results"),
}


def main(argv=None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(prog="dbact", description="DBACT simulation and CCTA 2027 reproduction")
    parser.add_argument("command", choices=COMMANDS, help="; ".join(f"{k}: {v[1]}" for k, v in COMMANDS.items()))
    if not args or args[0] in ("-h", "--help"):
        parser.print_help()
        return 0
    parsed = parser.parse_args(args[:1])
    module = importlib.import_module(COMMANDS[parsed.command][0])
    return module.main(args[1:]) or 0
