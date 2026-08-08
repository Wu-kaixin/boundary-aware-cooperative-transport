"""DBACT: Decentralized Boundary-Aware Enclosure and Cooperative Transport."""

from .cargo import Cargo
from .contracts import (
    ContactSafetyContract,
    ContractViolation,
    CoverageContract,
    DirectionalProgressContract,
    SolverContract,
    SuccessVerdict,
)
from .controller import DBACTController, DBACTParams
from .types import AgentState, BoundaryObservation, ControlCommand

__version__ = "0.2.0"

__all__ = [
    "AgentState",
    "BoundaryObservation",
    "ControlCommand",
    "Cargo",
    "DBACTController",
    "DBACTParams",
    "ContractViolation",
    "ContactSafetyContract",
    "SolverContract",
    "DirectionalProgressContract",
    "SuccessVerdict",
    "CoverageContract",
]
