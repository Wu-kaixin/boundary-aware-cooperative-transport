"""Backward-compatible CBF module.

Paper-grade implementation lives in ``distributed_cbf.DistributedCBFQP``
(pairwise responsibility splitting + object-boundary CBF).
"""

from __future__ import annotations

from .distributed_cbf import DistributedCBFQP, LocalCBFQP

__all__ = ["DistributedCBFQP", "LocalCBFQP"]
