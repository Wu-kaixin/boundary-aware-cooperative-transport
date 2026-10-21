"""Compatibility alias for :mod:`dbact.deployment.theorem`."""
import sys
from dbact.deployment import theorem as _implementation

sys.modules[__name__] = _implementation
