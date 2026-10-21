"""Compatibility alias for :mod:`dbact.sensing.boundary_map`."""
import sys
from dbact.sensing import boundary_map as _implementation

sys.modules[__name__] = _implementation
