"""Compatibility alias for :mod:`dbact.safety.qp2d`."""
import sys
from dbact.safety import qp2d as _implementation

sys.modules[__name__] = _implementation
