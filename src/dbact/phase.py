"""Compatibility alias for :mod:`dbact.control.phase`."""
import sys
from dbact.control import phase as _implementation

sys.modules[__name__] = _implementation
