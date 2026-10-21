"""Compatibility alias for :mod:`dbact.safety.filter`."""
import sys
from dbact.safety import filter as _implementation

sys.modules[__name__] = _implementation
