"""Compatibility alias for :mod:`dbact.sensing.sensor`."""
import sys
from dbact.sensing import sensor as _implementation

sys.modules[__name__] = _implementation
