"""Compatibility alias for :mod:`dbact.control.controller`."""
import sys
from dbact.control import controller as _implementation

sys.modules[__name__] = _implementation
