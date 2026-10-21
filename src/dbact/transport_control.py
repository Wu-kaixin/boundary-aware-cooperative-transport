"""Compatibility alias for :mod:`dbact.control.transport`."""
import sys
from dbact.control import transport as _implementation

sys.modules[__name__] = _implementation
