"""Compatibility alias for :mod:`dbact.sensing.features`."""
import sys
from dbact.sensing import features as _implementation

sys.modules[__name__] = _implementation
