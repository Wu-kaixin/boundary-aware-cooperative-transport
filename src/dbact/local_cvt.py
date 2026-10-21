"""Compatibility alias for :mod:`dbact.deployment.cvt`."""
import sys
from dbact.deployment import cvt as _implementation

sys.modules[__name__] = _implementation
