"""Compatibility alias for :mod:`dbact.deployment.density`."""
import sys
from dbact.deployment import density as _implementation

sys.modules[__name__] = _implementation
