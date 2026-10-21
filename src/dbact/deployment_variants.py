"""Compatibility alias for :mod:`dbact.deployment.variants`."""
import sys
from dbact.deployment import variants as _implementation

sys.modules[__name__] = _implementation
