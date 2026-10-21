"""Compatibility alias for :mod:`dbact.control.agent_control`."""
import sys
from dbact.control import agent_control as _implementation

sys.modules[__name__] = _implementation
