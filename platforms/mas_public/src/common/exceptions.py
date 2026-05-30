class MASException(Exception):
    """项目基础异常类型。 / Base exception type for the MAS project."""


class ConfigError(MASException):
    """配置文件缺失或字段非法。 / Raised when a config file is missing or invalid."""


class TransportError(MASException):
    """通信层异常。 / Raised by the messaging layer."""


class HardwareUnavailableError(MASException):
    """真实硬件 SDK 或连接不可用。 / Raised when hardware SDK or connection is unavailable."""
