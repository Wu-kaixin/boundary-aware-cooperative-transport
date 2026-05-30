from __future__ import annotations

import logging
import logging.config

from src.common.config_loader import get_project_root, load_config


def setup_logging(module_name: str) -> logging.Logger:
    """按 logging.yaml 初始化模块 logger，并确保 logs 目录存在。 / Configure a module logger from logging.yaml."""
    config = load_config("logging.yaml")
    root = get_project_root()
    log_dir = root / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    level = config.get("log_level", "INFO")
    log_format = config.get(
        "log_format", "%(asctime)s | %(name)s | %(levelname)s | %(message)s"
    )
    handlers: dict[str, dict] = {}
    root_handlers: list[str] = []

    if config.get("log_to_console", True):
        handlers["console"] = {
            "class": "logging.StreamHandler",
            "level": level,
            "formatter": "standard",
        }
        root_handlers.append("console")

    if config.get("log_to_file", True):
        handlers["file"] = {
            "class": "logging.handlers.RotatingFileHandler",
            "level": level,
            "formatter": "standard",
            "filename": str(log_dir / f"{module_name}.log"),
            "maxBytes": 5_000_000,
            "backupCount": 3,
            "encoding": "utf-8",
        }
        root_handlers.append("file")

    logging.config.dictConfig(
        {
            "version": 1,
            "disable_existing_loggers": False,
            "formatters": {"standard": {"format": log_format}},
            "handlers": handlers,
            "loggers": {
                module_name: {
                    "handlers": root_handlers,
                    "level": level,
                    "propagate": False,
                }
            },
        }
    )
    return logging.getLogger(module_name)
