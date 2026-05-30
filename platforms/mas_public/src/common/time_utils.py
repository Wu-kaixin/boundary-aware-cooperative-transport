from __future__ import annotations

import time
from datetime import datetime


def now_s() -> float:
    """返回用于消息和 CSV 的 wall time 秒。 / Return wall-clock seconds for messages and CSV records."""
    return time.time()


def monotonic_s() -> float:
    """返回单调时钟，用于循环周期和 watchdog 判断。 / Return monotonic seconds for loops and timeouts."""
    return time.monotonic()


def timestamp_for_dir() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def sleep_until_next(start_time: float, period_s: float) -> None:
    elapsed = monotonic_s() - start_time
    remaining = period_s - elapsed
    if remaining > 0:
        time.sleep(remaining)
