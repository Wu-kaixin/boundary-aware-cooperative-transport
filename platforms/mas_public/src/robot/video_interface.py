from __future__ import annotations


class VideoInterface:
    """视频模块预留接口；视觉识别应独立于 Robot Module 实现。 / Reserved interface for a future vision module."""

    def start(self) -> None:
        raise NotImplementedError("Video Module will be implemented independently later.")

    def stop(self) -> None:
        raise NotImplementedError("Video Module will be implemented independently later.")
