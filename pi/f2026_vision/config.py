from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CameraConfig:
    """Fixed capture configuration for the Orange Pi IMX219 helper."""

    width: int = 640
    height: int = 480
    capture_fps: float = 30.0
