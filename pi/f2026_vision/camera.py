from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterator

import cv2
import numpy as np

from .config import CameraConfig


@dataclass(frozen=True)
class CameraFrame:
    timestamp_seconds: float
    bgr: np.ndarray
    exposure_us: int
    analogue_gain: float


class PiCameraSource:
    """Small Picamera2 adapter; Picamera2 remains an OS package dependency."""

    def __init__(self, config: CameraConfig | None = None) -> None:
        self.config = config or CameraConfig()
        self._camera = None
        self._last_processed = 0.0

    def start(self) -> None:
        try:
            from picamera2 import Picamera2
        except ImportError as error:
            raise RuntimeError(
                "Picamera2 is missing; install python3-picamera2 from Raspberry Pi OS"
            ) from error

        camera = Picamera2()
        frame_duration_us = int(round(1_000_000.0 / self.config.capture_fps))
        configuration = camera.create_video_configuration(
            main={
                "size": (self.config.width, self.config.height),
                "format": "RGB888",
            },
            buffer_count=4,
        )
        camera.configure(configuration)
        camera.start()
        camera.set_controls(
            {
                "AeEnable": False,
                "ExposureTime": self.config.exposure_us,
                "AnalogueGain": self.config.analogue_gain,
                "FrameDurationLimits": (frame_duration_us, frame_duration_us),
            }
        )
        self._camera = camera
        self._last_processed = 0.0

    def stop(self) -> None:
        if self._camera is not None:
            self._camera.stop()
            self._camera.close()
            self._camera = None

    @property
    def exposure_range_us(self):
        if self._camera is None:
            raise RuntimeError("camera is not running")
        return self._camera.camera_controls.get("ExposureTime")

    def capture(self) -> CameraFrame:
        if self._camera is None:
            raise RuntimeError("camera is not running")
        request = self._camera.capture_request()
        try:
            rgb = request.make_array("main")
            metadata = request.get_metadata()
        finally:
            request.release()
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        timestamp_ns = metadata.get("SensorTimestamp")
        timestamp = (
            float(timestamp_ns) * 1e-9
            if timestamp_ns is not None
            else time.monotonic()
        )
        return CameraFrame(
            timestamp_seconds=timestamp,
            bgr=bgr,
            exposure_us=int(metadata.get("ExposureTime", self.config.exposure_us)),
            analogue_gain=float(
                metadata.get("AnalogueGain", self.config.analogue_gain)
            ),
        )

    def frames(self) -> Iterator[CameraFrame]:
        minimum_interval = 1.0 / self.config.process_fps
        while True:
            frame = self.capture()
            if frame.timestamp_seconds - self._last_processed < minimum_interval * 0.90:
                continue
            self._last_processed = frame.timestamp_seconds
            yield frame

    def __enter__(self) -> "PiCameraSource":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()
