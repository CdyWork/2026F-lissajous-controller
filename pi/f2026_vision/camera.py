from __future__ import annotations

import time
from dataclasses import dataclass
import sys

import cv2
import numpy as np

from .config import CameraConfig


@dataclass(frozen=True)
class CameraFrame:
    timestamp_seconds: float
    bgr: np.ndarray


class OrangePiV4L2CameraSource:
    """Orange Pi Zero3W IMX219 capture through the vendor v4l2cam helper.

    The A733 Bullseye image used in this project provides ``v4l2cam`` under
    ``/opt/v4l2_opencv_demo``. It returns ISP-processed RGB24 frames, which is a
    direct RGB frames for sweep photographs.
    """

    def __init__(
        self,
        config: CameraConfig | None = None,
        device: str = "/dev/video0",
        warmup_frames: int = 12,
    ) -> None:
        self.config = config or CameraConfig()
        self.device = device
        self.warmup_frames = max(0, warmup_frames)
        self._camera = None

    def start(self) -> None:
        try:
            import v4l2cam
        except ImportError as error:
            helper_dir = "/opt/v4l2_opencv_demo"
            if helper_dir not in sys.path:
                sys.path.append(helper_dir)
            try:
                import v4l2cam
            except ImportError as retry_error:
                raise RuntimeError(
                    "v4l2cam is missing; on Orange Pi run with "
                    "PYTHONPATH=/opt/v4l2_opencv_demo or install the vendor "
                    "OpenCV/V4L2 demo package"
                ) from retry_error

        camera = v4l2cam.V4L2Camera(
            self.device,
            self.config.width,
            self.config.height,
            enable_isp=True,
            format="RGB24",
        )
        if not camera.init():
            raise RuntimeError(f"cannot initialise Orange Pi camera {self.device}")
        try:
            camera.set_fps(int(round(self.config.capture_fps)))
        except Exception:
            pass
        if not camera.start():
            raise RuntimeError(f"cannot start Orange Pi camera {self.device}")
        self._camera = camera
        for _ in range(self.warmup_frames):
            self.capture()
            time.sleep(0.01)

    def stop(self) -> None:
        if self._camera is not None:
            try:
                self._camera.stop()
            finally:
                self._camera = None

    def capture(self) -> CameraFrame:
        if self._camera is None:
            raise RuntimeError("camera is not running")
        rgb = self._camera.get_frame()
        if rgb is None:
            raise RuntimeError("Orange Pi camera returned no frame")
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        return CameraFrame(
            timestamp_seconds=time.monotonic(),
            bgr=bgr,
        )

    def __enter__(self) -> "OrangePiV4L2CameraSource":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()
