from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .frequency import FrequencyEstimate, FrequencyEstimator
from .geometry import ScopeCalibration, ScopeLocator
from .trace import TraceExtractor, TraceObservation


@dataclass(frozen=True)
class FrameMeasurement:
    trace: TraceObservation
    frequency: FrequencyEstimate
    rectified_bgr: np.ndarray


class VisionPipeline:
    def __init__(
        self,
        calibration: ScopeCalibration | None = None,
        background_bgr: np.ndarray | None = None,
        locator: ScopeLocator | None = None,
        extractor: TraceExtractor | None = None,
        estimator: FrequencyEstimator | None = None,
    ) -> None:
        self.calibration = calibration
        self.background_bgr = background_bgr
        self.locator = locator or ScopeLocator()
        self.extractor = extractor or TraceExtractor()
        self.estimator = estimator or FrequencyEstimator()

    def calibrate(self, trace_free_frames: Iterable[np.ndarray]) -> ScopeCalibration:
        frames = [np.asarray(frame) for frame in trace_free_frames]
        if not frames:
            raise ValueError("at least one trace-free calibration frame is required")
        self.calibration = self.locator.locate(frames)
        rectified = [self.calibration.warp(frame) for frame in frames]
        self.background_bgr = np.median(np.stack(rectified), axis=0).astype(np.uint8)
        return self.calibration

    def process(self, frame_bgr: np.ndarray, ramp_seconds: float) -> FrameMeasurement:
        if self.calibration is None:
            self.calibration = self.locator.locate(frame_bgr)
        rectified = self.calibration.warp(frame_bgr)
        trace = self.extractor.extract(rectified, self.background_bgr)
        frequency = self.estimator.estimate(trace, ramp_seconds)
        return FrameMeasurement(trace, frequency, rectified)

    def extract(self, frame_bgr: np.ndarray) -> tuple[TraceObservation, np.ndarray]:
        if self.calibration is None:
            raise RuntimeError("scope calibration is not available")
        rectified = self.calibration.warp(frame_bgr)
        trace = self.extractor.extract(rectified, self.background_bgr)
        return trace, rectified
