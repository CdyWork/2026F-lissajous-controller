from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PhaseFrequencyEstimate:
    residual_hz: float
    frequency_hz: float
    phase_at_latest_rad: float
    phase_rmse_rad: float
    sample_count: int
    time_span_seconds: float


class PhaseFrequencyTracker:
    """Estimate sub-bin frequency from the slope of fitted probe phase."""

    def __init__(
        self,
        nominal_hz: float,
        maximum_samples: int = 120,
        minimum_samples: int = 6,
        minimum_span_seconds: float = 0.15,
    ) -> None:
        self.nominal_hz = float(nominal_hz)
        self.minimum_samples = minimum_samples
        self.minimum_span_seconds = minimum_span_seconds
        self._samples: deque[tuple[float, float]] = deque(maxlen=maximum_samples)

    def add(self, timestamp_seconds: float, phase_rad: float) -> None:
        if self._samples and timestamp_seconds <= self._samples[-1][0]:
            raise ValueError("phase timestamps must be strictly increasing")
        self._samples.append((float(timestamp_seconds), float(phase_rad)))

    def clear(self) -> None:
        self._samples.clear()

    def estimate(self) -> PhaseFrequencyEstimate | None:
        if len(self._samples) < self.minimum_samples:
            return None
        values = np.asarray(self._samples, dtype=np.float64)
        timestamps = values[:, 0]
        phases = np.unwrap(values[:, 1])
        relative_time = timestamps - timestamps[0]
        span = float(relative_time[-1])
        if span < self.minimum_span_seconds:
            return None

        design = np.column_stack((relative_time, np.ones_like(relative_time)))
        coefficients, _, _, _ = np.linalg.lstsq(design, phases, rcond=None)
        residual = phases - design @ coefficients

        # Reject isolated phase fits caused by a partial LCD refresh or a
        # momentarily broken trace, then refit the phase slope.
        median = float(np.median(residual))
        mad = float(np.median(np.abs(residual - median)))
        limit = max(0.08, 4.0 * 1.4826 * mad)
        keep = np.abs(residual - median) <= limit
        if np.count_nonzero(keep) >= self.minimum_samples:
            coefficients, _, _, _ = np.linalg.lstsq(
                design[keep], phases[keep], rcond=None
            )
            residual = phases[keep] - design[keep] @ coefficients

        slope, intercept = coefficients
        residual_hz = float(slope / (2.0 * np.pi))
        latest_phase = float(intercept + slope * relative_time[-1])
        return PhaseFrequencyEstimate(
            residual_hz=residual_hz,
            frequency_hz=self.nominal_hz + residual_hz,
            phase_at_latest_rad=latest_phase,
            phase_rmse_rad=float(np.sqrt(np.mean(residual * residual))),
            sample_count=int(np.count_nonzero(keep)),
            time_span_seconds=span,
        )
