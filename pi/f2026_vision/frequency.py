from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import EstimatorConfig
from .trace import TraceObservation


@dataclass(frozen=True)
class FrequencyEstimate:
    frequency_hz: float
    nominal_100hz: int
    cycles: float
    phase_rad: float
    amplitude_px: float
    offset_px: float
    trend_px: float
    rmse_px: float
    normalized_rmse: float


class FrequencyEstimator:
    def __init__(self, config: EstimatorConfig | None = None) -> None:
        self.config = config or EstimatorConfig()

    @staticmethod
    def _fit(u: np.ndarray, x: np.ndarray, cycles: float) -> tuple[np.ndarray, float]:
        angle = 2.0 * np.pi * cycles * u
        design = np.column_stack(
            (np.sin(angle), np.cos(angle), np.ones_like(u), u)
        )
        coefficients, _, _, _ = np.linalg.lstsq(design, x, rcond=None)
        residual = x - design @ coefficients
        mse = float(np.mean(residual * residual))
        return coefficients, mse

    def estimate(self, trace: TraceObservation, ramp_seconds: float) -> FrequencyEstimate:
        if ramp_seconds <= 0.0:
            raise ValueError("ramp_seconds must be positive")
        signal = trace.x - np.mean(trace.x)
        deviation = float(np.std(signal))
        if deviation < 1e-9:
            raise RuntimeError("trace has no horizontal variation")

        windowed = (signal / deviation) * np.hanning(signal.size)
        spectrum = np.abs(np.fft.rfft(windowed, n=self.config.fft_size))
        cycle_axis = np.fft.rfftfreq(
            self.config.fft_size,
            d=1.0 / (signal.size - 1),
        )
        search = (
            (cycle_axis >= self.config.minimum_cycles)
            & (cycle_axis <= self.config.maximum_cycles)
        )
        if not np.any(search):
            raise RuntimeError("empty FFT search interval")
        coarse = float(cycle_axis[search][np.argmax(spectrum[search])])

        half_width = self.config.refinement_half_width_cycles
        low = max(self.config.minimum_cycles, coarse - half_width)
        high = min(self.config.maximum_cycles, coarse + half_width)
        golden = 0.5 * (np.sqrt(5.0) - 1.0)
        left = high - golden * (high - low)
        right = low + golden * (high - low)
        left_error = self._fit(trace.u, trace.x, left)[1]
        right_error = self._fit(trace.u, trace.x, right)[1]

        for _ in range(self.config.refinement_iterations):
            if left_error < right_error:
                high = right
                right = left
                right_error = left_error
                left = high - golden * (high - low)
                left_error = self._fit(trace.u, trace.x, left)[1]
            else:
                low = left
                left = right
                left_error = right_error
                right = low + golden * (high - low)
                right_error = self._fit(trace.u, trace.x, right)[1]

        cycles = float(0.5 * (low + high))
        coefficients, mse = self._fit(trace.u, trace.x, cycles)
        sine_coefficient, cosine_coefficient, offset, trend = coefficients
        amplitude = float(np.hypot(sine_coefficient, cosine_coefficient))
        phase = float(np.arctan2(cosine_coefficient, sine_coefficient))
        rmse = float(np.sqrt(mse))
        normalized_rmse = rmse / max(amplitude, 1e-9)

        if amplitude < self.config.minimum_amplitude_px:
            raise RuntimeError(f"trace amplitude is too small: {amplitude:.1f} px")
        if normalized_rmse > self.config.maximum_normalized_rmse:
            raise RuntimeError(
                f"poor sine fit: normalized RMSE {normalized_rmse:.3f}"
            )

        frequency = cycles / ramp_seconds
        return FrequencyEstimate(
            frequency_hz=frequency,
            nominal_100hz=int(round(frequency / 100.0) * 100),
            cycles=cycles,
            phase_rad=phase,
            amplitude_px=amplitude,
            offset_px=float(offset),
            trend_px=float(trend),
            rmse_px=rmse,
            normalized_rmse=normalized_rmse,
        )

    def phase_at_frequency(
        self,
        trace: TraceObservation,
        frequency_hz: float,
        ramp_seconds: float,
    ) -> FrequencyEstimate:
        cycles = frequency_hz * ramp_seconds
        coefficients, mse = self._fit(trace.u, trace.x, cycles)
        sine_coefficient, cosine_coefficient, offset, trend = coefficients
        amplitude = float(np.hypot(sine_coefficient, cosine_coefficient))
        phase = float(np.arctan2(cosine_coefficient, sine_coefficient))
        rmse = float(np.sqrt(mse))
        normalized_rmse = rmse / max(amplitude, 1e-9)
        return FrequencyEstimate(
            frequency_hz=float(frequency_hz),
            nominal_100hz=int(round(frequency_hz / 100.0) * 100),
            cycles=cycles,
            phase_rad=phase,
            amplitude_px=amplitude,
            offset_px=float(offset),
            trend_px=float(trend),
            rmse_px=rmse,
            normalized_rmse=normalized_rmse,
        )
