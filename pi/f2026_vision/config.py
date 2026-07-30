from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CameraConfig:
    width: int = 640
    height: int = 480
    capture_fps: float = 60.0
    process_fps: float = 30.0
    exposure_us: int = 800
    analogue_gain: float = 16.0


@dataclass(frozen=True)
class LocatorConfig:
    output_width: int = 500
    output_height: int = 380
    neutral_threshold_delta: int = 11
    minimum_span_fraction: float = 0.45
    minimum_area_fraction: float = 0.12
    maximum_aspect_ratio: float = 2.2


@dataclass(frozen=True)
class TraceConfig:
    background_threshold: int = 24
    green_minimum: int = 56
    green_over_red: int = 25
    green_over_blue: int = 18
    minimum_valid_fraction: float = 0.70
    # A high-frequency sine crosses a camera row at a steep angle, so its
    # horizontal support is naturally wide even when the trace itself is thin.
    maximum_mean_row_spread_px: float = 140.0
    edge_margin_rows: int = 2


@dataclass(frozen=True)
class EstimatorConfig:
    fft_size: int = 65536
    minimum_cycles: float = 0.5
    maximum_cycles: float = 110.0
    refinement_half_width_cycles: float = 0.8
    refinement_iterations: int = 36
    minimum_amplitude_px: float = 20.0
    maximum_normalized_rmse: float = 0.25
