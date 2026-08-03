"""Visual phase lock for the Q5 free-running output in XY scope mode."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from q5_fpga_sweep import Camera, Uart, median_frame, rectify


# The 200 kpoint scope display was measured on hardware: a PHASEQ step becomes
# visible after about 0.42 s, but the complete scope/camera refresh varies with
# the acquisition cycle. Poll until several frames agree instead of assuming a
# fixed delay and accidentally fitting the previous phase.
MIN_OBSERVATION_S = 0.45
MAX_OBSERVATION_S = 3.50
OBSERVATION_STABLE_FRAMES = 3
OBSERVATION_STABILITY_DEGREES = 5.0
MIN_PROBE_RESPONSE_DEGREES = 12.0
MEASURED_DISPLAY_LATENCY_S = 0.42
MIN_TRACE_PIXELS = 500
PHASE_UPDATE_INTERVAL_S = 0.020
VISUAL_UPDATE_INTERVAL_S = 0.80
POST_LOCK_VISUAL_HOLDOFF_S = 0.80
VISUAL_PHASE_DEADBAND_DEGREES = 5.0
VISUAL_PHASE_RESUME_DEGREES = 7.0
VISUAL_PHASE_MIN_STEP_DEGREES = 0.5
VISUAL_PHASE_MAX_STEP_DEGREES = 2.0
VISUAL_PHASE_IMPROVEMENT_DEGREES = 0.5
DEFAULT_STATIC_PHASE_DEGREES = 14.0
LOCK_ACCEPT_ERROR_DEGREES = 10.0
FINE_LOCK_ERROR_DEGREES = 5.0
ZERO_REFINE_ACCEPT_DEGREES = 7.0
ZERO_REFINE_MIN_OBSERVATION_S = 0.70
ZERO_REFINE_MAX_OBSERVATION_S = 1.20
MIN_MEASURED_DRIFT_RATE_DEGREES_PER_S = 0.15
MAX_MEASURED_DRIFT_RATE_DEGREES_PER_S = 12.0
MAX_DRIFT_FIT_RMS_DEGREES = 0.75
MAX_DRIFT_SEGMENT_SPREAD_DEGREES_PER_S = 3.0
MIN_DRIFT_SAMPLE_INTERVAL_S = 0.75
# The 90.9 kHz bench point normally measures about +9..13 deg/s before
# correction. Each 100 kHz reference calibration shifts it slightly, so this
# value is only a fallback when the current run cannot produce a reliable fit.
REFERENCE_CALIBRATION_HZ = 100_000
REFERENCE_CALIBRATION_PERIODS = 50_000
REFERENCE_CALIBRATION_EXPECTED_TICKS = 25_000_000
FPGA_NOMINAL_CLOCK_HZ = 50_000_000
PHASE_ACCUMULATOR_MODULUS = 1 << 40
DOUBLE_SETTLE_S = 0.45
DOUBLE_OBSERVATION_TIMEOUT_S = 0.80
DOUBLE_STABLE_FRAMES = 2
DOUBLE_STABILITY_DEGREES = 4.0
DOUBLE_PHASE_DEADBAND_DEGREES = 2.0
DOUBLE_COARSE_MAX_DEGREES = 45.0
DOUBLE_FINE_GAIN = 0.70
DOUBLE_FINE_MAX_DEGREES = 10.0
DOUBLE_MAX_X_OFFSET_PX = 18.0
Q5_CLEAR_OUTPUT_S = 0.16
TRACK_LOCK_TIMEOUT_S = 2.0
TRACK_OBSERVATION_TIMEOUT_S = 1.15
TRACK_MIN_OBSERVATION_S = 0.72
TRACK_OBSERVATION_STABLE_FRAMES = 2
# The measured display latency is about 0.42 s. A shorter IDLE interval never
# reaches the visible scope frame and leaves old/new phase traces superimposed.
TRACK_CLEAR_OUTPUT_S = 0.50
TRACK_FALLBACK_STABILITY_DEGREES = 18.0
TRACK_PROBE_DEGREES = 90.0
TRACK_SECONDARY_PROBE_DEGREES = 45.0
TRACK_MIN_TRANSITION_DEGREES = 4.0
TRACK_ACCEPT_ERROR_DEGREES = 8.0
TRACK_MODEL_MIN_IMPROVEMENT_DEGREES = 2.0
TRACK_FINE_MAX_DEGREES = 35.0
TRACK_FINAL_REFINE_LIMIT_DEGREES = 15.0
TRACK_FINAL_REFINE_MAX_DEGREES = 15.0
TRACK_LINE_CENTER_HALF_WIDTH_PX = 10.0
TRACK_LINE_MIN_CENTER_OCCUPANCY = 0.55
TRACK_LINE_MAX_MEDIAN_HALF_WIDTH_PX = 10.0
TRACK_Q2_VERIFY_ERROR_DEGREES = 15.0


def compensation_from_calibration(
    frequency_hz: int, calibration_ticks: int
) -> dict[str, float | int]:
    """Replicate the MCU DDS rounding and derive its residual phase rate."""
    nominal_increment = (
        frequency_hz * PHASE_ACCUMULATOR_MODULUS + FPGA_NOMINAL_CLOCK_HZ // 2
    ) // FPGA_NOMINAL_CLOCK_HZ
    calibrated_increment = (
        nominal_increment * REFERENCE_CALIBRATION_EXPECTED_TICKS
        + calibration_ticks // 2
    ) // calibration_ticks
    output_frequency_hz = (
        calibrated_increment
        * calibration_ticks
        * REFERENCE_CALIBRATION_HZ
        / (REFERENCE_CALIBRATION_PERIODS * PHASE_ACCUMULATOR_MODULUS)
    )
    frequency_error_hz = output_frequency_hz - frequency_hz
    return {
        "calibration_ticks": calibration_ticks,
        "nominal_phase_increment": nominal_increment,
        "calibrated_phase_increment": calibrated_increment,
        "predicted_output_frequency_hz": output_frequency_hz,
        "predicted_frequency_error_hz": frequency_error_hz,
        "compensation_rate_degrees_per_s": -360.0 * frequency_error_hz,
    }


def capture_phase_error(
    camera: Camera,
    corners: np.ndarray,
    blank: np.ndarray,
    diagnostic_dir: Path | None = None,
    diagnostic_name: str | None = None,
) -> tuple[float, dict[str, float]]:
    """Return the centreline-ellipse phase error from one settled XY image."""
    frame_period = 1.0 / max(camera.fps, 1.0)
    # Do not median several camera frames here.  The input and output are
    # intentionally free-running, so a multi-frame blend is a time-average of
    # different phase errors and makes the drift estimate late.  One fresh
    # frame is the correct observation for the subsequent open-loop model.
    time.sleep(frame_period)
    xy = rectify(camera.capture().bgr, corners)
    current_gray = cv2.cvtColor(xy, cv2.COLOR_BGR2GRAY).astype(np.float32)
    blank_gray = cv2.cvtColor(blank, cv2.COLOR_BGR2GRAY).astype(np.float32)
    interior = np.s_[16:-16, 16:-16]
    reference = (blank_gray[interior] >= 16.0) & (blank_gray[interior] <= 220.0)
    if int(reference.sum()) < 1000:
        raise RuntimeError("phase background has insufficient reference pixels")
    background_gain = float(np.median(
        current_gray[interior][reference] / blank_gray[interior][reference]
    ))
    # Auto exposure can change substantially while the scope switches from an
    # idle grid to a bright XY trace.  Do not leave the whole graticule in the
    # subtraction residual by clipping this normalization too tightly.
    background_gain = float(np.clip(background_gain, 0.50, 2.00))
    gray = np.clip(current_gray - background_gain * blank_gray, 0.0, 255.0).astype(np.uint8)
    # The trace is cyan/white while the scope grid and axes are orange.  A
    # blue-channel mask is insensitive to the exposure difference that makes
    # background subtraction temporarily light up the entire graticule.
    blue = xy[:, :, 0]
    blue_reference = int(np.percentile(blue[interior], 99.4))
    blue_threshold = max(90, blue_reference * 72 // 100)
    mask = np.where(blue >= blue_threshold, 255, 0).astype(np.uint8)
    # Residual graticule axes cross the image centre. Remove only their narrow
    # bands; the wave trace remains available on both sides for ellipse fitting.
    centre = mask.shape[0] // 2
    mask[centre - 3:centre + 4, :] = 0
    mask[:, centre - 3:centre + 4] = 0
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    mask[:12, :] = 0
    mask[-12:, :] = 0
    mask[:, :12] = 0
    mask[:, -12:] = 0
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    kept = np.zeros_like(mask)
    kept_components = 0
    for label in range(1, component_count):
        _, _, width, height, area = stats[label]
        if area >= 40 and max(width, height) >= 28:
            kept[labels == label] = 255
            kept_components += 1
    if diagnostic_dir is not None and diagnostic_name is not None:
        diagnostic_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(diagnostic_dir / f"{diagnostic_name}_xy.png"), xy)
        cv2.imwrite(str(diagnostic_dir / f"{diagnostic_name}_residual.png"), gray)
        cv2.imwrite(str(diagnostic_dir / f"{diagnostic_name}_mask.png"), kept)
    rows, columns = np.nonzero(kept)
    if len(rows) < MIN_TRACE_PIXELS:
        raise RuntimeError(f"phase trace too sparse: {len(rows)} pixels")
    if len(rows) > 30000:
        raise RuntimeError(f"phase background residual too broad: {len(rows)} pixels")

    points = np.column_stack((columns.astype(np.float64), rows.astype(np.float64)))
    covariance = np.cov(points, rowvar=False)
    variance_x = float(covariance[0, 0])
    variance_y = float(covariance[1, 1])
    if variance_x < 25.0 or variance_y < 25.0:
        raise RuntimeError("phase trace has insufficient XY span")
    correlation = float(covariance[0, 1] / np.sqrt(variance_x * variance_y))
    correlation = float(np.clip(correlation, -1.0, 1.0))
    centered_points = points - np.median(points, axis=0)
    _, eigenvectors = np.linalg.eigh(covariance)
    major_minor = centered_points @ eigenvectors[:, ::-1]
    major = major_minor[:, 0]
    minor = major_minor[:, 1]
    major_extent = float(np.percentile(np.abs(major), 95.0))
    central_minor = np.abs(minor[np.abs(major) < 0.5 * major_extent])
    if len(central_minor) == 0:
        line_center_occupancy = 0.0
        line_median_half_width = float("inf")
    else:
        line_center_occupancy = float(np.mean(
            central_minor < TRACK_LINE_CENTER_HALF_WIDTH_PX
        ))
        line_median_half_width = float(np.median(central_minor))
    # Normalize the unequal screen gains before fitting. The ellipse axis
    # ratio is tan(|phi| / 2), unlike pixel covariance it is not biased by
    # the non-uniform density of a rasterized ellipse outline.
    x_span = float(np.percentile(points[:, 0], 95.0) - np.percentile(points[:, 0], 5.0))
    y_span = float(np.percentile(points[:, 1], 95.0) - np.percentile(points[:, 1], 5.0))
    if x_span < 20.0 or y_span < 20.0:
        raise RuntimeError("phase trace has insufficient axis amplitude")
    fit_points = points.copy()
    fit_points[:, 1] *= x_span / y_span
    contours, _ = cv2.findContours(kept, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contour_points = [
        contour.reshape(-1, 2)
        for contour in contours
        if len(contour) >= 12
    ]
    ellipse_points = (
        np.concatenate(contour_points, axis=0).astype(np.float32)
        if contour_points else points.astype(np.float32)
    )
    ellipse_points[:, 1] *= x_span / y_span
    _, axes, _ = cv2.fitEllipse(ellipse_points)
    minor_axis, major_axis = sorted(float(axis) for axis in axes)
    error_degrees = float(np.degrees(2.0 * np.arctan(minor_axis / major_axis)))
    # Screen voltage increases upward while OpenCV image Y increases downward.
    # Hence a same-phase diagonal has negative image covariance. A positive
    # covariance represents the supplementary (near-180 degree) phase.
    if correlation > 0.0:
        error_degrees = 180.0 - error_degrees
    return error_degrees, {
        "correlation": correlation,
        "trace_pixels": float(len(rows)),
        "threshold": float(blue_threshold),
        "background_gain": background_gain,
        "components": float(kept_components),
        "line_center_occupancy": line_center_occupancy,
        "line_median_half_width_px": line_median_half_width,
    }


def _double_axis_center(blank: np.ndarray) -> tuple[float, float]:
    """Locate the two bright scope axes in an already rectified blank image."""
    gray = cv2.cvtColor(blank, cv2.COLOR_BGR2GRAY).astype(np.float32)
    height, width = gray.shape
    margin = max(16, min(height, width) // 20)
    column_score = gray[margin:height - margin, :].mean(axis=0)
    row_score = gray[:, margin:width - margin].mean(axis=1)
    smooth = np.ones(5, dtype=np.float32) / 5.0
    column_score = np.convolve(column_score, smooth, mode="same")
    row_score = np.convolve(row_score, smooth, mode="same")

    x_start, x_end = width // 4, width * 3 // 4
    y_start, y_end = height // 4, height * 3 // 4
    axis_x = x_start + int(np.argmax(column_score[x_start:x_end]))
    axis_y = y_start + int(np.argmax(row_score[y_start:y_end]))
    if (
        column_score[axis_x] < np.median(column_score[x_start:x_end]) * 1.5
        or row_score[axis_y] < np.median(row_score[y_start:y_end]) * 1.5
    ):
        raise RuntimeError("cannot locate both bright XY axes for DOUBLE calibration")
    return float(axis_x), float(axis_y)


def analyse_double_intersection(
    xy: np.ndarray, blank: np.ndarray
) -> tuple[float, dict[str, float], np.ndarray, np.ndarray]:
    """Return signed 2:1 Lissajous phase error from its self-intersection."""
    height, width = xy.shape[:2]
    axis_x, axis_y = _double_axis_center(blank)
    screen_x = (width - 1) * 0.5
    screen_y = (height - 1) * 0.5
    translation = np.float32((
        (1.0, 0.0, screen_x - axis_x),
        (0.0, 1.0, screen_y - axis_y),
    ))
    centred = cv2.warpAffine(
        xy, translation, (width, height), flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )

    blue = centred[:, :, 0]
    interior = blue[16:-16, 16:-16]
    blue_reference = int(np.percentile(interior, 99.4))
    blue_threshold = max(90, blue_reference * 72 // 100)
    mask = np.where(blue >= blue_threshold, 255, 0).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    mask[:12, :] = 0
    mask[-12:, :] = 0
    mask[:, :12] = 0
    mask[:, -12:] = 0

    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    kept = np.zeros_like(mask)
    kept_components = 0
    for label in range(1, component_count):
        _, _, component_width, component_height, area = stats[label]
        if area >= 40 and component_width >= 24 and component_height >= 24:
            kept[labels == label] = 255
            kept_components += 1

    rows, columns = np.nonzero(kept)
    if len(rows) < MIN_TRACE_PIXELS:
        raise RuntimeError(f"DOUBLE trace too sparse: {len(rows)} pixels")
    if len(rows) > 30000:
        raise RuntimeError(f"DOUBLE trace too broad: {len(rows)} pixels")
    x_low, x_high = np.percentile(columns, (1.0, 99.0))
    y_low, y_high = np.percentile(rows, (1.0, 99.0))
    x_span = float(x_high - x_low)
    y_span = float(y_high - y_low)
    if x_span < 80.0 or y_span < 80.0:
        raise RuntimeError("DOUBLE trace has insufficient XY span")

    trace_center_x = float((x_low + x_high) * 0.5)
    horizontal_offset = trace_center_x - screen_x
    if abs(horizontal_offset) > DOUBLE_MAX_X_OFFSET_PX:
        raise RuntimeError(
            f"DOUBLE trace remains horizontally offset by {horizontal_offset:.1f} px"
        )
    half_band = max(4.0, min(9.0, x_span * 0.018))
    crossing_rows = rows[
        np.abs(columns.astype(np.float64) - trace_center_x) <= half_band
    ]
    if len(crossing_rows) < 40:
        raise RuntimeError("DOUBLE self-intersection has insufficient pixels")

    intersection_y = float(np.median(crossing_rows))
    amplitude_y = y_span * 0.5
    normalized_y = float(np.clip(
        (screen_y - intersection_y) / amplitude_y, -1.0, 1.0
    ))
    phase_error = float(np.degrees(np.arcsin(normalized_y)))
    return phase_error, {
        "axis_x_before_recentering": axis_x,
        "axis_y_before_recentering": axis_y,
        "intersection_x": trace_center_x,
        "intersection_y": intersection_y,
        "screen_center_x": screen_x,
        "screen_center_y": screen_y,
        "horizontal_offset_px": horizontal_offset,
        "vertical_offset_px": intersection_y - screen_y,
        "x_span_px": x_span,
        "y_span_px": y_span,
        "normalized_intersection_y": normalized_y,
        "trace_pixels": float(len(rows)),
        "intersection_pixels": float(len(crossing_rows)),
        "threshold": float(blue_threshold),
        "components": float(kept_components),
    }, centred, kept


def capture_double_phase_error(
    camera: Camera,
    corners: np.ndarray,
    blank: np.ndarray,
    diagnostic_dir: Path | None = None,
    diagnostic_name: str | None = None,
) -> tuple[float, dict[str, float]]:
    """Capture one live DOUBLE frame and measure its signed phase error."""
    time.sleep(1.0 / max(camera.fps, 1.0))
    raw = camera.capture().bgr
    error, features, centred, mask = analyse_double_intersection(
        rectify(raw, corners), blank
    )
    if diagnostic_dir is not None and diagnostic_name is not None:
        diagnostic_dir.mkdir(parents=True, exist_ok=True)
        overlay = centred.copy()
        centre = int(round(features["screen_center_x"]))
        intersection = (
            int(round(features["intersection_x"])),
            int(round(features["intersection_y"])),
        )
        cv2.line(
            overlay, (0, centre), (overlay.shape[1] - 1, centre),
            (0, 180, 255), 1,
        )
        cv2.line(overlay, (centre, 0), (centre, overlay.shape[0] - 1), (0, 180, 255), 1)
        cv2.circle(overlay, intersection, 7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.imwrite(str(diagnostic_dir / f"{diagnostic_name}_xy.png"), centred)
        cv2.imwrite(str(diagnostic_dir / f"{diagnostic_name}_mask.png"), mask)
        cv2.imwrite(str(diagnostic_dir / f"{diagnostic_name}_overlay.png"), overlay)
    return error, features


def set_phase(uart: Uart, degrees: float) -> float:
    wrapped = degrees % 360.0
    millidegrees = int(round(wrapped * 1000.0)) % 360000
    reply = uart.command(f"PHASEQ {millidegrees}")
    if not reply.startswith("OK PHASEQ"):
        raise RuntimeError(f"PHASEQ command failed: {reply!r}")
    return wrapped


@dataclass
class Q5PhaseFeedForward:
    """Hold the calibrated phase after the one-time visual alignment."""

    uart: Uart
    reference_phase_degrees: float
    compensation_rate_degrees_per_s: float
    reference_time_s: float
    last_sent_millidegrees: int
    next_due_s: float
    next_visual_due_s: float = 0.0
    visual_direction: float = 1.0
    visual_previous_error_degrees: float | None = None
    visual_pending_delta_degrees: float = 0.0
    visual_active: bool = False

    def phase_at(self, now: float | None = None) -> float:
        if now is None:
            now = time.monotonic()
        return (
            self.reference_phase_degrees
            + self.compensation_rate_degrees_per_s * (now - self.reference_time_s)
        ) % 360.0

    def step(self) -> dict[str, object]:
        """Write the predicted phase only when its one-degree word changes."""
        now = time.monotonic()
        target_phase = self.phase_at(now)
        phase_millidegrees = int(round(target_phase * 1000.0)) % 360000
        changed = phase_millidegrees != self.last_sent_millidegrees
        if changed:
            set_phase(self.uart, phase_millidegrees / 1000.0)
            self.last_sent_millidegrees = phase_millidegrees
        self.next_due_s = time.monotonic() + PHASE_UPDATE_INTERVAL_S
        return {
            "changed": changed,
            "phase_degrees": phase_millidegrees / 1000.0,
            "compensation_rate_degrees_per_s": self.compensation_rate_degrees_per_s,
        }

    def apply_output_transform(self, frequency_multiplier: float = 1.0) -> dict[str, object]:
        """Rebase compensation after a post-lock FPGA output-mode change."""
        now = time.monotonic()
        source_phase = self.phase_at(now)
        transformed_phase = (source_phase * frequency_multiplier) % 360.0
        if abs(frequency_multiplier - 1.0) > 1e-9:
            transformed_phase = set_phase(self.uart, transformed_phase)
        self.reference_phase_degrees = transformed_phase
        self.reference_time_s = now
        self.compensation_rate_degrees_per_s *= frequency_multiplier
        self.last_sent_millidegrees = int(round(transformed_phase * 1000.0)) % 360000
        self.next_due_s = now + PHASE_UPDATE_INTERVAL_S
        # A deliberate 90-degree or 2x waveform is no longer a zero-phase
        # ellipse, so the post-lock visual hill climb must not undo it.
        self.next_visual_due_s = float("inf")
        self.visual_active = False
        self.visual_pending_delta_degrees = 0.0
        return {
            "source_phase_degrees": source_phase,
            "phase_degrees": transformed_phase,
            "frequency_multiplier": frequency_multiplier,
            "compensation_rate_degrees_per_s": self.compensation_rate_degrees_per_s,
        }

    def shift_phase(self, delta_degrees: float) -> dict[str, object]:
        """Apply one visual correction while preserving continuous feed-forward."""
        now = time.monotonic()
        target_phase = set_phase(self.uart, self.phase_at(now) + delta_degrees)
        self.reference_phase_degrees = target_phase
        self.reference_time_s = now
        self.last_sent_millidegrees = int(round(target_phase * 1000.0)) % 360000
        self.next_due_s = now + PHASE_UPDATE_INTERVAL_S
        return {
            "applied_delta_degrees": delta_degrees,
            "phase_degrees": target_phase,
            "compensation_rate_degrees_per_s": self.compensation_rate_degrees_per_s,
        }

    def visual_step(
        self, camera: Camera, corners: np.ndarray, blank: np.ndarray
    ) -> dict[str, object]:
        """Use delayed unsigned phase images as a bounded hill-climbing loop."""
        error, features = capture_phase_error(camera, corners, blank)
        previous_error = self.visual_previous_error_degrees
        pending_delta = self.visual_pending_delta_degrees
        direction_changed = False

        if pending_delta != 0.0 and previous_error is not None:
            if error > previous_error + VISUAL_PHASE_IMPROVEMENT_DEGREES:
                self.visual_direction = -1.0 if pending_delta > 0.0 else 1.0
                direction_changed = True
            elif error < previous_error - VISUAL_PHASE_IMPROVEMENT_DEGREES:
                self.visual_direction = 1.0 if pending_delta > 0.0 else -1.0

        if error <= VISUAL_PHASE_DEADBAND_DEGREES:
            self.visual_active = False
        elif error >= VISUAL_PHASE_RESUME_DEGREES:
            self.visual_active = True

        applied_delta = 0.0
        if self.visual_active:
            step = float(np.clip(
                error * 0.25,
                VISUAL_PHASE_MIN_STEP_DEGREES,
                VISUAL_PHASE_MAX_STEP_DEGREES,
            ))
            applied_delta = self.visual_direction * step
            now = time.monotonic()
            target_phase = set_phase(self.uart, self.phase_at(now) + applied_delta)
            self.reference_phase_degrees = target_phase
            self.reference_time_s = now
            self.last_sent_millidegrees = int(round(target_phase * 1000.0)) % 360000
            self.next_due_s = now + PHASE_UPDATE_INTERVAL_S

        self.visual_previous_error_degrees = error
        self.visual_pending_delta_degrees = applied_delta
        self.next_visual_due_s = time.monotonic() + VISUAL_UPDATE_INTERVAL_S
        return {
            "phase_error_degrees": error,
            "applied_delta_degrees": applied_delta,
            "direction_changed": direction_changed,
            "visual_active": self.visual_active,
            "phase_degrees": self.phase_at(),
            "compensation_rate_degrees_per_s": self.compensation_rate_degrees_per_s,
            "features": features,
        }


def calibrate_double_phase(
    feedforward: Q5PhaseFeedForward,
    camera: Camera,
    corners: np.ndarray,
    blank: np.ndarray,
    diagnostic_dir: Path | None = None,
) -> dict[str, object]:
    """Centre the DOUBLE self-intersection with one coarse and one fine step."""
    calibration_started_s = time.monotonic()

    def observe(label: str) -> dict[str, object]:
        started_s = time.monotonic()
        deadline_s = started_s + DOUBLE_OBSERVATION_TIMEOUT_S
        samples: list[tuple[float, dict[str, float]]] = []
        failures: list[str] = []
        while time.monotonic() < deadline_s:
            if time.monotonic() >= feedforward.next_due_s:
                feedforward.step()
            settled = time.monotonic() - started_s >= DOUBLE_SETTLE_S
            try:
                error, features = capture_double_phase_error(
                    camera,
                    corners,
                    blank,
                    diagnostic_dir if settled else None,
                    label if settled else None,
                )
            except RuntimeError as capture_error:
                failures.append(str(capture_error))
                continue
            if not settled:
                continue
            samples.append((error, features))
            if len(samples) >= DOUBLE_STABLE_FRAMES:
                recent = samples[-DOUBLE_STABLE_FRAMES:]
                spread = max(item[0] for item in recent) - min(
                    item[0] for item in recent
                )
                if spread <= DOUBLE_STABILITY_DEGREES:
                    break
        if len(samples) < DOUBLE_STABLE_FRAMES:
            reason = failures[-1] if failures else "phase frames did not settle"
            raise RuntimeError(f"no stable DOUBLE frame: {reason}")
        recent = samples[-DOUBLE_STABLE_FRAMES:]
        return {
            "phase_error_degrees": float(np.median([item[0] for item in recent])),
            "frames_used": len(samples),
            "features": recent[-1][1],
        }

    before = observe("double_before")
    initial_error = float(before["phase_error_degrees"])
    coarse_delta = 0.0
    coarse_report: dict[str, object] | None = None
    after_coarse: dict[str, object] | None = None
    fine_delta = 0.0
    fine_report: dict[str, object] | None = None
    restored = False

    if abs(initial_error) > DOUBLE_PHASE_DEADBAND_DEGREES:
        coarse_delta = float(np.clip(
            -initial_error,
            -DOUBLE_COARSE_MAX_DEGREES,
            DOUBLE_COARSE_MAX_DEGREES,
        ))
        coarse_report = feedforward.shift_phase(coarse_delta)
        after_coarse = observe("double_after_coarse")
        residual_error = float(after_coarse["phase_error_degrees"])
        if abs(residual_error) > abs(initial_error) + DOUBLE_STABILITY_DEGREES:
            feedforward.shift_phase(-coarse_delta)
            restored = True
        elif abs(residual_error) > DOUBLE_PHASE_DEADBAND_DEGREES:
            fine_delta = float(np.clip(
                -residual_error * DOUBLE_FINE_GAIN,
                -DOUBLE_FINE_MAX_DEGREES,
                DOUBLE_FINE_MAX_DEGREES,
            ))
            fine_report = feedforward.shift_phase(fine_delta)

    if restored:
        status = "unreliable_restored"
    elif coarse_delta == 0.0:
        status = "centred"
    elif fine_delta != 0.0:
        status = "fine_corrected"
    else:
        status = "verified"
    return {
        "status": status,
        "before": before,
        "coarse_delta_degrees": coarse_delta,
        "coarse": coarse_report,
        "after_coarse": after_coarse,
        "fine_delta_degrees": fine_delta,
        "fine": fine_report,
        "restored": restored,
        "elapsed_s": time.monotonic() - calibration_started_s,
        "phase_degrees": feedforward.phase_at(),
        "compensation_rate_degrees_per_s": (
            feedforward.compensation_rate_degrees_per_s
        ),
    }


def run_tracking_phase_calibration(
    uart: Uart,
    camera: Camera,
    corners: np.ndarray,
    blank: np.ndarray,
    question_number: int,
    diagnostic_dir: Path | None = None,
) -> dict[str, object]:
    """Hard-lock the input, visually zero static phase, then select Q1-Q3."""
    if question_number not in (1, 2, 3):
        raise ValueError(f"unsupported tracking question {question_number}")

    started_s = time.monotonic()

    def command_ok(command: str, expected: str) -> str:
        reply = uart.command(command)
        if not reply.startswith(expected):
            raise RuntimeError(f"MCU rejected {command}: {reply!r}")
        return reply

    def status_fields() -> tuple[str, dict[str, str]]:
        reply = uart.command("STATUS")
        fields: dict[str, str] = {}
        for token in reply.split():
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key] = value
        return reply, fields

    def wait_for_lock() -> str:
        deadline_s = time.monotonic() + TRACK_LOCK_TIMEOUT_S
        last_reply = ""
        while time.monotonic() < deadline_s:
            last_reply, fields = status_fields()
            if fields.get("LOCK") == "1" and fields.get("FOUT") == "1":
                return last_reply
            time.sleep(0.04)
        raise RuntimeError(f"FPGA hard lock did not become ready: {last_reply!r}")

    def observe(label: str) -> dict[str, object]:
        observed_s = time.monotonic()
        deadline_s = observed_s + TRACK_OBSERVATION_TIMEOUT_S
        samples: list[tuple[float, dict[str, float]]] = []
        failures: list[str] = []
        while time.monotonic() < deadline_s:
            try:
                error, features = capture_phase_error(
                    camera,
                    corners,
                    blank,
                    diagnostic_dir if time.monotonic() - observed_s >= TRACK_MIN_OBSERVATION_S else None,
                    label if time.monotonic() - observed_s >= TRACK_MIN_OBSERVATION_S else None,
                )
            except RuntimeError as capture_error:
                failures.append(str(capture_error))
                continue
            if time.monotonic() - observed_s < TRACK_MIN_OBSERVATION_S:
                continue
            samples.append((error, features))
            if len(samples) >= TRACK_OBSERVATION_STABLE_FRAMES:
                recent = samples[-TRACK_OBSERVATION_STABLE_FRAMES:]
                errors = [item[0] for item in recent]
                if max(errors) - min(errors) <= OBSERVATION_STABILITY_DEGREES:
                    return {
                        "error_degrees": float(np.median(errors)),
                        "frames_used": len(samples),
                        "features": recent[-1][1],
                        "stable": True,
                    }
        if len(samples) >= TRACK_OBSERVATION_STABLE_FRAMES:
            recent = samples[-min(5, len(samples)):]
            errors = [item[0] for item in recent]
            spread = max(errors) - min(errors)
            if spread <= TRACK_FALLBACK_STABILITY_DEGREES:
                return {
                    "error_degrees": float(np.median(errors)),
                    "frames_used": len(samples),
                    "features": recent[-1][1],
                    "stable": False,
                    "stability_spread_degrees": float(spread),
                }
        reason = failures[-1] if failures else "phase frames did not settle"
        raise RuntimeError(f"no stable tracking phase frame for {label}: {reason}")

    def observe_after_change(
        label: str,
        previous: dict[str, object],
        minimum_change_degrees: float = TRACK_MIN_TRANSITION_DEGREES,
        max_attempts: int = 2,
    ) -> dict[str, object]:
        observation: dict[str, object] | None = None
        for attempt in range(max_attempts):
            suffix = "" if attempt == 0 else f"_retry_{attempt}"
            observation = observe(f"{label}{suffix}")
            change = abs(
                float(observation["error_degrees"])
                - float(previous["error_degrees"])
            )
            observation["transition_attempts"] = attempt + 1
            if change >= minimum_change_degrees:
                return observation
        assert observation is not None
        return observation

    def wrap_signed(degrees: float) -> float:
        return (degrees + 180.0) % 360.0 - 180.0

    def alignment_acceptable(observation: dict[str, object]) -> bool:
        features = observation.get("features")
        line_accepted = False
        if isinstance(features, dict):
            line_accepted = (
                float(features.get("correlation", 0.0)) < 0.0
                and float(features.get("line_center_occupancy", 0.0))
                >= TRACK_LINE_MIN_CENTER_OCCUPANCY
                and float(features.get("line_median_half_width_px", float("inf")))
                <= TRACK_LINE_MAX_MEDIAN_HALF_WIDTH_PX
            )
        observation["line_accepted"] = line_accepted
        return (
            float(observation["error_degrees"]) <= TRACK_ACCEPT_ERROR_DEGREES
            or line_accepted
        )

    def apply_tracking_phase(degrees: float) -> tuple[float, str]:
        command_ok("IDLE", "OK IDLE")
        time.sleep(TRACK_CLEAR_OUTPUT_S)
        phase = set_phase(uart, degrees)
        command_ok("TRACK DIAG", "OK TRACK")
        return phase, wait_for_lock()

    def solve_target(
        base_command: float,
        base_error: float,
        probe_error: float,
        probe_step: float,
    ) -> dict[str, float]:
        candidates: list[tuple[float, float, float, float]] = []
        for signed_base in (base_error, -base_error):
            for signed_probe in (probe_error, -probe_error):
                response = wrap_signed(signed_probe - signed_base)
                gain = response / probe_step
                if abs(gain) < 0.25:
                    continue
                target = (base_command - signed_base / gain) % 360.0
                score = abs(abs(gain) - 1.0)
                candidates.append((score, target, signed_base, gain))
        if not candidates:
            raise RuntimeError("tracking phase probe produced no usable response")
        score, target, signed_base, gain = min(candidates, key=lambda item: item[0])
        if score > 0.60:
            raise RuntimeError(f"tracking phase response gain is implausible: {gain:.3f}")
        return {
            "target_phase_degrees": target,
            "signed_base_error_degrees": signed_base,
            "response_gain": gain,
            "response_score": score,
        }

    def select_final_mode(
        target_phase: float,
    ) -> tuple[str, dict[str, object] | None, dict[str, object] | None, float]:
        final_mode = "DIAG"
        mode_verification: dict[str, object] | None = None
        double_calibration: dict[str, object] | None = None
        if question_number == 2:
            final_mode = "CIRCLE"
            command_ok("TRACK CIRCLE", "OK TRACK")
            wait_for_lock()
            mode_verification = observe("tracking_circle_verify")
            circle_error = abs(float(mode_verification["error_degrees"]) - 90.0)
            mode_verification["target_error_degrees"] = circle_error
            if circle_error > TRACK_Q2_VERIFY_ERROR_DEGREES:
                raise RuntimeError(
                    f"CIRCLE phase verification error is {circle_error:.2f} degrees"
                )
        elif question_number == 3:
            final_mode = "DOUBLE"
            command_ok("IDLE", "OK IDLE")
            time.sleep(TRACK_CLEAR_OUTPUT_S)
            double_phase = set_phase(uart, target_phase * 2.0)
            command_ok("TRACK DOUBLE", "OK TRACK")
            wait_for_lock()
            now = time.monotonic()
            double_servo = Q5PhaseFeedForward(
                uart=uart,
                reference_phase_degrees=double_phase,
                compensation_rate_degrees_per_s=0.0,
                reference_time_s=now,
                last_sent_millidegrees=int(round(double_phase * 1000.0)) % 360000,
                next_due_s=float("inf"),
                next_visual_due_s=float("inf"),
            )
            double_calibration = calibrate_double_phase(
                double_servo, camera, corners, blank, diagnostic_dir
            )
            if double_calibration["status"] == "unreliable_restored":
                raise RuntimeError("DOUBLE self-intersection calibration was unreliable")
            target_phase = double_servo.phase_at()
        return final_mode, mode_verification, double_calibration, target_phase

    seed_phase, lock_status = apply_tracking_phase(DEFAULT_STATIC_PHASE_DEGREES)
    base = observe("tracking_base")
    if alignment_acceptable(base):
        final_mode, mode_verification, double_calibration, target_phase = (
            select_final_mode(seed_phase)
        )
        return {
            "status": "calibrated",
            "question_number": question_number,
            "mode": final_mode,
            "lock_status": lock_status,
            "base": base,
            "probe": None,
            "solution": {
                "strategy": "accepted_initial_alignment",
                "target_phase_degrees": seed_phase,
            },
            "refinement": None,
            "final": base,
            "mode_verification": mode_verification,
            "double_calibration": double_calibration,
            "phase_degrees": target_phase,
            "elapsed_s": time.monotonic() - started_s,
        }
    probe_step = TRACK_PROBE_DEGREES
    apply_tracking_phase(seed_phase + probe_step)
    probe = observe_after_change(
        "tracking_probe", base, MIN_PROBE_RESPONSE_DEGREES
    )
    if abs(float(probe["error_degrees"]) - float(base["error_degrees"])) < MIN_PROBE_RESPONSE_DEGREES:
        probe_step = TRACK_SECONDARY_PROBE_DEGREES
        apply_tracking_phase(seed_phase + probe_step)
        probe = observe_after_change(
            "tracking_secondary_probe", base, MIN_PROBE_RESPONSE_DEGREES
        )
    if abs(float(probe["error_degrees"]) - float(base["error_degrees"])) < MIN_PROBE_RESPONSE_DEGREES:
        raise RuntimeError("tracking phase probe did not produce a fresh visible response")
    solution = solve_target(
        seed_phase,
        float(base["error_degrees"]),
        float(probe["error_degrees"]),
        probe_step,
    )
    target_phase, _ = apply_tracking_phase(solution["target_phase_degrees"])
    final = observe_after_change("tracking_final", probe)

    if (
        float(final["error_degrees"])
        >= float(base["error_degrees"]) - TRACK_MODEL_MIN_IMPROVEMENT_DEGREES
    ):
        # The camera reports phase magnitude, so a noisy 90-degree probe can
        # select the wrong signed branch. Do not refine around a model target
        # that is demonstrably no better than the observed seed phase.
        solution["rejected_target_phase_degrees"] = target_phase
        solution["rejected_target_error_degrees"] = float(final["error_degrees"])
        solution["strategy"] = "fallback_to_observed_seed"
        target_phase = seed_phase
        final = base

    refinement: dict[str, object] | None = None
    if not alignment_acceptable(final):
        # The ellipse gives |phase error|. Near 0/180 degrees a small probe can
        # cross that unsigned branch and make its apparent gain meaningless.
        # Test both residual directions and keep the visibly straighter trace.
        correction = float(np.clip(
            float(final["error_degrees"]) / abs(float(solution["response_gain"])),
            0.0,
            TRACK_FINE_MAX_DEGREES,
        ))
        refinement = {
            "correction_degrees": correction,
            "initial": final,
            "candidates": [],
        }
        best_phase = target_phase
        best_observation = final
        current_phase = target_phase
        previous_observation = final
        for index, direction in enumerate((-1.0, 1.0), start=1):
            current_phase, _ = apply_tracking_phase(
                target_phase + direction * correction
            )
            candidate = observe_after_change(
                f"tracking_fine_candidate_{index}", previous_observation
            )
            refinement["candidates"].append({
                "direction": direction,
                "phase_degrees": current_phase,
                "observation": candidate,
            })
            if float(candidate["error_degrees"]) < float(best_observation["error_degrees"]):
                best_phase = current_phase
                best_observation = candidate
            if alignment_acceptable(candidate):
                break
            previous_observation = candidate

        best_error = float(best_observation["error_degrees"])
        best_offset = wrap_signed(best_phase - target_phase)
        if (
            TRACK_ACCEPT_ERROR_DEGREES < best_error
            <= TRACK_FINAL_REFINE_LIMIT_DEGREES
            and abs(best_offset) > 0.001
        ):
            # The symmetric pair identifies which side approaches zero. Use
            # the measured probe gain for one bounded extrapolation instead
            # of merely restoring the still-marginal best candidate.
            final_step = float(np.clip(
                best_error / abs(float(solution["response_gain"])),
                0.0,
                TRACK_FINAL_REFINE_MAX_DEGREES,
            ))
            final_direction = 1.0 if best_offset > 0.0 else -1.0
            final_phase, _ = apply_tracking_phase(
                best_phase + final_direction * final_step
            )
            final_candidate = observe_after_change(
                "tracking_final_refine", previous_observation
            )
            refinement["final_refinement"] = {
                "direction": final_direction,
                "correction_degrees": final_step,
                "phase_degrees": final_phase,
                "observation": final_candidate,
            }
            current_phase = final_phase
            if float(final_candidate["error_degrees"]) < best_error:
                best_phase = final_phase
                best_observation = final_candidate
            else:
                # The already-observed best remains valid; restore its phase
                # without spending another camera acquisition interval.
                target_phase, _ = apply_tracking_phase(best_phase)
                current_phase = target_phase

        if abs(wrap_signed(current_phase - best_phase)) > 0.001:
            target_phase, _ = apply_tracking_phase(best_phase)
            final = observe("tracking_fine_best")
        else:
            target_phase = best_phase
            final = best_observation
        refinement["selected_phase_degrees"] = target_phase
        refinement["final"] = final

    if not alignment_acceptable(final):
        candidate_errors = []
        if refinement is not None:
            candidate_errors = [
                round(float(item["observation"]["error_degrees"]), 2)
                for item in refinement["candidates"]
            ]
        raise RuntimeError(
            "tracking phase residual remains "
            f"{float(final['error_degrees']):.2f} degrees "
            f"(base={float(base['error_degrees']):.2f}, "
            f"probe={float(probe['error_degrees']):.2f}, "
            f"gain={float(solution['response_gain']):.3f}, "
            f"candidates={candidate_errors})"
        )

    final_mode, mode_verification, double_calibration, target_phase = (
        select_final_mode(target_phase)
    )

    return {
        "status": "calibrated",
        "question_number": question_number,
        "mode": final_mode,
        "lock_status": lock_status,
        "base": base,
        "probe": probe,
        "solution": solution,
        "refinement": refinement,
        "final": final,
        "mode_verification": mode_verification,
        "double_calibration": double_calibration,
        "phase_degrees": target_phase,
        "elapsed_s": time.monotonic() - started_s,
    }


def start_q5_phase_feedforward(
    uart: Uart,
    camera: Camera,
    corners: np.ndarray,
    blank: np.ndarray,
    initial_lock: dict[str, object],
    diagnostic_dir: Path | None = None,
) -> tuple[Q5PhaseFeedForward, dict[str, object]]:
    """Resolve XY phase sign with one 90-degree probe and start feed-forward."""
    base_phase = float(initial_lock["phase_degrees"])

    def observe(
        phase_degrees: float,
        label: str,
        previous_error: float | None = None,
        min_response_degrees: float = MIN_PROBE_RESPONSE_DEGREES,
        expected_errors: tuple[float, ...] | None = None,
        max_observation_s: float = MAX_OBSERVATION_S,
        min_observation_s: float = MIN_OBSERVATION_S,
        expected_tolerance_degrees: float = 20.0,
        set_command: bool = True,
        service: Q5PhaseFeedForward | None = None,
    ) -> dict[str, object]:
        phase = set_phase(uart, phase_degrees) if set_command else phase_degrees % 360.0
        set_at_s = time.monotonic()
        deadline_s = set_at_s + max_observation_s
        samples: list[tuple[float, float, dict[str, float]]] = []
        accepted = False
        while time.monotonic() < deadline_s:
            if service is not None and time.monotonic() >= service.next_due_s:
                service.step()
            try:
                error, features = capture_phase_error(camera, corners, blank)
            except RuntimeError:
                continue
            captured_at_s = time.monotonic()
            samples.append((captured_at_s, error, features))
            if captured_at_s - set_at_s < min_observation_s:
                continue
            if len(samples) < OBSERVATION_STABLE_FRAMES:
                continue
            recent = samples[-OBSERVATION_STABLE_FRAMES:]
            recent_errors = [sample[1] for sample in recent]
            stable = max(recent_errors) - min(recent_errors) <= OBSERVATION_STABILITY_DEGREES
            median_error = float(np.median(recent_errors))
            if expected_errors is not None:
                changed = (
                    min(abs(median_error - expected) for expected in expected_errors)
                    <= expected_tolerance_degrees
                )
            else:
                changed = previous_error is None or abs(median_error - previous_error) >= min_response_degrees
            if stable and changed:
                accepted = True
                break
        if not samples:
            raise RuntimeError(f"no valid phase frame for {label}")
        recent = samples[-min(OBSERVATION_STABLE_FRAMES, len(samples)):]
        error = float(np.median([sample[1] for sample in recent]))
        captured_at_s, _, features = recent[-1]
        if diagnostic_dir is not None:
            capture_phase_error(
                camera, corners, blank, diagnostic_dir, label
            )
        return {
            "phase_degrees": phase,
            "set_at_s": set_at_s,
            "captured_at_s": captured_at_s,
            "error_degrees": error,
            "features": features,
            "stable": accepted,
            "frames_examined": len(samples),
        }

    frequency_hz = int(initial_lock.get("frequency_hz", 0))
    calibration_model: dict[str, float | int] | None = None
    reference_calibration = initial_lock.get("reference_calibration")
    if isinstance(reference_calibration, dict):
        calibration_ticks = int(reference_calibration.get("ticks", 0))
        if bool(reference_calibration.get("valid")) and calibration_ticks > 0:
            calibration_model = compensation_from_calibration(
                frequency_hz, calibration_ticks
            )

    # Every allowed Q5 point gets the same visual frequency-trim pass. The
    # 100 kHz reference supplies the initial DDS word; these settled frames
    # measure the remaining error against the actual, unconnected test signal.
    measure_visual_drift = True
    observations = [observe(base_phase, "model_base")]
    if measure_visual_drift:
        for index in range(4):
            observations.append(observe(
                base_phase,
                f"model_drift_{index}",
                set_command=False,
            ))

    origin_s = float(observations[0]["captured_at_s"])
    times = np.asarray([
        float(sample["captured_at_s"]) - origin_s for sample in observations
    ])
    errors = np.asarray([float(sample["error_degrees"]) for sample in observations])
    if measure_visual_drift:
        slope, intercept = np.polyfit(times[1:], errors[1:], 1)
        fitted = np.polyval((slope, intercept), times[1:])
        slope_rms = float(np.sqrt(np.mean((errors[1:] - fitted) ** 2)))
        sample_interval_s = float(times[-1] - times[1])
        segment_slopes = np.diff(errors[1:]) / np.diff(times[1:])
        segment_slope_spread = float(np.ptp(segment_slopes))
        measured_drift_rate = abs(float(slope))
        drift_rate_reliable = (
            sample_interval_s >= 1.20
            and slope_rms <= 1.50
            and segment_slope_spread <= 10.0
            and measured_drift_rate <= 25.0
        )
    else:
        slope = 0.0
        slope_rms = 0.0
        sample_interval_s = 0.0
        segment_slope_spread = 0.0
        measured_drift_rate = 0.0
        drift_rate_reliable = True

    last_error = float(errors[-1])

    def wrap_signed(degrees: float) -> float:
        return (degrees + 180.0) % 360.0 - 180.0

    # A static XY ellipse cannot distinguish +phi from -phi. Shift the FPGA
    # output by +90 degrees once; removing that known shift resolves the sign.
    sign_probe_expected_errors = tuple(
        abs(wrap_signed(signed_base + 90.0))
        for signed_base in (last_error, -last_error)
    )
    probe = observe(
        base_phase + 90.0,
        "sign_probe",
        expected_errors=sign_probe_expected_errors,
        max_observation_s=1.60,
    )
    probe_error = float(probe["error_degrees"])
    probe_output_candidates = (probe_error, -probe_error)
    elapsed_to_probe_s = (
        float(probe["captured_at_s"]) - float(observations[-1]["captured_at_s"])
    )
    expected_unsigned = float(np.clip(
        last_error + float(slope) * elapsed_to_probe_s, 0.0, 180.0
    ))
    signed_candidates = [
        (wrap_signed(output_error - 90.0), output_error)
        for output_error in probe_output_candidates
    ]
    probe_base_signed, probe_output_signed = min(
        signed_candidates,
        key=lambda candidate: abs(abs(candidate[0]) - expected_unsigned),
    )
    phase_sign = 0.0 if abs(probe_base_signed) < 1e-6 else float(np.sign(probe_base_signed))

    # Reconstruct a continuous signed phase history backwards from the known
    # +90-degree probe branch, then use a Theil-Sen-style median of pairwise
    # slopes. This remains stable when the ellipse angle folds at 0/180 degrees.
    unwrapped_errors = np.empty_like(errors)
    next_signed_error = probe_base_signed
    for index in range(len(errors) - 1, -1, -1):
        magnitude = float(errors[index])
        phase_candidates = [
            sign * magnitude + 360.0 * turn
            for sign in (1.0, -1.0)
            for turn in (-1.0, 0.0, 1.0)
        ]
        selected_error = min(
            phase_candidates,
            key=lambda candidate: abs(candidate - next_signed_error),
        )
        unwrapped_errors[index] = selected_error
        next_signed_error = selected_error

    fit_times = times[1:]
    fit_errors = unwrapped_errors[1:]
    pairwise_slopes = [
        float((fit_errors[right] - fit_errors[left]) /
              (fit_times[right] - fit_times[left]))
        for left in range(len(fit_times) - 1)
        for right in range(left + 1, len(fit_times))
        if fit_times[right] > fit_times[left]
    ]
    signed_visual_drift_rate = float(np.median(pairwise_slopes))
    robust_intercept = float(np.median(fit_errors - signed_visual_drift_rate * fit_times))
    fitted_errors = signed_visual_drift_rate * fit_times + robust_intercept
    slope_rms = float(np.sqrt(np.mean((fit_errors - fitted_errors) ** 2)))
    segment_slopes = np.diff(fit_errors) / np.diff(fit_times)
    segment_slope_spread = float(np.ptp(segment_slopes))
    sample_interval_s = float(fit_times[-1] - fit_times[0])
    measured_drift_rate = abs(signed_visual_drift_rate)
    drift_rate_reliable = (
        sample_interval_s >= 1.20
        and slope_rms <= 1.50
        and segment_slope_spread <= 10.0
        and measured_drift_rate <= 25.0
    )
    slope = signed_visual_drift_rate

    calibration_visual_difference = 0.0
    calibration_confirmed = calibration_model is not None
    if calibration_model is not None:
        calibration_physical_drift = -float(
            calibration_model["compensation_rate_degrees_per_s"]
        )
        if measure_visual_drift:
            calibration_visual_difference = abs(
                calibration_physical_drift - signed_visual_drift_rate
            )
    if calibration_model is None:
        raise RuntimeError("100 kHz reference calibration is required before Q5")

    trim_word = 0
    trim_reply = "not applied"
    trim_phase_rate = 0.0
    trim_applied_s: float | None = None
    if drift_rate_reliable:
        calibrated_clock_hz = (
            calibration_ticks * REFERENCE_CALIBRATION_HZ
            / REFERENCE_CALIBRATION_PERIODS
        )
        trim_word = int(round(
            -signed_visual_drift_rate / 360.0
            * PHASE_ACCUMULATOR_MODULUS / calibrated_clock_hz
        ))
        trim_word = int(np.clip(trim_word, -1_000_000, 1_000_000))
        trim_reply = uart.command(f"TRIMQ {trim_word}")
        if not trim_reply.startswith("OK TRIMQ"):
            raise RuntimeError(f"DDS frequency trim failed: {trim_reply!r}")
        trim_applied_s = time.monotonic()
        trim_phase_rate = (
            trim_word * calibrated_clock_hz
            / PHASE_ACCUMULATOR_MODULUS * 360.0
        )
        physical_drift_rate = signed_visual_drift_rate + trim_phase_rate
        drift_rate_source = "visual_40bit_dds_trim"
    else:
        physical_drift_rate = -float(
            calibration_model["compensation_rate_degrees_per_s"]
        )
        drift_rate_source = "reference_calibration_ticks_fallback"

    candidate_timing: dict[str, float] = {}

    def try_candidate(
        sign: float, label: str
    ) -> tuple[Q5PhaseFeedForward, dict[str, object], float, float]:
        # `sign` resolves only the +/- static ellipse ambiguity. Frequency
        # drift direction is a property of the hardware clocks and must never
        # flip when the opposite static phase candidate is tried.
        candidate_physical_drift = physical_drift_rate
        compensation_rate = -candidate_physical_drift
        now = time.monotonic()
        if trim_applied_s is not None:
            pretrim_age_s = max(
                0.0,
                trim_applied_s - float(probe["captured_at_s"])
                + MEASURED_DISPLAY_LATENCY_S,
            )
            posttrim_age_s = max(0.0, now - trim_applied_s)
            delay_phase = (
                signed_visual_drift_rate * pretrim_age_s
                + candidate_physical_drift * posttrim_age_s
            )
        else:
            pretrim_age_s = 0.0
            posttrim_age_s = (
                now - float(probe["captured_at_s"])
                + MEASURED_DISPLAY_LATENCY_S
            )
            delay_phase = candidate_physical_drift * posttrim_age_s
        predicted_error = sign * probe_output_signed + delay_phase
        candidate_timing.update({
            "pretrim_delay_s": pretrim_age_s,
            "posttrim_delay_s": posttrim_age_s,
            "delay_phase_degrees": delay_phase,
        })
        target_phase = set_phase(
            uart, float(probe["phase_degrees"]) - predicted_error
        )
        reference_time_s = time.monotonic()
        servo = Q5PhaseFeedForward(
            uart=uart,
            reference_phase_degrees=target_phase,
            compensation_rate_degrees_per_s=compensation_rate,
            reference_time_s=reference_time_s,
            last_sent_millidegrees=int(round(target_phase * 1000.0)) % 360000,
            next_due_s=reference_time_s + PHASE_UPDATE_INTERVAL_S,
        )
        sample = observe(
            target_phase,
            label,
            expected_errors=(0.0,),
            max_observation_s=1.20,
            set_command=False,
            service=servo,
        )
        return servo, sample, predicted_error, candidate_physical_drift

    # The accepted +90-degree probe already resolves the ellipse's +/- phase
    # ambiguity. A second opposite coarse command only repeats a solved branch;
    # the signed +30-degree fine probe below corrects any remaining residual.
    candidates = [try_candidate(1.0, "final_primary")]
    feedforward, final, predicted_error, selected_physical_drift = candidates[0]

    fine_candidates: list[dict[str, object]] = []
    fine_probe_report: dict[str, object] | None = None
    coarse_error = float(final["error_degrees"])
    if coarse_error > FINE_LOCK_ERROR_DEGREES:
        coarse_final = final
        coarse_reference_phase = feedforward.reference_phase_degrees
        coarse_reference_time_s = feedforward.reference_time_s
        candidates_with_reference = [
            (coarse_final, coarse_reference_phase, coarse_reference_time_s, 0.0)
        ]
        errors: list[str] = []

        # A zero-phase ellipse gives only |phi|. Test the two physically
        # possible corrections around the same compensated reference and keep
        # the better live result. This avoids deriving the fine-correction sign
        # from a stale frame after the scope refreshes.
        for direction in (-1.0, 1.0):
            try:
                now = time.monotonic()
                compensated_base = (
                    coarse_reference_phase
                    + feedforward.compensation_rate_degrees_per_s
                    * (now - coarse_reference_time_s)
                )
                candidate_phase = set_phase(
                    uart, compensated_base + direction * coarse_error
                )
                reference_time_s = time.monotonic()
                feedforward.reference_phase_degrees = candidate_phase
                feedforward.reference_time_s = reference_time_s
                feedforward.last_sent_millidegrees = (
                    int(round(candidate_phase * 1000.0)) % 360000
                )
                feedforward.next_due_s = reference_time_s + PHASE_UPDATE_INTERVAL_S
                sample = observe(
                    candidate_phase,
                    "final_refine_minus" if direction < 0.0 else "final_refine_plus",
                    expected_errors=(0.0,),
                    max_observation_s=ZERO_REFINE_MAX_OBSERVATION_S,
                    min_observation_s=ZERO_REFINE_MIN_OBSERVATION_S,
                    expected_tolerance_degrees=ZERO_REFINE_ACCEPT_DEGREES,
                    set_command=False,
                    service=feedforward,
                )
                fine_candidates.append(sample)
                candidates_with_reference.append(
                    (sample, candidate_phase, reference_time_s, direction * coarse_error)
                )
                if float(sample["error_degrees"]) <= ZERO_REFINE_ACCEPT_DEGREES:
                    break
            except RuntimeError as fine_error:
                errors.append(str(fine_error))

        best_sample, best_phase, best_time_s, best_delta = min(
            candidates_with_reference,
            key=lambda candidate: float(candidate[0]["error_degrees"]),
        )
        final = best_sample
        if (
            feedforward.reference_phase_degrees != best_phase
            or feedforward.reference_time_s != best_time_s
        ):
            now = time.monotonic()
            best_phase = set_phase(
                uart,
                best_phase
                + feedforward.compensation_rate_degrees_per_s * (now - best_time_s),
            )
            best_time_s = time.monotonic()
            feedforward.reference_phase_degrees = best_phase
            feedforward.reference_time_s = best_time_s
            feedforward.last_sent_millidegrees = (
                int(round(best_phase * 1000.0)) % 360000
            )
        fine_probe_report = {
            "strategy": "symmetric_live_candidates",
            "coarse_error_degrees": coarse_error,
            "selected_delta_degrees": best_delta,
            "selected_error_degrees": float(final["error_degrees"]),
            "attempt_errors": errors,
        }

    feedforward.next_due_s = 0.0
    feedforward.step()

    reference_error = float(final["error_degrees"])
    static_accepted = reference_error <= LOCK_ACCEPT_ERROR_DEGREES
    # The camera establishes static phase once. Continuous unsigned visual
    # corrections can reverse direction near zero and must not perturb the
    # calibrated 40-bit DDS after the initial lock.
    feedforward.next_visual_due_s = float("inf")
    feedforward.visual_previous_error_degrees = reference_error
    feedforward.visual_direction = -1.0 if predicted_error > 0.0 else 1.0
    feedforward.visual_active = reference_error > VISUAL_PHASE_DEADBAND_DEGREES
    reference_phase = feedforward.phase_at()
    return feedforward, {
        "status": "running",
        "phase_degrees": reference_phase,
        "residual_error_degrees": reference_error,
        "compensation_rate_degrees_per_s": feedforward.compensation_rate_degrees_per_s,
        "raw_compensation_rate_degrees_per_s": measured_drift_rate,
        "signed_visual_drift_rate_degrees_per_s": signed_visual_drift_rate,
        "drift_rate_source": drift_rate_source,
        "frequency_trim": {
            "word_delta": trim_word,
            "mcu_reply": trim_reply,
            "applied_phase_rate_degrees_per_s": trim_phase_rate,
            "predicted_residual_rate_degrees_per_s": physical_drift_rate,
        },
        "reference_calibration_model": calibration_model,
        "calibration_visual_difference_degrees_per_s": (
            calibration_visual_difference
        ),
        "calibration_confirmed": calibration_confirmed,
        "drift_rate_reliable": drift_rate_reliable,
        "rate_measured": measure_visual_drift,
        "rate_fit_rms_degrees": slope_rms,
        "rate_segment_spread_degrees_per_s": segment_slope_spread,
        "static_accepted": static_accepted,
        "static_candidate_error_degrees": reference_error,
        "rate_accepted": drift_rate_reliable,
        "rate_sample_interval_s": sample_interval_s,
        "first": {
            "observations": observations,
            "error_slope_degrees_per_s": float(slope),
            "phase_sign": phase_sign,
            "physical_drift_rate_degrees_per_s": selected_physical_drift,
            "display_latency_s": MEASURED_DISPLAY_LATENCY_S,
            "predicted_error_degrees": predicted_error,
            **candidate_timing,
            "sign_probe": probe,
            "probe_base_signed_degrees": probe_base_signed,
            "probe_output_signed_degrees": probe_output_signed,
            "candidates": [candidate[1] for candidate in candidates],
            "fine_candidates": fine_candidates,
            "fine_probe": fine_probe_report,
            "final": final,
        },
        "second": None,
        "features": final["features"],
    }


def run_q5_phase_lock(
    uart: Uart, camera: Camera, corners: np.ndarray, blank: np.ndarray
) -> dict[str, object]:
    """Flush pre-RESULT sweep frames before phase/rate measurements begin."""
    # RESULT changes the FPGA from the probe table to free-run DIAG mode. The
    # first V4L2 frames are still the prior serpentine sweep, not an XY ellipse.
    # Do not use those stale frames to infer a phase direction.
    # start_q5_phase_feedforward continuously drains the display/camera queue
    # and accepts only a stable phase response, so no blind fixed delay is
    # needed here.
    idle_reply = uart.command("IDLE")
    if not idle_reply.startswith("OK IDLE"):
        raise RuntimeError(f"cannot clear the previous Q5 task: {idle_reply!r}")
    time.sleep(Q5_CLEAR_OUTPUT_S)
    final_phase = set_phase(uart, DEFAULT_STATIC_PHASE_DEGREES)
    auto_reply = uart.command("AUTO DIAG")
    if not auto_reply.startswith("OK AUTO"):
        raise RuntimeError(f"cannot restart the Q5 output: {auto_reply!r}")
    trim_reply = uart.command("TRIMQ 0")
    if not trim_reply.startswith("OK TRIMQ"):
        raise RuntimeError(f"cannot clear the previous DDS trim: {trim_reply!r}")
    return {
        "status": "seeded",
        "phase_degrees": final_phase,
        "residual_error_degrees": None,
        "history": [],
        "samples": 0,
    }
