"""Q5 visual lookup for the three frequencies specified by the contest."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from q5_fpga_sweep import (
    CALIBRATION_FRAMES,
    CAMERA_PIPELINE_FLUSH_FRAMES,
    XY_AXIS_CENTER_HINT,
    AxisCalibration,
    Camera,
    Uart,
    calibrate_xy,
    largest_active_span,
    median_frame,
    rectify,
    rolling_median,
    write_image,
)


TABLE_RAMPS_US = (
    10, 12, 13, 16, 18, 21, 24, 28, 33, 38, 44, 51, 59, 69, 80, 93,
    108, 125, 145, 168, 195, 226, 263, 305, 353, 410, 476, 552, 640,
    743, 862, 1000, 2000, 6000,
)
KNOWN_FREQUENCIES_HZ = (1100, 49900, 90900)

# At 125 us the three allowed inputs produce approximately 0.13, 6.3, and
# 11.5 midline crossings. This keeps 90.9 kHz below the dense-trace regime
# while retaining enough crossings to separate it from 49.9 kHz. The sparse
# 1.1 kHz candidate is always confirmed with the 6 ms low-frequency ramp.
CLASSIFY_SLOT = 17
LOW_MID_CROSSING_BOUNDARY = 3.5
# Thick scope traces merge some adjacent crossings, so use a slightly lower
# boundary than the theoretical midpoint of 8.94 crossings.
MID_HIGH_CROSSING_BOUNDARY = 8.0

# Each candidate gets a dedicated ramp that yields about six crossings. This
# rejects a stale or partially settled classification capture.
CONFIRMATION_SLOTS = {
    1100: 33,   # 6000 us: about 6.0 crossings
    49900: 17,  # 125 us: about 6.3 crossings
    90900: 13,  # 69 us: about 6.4 crossings
}

NORMAL_SETTLE_S = 0.50
POST_STEP_FLUSH_FRAMES = 3
POST_STEP_CANDIDATE_FRAMES = 5
POST_STEP_CONFIRMATION_MAX_FRAMES = 12
# Rebuilding the blank/axis cache blocks the key path for roughly 2-3 seconds.
# Scope geometry and manual exposure are fixed during a contest run, so keep
# the startup calibration for the lifetime of the resident service.
CACHED_BLANK_MAX_AGE_S = float("inf")
MIN_XY_CALIBRATION_CONFIDENCE = 0.50
IDLE_XY_CALIBRATION_MAX_ATTEMPTS = 6
IDLE_XY_CALIBRATION_RETRY_S = 0.25
# The Hough axes can be geometrically correct while a small camera/scope shift
# lowers the legacy center-hint score. Three consistent detections are safer
# than either accepting one weak frame or restarting the whole service.
IDLE_XY_STABLE_DETECTIONS = 3
IDLE_XY_FALLBACK_MIN_CONFIDENCE = 0.28
IDLE_XY_FALLBACK_MAX_CENTER_OFFSET_PX = 18.0
IDLE_XY_FALLBACK_MAX_CENTER_SPREAD_PX = 3.0
IDLE_XY_FALLBACK_MIN_AXIS_LENGTH_PX = 150.0
IDLE_XY_FALLBACK_MIN_ORTHOGONALITY = 0.97
MIN_TRACE_SPAN_PX = 220
FULL_TRACE_SPAN_PX = 380.0
TRACE_CONNECT_THRESHOLD = 18
# Rectified XY traces progress primarily along image rows. This narrow kernel
# closes short raster gaps without joining adjacent sine periods sideways.
TRACE_CONNECT_KERNEL = (3, 7)
MAX_ATTEMPTS = 2
INTER_ATTEMPT_IDLE_S = 0.35
DENSE_HIGH_OCCUPANCY = 0.55
DENSE_HIGH_BAND_COUNT = 8.0
MID_FREQUENCY_BAND_COUNT = 4.0

# Existing bench calibration maps visual cycles back to electrical frequency.
# It is used only to predict an allowed candidate's crossing-count window,
# never to produce a continuous-frequency result.
VISUAL_FREQUENCY_GAIN = 1.965686502
VISUAL_FREQUENCY_OFFSET_HZ = 111.135141


@dataclass
class Capture:
    slot: int
    bgr: np.ndarray
    xy: np.ndarray


@dataclass
class IdleCache:
    """XY geometry and an IDLE image captured by the resident service."""

    calibration_image: np.ndarray
    corners: np.ndarray
    blank: np.ndarray
    captured_at_s: float


def capture_slot(
    camera: Camera,
    uart: Uart,
    slot: int,
    corners: np.ndarray,
    blank: np.ndarray,
    target_crossings: float | None = None,
) -> Capture:
    """Capture a settled STEP frame, retaining the most complete trace.

    The scope can expose an occasional blank or partially refreshed raster
    immediately after a table change. A short burst costs only four extra
    camera periods and avoids allowing one such frame to reject the lookup.
    """
    frame_period = 1.0 / max(camera.fps, 1.0)
    for _ in range(max(CAMERA_PIPELINE_FLUSH_FRAMES, 4)):
        time.sleep(frame_period)
        camera.capture()
    reply = uart.command(f"STEP {slot}")
    if not reply.startswith("OK STEP"):
        raise RuntimeError(f"STEP {slot} failed: {reply!r}")
    time.sleep(NORMAL_SETTLE_S)
    for _ in range(POST_STEP_FLUSH_FRAMES):
        time.sleep(frame_period)
        camera.capture()

    best_capture: Capture | None = None
    best_score: tuple[float, ...] | None = None
    confirmation_matches = 0
    frame_limit = (
        POST_STEP_CONFIRMATION_MAX_FRAMES
        if target_crossings is not None else POST_STEP_CANDIDATE_FRAMES
    )
    for _ in range(frame_limit):
        time.sleep(frame_period)
        frame = camera.capture().bgr
        candidate = Capture(slot, frame, rectify(frame, corners))
        features = trace_features(
            cv2.subtract(candidate.xy, blank), TABLE_RAMPS_US[slot]
        )
        if target_crossings is None:
            # The low-frequency classes deliberately have fewer than five
            # crossings and do not pass the sinusoid quality gate. Complete
            # scan coverage must rank ahead of that gate so a stale partial
            # high-frequency trace cannot win.
            score = (
                float(int(features["trace_span_px"]) >= MIN_TRACE_SPAN_PX),
                float(features["trace_span_px"]),
                float(bool(features["accepted"])),
                float(features["amplitude_px"]),
            )
        else:
            # A prior setting can remain visible on the scope for one camera
            # frame. For confirmation, prefer a complete trace whose crossing
            # count belongs to the requested setting rather than that residue.
            score = (
                float(bool(features["accepted"])),
                float(int(features["trace_span_px"]) >= MIN_TRACE_SPAN_PX),
                -abs(projected_crossings(features) - target_crossings),
                float(features["trace_span_px"]),
                float(features["amplitude_px"]),
            )
        if best_score is None or score > best_score:
            best_capture = candidate
            best_score = score
        if target_crossings is not None:
            projected = projected_crossings(features)
            response_window = max(1.25, target_crossings * 0.25)
            if (
                bool(features["accepted"])
                and int(features["trace_span_px"]) >= MIN_TRACE_SPAN_PX
                and abs(projected - target_crossings) <= response_window
            ):
                confirmation_matches += 1
                if confirmation_matches >= 2:
                    break
            else:
                confirmation_matches = 0

    if best_capture is None:
        raise RuntimeError("camera returned no candidate frame")
    return best_capture


def idle_calibration_is_stable(
    candidates: list[tuple[np.ndarray, AxisCalibration]],
) -> bool:
    """Accept repeated strong axis geometry despite a stale center hint."""
    if len(candidates) < IDLE_XY_STABLE_DETECTIONS:
        return False
    recent = candidates[-IDLE_XY_STABLE_DETECTIONS:]
    calibrations = [candidate[1] for candidate in recent]
    centers = np.asarray([calibration.center for calibration in calibrations])
    median_center = np.median(centers, axis=0)
    center_spread = float(np.max(np.linalg.norm(centers - median_center, axis=1)))
    for calibration in calibrations:
        horizontal = calibration.horizontal.astype(float)
        vertical = calibration.vertical.astype(float)
        horizontal_direction = horizontal[2:] - horizontal[:2]
        vertical_direction = vertical[2:] - vertical[:2]
        horizontal_length = float(np.linalg.norm(horizontal_direction))
        vertical_length = float(np.linalg.norm(vertical_direction))
        orthogonality = abs(float(
            horizontal_direction[0] * vertical_direction[1]
            - horizontal_direction[1] * vertical_direction[0]
        )) / (horizontal_length * vertical_length)
        center_offset = float(np.linalg.norm(calibration.center - median_center))
        if (
            calibration.confidence < IDLE_XY_FALLBACK_MIN_CONFIDENCE
            or center_offset > IDLE_XY_FALLBACK_MAX_CENTER_SPREAD_PX
            or horizontal_length < IDLE_XY_FALLBACK_MIN_AXIS_LENGTH_PX
            or vertical_length < IDLE_XY_FALLBACK_MIN_AXIS_LENGTH_PX
            or orthogonality < IDLE_XY_FALLBACK_MIN_ORTHOGONALITY
        ):
            return False
    # Keep the repeated pair close enough to the known oscilloscope plot that
    # adjacent grid lines cannot become a stable but incorrect fallback.
    return float(np.linalg.norm(median_center - XY_AXIS_CENTER_HINT)) <= (
        IDLE_XY_FALLBACK_MAX_CENTER_OFFSET_PX
    ) and center_spread <= IDLE_XY_FALLBACK_MAX_CENTER_SPREAD_PX


def idle_cache_from_calibration(
    calibration_image: np.ndarray, calibration: AxisCalibration,
) -> IdleCache:
    return IdleCache(
        calibration_image=calibration_image,
        corners=calibration.corners,
        blank=rectify(calibration_image, calibration.corners),
        captured_at_s=time.monotonic(),
    )


def prepare_idle_cache(
    uart: Uart, camera: Camera, retry_until_valid: bool = False,
) -> IdleCache:
    if not uart.command("IDLE").startswith("OK IDLE"):
        raise RuntimeError("cannot park FPGA before XY calibration")
    # The V4L2 queue may still contain frames captured before IDLE reached the
    # FPGA, and the scope phosphor needs time to clear the prior Lissajous
    # trace. A contaminated blank turns phase subtraction into the difference
    # of two ellipses, so discard the complete old camera pipeline first.
    time.sleep(1.50)
    frame_period = 1.0 / max(camera.fps, 1.0)
    for _ in range(12):
        time.sleep(frame_period)
        camera.capture()
    candidates: list[tuple[np.ndarray, AxisCalibration]] = []
    attempt = 0
    last_reason = "no XY calibration frame"
    while retry_until_valid or attempt < IDLE_XY_CALIBRATION_MAX_ATTEMPTS:
        attempt += 1
        frames: list[np.ndarray] = []
        for _ in range(CALIBRATION_FRAMES):
            time.sleep(frame_period)
            frames.append(camera.capture().bgr)
        calibration_image = median_frame(frames)
        try:
            calibration = calibrate_xy(calibration_image)
        except RuntimeError as error:
            last_reason = str(error)
            if attempt <= IDLE_XY_CALIBRATION_MAX_ATTEMPTS or attempt % 10 == 0:
                print(json.dumps({
                    "xy_idle_calibration_attempt": attempt,
                    "accepted": False,
                    "reason": last_reason,
                }), flush=True)
        else:
            candidates.append((calibration_image, calibration))
            candidates = candidates[-IDLE_XY_STABLE_DETECTIONS:]
            print(json.dumps({
                "xy_idle_calibration_attempt": attempt,
                "confidence": calibration.confidence,
                "center": calibration.center.tolist(),
            }), flush=True)
            if calibration.confidence >= MIN_XY_CALIBRATION_CONFIDENCE:
                return idle_cache_from_calibration(calibration_image, calibration)
            last_reason = f"XY calibration confidence too low: {calibration.confidence:.3f}"
            if idle_calibration_is_stable(candidates):
                selected_image, selected_calibration = max(
                    candidates, key=lambda candidate: candidate[1].confidence
                )
                print(json.dumps({
                    "xy_idle_calibration_fallback": "stable repeated axes",
                    "attempts": attempt,
                    "confidence": selected_calibration.confidence,
                    "center": selected_calibration.center.tolist(),
                }), flush=True)
                return idle_cache_from_calibration(
                    selected_image, selected_calibration
                )
        time.sleep(IDLE_XY_CALIBRATION_RETRY_S)
    raise RuntimeError(
        f"XY calibration failed after {attempt} in-process attempts: {last_reason}"
    )


def trace_features(trace: np.ndarray, ramp_us: int) -> dict[str, object]:
    """Return robust crossing features; no continuous frequency is estimated."""
    channels = trace.astype(np.int16)
    blue, green, red = cv2.split(channels)
    cyan_mask = (
        (np.minimum(blue, green) - red >= 20) & (blue >= 50)
    ).astype(np.float32)
    cyan_interior = cyan_mask[12:-12, 12:-12]
    cyan_occupancy = float(np.mean(cyan_interior))
    cyan_response = cv2.GaussianBlur(cyan_mask, (9, 9), 0)
    band_counts: list[int] = []
    for column in (100, 150, 200, 250, 300):
        active = cyan_response[12:-12, column] >= 0.18
        starts = active & ~np.r_[False, active[:-1]]
        band_counts.append(int(np.count_nonzero(starts)))
    cyan_band_count = float(np.median(band_counts))

    gray = cv2.cvtColor(trace, cv2.COLOR_BGR2GRAY)
    threshold = max(TRACE_CONNECT_THRESHOLD, int(gray.max()) * 15 // 100)
    mask = np.where(gray >= threshold, 255, 0).astype(np.uint8)
    connect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, TRACE_CONNECT_KERNEL)
    # Close gaps in one visible trace: dilate first, then erode by the same
    # kernel so the original line width is retained.
    mask = cv2.dilate(mask, connect_kernel, iterations=1)
    mask = cv2.erode(mask, connect_kernel, iterations=1)
    response = cv2.GaussianBlur(mask, (11, 5), 0)
    strength = response.max(axis=1)
    try:
        start, end = largest_active_span(strength >= max(20.0, float(strength.max()) * 0.35))
    except RuntimeError as error:
        return {
            "accepted": False,
            "reason": str(error),
            "ramp_us": ramp_us,
            "crossing_count": 0,
            "trace_span_px": 0,
            "amplitude_px": 0.0,
            "cyan_occupancy": cyan_occupancy,
            "cyan_band_count": cyan_band_count,
        }

    centers = response[start:end].argmax(axis=1).astype(np.float32)
    centers = rolling_median(centers, 9)
    centers = cv2.GaussianBlur(centers.reshape(1, -1), (15, 1), 0).reshape(-1)
    low, high = np.percentile(centers, (5.0, 95.0))
    centered = centers - (low + high) * 0.5
    crossings: list[tuple[float, str]] = []
    minimum_gap = max(12, len(centers) // 40)
    for row in range(len(centered) - 1):
        first, second = float(centered[row]), float(centered[row + 1])
        if not ((first < 0.0 <= second) or (first > 0.0 >= second)):
            continue
        position = row + (-first / (second - first))
        if not crossings or position - crossings[-1][0] >= minimum_gap:
            crossings.append((position, "up" if second > first else "down"))

    up = np.asarray([position for position, direction in crossings if direction == "up"])
    down = np.asarray([position for position, direction in crossings if direction == "down"])
    intervals = np.concatenate((np.diff(up), np.diff(down)))
    result: dict[str, object] = {
        "accepted": False,
        "ramp_us": ramp_us,
        "crossing_count": len(crossings),
        "trace_span_px": int(end - start),
        "amplitude_px": float((high - low) * 0.5),
        "cyan_occupancy": cyan_occupancy,
        "cyan_band_count": cyan_band_count,
    }
    if result["amplitude_px"] < 16.0:
        result["reason"] = "trace amplitude below 16 px"
    elif len(crossings) < 5:
        result["reason"] = "insufficient midline crossings"
    elif any(crossings[index][1] == crossings[index - 1][1] for index in range(1, len(crossings))):
        result["reason"] = "midline crossings do not alternate"
    elif len(intervals) < 1:
        result["reason"] = "not enough full-period intervals"
    else:
        spacing = float(np.median(intervals))
        spacing_cv = float(1.4826 * np.median(np.abs(intervals - spacing)) / spacing)
        result["crossing_spacing_cv"] = spacing_cv
        if spacing_cv > 0.15:
            result["reason"] = "unstable crossing intervals"
        else:
            result["accepted"] = True
    return result


def inspect(capture: Capture, blank: np.ndarray) -> dict[str, object]:
    result = trace_features(cv2.subtract(capture.xy, blank), TABLE_RAMPS_US[capture.slot])
    result["table_index"] = capture.slot
    return result


def projected_crossings(estimate: dict[str, object]) -> float:
    """Project an evenly spaced trace to the normal 380 px scan width."""
    span = max(float(estimate["trace_span_px"]), 1.0)
    return float(estimate["crossing_count"]) * FULL_TRACE_SPAN_PX / span


def crossing_class_errors(estimate: dict[str, object]) -> dict[int, float]:
    """Return crossing-count errors for each allowed frequency."""
    projected = projected_crossings(estimate)
    ramp_us = int(estimate["ramp_us"])
    return {
        frequency_hz: abs(projected - expected_crossings(frequency_hz, ramp_us))
        for frequency_hz in KNOWN_FREQUENCIES_HZ
    }


def stable_crossing_class(estimate: dict[str, object]) -> tuple[int, float] | None:
    """Trust a complete, regular ridge when its nearest class is unambiguous."""
    if (
        not bool(estimate.get("accepted", False))
        or int(estimate.get("trace_span_px", 0)) < 350
        or float(estimate.get("crossing_spacing_cv", 1.0)) > 0.12
    ):
        return None
    errors = crossing_class_errors(estimate)
    ranked = sorted(errors, key=errors.get)
    margin = errors[ranked[1]] - errors[ranked[0]]
    if margin < 1.0:
        return None
    return ranked[0], margin


def classify_estimate(estimate: dict[str, object]) -> int:
    """Select the allowed frequency nearest the observed crossing count."""
    projected = projected_crossings(estimate)
    estimate["projected_crossing_count"] = projected
    ramp_us = int(estimate["ramp_us"])
    expected = {
        frequency_hz: expected_crossings(frequency_hz, ramp_us)
        for frequency_hz in KNOWN_FREQUENCIES_HZ
    }
    estimate["class_expected_crossings"] = expected
    stable_class = stable_crossing_class(estimate)
    # A thick 90.9 kHz ridge may still contain eleven clean, evenly spaced
    # crossings while its cyan band count fluctuates into the mid-frequency
    # fallback window. Prefer that stronger geometric evidence.
    if stable_class is not None and stable_class[0] == KNOWN_FREQUENCIES_HZ[2]:
        estimate["classification_evidence"] = "stable high-frequency crossings"
        estimate["crossing_class_margin"] = stable_class[1]
        return stable_class[0]
    if dense_high_signature(estimate):
        estimate["classification_fallback"] = "dense cyan high-frequency trace"
        return KNOWN_FREQUENCIES_HZ[2]
    if float(estimate.get("cyan_band_count", 0.0)) >= MID_FREQUENCY_BAND_COUNT:
        estimate["classification_fallback"] = "cyan mid-frequency bands"
        return KNOWN_FREQUENCIES_HZ[1]
    if projected < LOW_MID_CROSSING_BOUNDARY:
        return KNOWN_FREQUENCIES_HZ[0]
    if projected < MID_HIGH_CROSSING_BOUNDARY:
        return KNOWN_FREQUENCIES_HZ[1]
    return KNOWN_FREQUENCIES_HZ[2]


def dense_high_signature(estimate: dict[str, object]) -> bool:
    """Recognize a high-frequency trace even when no single ridge exists."""
    return (
        float(estimate.get("cyan_occupancy", 0.0)) >= DENSE_HIGH_OCCUPANCY
        or float(estimate.get("cyan_band_count", 0.0)) >= DENSE_HIGH_BAND_COUNT
    )


def classification_is_decisive(estimate: dict[str, object], candidate_hz: int) -> bool:
    """Skip confirmation only for a complete, high-quality unambiguous trace."""
    projected = projected_crossings(estimate)
    class_errors = crossing_class_errors(estimate)
    errors = sorted(class_errors.values())
    expected = expected_crossings(candidate_hz, int(estimate["ramp_us"]))
    quality_ok = bool(estimate["accepted"])
    return (
        quality_ok
        and int(estimate["trace_span_px"]) >= 350
        # Cyan-band fallbacks must not bypass confirmation when the clean
        # crossings support a different allowed frequency.
        and class_errors[candidate_hz] <= errors[0] + 1e-6
        and errors[0] <= max(1.25, expected * 0.18)
        and errors[1] - errors[0] >= 1.0
    )


def expected_crossings(frequency_hz: int, ramp_us: int) -> float:
    visual_hz = (frequency_hz - VISUAL_FREQUENCY_OFFSET_HZ) / VISUAL_FREQUENCY_GAIN
    return 2.0 * visual_hz * ramp_us * 1e-6


def confirmation_matches(estimate: dict[str, object], frequency_hz: int) -> bool:
    expected = expected_crossings(frequency_hz, int(estimate["ramp_us"]))
    crossing_count = projected_crossings(estimate)
    tolerance = (
        2.5 if frequency_hz == 1100 else max(1.5, expected * 0.25)
    )
    estimate["expected_crossings"] = expected
    estimate["projected_crossing_count"] = crossing_count
    estimate["crossing_tolerance"] = tolerance
    complete_trace = int(estimate["trace_span_px"]) >= MIN_TRACE_SPAN_PX
    if (
        bool(estimate["accepted"])
        and complete_trace
        and abs(crossing_count - expected) <= tolerance
    ):
        return True

    # Thick cyan traces can have the right number of visible bands while the
    # single-ridge extractor reports non-alternating crossings. The candidate
    # ramp keeps these band windows unambiguous: 49.9 kHz at 125 us and
    # 90.9 kHz at 69 us both form roughly 4..7 resolvable bands.
    band_count = float(estimate.get("cyan_band_count", 0.0))
    occupancy = float(estimate.get("cyan_occupancy", 0.0))
    band_fallback = (
        frequency_hz in (49900, 90900)
        and complete_trace
        and float(estimate["amplitude_px"]) >= 16.0
        and occupancy < DENSE_HIGH_OCCUPANCY
        and 4.0 <= band_count < DENSE_HIGH_BAND_COUNT
    )
    if band_fallback:
        estimate["confirmation_fallback"] = "stable cyan band count"
    return band_fallback


def stale_high_frame_confirms(
    classification: dict[str, object], confirmation: dict[str, object]
) -> bool:
    """Accept 90.9 kHz when the scope repeats most of the 125 us high trace."""
    confirmation_high_floor = (
        expected_crossings(90900, int(confirmation["ramp_us"])) + 4.0
    )
    return (
        int(classification["trace_span_px"]) >= MIN_TRACE_SPAN_PX
        and int(confirmation["trace_span_px"]) >= MIN_TRACE_SPAN_PX
        and (
            projected_crossings(classification) >= MID_HIGH_CROSSING_BOUNDARY
            or dense_high_signature(classification)
        )
        # A settled 69 us high-frequency frame has only about 6.4 crossings;
        # 10+ crossings can only be the preceding 125 us display persisting.
        and projected_crossings(confirmation) >= confirmation_high_floor
    )


def archive(output_dir: Path, captures: list[Capture], cache: IdleCache) -> None:
    write_image(output_dir / "xy_axis_calibrated.png", cache.blank)
    for sequence, capture in enumerate(captures):
        stem = f"{sequence:02d}_idx{capture.slot:02d}_{TABLE_RAMPS_US[capture.slot]:04d}us"
        write_image(output_dir / f"{stem}.jpg", capture.bgr)
        write_image(output_dir / f"{stem}_xy.png", capture.xy)


def run_frequency_measurement(
    output_dir: Path,
    uart: Uart,
    camera: Camera,
    save_images: bool = False,
    idle_cache: IdleCache | None = None,
) -> dict[str, object]:
    """Classify the input into one of the three permitted frequencies."""
    started = time.monotonic()
    if idle_cache is None or time.monotonic() - idle_cache.captured_at_s > CACHED_BLANK_MAX_AGE_S:
        idle_cache = prepare_idle_cache(uart, camera)

    all_captures: list[Capture] = []
    attempts: list[dict[str, object]] = []
    result_hz: int | None = None
    for attempt in range(1, MAX_ATTEMPTS + 1):
        classification_capture = capture_slot(
            camera, uart, CLASSIFY_SLOT, idle_cache.corners, idle_cache.blank
        )
        all_captures.append(classification_capture)
        classification = inspect(classification_capture, idle_cache.blank)
        candidate_hz = classify_estimate(classification)
        if classification_is_decisive(classification, candidate_hz):
            classification["direct_lookup"] = True
            confirmation: dict[str, object] = {
                "skipped": True,
                "reason": "classification is clear of both lookup boundaries",
            }
            confirmed = True
        else:
            confirmation_slot = CONFIRMATION_SLOTS[candidate_hz]
            confirmation_capture = capture_slot(
                camera,
                uart,
                confirmation_slot,
                idle_cache.corners,
                idle_cache.blank,
                expected_crossings(candidate_hz, TABLE_RAMPS_US[confirmation_slot]),
            )
            all_captures.append(confirmation_capture)
            confirmation = inspect(confirmation_capture, idle_cache.blank)
            confirmed = confirmation_matches(confirmation, candidate_hz)
            if candidate_hz == 49900:
                projected = projected_crossings(confirmation)
                mid_expected = expected_crossings(
                    49900, TABLE_RAMPS_US[confirmation_slot]
                )
                high_expected = expected_crossings(
                    90900, TABLE_RAMPS_US[confirmation_slot]
                )
                mid_high_boundary = (mid_expected + high_expected) * 0.5
                decisive_high_confirmation = projected >= high_expected - 1.0
                if (
                    (bool(confirmation["accepted"]) or decisive_high_confirmation)
                    and int(confirmation["trace_span_px"]) >= MIN_TRACE_SPAN_PX
                    and projected >= mid_high_boundary
                ):
                    candidate_hz = 90900
                    confirmation["reclassified_from_hz"] = 49900
                    confirmation["reclassification_boundary"] = mid_high_boundary
                    confirmation["reason"] = (
                        "125 us confirmation decisively identifies high frequency"
                        if decisive_high_confirmation
                        else "125 us confirmation identifies high frequency"
                    )
                    confirmed = True
            if (
                not confirmed
                and candidate_hz == 90900
                and stale_high_frame_confirms(classification, confirmation)
            ):
                confirmation["stale_classification_frame"] = True
                confirmation["reason"] = (
                    "complete high-frequency main frame persisted during confirmation"
                )
                confirmed = True
        attempts.append({
            "attempt": attempt,
            "classification": classification,
            "candidate_frequency_hz": candidate_hz,
            "confirmation": confirmation,
            "confirmed": confirmed,
        })
        if confirmed:
            result_hz = candidate_hz
            break
        if attempt < MAX_ATTEMPTS:
            # Clear a saturated persistence frame before retrying. Reusing a
            # dense failed image makes every later ramp look high-frequency.
            uart.command("IDLE")
            time.sleep(INTER_ATTEMPT_IDLE_S)
            frame_period = 1.0 / max(camera.fps, 1.0)
            for _ in range(POST_STEP_FLUSH_FRAMES):
                time.sleep(frame_period)
                camera.capture()

    uart.command("IDLE")
    report: dict[str, object] = {
        "mode": "three_frequency_crossing_lookup",
        "allowed_frequencies_hz": KNOWN_FREQUENCIES_HZ,
        "classification": {
            "classify_slot": CLASSIFY_SLOT,
            "classify_ramp_us": TABLE_RAMPS_US[CLASSIFY_SLOT],
            "low_mid_crossing_boundary": LOW_MID_CROSSING_BOUNDARY,
            "mid_high_crossing_boundary": MID_HIGH_CROSSING_BOUNDARY,
            "expected_crossings": {
                frequency_hz: expected_crossings(
                    frequency_hz, TABLE_RAMPS_US[CLASSIFY_SLOT]
                )
                for frequency_hz in KNOWN_FREQUENCIES_HZ
            },
        },
        "confirmation_slots": CONFIRMATION_SLOTS,
        "attempts": attempts,
        "elapsed_s": time.monotonic() - started,
    }
    if result_hz is not None:
        report["frequency_hz"] = result_hz
    else:
        report["reason"] = "no candidate passed its lookup confirmation"

    output_dir.mkdir(parents=True, exist_ok=True)
    if save_images:
        archive(output_dir, all_captures, idle_cache)
    (output_dir / "frequency_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="ascii")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Q5 three-frequency visual lookup")
    parser.add_argument("--serial", default="/dev/ttyS2")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--output-dir", type=Path, default=Path("q5_frequency_measure"))
    parser.add_argument("--save-images", action="store_true")
    args = parser.parse_args()
    with Uart(args.serial) as uart, Camera(args.device, 30.0) as camera:
        report = run_frequency_measurement(args.output_dir, uart, camera, args.save_images)
        reply = uart.command(f"RESULT {int(report.get('frequency_hz', 0))}")
    report["mcu_result_reply"] = reply
    (args.output_dir / "frequency_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="ascii")
    print(json.dumps(report), flush=True)
    return 0 if "frequency_hz" in report and reply.startswith("OK RESULT") else 2


if __name__ == "__main__":
    raise SystemExit(main())
