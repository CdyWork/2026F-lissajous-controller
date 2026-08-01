"""Experimental Q5 visual binary search using FPGA ramp-table control.

This intentionally does not replace q5_fpga_sweep.py. It is a SRAM-test
entry point: STEP 0..31 selects a 2 ms-frame table entry. STEP 32 is used only
after normal measurement fails, and selects the 2 ms-ramp / 10 ms-frame
low-frequency fallback. Only selected camera frames remain in memory unless
--save-images is requested.
"""

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
    Camera,
    Uart,
    calibrate_xy,
    largest_active_span,
    median_frame,
    rectify,
    rolling_median,
    weighted_median,
    write_image,
)


TABLE_RAMPS_US = (
    10, 12, 13, 16, 18, 21, 24, 28,
    33, 38, 44, 51, 59, 69, 80, 93,
    108, 125, 145, 168, 195, 226, 263, 305,
    353, 410, 476, 552, 640, 743, 862, 1000,
    2000, 6000,
)
HIGH_TABLE_COUNT = 32
LOW_FREQUENCY_TABLE_INDEX = 32
LOW_FREQUENCY_CONFIRMATION_COUNT = 2
MID_INDEX = 16
# The scope XY persistence can outlast the camera queue. 300 ms plus the
# four-frame V4L2 flush gives a newly selected table setting time to dominate
# the displayed trace before it is scored.
SETTLE_S = 0.30
MAX_BINARY_STEPS = 6
MIN_TARGET_VERTICES = 6
MAX_TARGET_VERTICES = 18
MAX_RELATIVE_DEVIATION = 0.12
MIN_CONSISTENT_SETTINGS = 2
MIN_CROSSING_COUNT = 5
MAX_CROSSING_COUNT = 28
MAX_CROSSING_SPACING_CV = 0.15
REPEAT_CONFIRMATION_COUNT = 1
MIN_FULL_TRACE_SPAN_PX = 300
MAX_REPEAT_RELATIVE_DEVIATION = 0.05
FREQUENCY_CALIBRATION_GAIN = 1.9741
FREQUENCY_CALIBRATION_OFFSET_HZ = -314.0
LOW_FREQUENCY_CALIBRATION_GAIN = 2.0
LOW_FREQUENCY_CALIBRATION_OFFSET_HZ = 0.0


@dataclass
class TableCapture:
    index: int
    ramp_us: int
    bgr: np.ndarray
    xy: np.ndarray
    captured_at_s: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Experimental FPGA-table visual binary search")
    parser.add_argument("--serial", default="/dev/ttyS2")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--capture-fps", type=float, default=30.0)
    parser.add_argument("--output-dir", type=Path, default=Path("q5_adaptive_table_test"))
    parser.add_argument("--save-images", action="store_true")
    return parser.parse_args()


def capture_settled(camera: Camera, uart: Uart, index: int) -> TableCapture:
    reply = uart.command(f"STEP {index}")
    if not reply.startswith("OK STEP"):
        raise RuntimeError(f"STEP {index} failed: {reply!r}")

    frame_period_s = 1.0 / max(camera.fps, 1.0)
    time.sleep(SETTLE_S)
    # Frames queued before the STEP acknowledgement are never analysed.
    for _ in range(CAMERA_PIPELINE_FLUSH_FRAMES):
        time.sleep(frame_period_s)
        camera.capture()
    frame = camera.capture()
    return TableCapture(index, TABLE_RAMPS_US[index], frame.bgr, frame.bgr, frame.timestamp_s)


def rectify_capture(capture: TableCapture, corners: np.ndarray) -> TableCapture:
    capture.xy = rectify(capture.bgr, corners)
    return capture


def current_background(captures: list[TableCapture], blank: np.ndarray) -> np.ndarray:
    # A temporal low percentile removes an old scope trace as soon as three
    # distinct table settings are available. Before that, use the explicit
    # idle-screen blank; the final two-setting consistency gate prevents a
    # transient single-frame result from being returned.
    if len(captures) < 3:
        return blank
    return np.quantile(np.stack([capture.xy for capture in captures]), 0.20, axis=0).astype(np.uint8)


def analyse_zero_crossings(trace: np.ndarray, ramp_us: int) -> dict[str, object]:
    """Estimate visual frequency from robust same-direction midline crossings."""
    gray = cv2.cvtColor(trace, cv2.COLOR_BGR2GRAY)
    response = cv2.GaussianBlur(gray, (11, 5), 0)
    row_strength = response.max(axis=1)
    start, end = largest_active_span(
        row_strength >= max(20.0, float(row_strength.max()) * 0.35)
    )
    centers = response[start:end].argmax(axis=1).astype(np.float32)
    centers = rolling_median(centers, 9)
    centers = cv2.GaussianBlur(centers.reshape(1, -1), (15, 1), 0).reshape(-1)

    low = float(np.percentile(centers, 5.0))
    high = float(np.percentile(centers, 95.0))
    centerline = (low + high) * 0.5
    amplitude_px = (high - low) * 0.5
    centered = centers - centerline
    minimum_separation = max(12, len(centers) // 40)
    candidates: list[tuple[float, str, float]] = []
    for index in range(len(centered) - 1):
        first = float(centered[index])
        second = float(centered[index + 1])
        if (first < 0.0 <= second) or (first > 0.0 >= second):
            slope = second - first
            if slope == 0.0:
                continue
            position = index + (-first / slope)
            candidates.append((position, "up" if slope > 0.0 else "down", abs(slope)))

    # A thick trace can cross its midpoint over adjacent rows. Retain the
    # steepest observation from each local crossing region.
    crossings: list[tuple[float, str]] = []
    for position, direction, slope in candidates:
        if crossings and position - crossings[-1][0] < minimum_separation:
            continue
        crossings.append((position, direction))
    up = np.asarray([position for position, direction in crossings if direction == "up"], dtype=np.float64)
    down = np.asarray([position for position, direction in crossings if direction == "down"], dtype=np.float64)
    same_direction_spacing = np.concatenate((np.diff(up), np.diff(down))).astype(np.float64)
    all_crossing_positions = np.asarray([position for position, _ in crossings], dtype=np.float64)
    alternation_errors = sum(
        crossings[index][1] == crossings[index - 1][1]
        for index in range(1, len(crossings))
    )
    crossing_count = len(crossings)
    result: dict[str, object] = {
        "ramp_us": ramp_us,
        "trace_span_px": int(end - start),
        "midline_x": centerline,
        "amplitude_px": amplitude_px,
        "up_crossings_y": [start + float(position) for position in up],
        "down_crossings_y": [start + float(position) for position in down],
        "crossing_count": crossing_count,
        "accepted": False,
    }
    low_frequency_fallback = ramp_us >= 2000
    minimum_crossings = 2 if low_frequency_fallback else MIN_CROSSING_COUNT
    if amplitude_px < 16.0:
        result["reason"] = "trace amplitude below 16 px"
    elif not minimum_crossings <= crossing_count <= MAX_CROSSING_COUNT:
        result["reason"] = f"crossing count outside {minimum_crossings}..{MAX_CROSSING_COUNT}"
    elif alternation_errors != 0:
        result["reason"] = "midline crossings do not alternate"
    elif len(same_direction_spacing) < 2 and not low_frequency_fallback:
        result["reason"] = "not enough same-direction crossing intervals"
    else:
        if len(same_direction_spacing) >= 2:
            spacing_samples = same_direction_spacing
            spacing_px = float(np.median(spacing_samples))
        else:
            # The 2 ms fallback can show only one up/down pair at 1 kHz.
            # Its separation is one half visual period, so double it before
            # converting the displayed period to a frequency.
            spacing_samples = 2.0 * np.diff(all_crossing_positions)
            spacing_px = float(np.median(spacing_samples))
            result["low_frequency_half_period"] = True
        spacing_cv = (
            float(1.4826 * np.median(np.abs(spacing_samples - spacing_px)) / spacing_px)
            if len(spacing_samples) >= 2 else 0.0
        )
        result["crossing_spacing_px"] = spacing_px
        result["crossing_spacing_cv"] = spacing_cv
        if spacing_cv > MAX_CROSSING_SPACING_CV:
            result["reason"] = f"midline crossing spacing CV above {MAX_CROSSING_SPACING_CV:.2f}"
        else:
            cycles = float((end - start - 1) / spacing_px)
            uncalibrated_hz = cycles / (ramp_us * 1e-6)
            calibration_gain = (
                LOW_FREQUENCY_CALIBRATION_GAIN if low_frequency_fallback
                else FREQUENCY_CALIBRATION_GAIN
            )
            calibration_offset_hz = (
                LOW_FREQUENCY_CALIBRATION_OFFSET_HZ if low_frequency_fallback
                else FREQUENCY_CALIBRATION_OFFSET_HZ
            )
            result.update({
                "accepted": True,
                "visual_cycles": cycles,
                "uncalibrated_frequency_hz": uncalibrated_hz,
                "frequency_hz": (
                    uncalibrated_hz * calibration_gain + calibration_offset_hz
                ),
                "confidence": float(
                    (0.40 if result.get("low_frequency_half_period")
                     else min(1.0, (crossing_count - 4) / 10.0)) / (1.0 + spacing_cv)
                ),
            })
    return result


def inspect(capture: TableCapture, background: np.ndarray) -> dict[str, object]:
    trace = cv2.subtract(capture.xy, background)
    try:
        result = analyse_zero_crossings(trace, capture.ramp_us)
    except RuntimeError as error:
        result = {"accepted": False, "reason": str(error), "crossing_count": 0}
    result.update({"table_index": capture.index, "ramp_us": capture.ramp_us})
    return result


def inspect_best_background(
    capture: TableCapture, blank: np.ndarray, sweep_background: np.ndarray
) -> dict[str, object]:
    """Use the stronger of explicit blank and current-sweep q20 subtraction."""
    candidates = [
        ("idle_blank", inspect(capture, blank)),
        ("current_sweep_q20", inspect(capture, sweep_background)),
    ]

    def score(item: tuple[str, dict[str, object]]) -> tuple[float, float, float]:
        _, result = item
        if result.get("accepted"):
            # A cropped q20 trace underestimates visual cycles even when its
            # local crossing spacing is clean. Prefer the full-height trace.
            return (
                1.0,
                float(result.get("trace_span_px", 0)),
                float(result.get("confidence", 0.0)),
            )
        return (-1.0, float(result.get("trace_span_px", 0)), 0.0)

    source, result = max(candidates, key=score)
    result["background_source"] = source
    return result


def choose_direction(result: dict[str, object]) -> int:
    crossings = int(result.get("crossing_count", 0))
    if crossings < MIN_TARGET_VERTICES:
        return 1
    if crossings > MAX_TARGET_VERTICES:
        return -1
    return 0 if bool(result.get("accepted")) else 2


def retain_consistent_cluster(estimates: list[dict[str, object]]) -> list[dict[str, object]]:
    """Keep the strongest mutually consistent frequency cluster.

    A table-slot transition can leave a persistence artifact in an adjacent
    ramp image.  Two independent captures of the selected slot are more
    meaningful than forcing every accepted neighbour into one global median.
    """
    usable = [estimate for estimate in estimates if estimate.get("accepted")]
    if len(usable) < 2:
        return usable

    best_cluster: list[dict[str, object]] = []
    best_score = (-1, -1.0)
    for anchor in usable:
        anchor_hz = float(anchor["uncalibrated_frequency_hz"])
        cluster = [
            estimate for estimate in usable
            if abs(float(estimate["uncalibrated_frequency_hz"]) / anchor_hz - 1.0)
            <= MAX_RELATIVE_DEVIATION
        ]
        score = (len(cluster), sum(float(estimate["confidence"]) for estimate in cluster))
        if score > best_score:
            best_cluster, best_score = cluster, score

    cluster_ids = {id(estimate) for estimate in best_cluster}
    cluster_reference_hz = float(np.median([
        float(estimate["uncalibrated_frequency_hz"]) for estimate in best_cluster
    ]))
    for estimate in usable:
        deviation = abs(float(estimate["uncalibrated_frequency_hz"]) / cluster_reference_hz - 1.0)
        estimate["frequency_relative_deviation"] = deviation
        if id(estimate) not in cluster_ids:
            estimate["accepted"] = False
            estimate["reason"] = "frequency outlier outside strongest consistent cluster"
    return [estimate for estimate in estimates if estimate.get("accepted")]


def strongest_close_group(estimates: list[dict[str, object]]) -> list[dict[str, object]]:
    """Return the strongest group agreeing within the repeated-frame limit."""
    best: list[dict[str, object]] = []
    best_score = (-1, -1.0)
    for anchor in estimates:
        anchor_hz = float(anchor["uncalibrated_frequency_hz"])
        group = [
            estimate for estimate in estimates
            if abs(float(estimate["uncalibrated_frequency_hz"]) / anchor_hz - 1.0)
            <= MAX_REPEAT_RELATIVE_DEVIATION
        ]
        score = (len(group), sum(float(estimate["confidence"]) for estimate in group))
        if score > best_score:
            best, best_score = group, score
    return best


def preferred_full_trace_index(estimates: list[dict[str, object]]) -> int | None:
    """Choose the shortest ramp with a complete, stable final trace.

    The binary-search pass uses the idle screen and can temporarily miss a
    short-ramp trace.  After all captures are available, q20 background
    subtraction reveals those complete traces. Prefer their shortest ramp so
    long-ramp persistence cannot masquerade as a lower-frequency waveform.
    """
    candidates = [
        estimate for estimate in estimates
        if estimate.get("accepted")
        and int(estimate.get("trace_span_px", 0)) >= MIN_FULL_TRACE_SPAN_PX
    ]
    if not candidates:
        return None
    by_index: dict[int, list[dict[str, object]]] = {}
    for estimate in candidates:
        by_index.setdefault(int(estimate["table_index"]), []).append(estimate)
    stable_indices = [
        index for index, rows in by_index.items()
        if len(strongest_close_group(rows)) >= MIN_CONSISTENT_SETTINGS
    ]
    if stable_indices:
        return min(stable_indices)
    return min(int(estimate["table_index"]) for estimate in candidates)


def selected_slot_estimates(
    estimates: list[dict[str, object]], selected_index: int | None
) -> list[dict[str, object]]:
    if selected_index is None:
        return retain_consistent_cluster(estimates)
    usable = [
        estimate for estimate in estimates
        if estimate.get("accepted") and int(estimate["table_index"]) == selected_index
    ]
    cluster = strongest_close_group(usable)
    if len(cluster) < MIN_CONSISTENT_SETTINGS:
        return []
    cluster_ids = {id(estimate) for estimate in cluster}
    reference_hz = float(np.median([
        float(estimate["uncalibrated_frequency_hz"]) for estimate in cluster
    ]))
    for estimate in usable:
        deviation = abs(float(estimate["uncalibrated_frequency_hz"]) / reference_hz - 1.0)
        estimate["frequency_relative_deviation"] = deviation
        if id(estimate) not in cluster_ids:
            estimate["accepted"] = False
            estimate["reason"] = "repeat-frame frequency outlier"
    return [estimate for estimate in estimates if id(estimate) in cluster_ids]


def archive(output_dir: Path, captures: list[TableCapture], calibration_image: np.ndarray, corners: np.ndarray) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    write_image(output_dir / "xy_axis_calibrated.png", rectify(calibration_image, corners))
    for sequence, capture in enumerate(captures):
        stem = f"{sequence:02d}_idx{capture.index:02d}_{capture.ramp_us:04d}us"
        write_image(output_dir / f"{stem}.jpg", capture.bgr)
        write_image(output_dir / f"{stem}_xy.png", capture.xy)


def run_table_measurement(
    output_dir: Path,
    uart: Uart,
    camera: Camera,
    save_images: bool = False,
) -> dict[str, object]:
    """Measure with the normal table, then one 2 ms low-frequency fallback."""
    captures: list[TableCapture] = []
    started_at_s = time.monotonic()
    # Build a blank only after parking the DAC, so it cannot contain the table
    # waveform selected by a preceding measurement.
    if not uart.command("IDLE").startswith("OK IDLE"):
        raise RuntimeError("cannot park FPGA before XY calibration")
    time.sleep(0.20)
    calibration_image = median_frame([camera.capture().bgr for _ in range(CALIBRATION_FRAMES)])
    calibration = calibrate_xy(calibration_image)
    if calibration.confidence < 0.65:
        raise RuntimeError(f"XY calibration confidence too low: {calibration.confidence:.3f}")
    blank = rectify(calibration_image, calibration.corners)

    low, high, index = 0, HIGH_TABLE_COUNT - 1, MID_INDEX
    history: list[dict[str, object]] = []
    selected_index: int | None = None
    for _ in range(MAX_BINARY_STEPS):
        capture = rectify_capture(capture_settled(camera, uart, index), calibration.corners)
        captures.append(capture)
        result = inspect(capture, current_background(captures, blank))
        history.append(result)
        direction = choose_direction(result)
        print(json.dumps({"decision": result, "direction": direction}), flush=True)
        if direction == 0:
            selected_index = index
            break
        if direction == 1:
            low = max(low, index + 1)
        elif direction == -1:
            high = min(high, index - 1)
        else:
            low = max(low, index + 1)
        if low > high:
            break
        next_index = (low + high) // 2
        if next_index == index:
            break
        index = next_index

    if selected_index is not None:
        # Verify the chosen ramp with a fresh frame before asking adjacent
        # slots to corroborate it. This prevents a contaminated neighbour from
        # suppressing a stable selected-slot measurement.
        for _ in range(REPEAT_CONFIRMATION_COUNT):
            captures.append(rectify_capture(
                capture_settled(camera, uart, selected_index), calibration.corners
            ))
        for neighbour in (selected_index - 1, selected_index + 1):
            if 0 <= neighbour < HIGH_TABLE_COUNT:
                captures.append(rectify_capture(
                    capture_settled(camera, uart, neighbour), calibration.corners
                ))
    uart.command("IDLE")

    background = current_background(captures, blank)
    estimates: list[dict[str, object]] = [
        inspect_best_background(capture, blank, background) for capture in captures
    ]
    final_index = preferred_full_trace_index(estimates)
    preliminary_rows = [
        estimate for estimate in estimates
        if final_index is not None and int(estimate["table_index"]) == final_index
        and estimate.get("accepted")
    ]
    needs_repeat = len(strongest_close_group(preliminary_rows)) < MIN_CONSISTENT_SETTINGS
    if final_index is not None and (final_index != selected_index or needs_repeat):
        # The globally corrected background found a shorter, complete trace
        # that the initial idle-background binary pass missed. Verify it with
        # a new camera frame before using it for the result.
        captures.append(rectify_capture(
            capture_settled(camera, uart, final_index), calibration.corners
        ))
        background = current_background(captures, blank)
        estimates = [
            inspect_best_background(capture, blank, background) for capture in captures
        ]
        uart.command("IDLE")
    usable = selected_slot_estimates(estimates, final_index)
    low_frequency_fallback_used = False
    if len(usable) < MIN_CONSISTENT_SETTINGS:
        # The normal 10..1000 us table cannot expose enough cycles for low
        # frequencies. Test the dedicated 2 ms ramp / 10 ms frame once, with
        # two camera frames used only as a repeatability check.
        low_frequency_captures = [
            rectify_capture(
                capture_settled(camera, uart, LOW_FREQUENCY_TABLE_INDEX), calibration.corners
            )
            for _ in range(LOW_FREQUENCY_CONFIRMATION_COUNT)
        ]
        uart.command("IDLE")
        low_frequency_estimates = []
        for capture in low_frequency_captures:
            estimate = inspect(capture, blank)
            estimate["background_source"] = "idle_blank"
            low_frequency_estimates.append(estimate)
        low_frequency_usable = selected_slot_estimates(
            low_frequency_estimates, LOW_FREQUENCY_TABLE_INDEX
        )
        captures.extend(low_frequency_captures)
        estimates.extend(low_frequency_estimates)
        if len(low_frequency_usable) >= MIN_CONSISTENT_SETTINGS:
            final_index = LOW_FREQUENCY_TABLE_INDEX
            usable = low_frequency_usable
            low_frequency_fallback_used = True
    report: dict[str, object] = {
        "mode": "fpga_32_entry_table_binary_search",
        "table_ramps_us": TABLE_RAMPS_US,
        "binary_history": history,
        "captures": estimates,
        "selected_table_index": selected_index,
        "final_table_index": final_index,
        "low_frequency_fallback_used": low_frequency_fallback_used,
        "elapsed_s": time.monotonic() - started_at_s,
        "usable_settings": len(usable),
        "minimum_consistent_settings": MIN_CONSISTENT_SETTINGS,
        "max_frequency_relative_deviation": MAX_RELATIVE_DEVIATION,
        "processing_mode": "memory",
        "trace_background_method": "best_of_idle_blank_and_current_sweep_q20",
    }
    if len(usable) >= MIN_CONSISTENT_SETTINGS:
        values = [float(estimate["frequency_hz"]) for estimate in usable]
        weights = [float(estimate["confidence"]) for estimate in usable]
        report["frequency_hz"] = weighted_median(values, weights)
        report["frequency_estimates_hz"] = values
    else:
        report["reason"] = "fewer than two consistent table settings"
    output_dir.mkdir(parents=True, exist_ok=True)
    if save_images:
        archive(output_dir, captures, calibration_image, calibration.corners)
    (output_dir / "frequency_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="ascii"
    )
    print(json.dumps(report), flush=True)
    return report


def main() -> int:
    args = parse_args()
    if args.capture_fps <= 0:
        raise ValueError("capture-fps must be positive")
    with Uart(args.serial) as uart, Camera(args.device, args.capture_fps) as camera:
        report = run_table_measurement(args.output_dir, uart, camera, args.save_images)
        result_hz = int(round(float(report.get("frequency_hz", 0.0))))
        result_reply = uart.command(f"RESULT {result_hz}")
    if not result_reply.startswith("OK RESULT"):
        raise RuntimeError(f"MCU rejected RESULT: {result_reply!r}")
    report["mcu_result_reply"] = result_reply
    (args.output_dir / "frequency_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="ascii"
    )
    return 0 if "frequency_hz" in report else 2


if __name__ == "__main__":
    raise SystemExit(main())
