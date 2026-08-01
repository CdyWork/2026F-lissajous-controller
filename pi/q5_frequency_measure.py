"""Production Q5 visual frequency measurement.

The normal path samples four representative FPGA ramp slots, verifies one
selected slot with a second frame, and returns a calibrated frequency. If no
normal slot is usable, STEP 33 selects the 6 ms ramp / 10 ms frame fallback;
STEP 32 remains the shorter 2 ms fallback.
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
    10, 12, 13, 16, 18, 21, 24, 28, 33, 38, 44, 51, 59, 69, 80, 93,
    108, 125, 145, 168, 195, 226, 263, 305, 353, 410, 476, 552, 640,
    743, 862, 1000, 2000, 6000,
)
NORMAL_TABLE_COUNT = 32
INITIAL_NORMAL_SLOT = 16
LOW_FREQUENCY_SLOTS = (33, 32)
LOW_FREQUENCY_CAPTURE_COUNT = 3
NORMAL_CONFIRMATION_CAPTURE_LIMIT = 3
MAX_BINARY_STEPS = 5
MIN_TARGET_CROSSINGS = 6
MAX_TARGET_CROSSINGS = 18
NORMAL_SETTLE_S = 0.50
LOW_FREQUENCY_SETTLE_S = 0.75
IDLE_BLANK_SETTLE_S = 0.75
IDLE_BLANK_FLUSH_FRAMES = 12
POST_STEP_FLUSH_FRAMES = 8
CACHED_BLANK_MAX_AGE_S = 120.0
MIN_TRACE_SPAN_PX = 280
MAX_NORMAL_VISUAL_CYCLES = 14.0
MAX_REPEAT_DEVIATION = 0.05
NORMAL_CALIBRATION_GAIN = 1.965686502
NORMAL_CALIBRATION_OFFSET_HZ = 111.135141
LOW_CALIBRATION_GAIN = 1.95


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
    camera: Camera, uart: Uart, slot: int, corners: np.ndarray, settle_s: float = NORMAL_SETTLE_S,
) -> Capture:
    frame_period = 1.0 / max(camera.fps, 1.0)
    # Empty V4L2 frames left by the previous oscilloscope state must not be
    # associated with the new FPGA slot.
    for _ in range(max(CAMERA_PIPELINE_FLUSH_FRAMES, 4)):
        time.sleep(frame_period)
        camera.capture()
    reply = uart.command(f"STEP {slot}")
    if not reply.startswith("OK STEP"):
        raise RuntimeError(f"STEP {slot} failed: {reply!r}")
    time.sleep(settle_s)
    for _ in range(max(CAMERA_PIPELINE_FLUSH_FRAMES, POST_STEP_FLUSH_FRAMES)):
        time.sleep(frame_period)
        camera.capture()
    frame = camera.capture().bgr
    return Capture(slot, frame, rectify(frame, corners))


def prepare_idle_cache(uart: Uart, camera: Camera) -> IdleCache:
    """Capture a clean background once while FPGA output is parked."""
    if not uart.command("IDLE").startswith("OK IDLE"):
        raise RuntimeError("cannot park FPGA before calibration")
    time.sleep(IDLE_BLANK_SETTLE_S)
    frame_period = 1.0 / max(camera.fps, 1.0)
    for _ in range(IDLE_BLANK_FLUSH_FRAMES):
        time.sleep(frame_period)
        camera.capture()
    blank_frames: list[np.ndarray] = []
    for _ in range(CALIBRATION_FRAMES):
        time.sleep(frame_period)
        blank_frames.append(camera.capture().bgr)
    calibration_image = median_frame(blank_frames)
    calibration = calibrate_xy(calibration_image)
    if calibration.confidence < 0.65:
        raise RuntimeError(f"XY calibration confidence too low: {calibration.confidence:.3f}")
    return IdleCache(
        calibration_image=calibration_image,
        corners=calibration.corners,
        blank=rectify(calibration_image, calibration.corners),
        captured_at_s=time.monotonic(),
    )


def crossing_estimate(image: np.ndarray, ramp_us: int) -> dict[str, object]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    response = cv2.GaussianBlur(gray, (11, 5), 0)
    strength = response.max(axis=1)
    try:
        start, end = largest_active_span(strength >= max(20.0, float(strength.max()) * 0.35))
    except RuntimeError as error:
        return {
            "accepted": False,
            "trace_span_px": 0,
            "amplitude_px": 0.0,
            "crossing_count": 0,
            "ramp_us": ramp_us,
            "reason": str(error),
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
    same_direction = np.concatenate((np.diff(up), np.diff(down)))
    low_frequency = ramp_us >= 2000
    result: dict[str, object] = {
        "accepted": False,
        "trace_span_px": int(end - start),
        "amplitude_px": float((high - low) * 0.5),
        "crossing_count": len(crossings),
        "ramp_us": ramp_us,
    }
    minimum_amplitude = 120.0 if low_frequency else 16.0
    if result["amplitude_px"] < minimum_amplitude:
        result["reason"] = f"trace amplitude below {minimum_amplitude:.0f} px"
        return result
    if len(crossings) < (2 if low_frequency else 5):
        result["reason"] = "insufficient midline crossings"
        return result
    if any(crossings[index][1] == crossings[index - 1][1] for index in range(1, len(crossings))):
        result["reason"] = "midline crossings do not alternate"
        return result

    if len(same_direction) >= 1:
        intervals = same_direction
    elif low_frequency:
        # One up/down pair is a half visual period in the 2 ms fallback.
        intervals = 2.0 * np.diff(np.asarray([position for position, _ in crossings]))
        result["half_period_mode"] = True
    else:
        result["reason"] = "not enough full-period intervals"
        return result
    spacing = float(np.median(intervals))
    if spacing <= 0.0:
        result["reason"] = "invalid crossing interval"
        return result
    spacing_cv = (
        float(1.4826 * np.median(np.abs(intervals - spacing)) / spacing)
        if len(intervals) >= 2 else 0.0
    )
    if spacing_cv > 0.15:
        result["reason"] = "unstable crossing intervals"
        return result

    visual_cycles = float((end - start - 1) / spacing)
    raw_hz = visual_cycles / (ramp_us * 1e-6)
    gain = LOW_CALIBRATION_GAIN if low_frequency else NORMAL_CALIBRATION_GAIN
    offset = 0.0 if low_frequency else NORMAL_CALIBRATION_OFFSET_HZ
    result.update({
        "accepted": True,
        "crossing_spacing_cv": spacing_cv,
        "visual_cycles": visual_cycles,
        "uncalibrated_frequency_hz": raw_hz,
        "frequency_hz": raw_hz * gain + offset,
        "confidence": (0.40 if result.get("half_period_mode") else min(1.0, (len(crossings) - 4) / 10.0)) / (1.0 + spacing_cv),
    })
    return result


def sweep_background(captures: list[Capture], blank: np.ndarray) -> np.ndarray:
    """Remove traces that persist in the scope image across table changes."""
    if len({capture.slot for capture in captures}) < 3:
        return blank
    return np.quantile(np.stack([capture.xy for capture in captures]), 0.20, axis=0).astype(np.uint8)


def inspect(capture: Capture, blank: np.ndarray, background: np.ndarray | None = None) -> dict[str, object]:
    backgrounds = [("idle_blank", blank)]
    if background is not None:
        backgrounds.append(("sweep_q20", background))
    choices: list[dict[str, object]] = []
    for source, candidate_background in backgrounds:
        result = crossing_estimate(
            cv2.subtract(capture.xy, candidate_background), TABLE_RAMPS_US[capture.slot]
        )
        result.update({"background_source": source, "table_index": capture.slot})
        choices.append(result)
    return max(
        choices,
        key=lambda value: (
            bool(value["accepted"]),
            int(value["trace_span_px"]),
            float(value.get("confidence", 0.0)),
        ),
    )


def repeatable(values: list[dict[str, object]]) -> list[dict[str, object]]:
    if len(values) < 2:
        return []
    best: list[dict[str, object]] = []
    best_score = (-1, -1.0)
    for anchor in values:
        anchor_hz = float(anchor["uncalibrated_frequency_hz"])
        cluster = [
            value for value in values
            if abs(float(value["uncalibrated_frequency_hz"]) / anchor_hz - 1.0)
            <= MAX_REPEAT_DEVIATION
        ]
        score = (len(cluster), sum(float(value["confidence"]) for value in cluster))
        if score > best_score:
            best, best_score = cluster, score
    return best if len(best) >= 2 else []


def normal_usable(estimates: list[dict[str, object]], slot: int) -> list[dict[str, object]]:
    return repeatable([
        value for value in estimates
        if value["accepted"]
        and int(value["table_index"]) == slot
        and int(value["trace_span_px"]) >= MIN_TRACE_SPAN_PX
        and 2.5 <= float(value["visual_cycles"]) <= MAX_NORMAL_VISUAL_CYCLES
    ])


def binary_direction(estimate: dict[str, object]) -> int:
    """Return +1 for a longer ramp, -1 for shorter, 0 for a usable trace."""
    crossings = int(estimate["crossing_count"])
    if crossings < MIN_TARGET_CROSSINGS:
        return 1
    if crossings > MAX_TARGET_CROSSINGS:
        return -1
    return 0 if bool(estimate["accepted"]) else 1


def run_frequency_measurement(
    output_dir: Path,
    uart: Uart,
    camera: Camera,
    save_images: bool = False,
    idle_cache: IdleCache | None = None,
) -> dict[str, object]:
    started = time.monotonic()
    if idle_cache is None or time.monotonic() - idle_cache.captured_at_s > CACHED_BLANK_MAX_AGE_S:
        idle_cache = prepare_idle_cache(uart, camera)
    calibration_image = idle_cache.calibration_image
    corners = idle_cache.corners
    blank = idle_cache.blank

    captures: list[Capture] = []
    estimates: list[dict[str, object]] = []
    selected_slot: int | None = None
    low_fallback = False
    usable: list[dict[str, object]] = []
    binary_history: list[dict[str, object]] = []
    low, high, candidate_slot = 0, NORMAL_TABLE_COUNT - 1, INITIAL_NORMAL_SLOT

    for _ in range(MAX_BINARY_STEPS):
        capture = capture_slot(camera, uart, candidate_slot, corners)
        captures.append(capture)
        estimate = inspect(capture, blank, sweep_background(captures, blank))
        estimates.append(estimate)
        direction = binary_direction(estimate)
        binary_history.append({
            "table_index": candidate_slot,
            "ramp_us": TABLE_RAMPS_US[candidate_slot],
            "direction": direction,
            "crossing_count": estimate["crossing_count"],
            "accepted": estimate["accepted"],
        })
        if direction == 0:
            selected_slot = candidate_slot
            break
        if direction > 0:
            low = candidate_slot + 1
        else:
            high = candidate_slot - 1
        if low > high:
            break
        if candidate_slot == INITIAL_NORMAL_SLOT and direction > 0:
            crossings = int(estimate["crossing_count"])
            next_slot = 30 if crossings <= 3 else 28
            if not low <= next_slot <= high:
                next_slot = (low + high) // 2
        else:
            next_slot = (low + high) // 2
        if next_slot == candidate_slot:
            break
        candidate_slot = next_slot

    if selected_slot is not None:
        for _ in range(NORMAL_CONFIRMATION_CAPTURE_LIMIT):
            captures.append(capture_slot(camera, uart, selected_slot, corners))
            background = sweep_background(captures, blank)
            estimates = [inspect(candidate, blank, background) for candidate in captures]
            usable = normal_usable(estimates, selected_slot)
            if usable:
                break

    if not usable:
        low_fallback = True
        for low_slot in LOW_FREQUENCY_SLOTS:
            low_captures = [
                capture_slot(camera, uart, low_slot, corners, LOW_FREQUENCY_SETTLE_S)
                for _ in range(LOW_FREQUENCY_CAPTURE_COUNT)
            ]
            low_estimates = [inspect(capture, blank) for capture in low_captures]
            captures.extend(low_captures)
            estimates.extend(low_estimates)
            usable = repeatable([estimate for estimate in low_estimates if estimate["accepted"]])
            if usable:
                selected_slot = low_slot
                break
    uart.command("IDLE")

    report: dict[str, object] = {
        "mode": "adaptive_table_binary_search_with_low_frequency_fallback",
        "normal_table_count": NORMAL_TABLE_COUNT,
        "binary_history": binary_history,
        "selected_table_index": selected_slot,
        "low_frequency_fallback_used": low_fallback,
        "calibration": {
            "normal_gain": NORMAL_CALIBRATION_GAIN,
            "normal_offset_hz": NORMAL_CALIBRATION_OFFSET_HZ,
            "low_gain": LOW_CALIBRATION_GAIN,
        },
        "estimates": estimates,
        "usable_settings": len(usable),
        "elapsed_s": time.monotonic() - started,
    }
    if usable:
        values = [float(estimate["frequency_hz"]) for estimate in usable]
        weights = [float(estimate["confidence"]) for estimate in usable]
        report["frequency_hz"] = weighted_median(values, weights)
        report["frequency_estimates_hz"] = values
    else:
        report["reason"] = "no repeatable visual frequency"
    output_dir.mkdir(parents=True, exist_ok=True)
    if save_images:
        write_image(output_dir / "xy_axis_calibrated.png", blank)
        for sequence, capture in enumerate(captures):
            stem = f"{sequence:02d}_idx{capture.slot:02d}_{TABLE_RAMPS_US[capture.slot]:04d}us"
            write_image(output_dir / f"{stem}.jpg", capture.bgr)
            write_image(output_dir / f"{stem}_xy.png", capture.xy)
    (output_dir / "frequency_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="ascii")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Q5 production visual frequency measurement")
    parser.add_argument("--serial", default="/dev/ttyS2")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--output-dir", type=Path, default=Path("q5_frequency_measure"))
    parser.add_argument("--save-images", action="store_true")
    args = parser.parse_args()
    with Uart(args.serial) as uart, Camera(args.device, 30.0) as camera:
        report = run_frequency_measurement(args.output_dir, uart, camera, args.save_images)
        reply = uart.command(f"RESULT {int(round(float(report.get('frequency_hz', 0.0))))}")
    report["mcu_result_reply"] = reply
    (args.output_dir / "frequency_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="ascii")
    print(json.dumps(report), flush=True)
    return 0 if "frequency_hz" in report and reply.startswith("OK RESULT") else 2


if __name__ == "__main__":
    raise SystemExit(main())
