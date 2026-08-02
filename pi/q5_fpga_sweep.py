"""Capture and process the fixed eight-setting Q5 FPGA sweep.

This is the single production entry point for the Orange Pi. It sends SWEEP
once, calibrates the XY view from the two central graticule axes, captures only
stable frames, rectifies every capture to 400x400, and removes the blank frame.
"""

from __future__ import annotations

import argparse
import json
import os
import select
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

if os.name == "posix":
    import termios
    import tty
else:
    termios = None
    tty = None


RAMPS_US = (10, 30, 70, 150, 300, 500, 750, 1000)
FRAME_US = 10000
DWELL_MS = 400
SETTLE_MS = 120
GUARD_MS = 40
XY_SIZE = 400
CALIBRATION_FRAMES = 4
CAMERA_PIPELINE_FLUSH_FRAMES = 4
CAMERA_PIPELINE_WARMUP_FRAMES = 2
INTER_ATTEMPT_IDLE_SETTLE_S = 0.15
MIN_TRACE_FOREGROUND_PIXELS = 200
MAX_MEASUREMENT_ATTEMPTS = 5
SERVICE_SOCKET_PATH = "/tmp/q5_fpga_sweep.sock"
KEY_MEASURE_EVENT = b"MEASURE"
KEY_MEASURE_TASKS = (1, 2, 3)
KEY_MEASURE_OUTPUT_ROOT = Path("/home/orangepi/2026F")
TRACE_THRESHOLD = 18
REPRESENTATIVE_TARGET_MS = 250.0
MIN_VERTEX_COUNT = 3
MAX_VERTEX_COUNT = 24
MAX_SPACING_CV = 0.15
MIN_TRACE_SPAN_PX = 300
MAX_SIDE_VERTEX_COUNT_DIFFERENCE = 1
MAX_ALTERNATION_ERRORS = 0
MAX_FREQUENCY_RELATIVE_DEVIATION = 0.12
MAX_XY_AXIS_CENTER_OFFSET_PX = 24.0
XY_AXIS_CENTER_HINT = np.array((319.0, 312.0), dtype=np.float64)
# The visual-cycle estimate X is calibrated to the electrical frequency Y:
# Y[kHz] = 1.9741 * X[kHz] - 0.314.
FREQUENCY_CALIBRATION_GAIN = 1.9741
FREQUENCY_CALIBRATION_OFFSET_HZ = -314.0
XY_HINT = np.array(
    ((194.0, 202.0), (412.0, 216.0), (416.0, 413.0), (204.0, 410.0)),
    dtype=np.float64,
)


@dataclass(frozen=True)
class CameraFrame:
    timestamp_s: float
    bgr: np.ndarray


@dataclass(frozen=True)
class AxisCalibration:
    corners: np.ndarray
    center: np.ndarray
    horizontal: np.ndarray
    vertical: np.ndarray
    confidence: float


@dataclass(frozen=True)
class SweepCapture:
    """One stable sweep frame held in RAM until the attempt is decided."""

    record: dict[str, object]
    bgr: np.ndarray
    xy: np.ndarray


@dataclass(frozen=True)
class SweepAttempt:
    """All information needed to analyse or archive one hardware sweep."""

    captures: list[SweepCapture]
    calibration_image: np.ndarray
    calibration: AxisCalibration
    blank: np.ndarray


class Camera:
    """Minimal IMX219 V4L2 wrapper used by this sweep only."""

    def __init__(self, device: str, fps: float) -> None:
        self.device = device
        self.fps = fps
        self.camera = None

    def start(self) -> None:
        try:
            import v4l2cam
        except ImportError:
            helper_dir = "/opt/v4l2_opencv_demo"
            if helper_dir not in sys.path:
                sys.path.append(helper_dir)
            import v4l2cam

        camera = v4l2cam.V4L2Camera(
            self.device, 640, 480, enable_isp=True, format="RGB24"
        )
        if not camera.init():
            raise RuntimeError(f"cannot initialise camera {self.device}")
        try:
            camera.set_fps(int(round(self.fps)))
        except Exception:
            pass
        if not camera.start():
            raise RuntimeError(f"cannot start camera {self.device}")
        self.camera = camera
        for _ in range(12):
            self.capture()
            time.sleep(0.01)

    def capture(self) -> CameraFrame:
        if self.camera is None:
            raise RuntimeError("camera is not running")
        rgb = self.camera.get_frame()
        if rgb is None:
            raise RuntimeError("camera returned no frame")
        return CameraFrame(time.monotonic(), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    def stop(self) -> None:
        if self.camera is not None:
            try:
                self.camera.stop()
            finally:
                self.camera = None

    def __enter__(self) -> Camera:
        self.start()
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.stop()


def parse_key_measure_event(line: bytes) -> int | None:
    """Decode MCU keypad notifications; bare MEASURE remains task 1."""
    fields = line.strip().upper().split()
    if not fields or fields[0] != KEY_MEASURE_EVENT:
        return None
    if len(fields) == 1:
        return 1
    if len(fields) != 2:
        return None
    try:
        task_number = int(fields[1])
    except ValueError:
        return None
    return task_number if task_number in KEY_MEASURE_TASKS else None


class Uart:
    """UART transport with a pyserial-free fallback for the Orange Pi."""

    def __init__(self, port: str, timeout_s: float = 1.0) -> None:
        self.timeout_s = timeout_s
        self.serial = None
        self.fd = -1
        self.pending_events: list[int] = []
        try:
            import serial
        except ImportError:
            self._open_posix(port)
        else:
            self.serial = serial.Serial(port, 115200, timeout=timeout_s)

    def _open_posix(self, port: str) -> None:
        if termios is None or tty is None:
            raise RuntimeError("pyserial is required for UART on this platform")
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
        attributes = termios.tcgetattr(self.fd)
        attributes[4] = termios.B115200
        attributes[5] = termios.B115200
        attributes[2] |= termios.CLOCAL | termios.CREAD
        attributes[2] &= ~(termios.CSTOPB | termios.PARENB | termios.CSIZE)
        attributes[2] |= termios.CS8
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attributes)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def command(self, command: str) -> str:
        payload = (command.strip().upper() + "\r\n").encode("ascii")
        if self.serial is not None:
            self.serial.write(payload)
            self.serial.flush()
            deadline = time.monotonic() + self.timeout_s
            while time.monotonic() < deadline:
                line = self.serial.readline().strip()
                measure_task = parse_key_measure_event(line)
                if measure_task is not None:
                    self.pending_events.append(measure_task)
                    continue
                return line.decode("ascii", errors="replace")
            return ""

        os.write(self.fd, payload)
        termios.tcdrain(self.fd)
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            response: list[bytes] = []
            while time.monotonic() < deadline:
                readable, _, _ = select.select([self.fd], [], [], deadline - time.monotonic())
                if not readable:
                    break
                byte = os.read(self.fd, 1)
                if byte:
                    response.append(byte)
                if byte == b"\n":
                    break
            line = b"".join(response).strip()
            measure_task = parse_key_measure_event(line)
            if measure_task is not None:
                self.pending_events.append(measure_task)
                continue
            return line.decode("ascii", errors="replace")
        return ""

    def pop_event(self) -> int | None:
        if not self.pending_events:
            return None
        return self.pending_events.pop(0)

    def fileno(self) -> int:
        if self.serial is not None:
            return int(self.serial.fileno())
        return self.fd

    def read_available(self, max_size: int = 256) -> bytes:
        """Read bytes already announced as ready by select(), without waiting."""
        if self.serial is not None:
            waiting = min(int(self.serial.in_waiting), max_size)
            return self.serial.read(waiting) if waiting > 0 else b""
        return os.read(self.fd, max_size)

    def close(self) -> None:
        if self.serial is not None:
            self.serial.close()
            self.serial = None
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def __enter__(self) -> Uart:
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()


def median_frame(frames: list[np.ndarray]) -> np.ndarray:
    return np.median(np.stack(frames, axis=0), axis=0).astype(np.uint8)


def expanded_polygon(corners: np.ndarray, scale: float = 1.12) -> np.ndarray:
    center = corners.mean(axis=0)
    return np.round(center + (corners - center) * scale).astype(np.int32)


def line_angle(line: np.ndarray) -> float:
    x1, y1, x2, y2 = line.astype(float)
    return (np.degrees(np.arctan2(y2 - y1, x2 - x1)) + 90.0) % 180.0 - 90.0


def line_score(line: np.ndarray, center: np.ndarray, brightness: np.ndarray) -> float:
    x1, y1, x2, y2 = line.astype(float)
    direction = np.array((x2 - x1, y2 - y1), dtype=np.float64)
    length = float(np.linalg.norm(direction))
    distance = abs(float(np.cross(direction, center - np.array((x1, y1))) / length))
    count = max(2, int(length // 3))
    xs = np.clip(np.linspace(x1, x2, count).round().astype(int), 0, brightness.shape[1] - 1)
    ys = np.clip(np.linspace(y1, y2, count).round().astype(int), 0, brightness.shape[0] - 1)
    return length * (0.25 + float(brightness[ys, xs].mean()) / 100.0) ** 2 * np.exp(-distance / 60.0)


def extended_line(line: np.ndarray, prefer_positive_x: bool) -> tuple[np.ndarray, np.ndarray]:
    x1, y1, x2, y2 = line.astype(float)
    direction = np.array((x2 - x1, y2 - y1), dtype=np.float64)
    direction /= np.linalg.norm(direction)
    if (prefer_positive_x and direction[0] < 0) or (not prefer_positive_x and direction[1] < 0):
        direction *= -1.0
    midpoint = np.array(((x1 + x2) * 0.5, (y1 + y2) * 0.5), dtype=np.float64)
    return midpoint - direction * 2000.0, midpoint + direction * 2000.0


def intersection(first: tuple[np.ndarray, np.ndarray], second: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    first_direction = first[1] - first[0]
    second_direction = second[1] - second[0]
    try:
        distance = np.linalg.solve(
            np.column_stack((first_direction, -second_direction)), second[0] - first[0]
        )[0]
    except np.linalg.LinAlgError as error:
        raise RuntimeError("detected XY axes are parallel") from error
    return first[0] + distance * first_direction


def calibrate_xy(image: np.ndarray) -> AxisCalibration:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    region = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(region, expanded_polygon(XY_HINT), 255)
    edges = cv2.bitwise_and(cv2.Canny(gray, 35, 100), region)
    segments = cv2.HoughLinesP(edges, 1, np.pi / 360.0, 20, minLineLength=70, maxLineGap=20)
    if segments is None:
        raise RuntimeError("cannot find XY axes")

    horizontal = [line for line in segments[:, 0, :] if abs(line_angle(line)) <= 12.0]
    vertical = [line for line in segments[:, 0, :] if abs(line_angle(line)) >= 78.0]
    if not horizontal or not vertical:
        raise RuntimeError("cannot find both central XY axes")
    center_hint = XY_AXIS_CENTER_HINT
    brightness = gray.astype(np.float32)
    best_pair: tuple[float, np.ndarray, np.ndarray, tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray], np.ndarray] | None = None
    for horizontal_line in horizontal:
        h_line = extended_line(horizontal_line, prefer_positive_x=True)
        h_score = line_score(horizontal_line, center_hint, brightness)
        for vertical_line in vertical:
            v_line = extended_line(vertical_line, prefer_positive_x=False)
            try:
                center = intersection(h_line, v_line)
            except RuntimeError:
                continue
            center_offset = float(np.linalg.norm(center - center_hint))
            if center_offset > MAX_XY_AXIS_CENTER_OFFSET_PX:
                continue
            h_direction = h_line[1] - h_line[0]
            h_direction /= np.linalg.norm(h_direction)
            v_direction = v_line[1] - v_line[0]
            v_direction /= np.linalg.norm(v_direction)
            orthogonality = abs(float(h_direction[0] * v_direction[1] - h_direction[1] * v_direction[0]))
            pair_score = (h_score + line_score(vertical_line, center_hint, brightness)) * orthogonality * np.exp(-center_offset / 12.0)
            if best_pair is None or pair_score > best_pair[0]:
                best_pair = (pair_score, horizontal_line, vertical_line, h_line, v_line, center)
    if best_pair is None:
        raise RuntimeError("cannot select central XY axis pair")
    _, horizontal_line, vertical_line, h_line, v_line, center = best_pair
    image_height, image_width = image.shape[:2]
    if not (-20 <= center[0] <= image_width + 20 and -20 <= center[1] <= image_height + 20):
        raise RuntimeError("XY axis intersection is outside the image")

    h_direction = h_line[1] - h_line[0]
    h_direction /= np.linalg.norm(h_direction)
    v_direction = v_line[1] - v_line[0]
    v_direction /= np.linalg.norm(v_direction)
    h_span = 0.5 * (np.linalg.norm(XY_HINT[1] - XY_HINT[0]) + np.linalg.norm(XY_HINT[2] - XY_HINT[3]))
    v_span = 0.5 * (np.linalg.norm(XY_HINT[3] - XY_HINT[0]) + np.linalg.norm(XY_HINT[2] - XY_HINT[1]))
    corners = np.array((
        center - h_direction * h_span * 0.5 - v_direction * v_span * 0.5,
        center + h_direction * h_span * 0.5 - v_direction * v_span * 0.5,
        center + h_direction * h_span * 0.5 + v_direction * v_span * 0.5,
        center - h_direction * h_span * 0.5 + v_direction * v_span * 0.5,
    ))
    sine = abs(float(h_direction[0] * v_direction[1] - h_direction[1] * v_direction[0]))
    center_offset = float(np.linalg.norm(center - center_hint))
    confidence = min(1.0, (np.linalg.norm(horizontal_line[2:] - horizontal_line[:2]) + np.linalg.norm(vertical_line[2:] - vertical_line[:2])) / 260.0) * sine * np.exp(-center_offset / 12.0)
    return AxisCalibration(corners, center, horizontal_line, vertical_line, float(confidence))


def rectify(image: np.ndarray, corners: np.ndarray) -> np.ndarray:
    destination = np.array(((0, 0), (XY_SIZE - 1, 0), (XY_SIZE - 1, XY_SIZE - 1), (0, XY_SIZE - 1)), dtype=np.float32)
    transform = cv2.getPerspectiveTransform(corners.astype(np.float32), destination)
    return cv2.warpPerspective(image, transform, (XY_SIZE, XY_SIZE), flags=cv2.INTER_LINEAR)


def overlay_axes(image: np.ndarray, calibration: AxisCalibration) -> np.ndarray:
    overlay = image.copy()
    cv2.polylines(overlay, [calibration.corners.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
    for line, color in ((calibration.horizontal, (0, 220, 255)), (calibration.vertical, (255, 80, 0))):
        x1, y1, x2, y2 = (round(float(value)) for value in line)
        cv2.line(overlay, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
    cv2.circle(overlay, tuple(np.round(calibration.center).astype(int)), 5, (0, 255, 0), -1)
    return overlay


def write_image(path: Path, image: np.ndarray) -> None:
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"failed to write {path}")


def make_capture(
    frame: CameraFrame,
    calibration: AxisCalibration,
    capture_index: int,
    index: int,
    elapsed_ms: float,
    within_setting_ms: float,
    received_elapsed_ms: float,
    pipeline_warmup_ms: float,
    threshold: int,
) -> SweepCapture:
    """Rectify and retain a stable capture without encoding it to disk."""
    ramp_us = RAMPS_US[index]
    stem = f"{capture_index:03d}_{index:02d}_{ramp_us:04d}us"
    raw_name = f"{stem}.jpg"
    xy_name = f"{stem}_xy.png"
    trace_name = f"{stem}_trace.png"
    mask_name = f"{stem}_trace_mask.png"
    xy = rectify(frame.bgr, calibration.corners)
    record = {
        "index": index,
        "ramp_us": ramp_us,
        "frame_us": FRAME_US,
        "capture_index": capture_index,
        "captured_at_s": frame.timestamp_s,
        "sweep_elapsed_ms": elapsed_ms,
        "camera_received_elapsed_ms": received_elapsed_ms,
        "camera_pipeline_warmup_ms": pipeline_warmup_ms,
        "camera_flush_frames": CAMERA_PIPELINE_FLUSH_FRAMES,
        "camera_warmup_frames": CAMERA_PIPELINE_WARMUP_FRAMES,
        "within_setting_ms": within_setting_ms,
        "sweep_settle_ms": SETTLE_MS,
        "sweep_guard_ms": GUARD_MS,
        "stable_capture": True,
        "image": raw_name,
        "xy_image": f"xy/{xy_name}",
        "trace_image": f"trace/{trace_name}",
        "trace_mask": f"trace_mask/{mask_name}",
        "trace_threshold": threshold,
    }
    print(json.dumps(record), flush=True)
    return SweepCapture(record, frame.bgr, xy)


def largest_active_span(active: np.ndarray) -> tuple[int, int]:
    """Return the longest contiguous run of rows that contains the trace."""
    best_start = best_end = start = 0
    in_run = False
    for index, value in enumerate(active):
        if value and not in_run:
            start = index
            in_run = True
        elif not value and in_run:
            if index - start > best_end - best_start:
                best_start, best_end = start, index
            in_run = False
    if in_run and len(active) - start > best_end - best_start:
        best_start, best_end = start, len(active)
    if best_end - best_start < 100:
        raise RuntimeError("trace does not span enough of the XY image")
    return best_start, best_end


def spaced_turning_points(values: np.ndarray, minimum_separation: int) -> tuple[np.ndarray, np.ndarray]:
    """Find right and left vertices from a smoothed horizontal center line."""
    slope = np.sign(np.diff(values))
    for index in range(1, len(slope)):
        if slope[index] == 0:
            slope[index] = slope[index - 1]
    for index in range(len(slope) - 2, -1, -1):
        if slope[index] == 0:
            slope[index] = slope[index + 1]

    right_candidates: list[tuple[int, float]] = []
    left_candidates: list[tuple[int, float]] = []
    for index in range(1, len(slope)):
        vertex = index
        curvature = abs(float(values[vertex] * 2.0 - values[vertex - 1] - values[vertex + 1]))
        if slope[index - 1] > 0 and slope[index] < 0:
            right_candidates.append((vertex, curvature))
        elif slope[index - 1] < 0 and slope[index] > 0:
            left_candidates.append((vertex, curvature))

    def keep_separated(candidates: list[tuple[int, float]]) -> np.ndarray:
        selected: list[int] = []
        for position, _ in sorted(candidates, key=lambda item: item[1], reverse=True):
            if all(abs(position - existing) >= minimum_separation for existing in selected):
                selected.append(position)
        return np.asarray(sorted(selected), dtype=np.int32)

    return keep_separated(right_candidates), keep_separated(left_candidates)


def rolling_median(values: np.ndarray, width: int) -> np.ndarray:
    half_width = width // 2
    padded = np.pad(values, (half_width, half_width), mode="edge")
    # Keep this explicit loop for the Orange Pi's older NumPy build.
    return np.asarray(
        [np.median(padded[index:index + width]) for index in range(len(values))],
        dtype=np.float32,
    )


def analyse_trace(trace: np.ndarray, ramp_us: int) -> tuple[dict[str, object], np.ndarray]:
    """Validate one representative trace and estimate its visual cycle count."""
    gray = cv2.cvtColor(trace, cv2.COLOR_BGR2GRAY)
    response = cv2.GaussianBlur(gray, (11, 5), 0)
    row_strength = response.max(axis=1)
    start, end = largest_active_span(row_strength >= max(20.0, float(row_strength.max()) * 0.35))
    centers = response[start:end].argmax(axis=1).astype(np.float32)
    centers = rolling_median(centers, 9)
    centers = cv2.GaussianBlur(centers.reshape(1, -1), (15, 1), 0).reshape(-1)
    minimum_separation = max(18, len(centers) // 18)
    right, left = spaced_turning_points(centers, minimum_separation)
    total_vertices = int(len(right) + len(left))
    ordered_vertices = sorted(
        [(int(position), "right") for position in right] +
        [(int(position), "left") for position in left]
    )
    alternation_errors = sum(
        ordered_vertices[index][1] == ordered_vertices[index - 1][1]
        for index in range(1, len(ordered_vertices))
    )
    same_side_spacing = np.concatenate((np.diff(right), np.diff(left))).astype(np.float64)
    alternating_spacing = 2.0 * np.diff(
        np.asarray([position for position, _ in ordered_vertices], dtype=np.float64)
    )
    spacing = np.concatenate((same_side_spacing, alternating_spacing))
    result: dict[str, object] = {
        "ramp_us": ramp_us,
        "trace_span_px": int(end - start),
        "right_vertices_y": (right + start).tolist(),
        "left_vertices_y": (left + start).tolist(),
        "vertex_count": total_vertices,
        "accepted": False,
    }

    if end - start < MIN_TRACE_SPAN_PX:
        result["reason"] = f"trace span below {MIN_TRACE_SPAN_PX} px"
    elif not MIN_VERTEX_COUNT <= total_vertices <= MAX_VERTEX_COUNT:
        result["reason"] = f"vertex count outside {MIN_VERTEX_COUNT}..{MAX_VERTEX_COUNT}"
    elif abs(len(right) - len(left)) > MAX_SIDE_VERTEX_COUNT_DIFFERENCE:
        result["reason"] = "left/right vertex counts differ too much"
    elif alternation_errors > MAX_ALTERNATION_ERRORS:
        result["reason"] = "too many left/right alternation errors"
    elif len(spacing) < 2:
        result["reason"] = "not enough vertex intervals"
    else:
        median_spacing = float(np.median(spacing))
        spacing_cv = float(1.4826 * np.median(np.abs(spacing - median_spacing)) / median_spacing)
        result["spacing_px"] = median_spacing
        result["spacing_cv"] = spacing_cv
        if spacing_cv > MAX_SPACING_CV:
            result["reason"] = f"vertex spacing CV above {MAX_SPACING_CV:.2f}"
        else:
            cycles = float((end - start - 1) / median_spacing)
            uncalibrated_frequency_hz = cycles / (ramp_us * 1e-6)
            frequency_hz = (
                uncalibrated_frequency_hz * FREQUENCY_CALIBRATION_GAIN
                + FREQUENCY_CALIBRATION_OFFSET_HZ
            )
            result.update({
                "accepted": True,
                "visual_cycles": cycles,
                "uncalibrated_frequency_hz": uncalibrated_frequency_hz,
                "frequency_hz": frequency_hz,
                # Three vertices can establish a period but are less reliable
                # than several repeated periods across the whole trace.
                "confidence": float(
                    min(1.0, (total_vertices - 2) / 5.0) / (1.0 + spacing_cv)
                ),
            })

    overlay = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for vertex in right + start:
        cv2.circle(overlay, (round(float(centers[vertex - start])), int(vertex)), 4, (0, 255, 0), -1)
    for vertex in left + start:
        cv2.circle(overlay, (round(float(centers[vertex - start])), int(vertex)), 4, (0, 0, 255), -1)
    color = (0, 255, 0) if result["accepted"] else (0, 0, 255)
    cv2.putText(overlay, f"vertices={total_vertices}", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return result, overlay


def weighted_median(values: list[float], weights: list[float]) -> float:
    ordered = sorted(zip(values, weights), key=lambda item: item[0])
    half_weight = sum(weights) * 0.5
    cumulative = 0.0
    for value, weight in ordered:
        cumulative += weight
        if cumulative >= half_weight:
            return value
    return ordered[-1][0]


def synthesize_sweep_background(images: list[np.ndarray]) -> np.ndarray:
    """Build a grid-only reference from the current sweep's own XY images.

    The scope may retain a previous Lissajous trace when an attempt begins, so
    its first camera frame is not a reliable blank. Grid/axis pixels are fixed
    across all settings, while the trace moves; a low temporal quantile retains
    the former and removes the latter without relying on screen persistence.
    """
    if len(images) < len(RAMPS_US):
        raise RuntimeError("not enough rectified XY images for sweep background")
    return np.quantile(np.stack(images, axis=0), 0.20, axis=0).astype(np.uint8)


def analyse_sweep(
    output_dir: Path,
    captures: list[SweepCapture] | None = None,
    write_diagnostics: bool = True,
) -> dict[str, object]:
    """Choose the cleanest candidate in every setting.

    Live measurements pass captures directly, so no image codec or filesystem
    round trip occurs. Disk mode remains available for `--analyze-dir` and for
    archived failed attempts.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    from_memory = captures is not None
    if captures is None:
        manifest_path = output_dir / "manifest.jsonl"
        rows = [json.loads(line) for line in manifest_path.read_text(encoding="ascii").splitlines() if line]
        candidates: list[tuple[dict[str, object], np.ndarray]] = []
        for row in rows:
            xy = cv2.imread(str(output_dir / str(row["xy_image"])), cv2.IMREAD_COLOR)
            if xy is not None and xy.shape[:2] == (XY_SIZE, XY_SIZE):
                candidates.append((row, xy))
    else:
        candidates = [(capture.record, capture.xy) for capture in captures]

    background = synthesize_sweep_background([xy for _, xy in candidates])
    analysis_dir = output_dir / "analysis"
    if write_diagnostics:
        analysis_dir.mkdir(exist_ok=True)
        (output_dir / "trace").mkdir(exist_ok=True)
        (output_dir / "trace_mask").mkdir(exist_ok=True)
        write_image(analysis_dir / "xy_sweep_background.png", background)

    traces: dict[int, np.ndarray] = {}
    for candidate, xy in candidates:
        trace = cv2.subtract(xy, background)
        traces[id(candidate)] = trace
        trace_gray = cv2.cvtColor(trace, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(trace_gray, int(candidate["trace_threshold"]), 255, cv2.THRESH_BINARY)
        candidate["trace_foreground_pixels"] = int(np.count_nonzero(mask))
        if write_diagnostics:
            write_image(output_dir / str(candidate["trace_image"]), trace)
            write_image(output_dir / str(candidate["trace_mask"]), mask)

    selected: list[dict[str, object]] = []
    for index, ramp_us in enumerate(RAMPS_US):
        setting_candidates = [candidate for candidate, _ in candidates if candidate.get("index") == index]
        if not setting_candidates:
            selected.append({"index": index, "ramp_us": ramp_us, "accepted": False, "reason": "no stable capture"})
            continue
        analysed: list[tuple[float, dict[str, object], np.ndarray | None, dict[str, object]]] = []
        for candidate in setting_candidates:
            try:
                result, overlay = analyse_trace(traces[id(candidate)], ramp_us)
            except RuntimeError as error:
                result = {
                    "ramp_us": ramp_us,
                    "accepted": False,
                    "reason": str(error),
                }
                overlay = None
            time_distance_ms = abs(float(candidate["within_setting_ms"]) - REPRESENTATIVE_TARGET_MS)
            # Prefer a valid, geometrically clean curve. The time target only
            # breaks near-ties, so a residual-heavy third frame is not forced.
            score = (
                float(result["confidence"]) - time_distance_ms / 1000.0
                if result.get("accepted") else -1.0 - time_distance_ms / 1000.0
            )
            analysed.append((score, result, overlay, candidate))

        _, result, overlay, chosen = max(analysed, key=lambda item: item[0])
        if overlay is not None:
            overlay_name = f"{Path(str(chosen['image'])).stem}_vertices.png"
            if write_diagnostics:
                write_image(analysis_dir / overlay_name, overlay)
                result["vertex_overlay"] = f"analysis/{overlay_name}"
        result.update({
            "index": index,
            "image": chosen["image"],
            "within_setting_ms": chosen["within_setting_ms"],
            "representative_target_ms": REPRESENTATIVE_TARGET_MS,
            "candidate_count": len(setting_candidates),
            "selection_method": "best_vertex_fit",
        })
        selected.append(result)

    usable = [row for row in selected if row.get("accepted")]
    if len(usable) > 1:
        raw_reference_hz = float(np.median([
            float(row["uncalibrated_frequency_hz"]) for row in usable
        ]))
        for row in usable:
            deviation = abs(float(row["uncalibrated_frequency_hz"]) / raw_reference_hz - 1.0)
            row["frequency_relative_deviation"] = deviation
            if deviation > MAX_FREQUENCY_RELATIVE_DEVIATION:
                row["accepted"] = False
                row["reason"] = "frequency outlier across sweep settings"
        usable = [row for row in selected if row.get("accepted")]
    report: dict[str, object] = {
        "representative_target_ms": REPRESENTATIVE_TARGET_MS,
        "vertex_count_range": [MIN_VERTEX_COUNT, MAX_VERTEX_COUNT],
        "max_spacing_cv": MAX_SPACING_CV,
        "min_trace_span_px": MIN_TRACE_SPAN_PX,
        "max_side_vertex_count_difference": MAX_SIDE_VERTEX_COUNT_DIFFERENCE,
        "max_alternation_errors": MAX_ALTERNATION_ERRORS,
        "max_frequency_relative_deviation": MAX_FREQUENCY_RELATIVE_DEVIATION,
        "frequency_calibration_gain": FREQUENCY_CALIBRATION_GAIN,
        "frequency_calibration_offset_hz": FREQUENCY_CALIBRATION_OFFSET_HZ,
        "frequency_mode": "affine_calibrated",
        "trace_background_method": "current_sweep_pixelwise_q20",
        "trace_background_image": "analysis/xy_sweep_background.png" if write_diagnostics else None,
        "processing_mode": "memory" if from_memory else "disk",
        "settings": selected,
        "usable_settings": len(usable),
    }
    if usable:
        values = [float(row["frequency_hz"]) for row in usable]
        weights = [float(row["confidence"]) for row in usable]
        report["frequency_hz"] = weighted_median(values, weights)
        report["frequency_estimates_hz"] = values
        report["frequency_relative_spread"] = (
            (max(values) - min(values)) / float(np.median(values))
            if len(values) > 1 else 0.0
        )
    (output_dir / "frequency_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="ascii")
    print(json.dumps(report), flush=True)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fixed FPGA Q5 sweep: capture, calibrate, and process")
    parser.add_argument("--serial", default="/dev/ttyS2")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--sweep-cycles", type=int, default=1)
    parser.add_argument("--capture-fps", type=float, default=30.0)
    parser.add_argument("--trace-threshold", type=int, default=TRACE_THRESHOLD)
    parser.add_argument("--output-dir", type=Path, default=Path("q5_fpga_sweep"))
    parser.add_argument("--socket-path", default=SERVICE_SOCKET_PATH)
    parser.add_argument("--key-output-root", type=Path, default=KEY_MEASURE_OUTPUT_ROOT)
    parser.add_argument("--save-images", action="store_true", help="archive successful captures and diagnostics")
    parser.add_argument("--serve", action="store_true", help="hold the camera and UART open as a resident service")
    parser.add_argument("--trigger", action="store_true", help="request one measurement from the resident service")
    parser.add_argument("--phase-check", action="store_true", help="read the current compensated phase error")
    parser.add_argument(
        "--analyze-dir",
        type=Path,
        help="re-run only this fixed vertex analysis for an existing sweep directory",
    )
    return parser


def capture_sweep_attempt(
    uart: Uart,
    camera: Camera,
    sweep_cycles: int,
    trace_threshold: int,
    attempt: int,
) -> SweepAttempt:
    """Capture one complete sweep attempt in RAM."""
    print(json.dumps({"measurement_attempt": attempt, "processing_mode": "memory"}), flush=True)
    calibration_image = median_frame([camera.capture().bgr for _ in range(CALIBRATION_FRAMES)])
    calibration = calibrate_xy(calibration_image)
    if calibration.confidence < 0.65:
        raise RuntimeError(f"XY calibration confidence too low: {calibration.confidence:.3f}")
    blank = rectify(calibration_image, calibration.corners)
    print(json.dumps({
        "measurement_attempt": attempt,
        "xy_auto_calibration": True,
        "xy_center": calibration.center.tolist(),
        "xy_confidence": calibration.confidence,
        "xy_blank": "memory",
    }), flush=True)

    captures: list[SweepCapture] = []
    try:
        # Drain queued V4L2 frames before the FPGA timing reference starts.
        # Draining after SWEEP used up the first setting's capture window and
        # made valid 10 us images appear under later setting labels.
        frame_period_s = 1.0 / max(camera.fps, 1.0)
        time.sleep(CAMERA_PIPELINE_WARMUP_FRAMES * frame_period_s)
        for _ in range(CAMERA_PIPELINE_FLUSH_FRAMES):
            time.sleep(frame_period_s)
            camera.capture()
        reply = uart.command("SWEEP")
        if not reply.startswith("OK SWEEP"):
            raise RuntimeError(f"SWEEP failed: {reply!r}")
        started_at_s = time.monotonic()
        camera_pipeline_warmup_s = 0.0
        dwell_s = DWELL_MS / 1000.0
        total_s = len(RAMPS_US) * dwell_s * sweep_cycles
        capture_index = 0
        while True:
            frame = camera.capture()
            received_elapsed_s = frame.timestamp_s - started_at_s
            image_elapsed_s = received_elapsed_s
            if image_elapsed_s >= total_s:
                break
            if image_elapsed_s < 0.0:
                continue
            setting_position = image_elapsed_s / dwell_s
            index = int(setting_position) % len(RAMPS_US)
            within_setting_ms = (setting_position - int(setting_position)) * DWELL_MS
            if SETTLE_MS <= within_setting_ms <= DWELL_MS - GUARD_MS:
                captures.append(make_capture(
                    frame, calibration,
                    capture_index, index, image_elapsed_s * 1000.0, within_setting_ms,
                    received_elapsed_s * 1000.0, camera_pipeline_warmup_s * 1000.0,
                    trace_threshold,
                ))
                capture_index += 1
    finally:
        uart.command("IDLE")
    return SweepAttempt(captures, calibration_image, calibration, blank)


def persist_attempt(output_dir: Path, attempt: SweepAttempt) -> None:
    """Archive an in-memory attempt only when diagnostics are required."""
    output_dir.mkdir(parents=True, exist_ok=True)
    xy_dir = output_dir / "xy"
    xy_dir.mkdir(exist_ok=True)
    write_image(output_dir / "xy_axis_calibration.png", overlay_axes(attempt.calibration_image, attempt.calibration))
    write_image(output_dir / "xy_axis_calibrated.png", attempt.blank)
    write_image(output_dir / "xy_blank.png", attempt.blank)
    with (output_dir / "manifest.jsonl").open("w", encoding="ascii") as manifest:
        for capture in attempt.captures:
            record = capture.record
            write_image(output_dir / str(record["image"]), capture.bgr)
            write_image(output_dir / str(record["xy_image"]), capture.xy)
            manifest.write(json.dumps(record) + "\n")


def run_measurement(
    output_dir: Path,
    uart: Uart,
    camera: Camera,
    sweep_cycles: int,
    trace_threshold: int,
    save_images: bool = False,
) -> dict[str, object]:
    """Run one measurement request using already-open hardware handles."""
    output_dir.mkdir(parents=True, exist_ok=True)
    attempts: list[dict[str, object]] = []
    final_report: dict[str, object] | None = None
    for attempt in range(1, MAX_MEASUREMENT_ATTEMPTS + 1):
        attempt_dir = output_dir / f"attempt_{attempt:02d}"
        captured: SweepAttempt | None = None
        try:
            if attempt > 1:
                time.sleep(INTER_ATTEMPT_IDLE_SETTLE_S)
            captured = capture_sweep_attempt(uart, camera, sweep_cycles, trace_threshold, attempt)
            report = analyse_sweep(attempt_dir, captured.captures, write_diagnostics=False)
        except Exception as error:
            if captured is not None:
                persist_attempt(attempt_dir, captured)
                analyse_sweep(attempt_dir)
            attempts.append({
                "attempt": attempt,
                "directory": attempt_dir.name,
                "status": "capture_or_processing_failed",
                "reason": str(error),
            })
            print(json.dumps(attempts[-1]), flush=True)
            continue

        success = int(report.get("usable_settings", 0)) > 0 and "frequency_hz" in report
        attempts.append({
            "attempt": attempt,
            "directory": attempt_dir.name,
            "status": "success" if success else "no_valid_frequency",
            "usable_settings": report.get("usable_settings", 0),
        })
        if success:
            if save_images:
                persist_attempt(attempt_dir, captured)
                report = analyse_sweep(attempt_dir)
            final_report = report
            break

        # Failed candidate sets are preserved for later inspection, but their
        # large image arrays are released before the next hardware attempt.
        persist_attempt(attempt_dir, captured)
        analyse_sweep(attempt_dir)

    if final_report is None:
        final_report = {
            "frequency_mode": "affine_calibrated",
            "status": "failed_after_retry",
            "usable_settings": 0,
        }
    final_report["attempts"] = attempts
    final_report["attempts_used"] = len(attempts)
    final_report["recovered_by_retry"] = final_report.get("status") != "failed_after_retry" and len(attempts) > 1
    (output_dir / "frequency_report.json").write_text(
        json.dumps(final_report, indent=2) + "\n", encoding="ascii"
    )
    print(json.dumps(final_report), flush=True)
    return final_report


def key_measurement_output_dir(root: Path) -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    return root / f"q5_measure_{timestamp}"


def send_measurement_result(uart: Uart, report: dict[str, object]) -> str:
    frequency_hz = report.get("frequency_hz")
    result_hz = int(round(float(frequency_hz))) if frequency_hz is not None else 0
    reply = uart.command(f"RESULT {result_hz}")
    if not reply.startswith("OK RESULT"):
        raise RuntimeError(f"MCU rejected RESULT: {reply!r}")
    return reply


def read_reference_calibration(uart: Uart) -> dict[str, object]:
    """Read the last 100 kHz FPGA clock calibration retained by the MCU."""
    reply = uart.command("STATUS")
    fields: dict[str, str] = {}
    for token in reply.split():
        if "=" in token:
            key, value = token.split("=", 1)
            fields[key] = value
    try:
        calibration_done = int(fields.get("CAL", "0")) == 1
        ticks = int(fields.get("CTICKS", "0"))
    except ValueError:
        calibration_done = False
        ticks = 0
    valid = calibration_done and 24_000_000 <= ticks <= 26_000_000
    return {
        "valid": valid,
        "calibration_done": calibration_done,
        "ticks": ticks,
        "status_reply": reply,
    }


def service_loop(args: argparse.Namespace) -> int:
    """Serve local requests and KEY3 measurement events without reopening hardware."""
    socket_path = Path(args.socket_path)
    if socket_path.exists():
        socket_path.unlink()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(str(socket_path))
        os.chmod(socket_path, 0o600)
        listener.listen(1)
        listener.setblocking(False)
        print(json.dumps({"service": "ready", "socket": str(socket_path)}), flush=True)
        try:
            with Uart(args.serial) as uart, Camera(args.device, args.capture_fps) as camera:
                from q5_frequency_measure import (
                    CACHED_BLANK_MAX_AGE_S,
                    prepare_idle_cache,
                    run_frequency_measurement,
                )
                from q5_phase_lock import (
                    calibrate_double_phase,
                    capture_phase_error,
                    start_q5_phase_feedforward,
                    run_q5_phase_lock,
                )

                idle_cache = prepare_idle_cache(uart, camera)
                phase_servo = None

                def current_idle_cache():
                    nonlocal idle_cache
                    if time.monotonic() - idle_cache.captured_at_s > CACHED_BLANK_MAX_AGE_S:
                        idle_cache = prepare_idle_cache(uart, camera)
                    return idle_cache

                def apply_result_and_phase_lock(
                    report: dict[str, object], output_dir: Path,
                    save_images: bool = False, task_number: int = 1,
                ) -> dict[str, object]:
                    nonlocal phase_servo
                    if task_number not in KEY_MEASURE_TASKS:
                        raise ValueError(f"unsupported Q5 task {task_number}")
                    report["task_number"] = task_number
                    # A new classification replaces the output frequency, so
                    # never let an earlier visual servo compensate the new one.
                    phase_servo = None
                    report["mcu_result_reply"] = send_measurement_result(uart, report)
                    if "frequency_hz" not in report:
                        return report
                    try:
                        reference_calibration = read_reference_calibration(uart)
                        report["reference_calibration"] = reference_calibration
                        cache = current_idle_cache()
                        initial_lock = run_q5_phase_lock(
                            uart, camera, cache.corners, cache.blank
                        )
                        initial_lock["frequency_hz"] = report["frequency_hz"]
                        initial_lock["reference_calibration"] = reference_calibration
                        report["phase_lock"] = initial_lock
                        phase_servo, feedforward_report = start_q5_phase_feedforward(
                            uart, camera, cache.corners, cache.blank, initial_lock,
                            output_dir / "phase" if save_images else None,
                        )
                        report["phase_feedforward"] = feedforward_report
                    except Exception as error:
                        # Preserve the valid frequency result even if an XY
                        # camera frame is unsuitable for phase refinement.
                        report["phase_lock"] = {"status": "failed", "reason": str(error)}
                        return report

                    post_modes = {2: "CIRCLE", 3: "DOUBLE"}
                    if task_number not in post_modes:
                        report["post_output"] = {
                            "status": "running",
                            "mode": "DIAG",
                            "phase_offset_degrees": 0.0,
                        }
                        return report
                    try:
                        mode = post_modes[task_number]
                        reply = uart.command(f"AUTO {mode}")
                        if not reply.startswith("OK AUTO"):
                            raise RuntimeError(
                                f"MCU rejected task {task_number} output: {reply!r}"
                            )
                        transform = phase_servo.apply_output_transform(
                            2.0 if task_number == 3 else 1.0
                        )
                        double_phase_calibration = None
                        if task_number == 3:
                            try:
                                double_phase_calibration = calibrate_double_phase(
                                    phase_servo,
                                    camera,
                                    cache.corners,
                                    cache.blank,
                                    output_dir / "phase" if save_images else None,
                                )
                            except Exception as calibration_error:
                                double_phase_calibration = {
                                    "status": "failed",
                                    "reason": str(calibration_error),
                                }
                        report["post_output"] = {
                            "status": "running",
                            "mode": mode,
                            "phase_offset_degrees": 90.0 if task_number == 2 else 0.0,
                            "mcu_reply": reply,
                            **transform,
                        }
                        if double_phase_calibration is not None:
                            report["post_output"]["double_phase_calibration"] = (
                                double_phase_calibration
                            )
                    except Exception as error:
                        report["post_output"] = {
                            "status": "failed",
                            "reason": str(error),
                        }
                    return report

                def run_key_measurement_event(task_number: int) -> None:
                    output_dir = key_measurement_output_dir(args.key_output_root)
                    print(json.dumps({
                        "key_measure": True,
                        "task_number": task_number,
                        "output_dir": str(output_dir),
                    }), flush=True)
                    try:
                        report = run_frequency_measurement(
                            output_dir, uart, camera, idle_cache=current_idle_cache()
                        )
                        report = apply_result_and_phase_lock(
                            report, output_dir, task_number=task_number
                        )
                        print(json.dumps({"key_measure_complete": True, "report": report}), flush=True)
                    except Exception as error:
                        try:
                            send_measurement_result(uart, {})
                        except Exception:
                            pass
                        print(json.dumps({"key_measure_error": str(error)}), flush=True)

                uart_line = bytearray()
                while True:
                    timeout_s = None
                    if phase_servo is not None:
                        timeout_s = max(
                            0.0,
                            min(phase_servo.next_due_s, phase_servo.next_visual_due_s)
                            - time.monotonic(),
                        )
                    if uart.pending_events:
                        timeout_s = 0.0
                    ready, _, _ = select.select([listener, uart.fileno()], [], [], timeout_s)
                    if listener in ready:
                        connection, _ = listener.accept()
                        with connection, connection.makefile("rwb") as stream:
                            request_line = stream.readline(8192)
                            try:
                                request = json.loads(request_line.decode("ascii"))
                                command = str(request.get("command", "")).upper()
                                if command == "PING":
                                    response: dict[str, object] = {"status": "ready"}
                                elif command == "MEASURE":
                                    output_value = request.get("output_dir")
                                    if not isinstance(output_value, str) or not output_value:
                                        raise ValueError("MEASURE requires output_dir")
                                    task_number = int(request.get("task_number", 1))
                                    response = run_frequency_measurement(
                                        Path(output_value), uart, camera,
                                        bool(request.get("save_images", False)),
                                        current_idle_cache(),
                                    )
                                    response = apply_result_and_phase_lock(
                                        response, Path(output_value),
                                        bool(request.get("save_images", False)), task_number,
                                    )
                                elif command == "PHASE_CHECK":
                                    if phase_servo is None:
                                        raise RuntimeError("phase compensation is not running")
                                    if time.monotonic() >= phase_servo.next_due_s:
                                        phase_servo.step()
                                    phase_error, features = capture_phase_error(
                                        camera, idle_cache.corners, idle_cache.blank
                                    )
                                    response = {
                                        "status": "ok",
                                        "phase_error_degrees": phase_error,
                                        "phase_degrees": phase_servo.phase_at(),
                                        "compensation_rate_degrees_per_s": (
                                            phase_servo.compensation_rate_degrees_per_s
                                        ),
                                        "features": features,
                                    }
                                else:
                                    raise ValueError("unsupported command")
                            except Exception as error:
                                response = {"status": "service_error", "reason": str(error)}
                                try:
                                    response["mcu_result_reply"] = send_measurement_result(uart, {})
                                except Exception as reply_error:
                                    response["mcu_result_reply"] = f"failed: {reply_error}"
                            stream.write((json.dumps(response) + "\n").encode("ascii"))
                            stream.flush()
                    if uart.fileno() in ready:
                        uart_line.extend(uart.read_available())
                        while b"\n" in uart_line:
                            raw_line, _, remainder = uart_line.partition(b"\n")
                            uart_line = bytearray(remainder)
                            task_number = parse_key_measure_event(raw_line.rstrip(b"\r"))
                            if task_number is not None:
                                run_key_measurement_event(task_number)
                    while True:
                        task_number = uart.pop_event()
                        if task_number is None:
                            break
                        run_key_measurement_event(task_number)
                    if phase_servo is not None and time.monotonic() >= phase_servo.next_due_s:
                        try:
                            phase_servo.step()
                        except Exception as error:
                            # A UART transient must not stall the service or
                            # leave it in a tight retry loop.
                            phase_servo.next_due_s = time.monotonic() + 0.5
                            print(json.dumps({
                                "phase_feedforward": {"status": "retry", "reason": str(error)}
                            }), flush=True)
                    if phase_servo is not None and time.monotonic() >= phase_servo.next_visual_due_s:
                        try:
                            visual_report = phase_servo.visual_step(
                                camera, idle_cache.corners, idle_cache.blank
                            )
                            print(json.dumps({"phase_visual_servo": visual_report}), flush=True)
                        except Exception as error:
                            phase_servo.next_visual_due_s = time.monotonic() + 0.5
                            print(json.dumps({
                                "phase_visual_servo": {"status": "retry", "reason": str(error)}
                            }), flush=True)
        finally:
            try:
                socket_path.unlink()
            except FileNotFoundError:
                pass


def trigger_measurement(args: argparse.Namespace) -> int:
    request = {
        "command": "MEASURE",
        "output_dir": str(args.output_dir.resolve()),
        "save_images": args.save_images,
    }
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.connect(args.socket_path)
        with client.makefile("rwb") as stream:
            stream.write((json.dumps(request) + "\n").encode("ascii"))
            stream.flush()
            response_line = stream.readline(65536)
    if not response_line:
        raise RuntimeError("resident service closed without a response")
    response = json.loads(response_line.decode("ascii"))
    print(json.dumps(response), flush=True)
    return 0 if response.get("status") != "failed_after_retry" and "frequency_hz" in response else 2


def trigger_phase_check(args: argparse.Namespace) -> int:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.connect(args.socket_path)
        with client.makefile("rwb") as stream:
            stream.write(b'{"command":"PHASE_CHECK"}\n')
            stream.flush()
            response_line = stream.readline(65536)
    if not response_line:
        raise RuntimeError("resident service closed without a response")
    response = json.loads(response_line.decode("ascii"))
    print(json.dumps(response), flush=True)
    return 0 if response.get("status") == "ok" else 2


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if sum((args.analyze_dir is not None, args.serve, args.trigger, args.phase_check)) > 1:
        raise ValueError("choose only one operating mode")
    if args.analyze_dir is not None:
        analyse_sweep(args.analyze_dir)
        return 0
    if args.sweep_cycles < 1:
        raise ValueError("sweep-cycles must be positive")
    if args.capture_fps <= 0:
        raise ValueError("capture-fps must be positive")
    if not 1 <= args.trace_threshold <= 255:
        raise ValueError("trace-threshold must be within 1..255")
    if args.serve:
        return service_loop(args)
    if args.trigger:
        return trigger_measurement(args)
    if args.phase_check:
        return trigger_phase_check(args)
    with Uart(args.serial) as uart, Camera(args.device, args.capture_fps) as camera:
        report = run_measurement(
            args.output_dir, uart, camera, args.sweep_cycles, args.trace_threshold, args.save_images
        )
    return 0 if report.get("status") != "failed_after_retry" else 2


if __name__ == "__main__":
    raise SystemExit(main())
