"""Axis-driven extraction of the oscilloscope XY graticule.

The central horizontal and vertical axes are the calibration reference.  They
set the output origin and directions; a previous four-corner calibration is
used only as a loose search/crop envelope.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


DEFAULT_XY_HINT = "194,202,412,216,416,413,204,410"


@dataclass(frozen=True)
class XYAxisCalibration:
    """Source quadrilateral and detected central XY axes."""

    corners: tuple[float, float, float, float, float, float, float, float]
    center: tuple[float, float]
    horizontal: tuple[float, float, float, float]
    vertical: tuple[float, float, float, float]
    confidence: float


def parse_corners(text: str) -> tuple[float, float, float, float, float, float, float, float]:
    try:
        values = tuple(float(value.strip()) for value in text.split(","))
    except ValueError as error:
        raise ValueError("corners must contain eight comma-separated values") from error
    if len(values) != 8:
        raise ValueError("corners must contain four x,y point pairs")
    return values  # type: ignore[return-value]


def rectify_xy(
    image_bgr: np.ndarray,
    corners: tuple[float, float, float, float, float, float, float, float],
    width: int = 400,
    height: int = 400,
) -> np.ndarray:
    if width < 32 or height < 32:
        raise ValueError("rectified XY dimensions must be at least 32 pixels")
    source = np.asarray(corners, dtype=np.float32).reshape((4, 2))
    destination = np.array(
        ((0, 0), (width - 1, 0), (width - 1, height - 1), (0, height - 1)),
        dtype=np.float32,
    )
    transform = cv2.getPerspectiveTransform(source, destination)
    return cv2.warpPerspective(image_bgr, transform, (width, height), flags=cv2.INTER_LINEAR)


def median_frame(images: list[np.ndarray]) -> np.ndarray:
    if not images:
        raise ValueError("at least one image is required")
    shape = images[0].shape
    if any(image.shape != shape for image in images):
        raise ValueError("all calibration frames must have the same shape")
    return np.median(np.stack(images, axis=0), axis=0).astype(np.uint8)


def _expanded_polygon(corners: np.ndarray, scale: float) -> np.ndarray:
    center = corners.mean(axis=0)
    return np.round(center + (corners - center) * scale).astype(np.int32)


def _axis_mask(image: np.ndarray, hint: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Build line candidates from the graticule, independent of axis colour."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    polygon = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(polygon, _expanded_polygon(hint, 1.12), 255)
    edges = cv2.Canny(gray, 35, 100)
    return cv2.bitwise_and(edges, polygon), gray.astype(np.float32)


def _line_angle(line: np.ndarray) -> float:
    x1, y1, x2, y2 = line.astype(float)
    angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
    return (angle + 90.0) % 180.0 - 90.0


def _line_score(line: np.ndarray, center: np.ndarray, brightness_image: np.ndarray) -> float:
    x1, y1, x2, y2 = line.astype(float)
    length = float(np.hypot(x2 - x1, y2 - y1))
    # A central axis can be split by the waveform, so its midpoint is not a
    # useful centre estimate.  Score the distance from the *infinite* line to
    # the expected graticule centre; this rejects status-bar lines.  The
    # central axes are solid and brighter than the dotted grid lines.
    direction = np.array((x2 - x1, y2 - y1), dtype=np.float64)
    offset = center - np.array((x1, y1))
    distance = abs(float((direction[0] * offset[1] - direction[1] * offset[0]) / np.linalg.norm(direction)))
    samples = max(2, int(length // 3))
    xs = np.clip(np.linspace(x1, x2, samples).round().astype(int), 0, brightness_image.shape[1] - 1)
    ys = np.clip(np.linspace(y1, y2, samples).round().astype(int), 0, brightness_image.shape[0] - 1)
    brightness = float(brightness_image[ys, xs].mean())
    return length * (0.25 + brightness / 100.0) ** 2 * np.exp(-distance / 60.0)


def _extend_line(line: np.ndarray, length: float, prefer_positive_x: bool) -> tuple[np.ndarray, np.ndarray]:
    x1, y1, x2, y2 = line.astype(float)
    direction = np.array((x2 - x1, y2 - y1), dtype=np.float64)
    direction /= np.linalg.norm(direction)
    if (prefer_positive_x and direction[0] < 0) or (not prefer_positive_x and direction[1] < 0):
        direction *= -1.0
    midpoint = np.array(((x1 + x2) * 0.5, (y1 + y2) * 0.5), dtype=np.float64)
    return midpoint - direction * length, midpoint + direction * length


def _intersection(first: tuple[np.ndarray, np.ndarray], second: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    first_point, first_end = first
    second_point, second_end = second
    first_direction = first_end - first_point
    second_direction = second_end - second_point
    matrix = np.column_stack((first_direction, -second_direction))
    try:
        scale = np.linalg.solve(matrix, second_point - first_point)[0]
    except np.linalg.LinAlgError as error:
        raise RuntimeError("detected XY axes are parallel") from error
    return first_point + scale * first_direction


def detect_xy_axes(
    image_bgr: np.ndarray,
    hint_corners: tuple[float, float, float, float, float, float, float, float] | str = DEFAULT_XY_HINT,
) -> XYAxisCalibration:
    """Find central XY axes and build a centered, rotation-corrected crop."""
    hint_values = parse_corners(hint_corners) if isinstance(hint_corners, str) else hint_corners
    hint = np.asarray(hint_values, dtype=np.float64).reshape((4, 2))
    mask, brightness = _axis_mask(image_bgr, hint)
    segments = cv2.HoughLinesP(mask, 1, np.pi / 360.0, 20, minLineLength=70, maxLineGap=20)
    if segments is None:
        raise RuntimeError("cannot find XY axes: no graticule segments")

    center_hint = hint.mean(axis=0)
    horizontal_candidates: list[np.ndarray] = []
    vertical_candidates: list[np.ndarray] = []
    for segment in segments[:, 0, :]:
        angle = abs(_line_angle(segment))
        # The camera mounting allows a few degrees of roll.  Wider ranges
        # would admit the sloped cyan waveform when it crosses the centre.
        if angle <= 10.0:
            horizontal_candidates.append(segment)
        elif angle >= 80.0:
            vertical_candidates.append(segment)
    if not horizontal_candidates or not vertical_candidates:
        raise RuntimeError("cannot find both central XY axes")

    horizontal = max(horizontal_candidates, key=lambda line: _line_score(line, center_hint, brightness))
    vertical = max(vertical_candidates, key=lambda line: _line_score(line, center_hint, brightness))
    horizontal_line = _extend_line(horizontal, 2000.0, prefer_positive_x=True)
    vertical_line = _extend_line(vertical, 2000.0, prefer_positive_x=False)
    center = _intersection(horizontal_line, vertical_line)

    # The axes set position and rotation.  The loose hint supplies only the
    # graticule span (10 horizontal and 8 vertical divisions) for framing.
    h_span = 0.5 * (np.linalg.norm(hint[1] - hint[0]) + np.linalg.norm(hint[2] - hint[3]))
    v_span = 0.5 * (np.linalg.norm(hint[3] - hint[0]) + np.linalg.norm(hint[2] - hint[1]))
    h_direction = horizontal_line[1] - horizontal_line[0]
    h_direction /= np.linalg.norm(h_direction)
    v_direction = vertical_line[1] - vertical_line[0]
    v_direction /= np.linalg.norm(v_direction)
    half_h = h_span * 0.5
    half_v = v_span * 0.5
    source = np.array(
        (
            center - h_direction * half_h - v_direction * half_v,
            center + h_direction * half_h - v_direction * half_v,
            center + h_direction * half_h + v_direction * half_v,
            center - h_direction * half_h + v_direction * half_v,
        )
    )

    image_height, image_width = image_bgr.shape[:2]
    if not (-20 <= center[0] <= image_width + 20 and -20 <= center[1] <= image_height + 20):
        raise RuntimeError("detected XY axis intersection is outside the camera image")
    sine = abs(float(h_direction[0] * v_direction[1] - h_direction[1] * v_direction[0]))
    confidence = float(min(1.0, (np.linalg.norm(horizontal[2:] - horizontal[:2]) + np.linalg.norm(vertical[2:] - vertical[:2])) / 260.0) * sine)
    corners = tuple(float(value) for value in source.reshape(-1))
    return XYAxisCalibration(
        corners=corners, center=(float(center[0]), float(center[1])),
        horizontal=tuple(float(value) for value in horizontal),
        vertical=tuple(float(value) for value in vertical), confidence=confidence,
    )


def draw_detection(image_bgr: np.ndarray, calibration: XYAxisCalibration) -> np.ndarray:
    preview = image_bgr.copy()
    corners = np.asarray(calibration.corners, dtype=np.int32).reshape((4, 2))
    cv2.polylines(preview, [corners], True, (0, 255, 0), 2, cv2.LINE_AA)
    for line, color in ((calibration.horizontal, (0, 220, 255)), (calibration.vertical, (255, 80, 0))):
        x1, y1, x2, y2 = (round(value) for value in line)
        cv2.line(preview, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
    cv2.circle(preview, (round(calibration.center[0]), round(calibration.center[1])), 5, (0, 255, 0), -1)
    return preview


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Extract XY view from central scope axes")
    parser.add_argument("images", type=Path, nargs="+", help="fresh full camera JPG/PNG frames")
    parser.add_argument("--hint-corners", default=DEFAULT_XY_HINT, help="loose TL,TR,BR,BL crop envelope")
    parser.add_argument("--overlay", type=Path, default=Path("xy_axes_overlay.png"))
    parser.add_argument("--output", type=Path, default=Path("xy_axes_rectified.png"))
    args = parser.parse_args(argv)
    images = [cv2.imread(str(path)) for path in args.images]
    if any(image is None for image in images):
        missing = next(str(path) for path, image in zip(args.images, images) if image is None)
        raise RuntimeError(f"cannot read {missing}")
    frame = median_frame([image for image in images if image is not None])
    calibration = detect_xy_axes(frame, args.hint_corners)
    if not cv2.imwrite(str(args.overlay), draw_detection(frame, calibration)):
        raise RuntimeError(f"cannot write {args.overlay}")
    if not cv2.imwrite(str(args.output), rectify_xy(frame, calibration.corners)):
        raise RuntimeError(f"cannot write {args.output}")
    print("XY_CORNERS=" + ",".join(f"{value:.2f}" for value in calibration.corners))
    print(f"XY_CENTER={calibration.center[0]:.2f},{calibration.center[1]:.2f}")
    print(f"CONFIDENCE={calibration.confidence:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
