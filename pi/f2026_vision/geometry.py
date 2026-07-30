from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

from .config import LocatorConfig


def _order_corners(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    sums = points.sum(axis=1)
    differences = points[:, 0] - points[:, 1]
    return np.array(
        [
            points[np.argmin(sums)],
            points[np.argmax(differences)],
            points[np.argmax(sums)],
            points[np.argmin(differences)],
        ],
        dtype=np.float32,
    )


@dataclass(frozen=True)
class ScopeCalibration:
    """Four graticule corners in camera coordinates, ordered TL/TR/BR/BL."""

    corners: np.ndarray
    output_width: int = 500
    output_height: int = 380

    def __post_init__(self) -> None:
        ordered = _order_corners(self.corners)
        object.__setattr__(self, "corners", ordered)

    @property
    def destination(self) -> np.ndarray:
        return np.float32(
            [
                [0, 0],
                [self.output_width - 1, 0],
                [self.output_width - 1, self.output_height - 1],
                [0, self.output_height - 1],
            ]
        )

    @property
    def matrix(self) -> np.ndarray:
        return cv2.getPerspectiveTransform(self.corners, self.destination)

    def warp(self, frame: np.ndarray) -> np.ndarray:
        return cv2.warpPerspective(
            frame,
            self.matrix,
            (self.output_width, self.output_height),
            flags=cv2.INTER_LINEAR,
        )

    def save(self, path: str | Path) -> None:
        payload = {
            "corners": self.corners.tolist(),
            "output_width": self.output_width,
            "output_height": self.output_height,
        }
        Path(path).write_text(json.dumps(payload, indent=2), encoding="ascii")

    @classmethod
    def load(cls, path: str | Path) -> "ScopeCalibration":
        payload = json.loads(Path(path).read_text(encoding="ascii"))
        return cls(
            corners=np.asarray(payload["corners"], dtype=np.float32),
            output_width=int(payload["output_width"]),
            output_height=int(payload["output_height"]),
        )


class ScopeLocator:
    """Locate the oscilloscope graticule without external fiducial markers."""

    def __init__(self, config: LocatorConfig | None = None) -> None:
        self.config = config or LocatorConfig()

    def locate(self, frames: np.ndarray | Iterable[np.ndarray]) -> ScopeCalibration:
        image = self._median_image(frames)
        mask = self._neutral_grid_mask(image, suppress_colored_trace=True)
        try:
            rough_corners = self._largest_grid_component(mask)
            corners = self._refine_outer_lines(mask, rough_corners)
        except RuntimeError:
            # A saved image may already contain a dense trace which breaks the
            # neutral grid at many intersections. This fallback is sufficient
            # for diagnostics, but live high-range measurement should always
            # calibrate from trace-free frames.
            mask = self._neutral_grid_mask(image, suppress_colored_trace=False)
            corners = self._largest_grid_component(mask)
        self._validate(corners, image.shape[1], image.shape[0])
        return ScopeCalibration(
            corners,
            self.config.output_width,
            self.config.output_height,
        )

    @staticmethod
    def _median_image(frames: np.ndarray | Iterable[np.ndarray]) -> np.ndarray:
        if isinstance(frames, np.ndarray) and frames.ndim == 3:
            return frames
        frame_list = [np.asarray(frame) for frame in frames]
        if not frame_list:
            raise ValueError("at least one calibration frame is required")
        return np.median(np.stack(frame_list), axis=0).astype(np.uint8)

    def _neutral_grid_mask(
        self, image: np.ndarray, suppress_colored_trace: bool = True
    ) -> np.ndarray:
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("locator expects an 8-bit BGR image")

        # A scope grid is approximately neutral grey. Taking the minimum color
        # channel suppresses a bright green/yellow trace before thresholding.
        minimum_channel = np.min(image, axis=2)
        maximum_channel = np.max(image, axis=2)
        neutral = minimum_channel.astype(np.uint8)
        background = float(np.percentile(neutral, 45.0))
        threshold = int(min(250, background + self.config.neutral_threshold_delta))
        if suppress_colored_trace:
            low_chroma = (
                maximum_channel.astype(np.int16) - minimum_channel.astype(np.int16)
            ) <= 35
        else:
            low_chroma = np.ones_like(neutral, dtype=bool)
        mask = ((neutral >= threshold) & low_chroma).astype(np.uint8) * 255
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        )
        return mask

    def _largest_grid_component(self, mask: np.ndarray) -> np.ndarray:
        connected_mask = cv2.dilate(
            mask,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
            iterations=1,
        )
        count, labels, stats, _ = cv2.connectedComponentsWithStats(connected_mask, 8)
        height, width = mask.shape
        candidates: list[tuple[float, int]] = []

        for label in range(1, count):
            x, y, box_width, box_height, area = stats[label]
            if box_width < width * self.config.minimum_span_fraction:
                continue
            if box_height < height * self.config.minimum_span_fraction:
                continue
            span_score = float(box_width * box_height)
            candidates.append((span_score + float(area), label))

        if not candidates:
            raise RuntimeError("oscilloscope grid not found")

        label = max(candidates)[1]
        ys, xs = np.nonzero(labels == label)
        points = np.column_stack((xs, ys)).astype(np.float32)
        hull = cv2.convexHull(points.reshape(-1, 1, 2))
        perimeter = cv2.arcLength(hull, True)
        polygon = cv2.approxPolyDP(hull, 0.018 * perimeter, True)

        if polygon.shape[0] == 4:
            return _order_corners(polygon.reshape(4, 2))

        # This fallback is stable for the small camera angles used by the
        # fixture and avoids depending on four perfect outer-grid segments.
        rectangle = cv2.boxPoints(cv2.minAreaRect(points))
        return _order_corners(rectangle)

    @staticmethod
    def _refine_outer_lines(mask: np.ndarray, corners: np.ndarray) -> np.ndarray:
        """Fit the four outer grid lines to sub-pixel precision."""
        ys, xs = np.nonzero(mask)
        points = np.column_stack((xs, ys)).astype(np.float64)
        fitted_lines: list[tuple[np.ndarray, np.ndarray]] = []

        for start, end in zip(corners, np.roll(corners, -1, axis=0)):
            edge = end.astype(np.float64) - start.astype(np.float64)
            length = float(np.linalg.norm(edge))
            direction = edge / max(length, 1e-9)
            relative = points - start
            projection = relative @ direction
            distance = np.abs(relative[:, 0] * direction[1] - relative[:, 1] * direction[0])
            selected = points[
                (distance <= 3.0)
                & (projection >= 0.04 * length)
                & (projection <= 0.96 * length)
            ]
            if selected.shape[0] < 20:
                return corners
            vx, vy, x0, y0 = cv2.fitLine(
                selected.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01
            ).reshape(4)
            fitted_lines.append(
                (
                    np.array([x0, y0], dtype=np.float64),
                    np.array([vx, vy], dtype=np.float64),
                )
            )

        def intersection(
            first: tuple[np.ndarray, np.ndarray],
            second: tuple[np.ndarray, np.ndarray],
        ) -> np.ndarray:
            p1, d1 = first
            p2, d2 = second
            matrix = np.column_stack((d1, -d2))
            if abs(float(np.linalg.det(matrix))) < 1e-6:
                raise RuntimeError("grid boundary lines are nearly parallel incorrectly")
            parameters = np.linalg.solve(matrix, p2 - p1)
            return p1 + parameters[0] * d1

        top, right, bottom, left = fitted_lines
        return _order_corners(
            np.array(
                [
                    intersection(left, top),
                    intersection(top, right),
                    intersection(right, bottom),
                    intersection(bottom, left),
                ],
                dtype=np.float32,
            )
        )

    def _validate(self, corners: np.ndarray, image_width: int, image_height: int) -> None:
        top = np.linalg.norm(corners[1] - corners[0])
        bottom = np.linalg.norm(corners[2] - corners[3])
        left = np.linalg.norm(corners[3] - corners[0])
        right = np.linalg.norm(corners[2] - corners[1])
        mean_width = 0.5 * (top + bottom)
        mean_height = 0.5 * (left + right)
        area = abs(float(cv2.contourArea(corners)))

        if area < image_width * image_height * self.config.minimum_area_fraction:
            raise RuntimeError("detected grid is too small")
        if mean_height <= 1.0:
            raise RuntimeError("detected grid has invalid height")
        aspect = mean_width / mean_height
        if not (1.0 / self.config.maximum_aspect_ratio <= aspect <= self.config.maximum_aspect_ratio):
            raise RuntimeError(f"implausible grid aspect ratio: {aspect:.3f}")
