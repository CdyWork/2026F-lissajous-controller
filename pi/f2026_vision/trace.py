from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .config import TraceConfig


@dataclass(frozen=True)
class TraceObservation:
    u: np.ndarray
    x: np.ndarray
    mask: np.ndarray
    valid_fraction: float
    mean_row_spread_px: float


class TraceExtractor:
    def __init__(self, config: TraceConfig | None = None) -> None:
        self.config = config or TraceConfig()

    def extract(
        self,
        rectified_bgr: np.ndarray,
        background_bgr: np.ndarray | None = None,
    ) -> TraceObservation:
        frame = np.asarray(rectified_bgr)
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("trace extractor expects an 8-bit BGR image")
        if background_bgr is not None and background_bgr.shape != frame.shape:
            raise ValueError("background and frame dimensions must match")

        blue, green, red = cv2.split(frame)
        green_i = green.astype(np.int16)
        green_score = green_i - np.maximum(red.astype(np.int16), blue.astype(np.int16))
        color_mask = (
            (green >= self.config.green_minimum)
            & (green_i - red.astype(np.int16) >= self.config.green_over_red)
            & (green_i - blue.astype(np.int16) >= self.config.green_over_blue)
        )

        if background_bgr is None:
            mask_bool = color_mask
            weights = np.maximum(green_score, 1).astype(np.float64)
        else:
            difference = cv2.absdiff(frame, background_bgr)
            difference_score = np.max(difference, axis=2)
            difference_mask = difference_score >= self.config.background_threshold
            # Background subtraction handles yellow/cyan traces. The color
            # condition rejects most LCD refresh noise when the trace is green.
            mask_bool = difference_mask & (color_mask | (difference_score >= 48))
            weights = np.maximum(difference_score, 1).astype(np.float64)

        mask = mask_bool.astype(np.uint8) * 255
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        )
        mask_bool = mask > 0

        margin = self.config.edge_margin_rows
        if frame.shape[0] <= 2 * margin + 4:
            raise ValueError("rectified image is too short")
        row_slice = slice(margin, frame.shape[0] - margin)
        selected = mask_bool[row_slice]
        selected_weights = weights[row_slice] * selected
        x_axis = np.arange(frame.shape[1], dtype=np.float64)
        row_weight = selected_weights.sum(axis=1)
        valid = row_weight > 0.0
        valid_fraction = float(np.mean(valid))

        if valid_fraction < self.config.minimum_valid_fraction:
            raise RuntimeError(
                f"insufficient trace rows: {valid_fraction:.1%} "
                f"< {self.config.minimum_valid_fraction:.1%}"
            )

        centers = np.full(row_weight.size, np.nan, dtype=np.float64)
        centers[valid] = (selected_weights[valid] @ x_axis) / row_weight[valid]

        second_moment = np.zeros_like(row_weight)
        second_moment[valid] = (selected_weights[valid] @ (x_axis * x_axis)) / row_weight[valid]
        variances = np.maximum(second_moment[valid] - centers[valid] ** 2, 0.0)
        mean_spread = float(np.mean(np.sqrt(variances)))
        if mean_spread > self.config.maximum_mean_row_spread_px:
            raise RuntimeError(
                f"trace support is implausibly wide: row spread {mean_spread:.2f} px"
            )

        indices = np.arange(centers.size)
        centers = np.interp(indices, indices[valid], centers[valid])
        centers = centers[::-1]
        image_rows = np.arange(margin, frame.shape[0] - margin, dtype=np.float64)
        u = ((frame.shape[0] - 1.0 - image_rows) / (frame.shape[0] - 1.0))[::-1]
        return TraceObservation(u, centers, mask, valid_fraction, mean_spread)
