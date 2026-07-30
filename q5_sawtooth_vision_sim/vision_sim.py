from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np


@dataclass(frozen=True)
class SimulationConfig:
    width: int = 640
    height: int = 480
    scope_left: int = 70
    scope_right: int = 570
    scope_top: int = 50
    scope_bottom: int = 430
    frame_seconds: float = 0.010
    ramp_seconds: float = 0.001
    accumulated_frames: int = 3
    source_clock_ppm: float = 5.0
    sensor_noise_sigma: float = 3.0
    trace_blur_sigma: float = 0.7


def draw_scope(frequency_hz: float, cfg: SimulationConfig, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    image = np.zeros((cfg.height, cfg.width, 3), dtype=np.uint8)
    image[:] = (7, 10, 7)

    grid_color = (35, 43, 35)
    axis_color = (54, 61, 54)
    for index in range(11):
        x = round(cfg.scope_left + index * (cfg.scope_right - cfg.scope_left) / 10)
        cv2.line(image, (x, cfg.scope_top), (x, cfg.scope_bottom),
                 axis_color if index == 5 else grid_color, 1, cv2.LINE_AA)
    for index in range(9):
        y = round(cfg.scope_top + index * (cfg.scope_bottom - cfg.scope_top) / 8)
        cv2.line(image, (cfg.scope_left, y), (cfg.scope_right, y),
                 axis_color if index == 4 else grid_color, 1, cv2.LINE_AA)

    actual_frequency = frequency_hz * (1.0 + cfg.source_clock_ppm * 1e-6)
    cycles = actual_frequency * cfg.ramp_seconds
    sample_count = max(3000, int(np.ceil(cycles * 80)))
    u = np.linspace(0.0, 1.0, sample_count, dtype=np.float64)
    half_width = 0.46 * (cfg.scope_right - cfg.scope_left)
    center_x = 0.5 * (cfg.scope_left + cfg.scope_right)
    y_values = cfg.scope_bottom - u * (cfg.scope_bottom - cfg.scope_top)

    for frame_index in range(cfg.accumulated_frames):
        phase = 2.0 * np.pi * actual_frequency * cfg.frame_seconds * frame_index
        x_values = center_x + half_width * np.sin(2.0 * np.pi * cycles * u + phase)
        points = np.column_stack((x_values, y_values)).round().astype(np.int32)
        cv2.polylines(image, [points], False, (48, 232, 82), 1, cv2.LINE_AA)

    # The high-speed flyback may leave a faint vertical artifact. The 9 ms park
    # interval is above the visible screen and therefore contributes no line.
    flyback_x = int(round(center_x + half_width * np.sin(2.0 * np.pi * cycles)))
    cv2.line(image, (flyback_x, cfg.scope_top), (flyback_x, cfg.scope_bottom),
             (18, 55, 24), 1, cv2.LINE_AA)

    if cfg.trace_blur_sigma > 0:
        image = cv2.GaussianBlur(image, (0, 0), cfg.trace_blur_sigma)

    noise = rng.normal(0.0, cfg.sensor_noise_sigma, image.shape)
    image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return image


def camera_warp(scope_image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    height, width = scope_image.shape[:2]
    source = np.float32([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])
    camera_corners = np.float32([[16, 11], [width - 25, 3],
                                 [width - 8, height - 17], [28, height - 3]])
    to_camera = cv2.getPerspectiveTransform(source, camera_corners)
    camera = cv2.warpPerspective(scope_image, to_camera, (width, height),
                                 flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT,
                                 borderValue=(5, 7, 5))
    to_scope = cv2.getPerspectiveTransform(camera_corners, source)
    rectified = cv2.warpPerspective(camera, to_scope, (width, height),
                                    flags=cv2.INTER_LINEAR)
    return camera, rectified


def extract_trace(rectified: np.ndarray, cfg: SimulationConfig) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    blue, green, red = cv2.split(rectified)
    mask = ((green.astype(np.int16) > red.astype(np.int16) + 28) &
            (green.astype(np.int16) > blue.astype(np.int16) + 20) &
            (green > 55)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))

    rows = np.arange(cfg.scope_top + 2, cfg.scope_bottom - 1)
    centers = np.full(rows.size, np.nan, dtype=np.float64)
    weights = green.astype(np.float64)

    for output_index, row in enumerate(rows):
        xs = np.flatnonzero(mask[row, cfg.scope_left:cfg.scope_right + 1]) + cfg.scope_left
        if xs.size:
            row_weights = weights[row, xs]
            centers[output_index] = np.average(xs, weights=np.maximum(row_weights, 1.0))

    valid = np.isfinite(centers)
    if np.count_nonzero(valid) < int(0.75 * rows.size):
        raise RuntimeError("insufficient trace rows after segmentation")

    centers = np.interp(np.arange(rows.size), np.flatnonzero(valid), centers[valid])
    # Image rows increase downwards; reverse them so u=0 is ramp start.
    centers = centers[::-1]
    rows = rows[::-1]
    u = (cfg.scope_bottom - rows.astype(np.float64)) / (cfg.scope_bottom - cfg.scope_top)
    return u, centers, mask


def estimate_cycles(u: np.ndarray, centers: np.ndarray) -> float:
    signal = centers - np.mean(centers)
    signal /= max(np.std(signal), 1e-9)
    window = np.hanning(signal.size)
    windowed = signal * window

    fft_size = 65536
    spectrum = np.abs(np.fft.rfft(windowed, n=fft_size))
    cycle_axis = np.fft.rfftfreq(fft_size, d=1.0 / (signal.size - 1))
    search = (cycle_axis >= 0.5) & (cycle_axis <= 110.0)
    coarse = float(cycle_axis[search][np.argmax(spectrum[search])])

    def fit_error(cycles: float) -> float:
        phase = 2.0 * np.pi * cycles * u
        design = np.column_stack((np.sin(phase), np.cos(phase), np.ones_like(u), u))
        coefficients, _, _, _ = np.linalg.lstsq(design, centers, rcond=None)
        residual = centers - design @ coefficients
        return float(np.mean(residual * residual))

    low = max(0.5, coarse - 0.8)
    high = min(110.0, coarse + 0.8)
    golden = 0.5 * (np.sqrt(5.0) - 1.0)
    left = high - golden * (high - low)
    right = low + golden * (high - low)
    left_error = fit_error(left)
    right_error = fit_error(right)

    for _ in range(36):
        if left_error < right_error:
            high = right
            right = left
            right_error = left_error
            left = high - golden * (high - low)
            left_error = fit_error(left)
        else:
            low = left
            left = right
            left_error = right_error
            right = low + golden * (high - low)
            right_error = fit_error(right)

    return float(0.5 * (low + high))


def simulate_one(frequency_hz: float, cfg: SimulationConfig, seed: int):
    scope = draw_scope(frequency_hz, cfg, seed)
    camera, rectified = camera_warp(scope)
    u, centers, mask = extract_trace(rectified, cfg)
    cycles = estimate_cycles(u, centers)
    estimated_hz = cycles / cfg.ramp_seconds
    return estimated_hz, scope, camera, rectified, mask, centers


def measure_frequency(frequency_hz: float, cfg: SimulationConfig, seed: int):
    primary = simulate_one(frequency_hz, cfg, seed)
    if primary[0] < 70000.0:
        return primary, cfg.ramp_seconds

    # Above roughly 70 kHz, a 1 ms ramp puts too many cycles into the
    # 380-pixel vertical scope area. A second 0.5 ms probe doubles the pixels
    # per cycle while the fixed 10 ms frame still preserves phase repetition.
    high_range_cfg = replace(cfg, ramp_seconds=0.0005)
    return simulate_one(frequency_hz, high_range_cfg, seed + 100000), high_range_cfg.ramp_seconds


def save_montage(output_dir: Path, cfg: SimulationConfig) -> None:
    frequencies = [1000.0, 10000.0, 37400.0, 100000.0]
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for axis, frequency in zip(axes.flat, frequencies):
        result, ramp_seconds = measure_frequency(frequency, cfg, int(frequency))
        estimate, _, camera, _, _, _ = result
        axis.imshow(cv2.cvtColor(camera, cv2.COLOR_BGR2RGB))
        axis.set_title(
            f"input {frequency / 1000:.1f} kHz, estimate {estimate / 1000:.3f} kHz, "
            f"ramp {ramp_seconds * 1e3:.1f} ms"
        )
        axis.axis("off")
    figure.savefig(output_dir / "montage.png", dpi=160)
    plt.close(figure)


def run_regression(output_dir: Path, cfg: SimulationConfig) -> dict:
    frequencies = np.arange(1000.0, 100000.0 + 0.1, 100.0)
    estimates = np.empty_like(frequencies)
    ramp_times = np.empty_like(frequencies)

    for index, frequency in enumerate(frequencies):
        result, ramp_times[index] = measure_frequency(frequency, cfg, 1000 + index)
        estimates[index] = result[0]

    actual = frequencies * (1.0 + cfg.source_clock_ppm * 1e-6)
    errors = estimates - actual
    rounded = np.round(estimates / 100.0) * 100.0
    exact = rounded == frequencies

    figure, axis = plt.subplots(figsize=(11, 4.5), constrained_layout=True)
    axis.plot(frequencies / 1000.0, errors, linewidth=1.0)
    axis.axhline(50.0, color="tab:red", linewidth=1.0, linestyle="--", label="100 Hz decision boundary")
    axis.axhline(-50.0, color="tab:red", linewidth=1.0, linestyle="--")
    axis.set_xlabel("Input frequency (kHz)")
    axis.set_ylabel("Estimate error (Hz)")
    axis.set_title("Sawtooth image frequency estimation error")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="upper right")
    figure.savefig(output_dir / "frequency_error.png", dpi=160)
    plt.close(figure)

    summary = {
        "configuration": asdict(cfg),
        "high_range_ramp_seconds": 0.0005,
        "high_range_switch_estimate_hz": 70000.0,
        "cases": int(frequencies.size),
        "exact_100_hz_bin_cases": int(np.count_nonzero(exact)),
        "exact_100_hz_bin_rate": float(np.mean(exact)),
        "mean_absolute_error_hz": float(np.mean(np.abs(errors))),
        "p95_absolute_error_hz": float(np.percentile(np.abs(errors), 95)),
        "max_absolute_error_hz": float(np.max(np.abs(errors))),
        "worst_frequency_hz": float(frequencies[np.argmax(np.abs(errors))]),
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="ascii")
    np.savetxt(output_dir / "frequency_results.csv",
               np.column_stack((frequencies, actual, estimates, errors,
                                ramp_times, exact.astype(int))),
               delimiter=",",
               header="nominal_hz,actual_hz,estimated_hz,error_hz,ramp_seconds,correct_100hz_bin",
               comments="")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parent / "outputs")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    cfg = SimulationConfig()
    save_montage(args.output, cfg)
    summary = run_regression(args.output, cfg)
    print(json.dumps(summary, indent=2))
    return 0 if summary["exact_100_hz_bin_rate"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
