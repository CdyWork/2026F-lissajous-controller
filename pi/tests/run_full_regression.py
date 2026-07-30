from __future__ import annotations

import importlib.util
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np


PI_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = PI_ROOT.parent
sys.path.insert(0, str(PI_ROOT))

from f2026_vision.pipeline import VisionPipeline


def load_simulator():
    path = PROJECT_ROOT / "q5_sawtooth_vision_sim" / "vision_sim.py"
    specification = importlib.util.spec_from_file_location("q5_full_sim", path)
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


def trace_free_scope(config) -> np.ndarray:
    image = np.zeros((config.height, config.width, 3), dtype=np.uint8)
    image[:] = (7, 10, 7)
    for index in range(11):
        x = round(
            config.scope_left
            + index * (config.scope_right - config.scope_left) / 10
        )
        color = (54, 61, 54) if index == 5 else (35, 43, 35)
        cv2.line(
            image,
            (x, config.scope_top),
            (x, config.scope_bottom),
            color,
            1,
            cv2.LINE_AA,
        )
    for index in range(9):
        y = round(
            config.scope_top
            + index * (config.scope_bottom - config.scope_top) / 8
        )
        color = (54, 61, 54) if index == 4 else (35, 43, 35)
        cv2.line(
            image,
            (config.scope_left, y),
            (config.scope_right, y),
            color,
            1,
            cv2.LINE_AA,
        )
    return image


def main() -> int:
    simulator = load_simulator()
    config = simulator.SimulationConfig()
    background_camera, _ = simulator.camera_warp(trace_free_scope(config))
    pipeline = VisionPipeline()
    pipeline.calibrate([background_camera])

    frequencies = np.arange(1000.0, 100000.0 + 0.1, 100.0)
    errors: list[float] = []
    correct = 0
    rejected = 0
    worst = {"absolute_error_hz": -1.0}
    started = time.perf_counter()

    for index, frequency_hz in enumerate(frequencies):
        ramp_seconds = 0.001 if frequency_hz < 70000.0 else 0.0005
        case_config = simulator.replace(config, ramp_seconds=ramp_seconds)
        scope = simulator.draw_scope(float(frequency_hz), case_config, 1000 + index)
        camera, _ = simulator.camera_warp(scope)
        try:
            estimate = pipeline.process(camera, ramp_seconds).frequency.frequency_hz
        except RuntimeError:
            rejected += 1
            continue

        actual = frequency_hz * (1.0 + config.source_clock_ppm * 1e-6)
        error = float(estimate - actual)
        errors.append(abs(error))
        if round(estimate / 100.0) * 100.0 == frequency_hz:
            correct += 1
        if abs(error) > worst["absolute_error_hz"]:
            worst = {
                "absolute_error_hz": abs(error),
                "nominal_hz": float(frequency_hz),
                "estimate_hz": float(estimate),
            }

    summary = {
        "cases": int(frequencies.size),
        "correct_100hz_bins": correct,
        "rejected": rejected,
        "mean_absolute_error_hz": float(np.mean(errors)),
        "p95_absolute_error_hz": float(np.percentile(errors, 95)),
        "worst": worst,
        "elapsed_seconds": time.perf_counter() - started,
    }
    print(json.dumps(summary, indent=2))
    return 0 if correct == frequencies.size and rejected == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
