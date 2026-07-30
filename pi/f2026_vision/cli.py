from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from .camera import PiCameraSource
from .config import CameraConfig
from .frequency import FrequencyEstimator
from .geometry import ScopeCalibration
from .phase import PhaseFrequencyTracker
from .pipeline import VisionPipeline
from .serial_link import McuLink


def _measurement_payload(measurement) -> dict:
    estimate = measurement.frequency
    return {
        "frequency_hz": estimate.frequency_hz,
        "nominal_100hz": estimate.nominal_100hz,
        "cycles": estimate.cycles,
        "phase_degrees": float(np.degrees(estimate.phase_rad)),
        "amplitude_px": estimate.amplitude_px,
        "rmse_px": estimate.rmse_px,
        "normalized_rmse": estimate.normalized_rmse,
        "valid_rows": measurement.trace.valid_fraction,
        "mean_row_spread_px": measurement.trace.mean_row_spread_px,
    }


def _save_debug(directory: Path, measurement) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(directory / "rectified.png"), measurement.rectified_bgr)
    cv2.imwrite(str(directory / "trace_mask.png"), measurement.trace.mask)


def run_image(args: argparse.Namespace) -> int:
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot read image: {args.image}")
    calibration = ScopeCalibration.load(args.calibration) if args.calibration else None
    background = None
    if args.background:
        background_camera = cv2.imread(str(args.background), cv2.IMREAD_COLOR)
        if background_camera is None:
            raise RuntimeError(f"cannot read background: {args.background}")
        if calibration is None:
            calibration = VisionPipeline().locator.locate(background_camera)
        background = calibration.warp(background_camera)
    pipeline = VisionPipeline(calibration=calibration, background_bgr=background)
    measurement = pipeline.process(image, args.ramp_us * 1e-6)
    if args.save_calibration:
        pipeline.calibration.save(args.save_calibration)
    if args.debug_dir:
        _save_debug(args.debug_dir, measurement)
    print(json.dumps(_measurement_payload(measurement), indent=2))
    return 0


def _collect_calibration(camera: PiCameraSource, count: int) -> list[np.ndarray]:
    frames: list[np.ndarray] = []
    iterator = camera.frames()
    while len(frames) < count:
        frames.append(next(iterator).bgr)
    return frames


def run_camera(args: argparse.Namespace) -> int:
    camera_config = CameraConfig(
        capture_fps=args.capture_fps,
        process_fps=args.process_fps,
        exposure_us=args.exposure_us,
        analogue_gain=args.gain,
    )
    pipeline = VisionPipeline()
    if args.calibration:
        pipeline.calibration = ScopeCalibration.load(args.calibration)

    with PiCameraSource(camera_config) as camera:
        print(
            json.dumps(
                {
                    "exposure_range_us": camera.exposure_range_us,
                    "requested_exposure_us": args.exposure_us,
                }
            )
        )
        calibration_frames = _collect_calibration(camera, args.calibration_frames)
        if pipeline.calibration is None:
            pipeline.calibrate(calibration_frames)
        else:
            rectified = [pipeline.calibration.warp(frame) for frame in calibration_frames]
            pipeline.background_bgr = np.median(np.stack(rectified), axis=0).astype(np.uint8)
        if args.save_calibration:
            pipeline.calibration.save(args.save_calibration)

        estimates = []
        iterator = camera.frames()
        deadline = time.monotonic() + args.coarse_seconds
        last_frame = None
        while time.monotonic() < deadline:
            camera_frame = next(iterator)
            try:
                measurement = pipeline.process(camera_frame.bgr, args.ramp_us * 1e-6)
            except RuntimeError as error:
                print(json.dumps({"rejected": str(error)}))
                continue
            estimates.append(measurement.frequency.frequency_hz)
            last_frame = measurement

        if not estimates or last_frame is None:
            raise RuntimeError("no valid probe frames were measured")
        coarse_hz = float(np.median(estimates))
        nominal_hz = int(round(coarse_hz / 100.0) * 100)
        print(json.dumps({"coarse_hz": coarse_hz, "nominal_100hz": nominal_hz}))

        tracker = PhaseFrequencyTracker(nominal_hz)
        estimator = FrequencyEstimator()
        deadline = time.monotonic() + args.phase_seconds
        final = None
        while time.monotonic() < deadline:
            camera_frame = next(iterator)
            try:
                trace, _ = pipeline.extract(camera_frame.bgr)
                phase_fit = estimator.phase_at_frequency(
                    trace, nominal_hz, args.ramp_us * 1e-6
                )
            except RuntimeError as error:
                print(json.dumps({"rejected": str(error)}))
                continue
            if phase_fit.normalized_rmse > 0.30:
                continue
            tracker.add(camera_frame.timestamp_seconds, phase_fit.phase_rad)
            final = tracker.estimate()

        final_hz = final.frequency_hz if final is not None else coarse_hz
        result = {
            "final_hz": final_hz,
            "nominal_100hz": nominal_hz,
            "phase_tracking": None if final is None else final.__dict__,
        }
        print(json.dumps(result, indent=2))

        if args.apply:
            if not args.serial:
                raise RuntimeError("--apply requires --serial")
            with McuLink(args.serial) as link:
                # The current STM32 accepts integer Hz. The optical result is
                # retained as float for the future high-resolution DDS command.
                link.set_frequency_hz(final_hz)
                link.set_auto_mode(args.mode)
        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="2026F oscilloscope vision")
    subparsers = parser.add_subparsers(dest="command", required=True)

    image = subparsers.add_parser("image", help="measure a saved camera image")
    image.add_argument("image", type=Path)
    image.add_argument("--background", type=Path)
    image.add_argument("--calibration", type=Path)
    image.add_argument("--save-calibration", type=Path)
    image.add_argument("--debug-dir", type=Path)
    image.add_argument("--ramp-us", type=int, default=1000, choices=(500, 1000))
    image.set_defaults(handler=run_image)

    camera = subparsers.add_parser("camera", help="measure from OV5647")
    camera.add_argument("--calibration", type=Path)
    camera.add_argument("--save-calibration", type=Path, default=Path("scope_calibration.json"))
    camera.add_argument("--calibration-frames", type=int, default=20)
    camera.add_argument("--capture-fps", type=float, default=60.0)
    camera.add_argument("--process-fps", type=float, default=30.0)
    camera.add_argument("--exposure-us", type=int, default=800)
    camera.add_argument("--gain", type=float, default=16.0)
    camera.add_argument("--ramp-us", type=int, default=1000, choices=(500, 1000))
    camera.add_argument("--coarse-seconds", type=float, default=0.8)
    camera.add_argument("--phase-seconds", type=float, default=1.2)
    camera.add_argument("--serial")
    camera.add_argument("--apply", action="store_true")
    camera.add_argument("--mode", choices=("DIAG", "CIRCLE", "DOUBLE"), default="DIAG")
    camera.set_defaults(handler=run_camera)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.handler(args))
