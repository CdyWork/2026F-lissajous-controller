"""Capture one fresh oscilloscope frame for each cyclic rising Q5 ramp."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from .camera import OrangePiV4L2CameraSource
from .config import CameraConfig
from .serial_link import McuLink
from .xy_extract import detect_xy_axes, draw_detection, median_frame


# Fixed IMX219 mounting: measured outer XY-graticule corners, clockwise from
# top-left.  These are the true screen corners, not the inner trace area, so
# the homography removes both camera roll and keystone distortion.
DEFAULT_XY_CORNERS = "194,202,412,216,416,413,204,410"
FPGA_SWEEP_RAMPS_US = (10, 30, 70, 150, 300, 500, 750, 1000)
FPGA_SWEEP_FRAME_US = 2000
FPGA_SWEEP_DWELL_MS = 400


def _ramp_widths(start_us: int, stop_us: int, step_us: int) -> list[int]:
    if not (10 <= start_us <= stop_us <= 1800):
        raise ValueError("ramp widths must be within 10..1800 us")
    if step_us <= 0:
        raise ValueError("step-us must be positive")

    widths = list(range(start_us, stop_us + 1, step_us))
    if widths[-1] != stop_us:
        widths.append(stop_us)
    return widths


def _parse_ramp_list(text: str) -> list[int]:
    try:
        widths = [int(value.strip()) for value in text.split(",") if value.strip()]
    except ValueError as error:
        raise ValueError("ramps-us must be a comma-separated integer list") from error
    if not widths:
        raise ValueError("ramps-us must contain at least one ramp")
    if any(not 10 <= width <= 1800 for width in widths):
        raise ValueError("ramps-us values must be within 10..1800 us")
    return widths


def _parse_xy_corners(text: str) -> tuple[float, float, float, float, float, float, float, float]:
    try:
        values = tuple(float(value.strip()) for value in text.split(","))
    except ValueError as error:
        raise ValueError("xy-corners must contain eight comma-separated numbers") from error
    if len(values) != 8:
        raise ValueError("xy-corners must contain four x,y corner pairs")
    return values  # type: ignore[return-value]


def _rectify_xy(
    image_bgr: np.ndarray,
    corners: tuple[float, float, float, float, float, float, float, float],
    width: int,
    height: int,
) -> np.ndarray:
    if width < 32 or height < 32:
        raise ValueError("rectified XY dimensions must be at least 32 pixels")
    source = np.asarray(corners, dtype=np.float32).reshape((4, 2))
    destination = np.array(
        [
            (0.0, 0.0),
            (float(width - 1), 0.0),
            (float(width - 1), float(height - 1)),
            (0.0, float(height - 1)),
        ],
        dtype=np.float32,
    )
    transform = cv2.getPerspectiveTransform(source, destination)
    return cv2.warpPerspective(
        image_bgr, transform, (width, height), flags=cv2.INTER_LINEAR
    )


def _frame_period_us(ramp_us: int, frame_us: int) -> int:
    if not (200 <= frame_us <= 2000):
        raise ValueError("frame-us must be within 200..2000 us")
    if frame_us % 200 != 0:
        raise ValueError("frame-us must be a multiple of 200 us")
    if frame_us <= ramp_us:
        raise ValueError("frame-us must be longer than every ramp")
    return frame_us


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture fresh scope frames from cyclic rising Q5 probes"
    )
    parser.add_argument("--serial", default="/dev/ttyS2")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--start-us", type=int, default=10)
    parser.add_argument("--stop-us", type=int, default=1000)
    parser.add_argument("--step-us", type=int, default=20)
    parser.add_argument(
        "--ramps-us",
        help="comma-separated explicit ramp widths; overrides start/stop/step",
    )
    parser.add_argument(
        "--fpga-sweep",
        action="store_true",
        help="send one SWEEP command and capture the eight FPGA-resident ramp settings",
    )
    parser.add_argument(
        "--sweep-cycles",
        type=int,
        default=1,
        help="complete FPGA sweep cycles to record when --fpga-sweep is selected",
    )
    parser.add_argument(
        "--sweep-dwell-ms",
        type=int,
        default=FPGA_SWEEP_DWELL_MS,
        help="FPGA dwell time per setting; fixed at 400 ms in the current bitstream",
    )
    parser.add_argument(
        "--sweep-settle-ms",
        type=int,
        default=120,
        help="skip camera frames for this long after every FPGA sweep setting change",
    )
    parser.add_argument(
        "--sweep-guard-ms",
        type=int,
        default=40,
        help="skip camera frames this long before the next FPGA sweep setting change",
    )
    parser.add_argument(
        "--frame-us",
        type=int,
        default=2000,
        help="fixed cyclic period for every ramp; must be a 200 us multiple",
    )
    parser.add_argument(
        "--capture-delay-ms",
        type=int,
        default=100,
        help="delay after PROBE for the STM32 command and scope single trigger",
    )
    parser.add_argument(
        "--captures-per-ramp",
        type=int,
        default=1,
        help="fresh camera frames saved continuously for each ramp setting",
    )
    parser.add_argument(
        "--capture-gap-ms",
        type=int,
        default=34,
        help="gap between consecutive camera frames at one ramp setting",
    )
    parser.add_argument(
        "--discard-frames",
        type=int,
        default=0,
        help="camera frames discarded after the settle delay at every ramp setting",
    )
    parser.add_argument("--capture-fps", type=float, default=30.0)
    parser.add_argument(
        "--xy-corners",
        default=DEFAULT_XY_CORNERS,
        help="top-left,top-right,bottom-right,bottom-left x,y pairs in camera pixels",
    )
    parser.add_argument(
        "--auto-xy",
        action="store_true",
        help="fit the central XY axes before capture and use them for centered extraction",
    )
    parser.add_argument(
        "--auto-xy-frames",
        type=int,
        default=4,
        help="fresh frames combined for --auto-xy",
    )
    parser.add_argument(
        "--auto-xy-min-confidence",
        type=float,
        default=0.65,
        help="minimum accepted central-axis calibration confidence",
    )
    parser.add_argument(
        "--trace-threshold",
        type=int,
        default=18,
        help="minimum blank-subtracted gray level for the binary trace mask",
    )
    parser.add_argument("--xy-width", type=int, default=400)
    parser.add_argument("--xy-height", type=int, default=400)
    parser.add_argument(
        "--leave-active",
        action="store_true",
        help="keep the final PROBE ramp active instead of sending IDLE after capture",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("q5_live_captures"),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.capture_delay_ms < 20:
        raise ValueError("capture-delay-ms must be at least 20")
    if args.captures_per_ramp < 1:
        raise ValueError("captures-per-ramp must be positive")
    if args.capture_gap_ms < 0:
        raise ValueError("capture-gap-ms cannot be negative")
    if args.discard_frames < 0:
        raise ValueError("discard-frames cannot be negative")
    if args.sweep_cycles < 1:
        raise ValueError("sweep-cycles must be positive")
    if args.sweep_settle_ms < 0 or args.sweep_guard_ms < 0:
        raise ValueError("sweep settle and guard times cannot be negative")
    if args.fpga_sweep and args.sweep_settle_ms + args.sweep_guard_ms >= args.sweep_dwell_ms:
        raise ValueError("sweep settle plus guard time must be shorter than the dwell")
    if args.auto_xy_frames < 1:
        raise ValueError("auto-xy-frames must be positive")
    if not 0.0 <= args.auto_xy_min_confidence <= 1.0:
        raise ValueError("auto-xy-min-confidence must be within 0..1")
    if not 1 <= args.trace_threshold <= 255:
        raise ValueError("trace-threshold must be within 1..255")
    if args.fpga_sweep and args.sweep_dwell_ms != FPGA_SWEEP_DWELL_MS:
        raise ValueError("the FPGA sweep dwell is fixed at 400 ms")

    ramp_widths = list(FPGA_SWEEP_RAMPS_US) if args.fpga_sweep else (
        _parse_ramp_list(args.ramps_us)
        if args.ramps_us
        else _ramp_widths(args.start_us, args.stop_us, args.step_us)
    )
    xy_corners = _parse_xy_corners(args.xy_corners)
    if args.xy_width < 32 or args.xy_height < 32:
        raise ValueError("rectified XY dimensions must be at least 32 pixels")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    xy_directory = args.output_dir / "xy"
    xy_directory.mkdir(exist_ok=True)
    trace_directory = args.output_dir / "trace"
    trace_mask_directory = args.output_dir / "trace_mask"
    manifest_path = args.output_dir / "manifest.jsonl"
    camera_config = CameraConfig(
        capture_fps=args.capture_fps,
    )

    with McuLink(args.serial, timeout=1.0) as link, OrangePiV4L2CameraSource(
        camera_config, args.device
    ) as camera, manifest_path.open("w", encoding="ascii") as manifest:
        try:
            blank_xy: np.ndarray | None = None
            if args.auto_xy:
                calibration_frames = [camera.capture().bgr for _ in range(args.auto_xy_frames)]
                calibration_image = median_frame(calibration_frames)
                calibration = detect_xy_axes(calibration_image, xy_corners)
                if calibration.confidence < args.auto_xy_min_confidence:
                    raise RuntimeError(
                        f"XY axis calibration confidence too low: {calibration.confidence:.3f}"
                    )
                xy_corners = calibration.corners
                axis_path = args.output_dir / "xy_axis_calibration.png"
                rectified_path = args.output_dir / "xy_axis_calibrated.png"
                blank_path = args.output_dir / "xy_blank.png"
                if not cv2.imwrite(str(axis_path), draw_detection(calibration_image, calibration)):
                    raise RuntimeError(f"failed to write {axis_path}")
                blank_xy = _rectify_xy(
                    calibration_image, xy_corners, args.xy_width, args.xy_height
                )
                if not cv2.imwrite(str(rectified_path), blank_xy):
                    raise RuntimeError(f"failed to write {rectified_path}")
                if not cv2.imwrite(str(blank_path), blank_xy):
                    raise RuntimeError(f"failed to write {blank_path}")
                trace_directory.mkdir(exist_ok=True)
                trace_mask_directory.mkdir(exist_ok=True)
                print(
                    json.dumps(
                        {
                            "xy_auto_calibration": True,
                            "xy_corners": list(xy_corners),
                            "xy_center": list(calibration.center),
                            "xy_confidence": calibration.confidence,
                            "xy_blank": str(blank_path.relative_to(args.output_dir)),
                        }
                    ),
                    flush=True,
                )
            if args.fpga_sweep:
                _capture_fpga_sweep(
                    args, link, camera, manifest, xy_directory, trace_directory,
                    trace_mask_directory, xy_corners, blank_xy,
                )
            else:
                _capture_host_sweep(
                    args, link, camera, manifest, xy_directory, trace_directory,
                    trace_mask_directory, xy_corners, blank_xy, ramp_widths,
                )
        finally:
            if not args.leave_active:
                link.set_idle()

    return 0


def _write_capture(
    args: argparse.Namespace,
    frame: object,
    manifest: object,
    xy_directory: Path,
    trace_directory: Path,
    trace_mask_directory: Path,
    xy_corners: tuple[float, float, float, float, float, float, float, float],
    blank_xy: np.ndarray | None,
    record: dict[str, object],
) -> None:
    image_path = args.output_dir / str(record["image"])
    if not cv2.imwrite(str(image_path), frame.bgr):
        raise RuntimeError(f"failed to write {image_path}")
    xy_path = xy_directory / f"{image_path.stem}_xy.png"
    rectified = _rectify_xy(frame.bgr, xy_corners, args.xy_width, args.xy_height)
    if not cv2.imwrite(
        str(xy_path),
        rectified,
    ):
        raise RuntimeError(f"failed to write {xy_path}")
    record["xy_image"] = str(xy_path.relative_to(args.output_dir))
    record["xy_corners"] = list(xy_corners)
    record["xy_size"] = [args.xy_width, args.xy_height]
    if blank_xy is not None:
        trace = cv2.subtract(rectified, blank_xy)
        trace_gray = cv2.cvtColor(trace, cv2.COLOR_BGR2GRAY)
        _, trace_mask = cv2.threshold(
            trace_gray, args.trace_threshold, 255, cv2.THRESH_BINARY
        )
        trace_path = trace_directory / f"{image_path.stem}_trace.png"
        trace_mask_path = trace_mask_directory / f"{image_path.stem}_trace_mask.png"
        if not cv2.imwrite(str(trace_path), trace):
            raise RuntimeError(f"failed to write {trace_path}")
        if not cv2.imwrite(str(trace_mask_path), trace_mask):
            raise RuntimeError(f"failed to write {trace_mask_path}")
        record["trace_image"] = str(trace_path.relative_to(args.output_dir))
        record["trace_mask"] = str(trace_mask_path.relative_to(args.output_dir))
        record["trace_threshold"] = args.trace_threshold
    manifest.write(json.dumps(record) + "\n")
    manifest.flush()
    print(json.dumps(record), flush=True)


def _capture_host_sweep(
    args: argparse.Namespace,
    link: McuLink,
    camera: OrangePiV4L2CameraSource,
    manifest: object,
    xy_directory: Path,
    trace_directory: Path,
    trace_mask_directory: Path,
    xy_corners: tuple[float, float, float, float, float, float, float, float],
    blank_xy: np.ndarray | None,
    ramp_widths: list[int],
) -> None:
    for index, ramp_us in enumerate(ramp_widths):
        frame_us = _frame_period_us(ramp_us, args.frame_us)
        reply = link.request_probe(ramp_us, frame_us)
        if reply is None or not reply.startswith("OK"):
            raise RuntimeError(f"PROBE {ramp_us} {frame_us} failed: {reply!r}")
        time.sleep(args.capture_delay_ms / 1000.0)
        for _ in range(args.discard_frames):
            camera.capture()
        for capture_index in range(args.captures_per_ramp):
            if capture_index and args.capture_gap_ms:
                time.sleep(args.capture_gap_ms / 1000.0)
            frame = camera.capture()
            _write_capture(args, frame, manifest, xy_directory, trace_directory,
                           trace_mask_directory, xy_corners, blank_xy, {
                "index": index,
                "capture_index": capture_index,
                "ramp_us": ramp_us,
                "frame_us": frame_us,
                "discarded_frames": args.discard_frames,
                "probe_reply": reply,
                "captured_at_s": frame.timestamp_seconds,
                "image": f"{index:03d}_{ramp_us:04d}us_{capture_index:02d}.jpg",
            })


def _capture_fpga_sweep(
    args: argparse.Namespace,
    link: McuLink,
    camera: OrangePiV4L2CameraSource,
    manifest: object,
    xy_directory: Path,
    trace_directory: Path,
    trace_mask_directory: Path,
    xy_corners: tuple[float, float, float, float, float, float, float, float],
    blank_xy: np.ndarray | None,
) -> None:
    reply = link.start_probe_sweep()
    if reply is None or not reply.startswith("OK SWEEP"):
        raise RuntimeError(f"SWEEP failed: {reply!r}")

    started_at_s = time.monotonic()
    dwell_s = args.sweep_dwell_ms / 1000.0
    total_s = len(FPGA_SWEEP_RAMPS_US) * dwell_s * args.sweep_cycles
    capture_index = 0
    while True:
        frame = camera.capture()
        elapsed_s = frame.timestamp_seconds - started_at_s
        if elapsed_s >= total_s:
            break
        sweep_position = elapsed_s / dwell_s
        index = int(sweep_position) % len(FPGA_SWEEP_RAMPS_US)
        within_setting_ms = (sweep_position - int(sweep_position)) * args.sweep_dwell_ms
        if (within_setting_ms < args.sweep_settle_ms or
                within_setting_ms > args.sweep_dwell_ms - args.sweep_guard_ms):
            continue
        ramp_us = FPGA_SWEEP_RAMPS_US[index]
        _write_capture(args, frame, manifest, xy_directory, trace_directory,
                       trace_mask_directory, xy_corners, blank_xy, {
            "index": index,
            "capture_index": capture_index,
            "ramp_us": ramp_us,
            "frame_us": FPGA_SWEEP_FRAME_US,
            "probe_reply": reply,
            "captured_at_s": frame.timestamp_seconds,
            "sweep_started_at_s": started_at_s,
            "sweep_elapsed_ms": elapsed_s * 1000.0,
            "within_setting_ms": within_setting_ms,
            "sweep_settle_ms": args.sweep_settle_ms,
            "sweep_guard_ms": args.sweep_guard_ms,
            "stable_capture": True,
            "image": f"{capture_index:03d}_{index:02d}_{ramp_us:04d}us.jpg",
        })
        capture_index += 1


if __name__ == "__main__":
    raise SystemExit(main())
