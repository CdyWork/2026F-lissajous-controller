"""Interactively calibrate the four XY graticule corners from one camera frame."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from .live_sweep import _rectify_xy


def _parse_points(text: str) -> list[tuple[float, float]]:
    values = [float(value.strip()) for value in text.split(",") if value.strip()]
    if len(values) != 8:
        raise ValueError("corners must contain four x,y point pairs")
    return [(values[index], values[index + 1]) for index in range(0, 8, 2)]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Four-corner XY perspective calibration")
    parser.add_argument("image", type=Path, help="fresh full camera JPG")
    parser.add_argument("--corners", help="TL,TR,BR,BL; skips mouse selection")
    parser.add_argument("--output", type=Path, default=Path("xy_calibrated.png"))
    parser.add_argument("--width", type=int, default=400)
    parser.add_argument("--height", type=int, default=400)
    args = parser.parse_args(argv)

    image = cv2.imread(str(args.image))
    if image is None:
        raise RuntimeError(f"cannot read {args.image}")
    points = _parse_points(args.corners) if args.corners else []

    if not points:
        canvas = image.copy()
        window = "Click XY corners: TL, TR, BR, BL; R resets; Enter saves"

        def redraw() -> None:
            shown = canvas.copy()
            for index, point in enumerate(points):
                xy = (round(point[0]), round(point[1]))
                cv2.circle(shown, xy, 5, (0, 255, 0), -1)
                cv2.putText(shown, str(index + 1), (xy[0] + 7, xy[1] - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if len(points) > 1:
                cv2.polylines(shown, [np.asarray(points, dtype=np.int32)],
                              len(points) == 4, (0, 255, 0), 1)
            cv2.imshow(window, shown)

        def on_mouse(event: int, x: int, y: int, flags: int, parameter: object) -> None:
            del flags, parameter
            if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
                points.append((float(x), float(y)))
                redraw()

        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window, on_mouse)
        redraw()
        while True:
            key = cv2.waitKey(50) & 0xFF
            if key in (13, 10) and len(points) == 4:
                break
            if key in (ord("r"), ord("R")):
                points.clear()
                redraw()
            if key == 27:
                cv2.destroyAllWindows()
                return 1
        cv2.destroyAllWindows()

    corners = tuple(coordinate for point in points for coordinate in point)
    rectified = _rectify_xy(image, corners, args.width, args.height)
    if not cv2.imwrite(str(args.output), rectified):
        raise RuntimeError(f"cannot write {args.output}")
    print("XY_CORNERS=" + ",".join(f"{coordinate:.1f}" for coordinate in corners))
    print(f"PREVIEW={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
