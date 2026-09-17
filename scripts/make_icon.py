#!/usr/bin/env python3
"""Generate the add-on icon set (``addons/opencv_camera/icons/*.png``).

Pure Python (no Pillow): a tiny PNG writer plus signed-distance-field drawing with
3x3 super sampling.  Run with any python3:

    python3 scripts/make_icon.py

Style: one monochrome line drawing per menu entry (no colour), drawn with as few
strokes as possible so it stays readable at menu size (16 px):

===============  ==========================================================
``visionsim``    eye with a pupil - the add-on itself
``camera``       camera body with a lens, for the Camera submenu
``fisheye``      circle with a bowed cross (wide angle)
``brown_conrady``square with a bowed cross
``rational``     like brown_conrady plus a centre ring (higher order terms)
``pinhole``      square with a straight cross
``camera_scene`` cube outline (the checker camera scene)
===============  ==========================================================

The marks are original; do not ship the OpenCV logo (a trademark of the OpenCV
project).
"""

from __future__ import annotations

import math
import os
import struct
import zlib
from typing import Callable, Dict, Iterable, List, Sequence, Tuple

SIZE = 64
SS = 3                      # super sampling factor
HALF_WIDTH = 0.038          # stroke half width in normalised units
INK = (0.878, 0.902, 0.929)  # light neutral, readable on the dark UI
CORNER = 0.10

Point = Tuple[float, float]
Shape = Callable[[float, float], float]  # signed distance (negative inside)


# ---------------------------------------------------------------------------
# drawing primitives
# ---------------------------------------------------------------------------
def segment_distance(p: Point, a: Point, b: Point) -> float:
    px, py = p
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    length2 = vx * vx + vy * vy
    t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / length2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def line(a: Point, b: Point) -> Shape:
    return lambda x, y: segment_distance((x, y), a, b)


def polyline(points: Sequence[Point]) -> Shape:
    segments = [(line(points[i], points[i + 1])) for i in range(len(points) - 1)]
    return lambda x, y: min(shape(x, y) for shape in segments)


def circle(center: Point, radius: float) -> Shape:
    cx, cy = center
    return lambda x, y: abs(math.hypot(x - cx, y - cy) - radius)


def disc(center: Point, radius: float) -> Shape:
    cx, cy = center
    return lambda x, y: math.hypot(x - cx, y - cy) - radius


def rounded_rect(x0: float, y0: float, x1: float, y1: float, radius: float = CORNER) -> Shape:
    cx, cy = (x0 + x1) * 0.5, (y0 + y1) * 0.5
    hx, hy = (x1 - x0) * 0.5 - radius, (y1 - y0) * 0.5 - radius

    def sdf(x: float, y: float) -> float:
        dx = abs(x - cx) - hx
        dy = abs(y - cy) - hy
        outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
        inside = min(max(dx, dy), 0.0)
        return abs(outside + inside - radius)  # distance to the outline

    return sdf


def curve(function: Callable[[float], Point], t0: float, t1: float, samples: int = 48) -> Shape:
    points = [function(t0 + (t1 - t0) * i / samples) for i in range(samples + 1)]
    return polyline(points)


def warped_grid(cx: float, cy: float, half: float, lines: int, bend: float) -> Shape:
    """``lines`` vertical + horizontal lines, radially displaced by ``bend``.

    Positive ``bend`` bulges the lines outward at their ends (barrel), negative
    pinches them (pincushion).
    """
    shapes: List[Shape] = []
    steps = 5
    offsets = [(-half + 2.0 * half * i / (lines - 1)) for i in range(lines)] if lines > 1 else [0.0]
    for offset in offsets:
        for sign, axis in ((1.0, "v"), (1.0, "h")):
            if axis == "v":
                points = []
                for i in range(steps + 1):
                    t = -half + 2.0 * half * i / steps
                    bulge = bend * (offset / half) * ((t / half) ** 2) if half else 0.0
                    points.append((cx + offset + bulge, cy + t))
            else:
                points = []
                for i in range(steps + 1):
                    t = -half + 2.0 * half * i / steps
                    bulge = bend * (offset / half) * ((t / half) ** 2) if half else 0.0
                    points.append((cx + t, cy + offset + bulge))
            shapes.append(polyline(points))
    return lambda x, y: min(shape(x, y) for shape in shapes)


def eye_shape() -> Shape:
    """Almond outline: two mirrored elliptical arcs."""
    def top(t: float) -> Point:
        u = -1.0 + 2.0 * t
        return (0.5 + 0.40 * u, 0.5 - 0.26 * math.sqrt(max(0.0, 1.0 - u * u)))

    def bottom(t: float) -> Point:
        u = -1.0 + 2.0 * t
        return (0.5 + 0.40 * u, 0.5 + 0.26 * math.sqrt(max(0.0, 1.0 - u * u)))

    return lambda x, y: min(curve(top, 0.0, 1.0)(x, y), curve(bottom, 0.0, 1.0)(x, y))


def cube_shape() -> Shape:
    """Isometric cube: hexagon plus two inner edges (three strokes in total)."""
    top = (0.5, 0.18)
    left = (0.18, 0.36)
    right = (0.82, 0.36)
    front = (0.5, 0.54)
    bottom_left = (0.18, 0.72)
    bottom_right = (0.82, 0.72)
    bottom = (0.5, 0.90)
    shapes = [
        polyline([top, left, bottom_left, bottom, bottom_right, right, top]),  # hexagon
        line(front, bottom_left), line(front, bottom_right),                   # inner edges
    ]
    return lambda x, y: min(shape(x, y) for shape in shapes)


# ---------------------------------------------------------------------------
# the icon set
# ---------------------------------------------------------------------------
def build_icons() -> Dict[str, List[Shape]]:
    return {
        "visionsim": [eye_shape(), disc((0.5, 0.5), 0.105)],
        "camera": [
            rounded_rect(0.08, 0.28, 0.92, 0.84, 0.09),
            circle((0.50, 0.56), 0.17),
        ],
        "fisheye": [circle((0.5, 0.5), 0.41), warped_grid(0.5, 0.5, 0.17, 2, 0.20)],
        "brown_conrady": [rounded_rect(0.10, 0.10, 0.90, 0.90), warped_grid(0.5, 0.5, 0.19, 2, 0.17)],
        "rational": [rounded_rect(0.10, 0.10, 0.90, 0.90), warped_grid(0.5, 0.5, 0.19, 2, 0.17),
                     circle((0.5, 0.5), 0.065)],
        "pinhole": [rounded_rect(0.10, 0.10, 0.90, 0.90), warped_grid(0.5, 0.5, 0.22, 2, 0.0)],
        "camera_scene": [cube_shape()],
    }


# ---------------------------------------------------------------------------
# rasterise + PNG
# ---------------------------------------------------------------------------
def render(shapes: Iterable[Shape]) -> bytes:
    shapes = list(shapes)
    rows = []
    for py in range(SIZE):
        row = bytearray()
        for px in range(SIZE):
            covered = 0
            for sy in range(SS):
                for sx in range(SS):
                    x = (px + (sx + 0.5) / SS) / SIZE
                    y = (py + (sy + 0.5) / SS) / SIZE
                    if min(shape(x, y) for shape in shapes) <= HALF_WIDTH:
                        covered += 1
            alpha = max(0, min(255, int(round(255.0 * covered / (SS * SS)))))
            rgb = [max(0, min(255, int(round(255.0 * c)))) for c in INK]
            row.extend(rgb + [alpha])
        rows.append(bytes(row))
    return b"".join(b"\x00" + row for row in rows)


def write_png(path: str, raw: bytes) -> None:
    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    header = struct.pack(">IIBBBBB", SIZE, SIZE, 8, 6, 0, 0, 0)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header)
           + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as handle:
        handle.write(png)


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    directory = os.path.join(root, "addons", "opencv_camera", "icons")
    for name, shapes in build_icons().items():
        path = os.path.join(directory, f"{name}.png")
        write_png(path, render(shapes))
        print(f"written {path} ({os.path.getsize(path)} bytes)")


if __name__ == "__main__":
    main()