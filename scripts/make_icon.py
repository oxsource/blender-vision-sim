#!/usr/bin/env python3
"""Generate the add-on icon (``addons/opencv_camera/icons/visionsim.png``).

Pure Python (no Pillow): a tiny PNG writer plus analytic drawing with 3x3 super
sampling.  Run with any python3:

    python3 scripts/make_icon.py

Design: three coloured arcs (red/green/blue, echoing the OpenCV palette) around a
slightly barrel-warped 3x3 calibration grid.  No lens/aperture body, so the mark
does not read as "just another camera icon" at menu size.  It is an original mark -
do not ship the OpenCV logo itself, it is a trademark of the OpenCV project.
"""

from __future__ import annotations

import math
import os
import struct
import zlib

SIZE = 64
SS = 3  # super sampling factor

RED = (0.898, 0.157, 0.157)
GREEN = (0.196, 0.678, 0.239)
BLUE = (0.129, 0.451, 0.851)

ARC_RADIUS = 0.385
ARC_WIDTH = 0.098
ARC_SPAN = 96.0         # degrees per arc
ARC_CENTERS = ((90.0, RED), (210.0, GREEN), (330.0, BLUE))

#: five calibration dots (a cross) ; the outer ones are pushed further out to hint
#: at the barrel/fisheye warp this add-on is about.  Few and large so the mark
#: still reads at menu size (16 px).
GRID_STEP = 0.135
DOT_RADIUS = 0.042
WARP = 0.45
DOT_COLOR = (0.560, 0.830, 1.000)
DOT_CENTER_COLOR = (1.0, 1.0, 1.0)


def _mix(dst, src, alpha):
    return tuple(d + (s - d) * alpha for d, s in zip(dst, src))


def _dot_centres():
    """Centre plus four dots in a cross, radially displaced (barrel warp)."""
    points = [(0.0, 0.0, True)]
    for gx, gy in ((GRID_STEP, 0.0), (-GRID_STEP, 0.0), (0.0, GRID_STEP), (0.0, -GRID_STEP)):
        scale = 1.0 + WARP
        points.append((gx * scale, gy * scale, False))
    return points


DOTS = _dot_centres()


def shade(x: float, y: float) -> tuple:
    """Colour of the icon at normalised coordinates (0..1, y down)."""
    dx, dy = x - 0.5, y - 0.5
    radius = math.hypot(dx, dy)
    angle = math.degrees(math.atan2(dy, dx))

    # calibration grid
    for gx, gy, is_center in DOTS:
        if math.hypot(dx - gx, dy - gy) <= DOT_RADIUS:
            return DOT_CENTER_COLOR if is_center else DOT_COLOR

    # coloured arcs
    for center, rgb in ARC_CENTERS:
        delta = (angle - center + 180.0) % 360.0 - 180.0
        if abs(delta) <= ARC_SPAN * 0.5 and abs(radius - ARC_RADIUS) <= ARC_WIDTH * 0.5:
            offset = abs(radius - ARC_RADIUS) / (ARC_WIDTH * 0.5)
            return _mix(rgb, (1.0, 1.0, 1.0), 0.25 * (1.0 - offset) ** 2)
    return (0.0, 0.0, 0.0, 0.0)


def render() -> bytes:
    rows = []
    for py in range(SIZE):
        row = bytearray()
        for px in range(SIZE):
            acc = [0.0, 0.0, 0.0, 0.0]
            for sy in range(SS):
                for sx in range(SS):
                    x = (px + (sx + 0.5) / SS) / SIZE
                    y = (py + (sy + 0.5) / SS) / SIZE
                    colour = shade(x, y)
                    if len(colour) == 3:
                        colour = colour + (1.0,)
                    for i in range(4):
                        acc[i] += colour[i]
            samples = SS * SS
            rgba = [max(0, min(255, int(round(255.0 * c / samples)))) for c in acc]
            row.extend(rgba)
        rows.append(bytes(row))
    raw = b"".join(b"\x00" + row for row in rows)
    return raw


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
    path = os.path.join(root, "addons", "opencv_camera", "icons", "visionsim.png")
    write_png(path, render())
    print(f"written {path} ({os.path.getsize(path)} bytes)")


if __name__ == "__main__":
    main()