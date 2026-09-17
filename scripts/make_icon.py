#!/usr/bin/env python3
"""Generate the add-on icon (``addons/opencv_camera/icons/visionsim.png``).

Pure Python (no Pillow): a tiny PNG writer plus analytic drawing with 3x3 super
sampling.  Run with any python3:

    python3 scripts/make_icon.py

Design: three coloured arcs (red/green/blue, echoing the OpenCV logo palette)
around a lens with concentric fisheye rings.  It is an original mark - do not ship
the OpenCV logo itself, it is a trademark of the OpenCV project.
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
BODY = (0.078, 0.125, 0.180)
RIM = (0.694, 0.831, 0.945)
RING = (0.498, 0.820, 1.000)
PUPIL = (1.0, 1.0, 1.0)

ARC_RADIUS = 0.385
ARC_WIDTH = 0.105
ARC_SPAN = 100.0        # degrees per arc
ARC_CENTERS = ((90.0, RED), (210.0, GREEN), (330.0, BLUE))

LENS_RADIUS = 0.230
LENS_RIM = 0.255
RING_RADII = (0.155, 0.098)
RING_WIDTH = 0.020
PUPIL_RADIUS = 0.045


def _mix(dst, src, alpha):
    return tuple(d + (s - d) * alpha for d, s in zip(dst, src))


def shade(x: float, y: float) -> tuple:
    """Colour of the icon at normalised coordinates (0..1, y down)."""
    dx, dy = x - 0.5, y - 0.5
    radius = math.hypot(dx, dy)
    angle = math.degrees(math.atan2(dy, dx))

    # lens rim + body
    if radius <= LENS_RIM:
        base = BODY if radius <= LENS_RADIUS else RIM
        colour = base
        # fisheye rings inside the lens
        for ring in RING_RADII:
            if abs(radius - ring * LENS_RADIUS / 0.230) <= RING_WIDTH * 0.5:
                colour = _mix(colour, RING, 0.85)
        if radius <= PUPIL_RADIUS:
            colour = PUPIL
        return colour

    # coloured arcs
    for center, rgb in ARC_CENTERS:
        delta = (angle - center + 180.0) % 360.0 - 180.0
        if abs(delta) <= ARC_SPAN * 0.5 and abs(radius - ARC_RADIUS) <= ARC_WIDTH * 0.5:
            # slight shading across the arc thickness for a rounded look
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