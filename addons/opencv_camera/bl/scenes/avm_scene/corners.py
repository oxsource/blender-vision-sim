"""Corner detection for the AVM Scene camera images.

The reference AVM app detects the calibration blocks in every camera image with
OpenCV (``mediapipe_avm_calib``: ``CalibProjActivity`` -> ``FalconNative.corners``
-> ``perspective_calib_detect``).  Blender ships numpy but **not** cv2, so this is
a numpy port of the same idea for the synthetic scene:

1. render a camera to a PNG at its own output resolution;
2. project that camera's eight ``points_3d`` through the camera model - the
   projection is the search prior, the same role the reference's x/y range
   sliders play;
3. for every block, sample the image across each projected edge and find the
   sub-pixel black/white transition, fit a line to each edge and intersect the
   adjacent lines -> the four corners;
4. order the corners to match ``points_3d`` and store them as ``points_2d``.

The coordinates come from the rendered pixels; the projection only bounds the
search, so a wrong pose or a wrong intrinsics scale shows up as a large residual
instead of being silently copied.

A detection also writes an **annotated image** (green detected corners with
their ``points_3d`` index, red projected points, the block outlines) that can be
opened in the Image Editor, and caches the result per camera keyed on the scene
revision plus the camera pose/intrinsics/output, so a re-detect with unchanged
inputs reuses the cached points instead of rendering again.
"""

from __future__ import annotations

import math
import os
import shutil
import struct
import tempfile
import zlib
from typing import Dict, List, Optional, Sequence, Tuple

import bpy
import numpy as np

from ....core.scenes import avm_coverage, avm_layout
from ... import apply as apply_mod
from . import builder, io as io_mod

#: the eight ``points_3d`` of a camera split into its two visible blocks
#: (indices into ``avm_layout.points(camera, field)``; true for all four cameras)
BLOCK_GROUPS = ((0, 1, 4, 5), (2, 3, 6, 7))

#: luminance weights (Rec. 709)
LUMA = (0.2126, 0.7152, 0.0722)

#: annotation colours (RGB)
DETECTED_COLOR = (0, 220, 0)
PROJECTED_COLOR = (230, 40, 40)
OUTLINE_COLOR = (0, 150, 0)
LABEL_COLOR = (255, 255, 255)

#: minimal 5x7 bitmap font for the point indices (0..7)
DIGIT_FONT = {
    0: ("01110", "10001", "10011", "10101", "11001", "10001", "01110"),
    1: ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
    2: ("01110", "10001", "00001", "00010", "00100", "01000", "11111"),
    3: ("11111", "00010", "00100", "00010", "00001", "10001", "01110"),
    4: ("00010", "00110", "01010", "10010", "11111", "00010", "00010"),
    5: ("11111", "10000", "11110", "00001", "00001", "10001", "01110"),
    6: ("00110", "01000", "10000", "11110", "10001", "10001", "01110"),
    7: ("11111", "00001", "00010", "00100", "01000", "01000", "01000"),
    8: ("01110", "10001", "10001", "01110", "10001", "10001", "01110"),
    9: ("01110", "10001", "10001", "01111", "00001", "00010", "01100"),
}

#: cached annotated images live here (session temp; the Image Editor can show them)
IMAGE_DIR = "avm_corners"


# ---------------------------------------------------------------------------
# image access
# ---------------------------------------------------------------------------
def _load_pixels(path: str) -> Optional[np.ndarray]:
    """Load a rendered PNG as a top-left-origin float RGB array (0..1)."""
    try:
        image = bpy.data.images.load(path)
    except Exception:
        return None
    try:
        width, height = image.size
        pixels = np.array(image.pixels[:], dtype=np.float32).reshape(height, width, 4)
    finally:
        bpy.data.images.remove(image)
    # Blender stores rows bottom-up; points_2d use the top-left origin
    return np.flipud(pixels[..., :3])


def _load_image(path: str):
    """``(rgb, gray)`` for a rendered PNG, or ``(None, None)``."""
    rgb = _load_pixels(path)
    if rgb is None:
        return None, None
    gray = (rgb[..., 0] * LUMA[0] + rgb[..., 1] * LUMA[1] + rgb[..., 2] * LUMA[2])
    return rgb, gray


def _bilinear(gray: np.ndarray, xs, ys) -> np.ndarray:
    """Bilinear sample; outside the image the result is ``nan``."""
    height, width = gray.shape
    x = np.asarray(xs, dtype=np.float64)
    y = np.asarray(ys, dtype=np.float64)
    inside = (x >= 0) & (x <= width - 1) & (y >= 0) & (y <= height - 1)
    x0 = np.clip(np.floor(x), 0, width - 1).astype(int)
    y0 = np.clip(np.floor(y), 0, height - 1).astype(int)
    x1 = np.clip(x0 + 1, 0, width - 1)
    y1 = np.clip(y0 + 1, 0, height - 1)
    fx = x - x0
    fy = y - y0
    top = gray[y0, x0] * (1 - fx) + gray[y0, x1] * fx
    bottom = gray[y1, x0] * (1 - fx) + gray[y1, x1] * fx
    return np.where(inside, top * (1 - fy) + bottom * fy, np.nan)


# ---------------------------------------------------------------------------
# sub-pixel edge fitting
# ---------------------------------------------------------------------------
def _peak_offset(profile: np.ndarray, step: float) -> float:
    """Sub-pixel position of the ``|profile|`` peak, in samples, via a parabola."""
    index = int(np.argmax(profile))
    if index <= 0 or index >= len(profile) - 1:
        return float(index) * step
    left, centre, right = profile[index - 1], profile[index], profile[index + 1]
    denominator = left - 2.0 * centre + right
    if abs(denominator) < 1e-12:
        return float(index) * step
    return (index + 0.5 * (left - right) / denominator) * step


def _edge_points(gray: np.ndarray, p0, p1, radius: float,
                 samples: int = 32, step: float = 0.2) -> List[np.ndarray]:
    """Sub-pixel points along the strong intensity edge between ``p0`` and ``p1``.

    Samples the segment away from the corners, marches perpendicular over
    ``[-radius, +radius]`` and keeps the position of the largest gradient.  The
    black block against its white border gives a much stronger gradient than any
    other edge in the crop, so the strongest peak is the block boundary.  Peaks
    far from the projected edge (the white-border/ground edge, or noise) are
    dropped, since the projection is accurate to well under a pixel.
    """
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    edge = p1 - p0
    length = float(np.hypot(edge[0], edge[1]))
    if length < 4.0:
        return []
    direction = edge / length
    normal = np.array([-direction[1], direction[0]])
    offsets = np.arange(-radius, radius + 1e-9, step)
    fractions = np.linspace(0.15, 0.85, max(4, int(samples)))
    bases = p0[None, :] + fractions[:, None] * edge[None, :]
    points = bases[:, None, :] + offsets[None, :, None] * normal[None, None, :]
    values = _bilinear(gray, points[..., 0], points[..., 1])
    gradient = np.abs(np.gradient(values, step, axis=1))
    gradient = np.where(np.isfinite(values), gradient, 0.0)

    peaks = gradient.max(axis=1)
    positive = peaks[peaks > 0.0]
    if positive.size == 0:
        return []
    cutoff = 0.35 * float(np.median(positive))
    limit = max(2.0, min(0.4 * radius, 6.0))
    found: List[np.ndarray] = []
    for row in range(values.shape[0]):
        if peaks[row] < cutoff:
            continue
        offset = _peak_offset(gradient[row], step) - radius
        if abs(offset) > limit:
            continue
        found.append(bases[row] + offset * normal)
    return found


def _fit_line(points: Sequence[np.ndarray]):
    """Total-least-squares line ``(point, direction)`` through ``points``."""
    cloud = np.asarray(points, dtype=float)
    if cloud.shape[0] < 2:
        return None
    centre = cloud.mean(axis=0)
    try:
        _, _, vt = np.linalg.svd(cloud - centre)
    except np.linalg.LinAlgError:
        return None
    return centre, vt[0]


def _intersect(line_a, line_b) -> Optional[np.ndarray]:
    """Intersection of two ``(point, direction)`` lines, or ``None``."""
    if line_a is None or line_b is None:
        return None
    c1, d1 = line_a
    c2, d2 = line_b
    matrix = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]])
    if abs(float(np.linalg.det(matrix))) < 1e-9:
        return None
    t = np.linalg.solve(matrix, c2 - c1)
    return c1 + t[0] * d1


def _cyclic_order(points: np.ndarray) -> np.ndarray:
    """Indices that put a convex quad's corners in angular (cyclic) order."""
    centre = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - centre[1], points[:, 0] - centre[0])
    return np.argsort(angles)


def _refine_quad(gray: np.ndarray, corners: np.ndarray) -> np.ndarray:
    """Sub-pixel corners of a quad given its projected corners (cyclic order)."""
    lines = []
    for index in range(4):
        p0, p1 = corners[index], corners[(index + 1) % 4]
        edge_length = float(np.hypot(*(p1 - p0)))
        radius = float(np.clip(0.35 * edge_length, 4.0, 24.0))
        edge = _edge_points(gray, p0, p1, radius)
        lines.append(_fit_line(edge) if len(edge) >= 3 else None)
    refined = []
    for index in range(4):
        point = _intersect(lines[(index - 1) % 4], lines[index])
        refined.append(np.asarray(corners[index], dtype=float) if point is None else point)
    return np.asarray(refined)


# ---------------------------------------------------------------------------
# per-camera detection
# ---------------------------------------------------------------------------
def _project_points(record: Dict, cam_settings, field: avm_layout.FieldSpec,
                    width: int, height: int):
    """The camera's eight ``points_3d`` as pixels at the render resolution."""
    intrinsics = apply_mod.effective_intrinsics(cam_settings, width, height)
    distortion = cam_settings.core_distortion()
    matrix = avm_coverage.object_matrix(record["location"], record["rotation"])
    projected = []
    for x, y, _ in avm_layout.points(record["name"], field):
        projected.append(avm_coverage.project_ground_point(
            (x, y), intrinsics, distortion, matrix))
    return projected


def detect_camera(gray: np.ndarray, record: Dict, cam_settings,
                  field: avm_layout.FieldSpec):
    """Detect the eight block corners in one camera image.

    Returns ``(detected, projected, residual)``: both ``(8, 2)`` arrays in the
    render pixels and the ``points_3d`` order, plus the RMS distance from the
    projection in pixels.  ``detected`` is ``None`` when the blocks cannot be
    found.
    """
    height, width = gray.shape
    projected = _project_points(record, cam_settings, field, width, height)
    if any(point is None for point in projected):
        return None, None, float("inf")
    projected = np.asarray(projected, dtype=float)

    detected = np.zeros((8, 2), dtype=float)
    for group in BLOCK_GROUPS:
        quad = projected[list(group)]
        order = _cyclic_order(quad)
        refined = _refine_quad(gray, quad[order])
        # refined[k] is the corner that was quad[order[k]]
        for k, quad_index in enumerate(order):
            detected[group[int(quad_index)]] = refined[k]

    residual = float(np.sqrt(np.mean(np.sum((detected - projected) ** 2, axis=1))))
    return detected, projected, residual


# ---------------------------------------------------------------------------
# annotated image (no PIL: draw into the pixels, write the PNG by hand)
# ---------------------------------------------------------------------------
def _draw_disc(rgb: np.ndarray, cx: float, cy: float, radius: float, color) -> None:
    height, width = rgb.shape[:2]
    x0 = max(0, int(math.floor(cx - radius)))
    x1 = min(width, int(math.ceil(cx + radius)) + 1)
    y0 = max(0, int(math.floor(cy - radius)))
    y1 = min(height, int(math.ceil(cy + radius)) + 1)
    if x0 >= x1 or y0 >= y1:
        return
    yy, xx = np.ogrid[y0:y1, x0:x1]
    mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius * radius
    rgb[y0:y1, x0:x1][mask] = color


def _draw_line(rgb: np.ndarray, p0, p1, color, thickness: float = 1.0) -> None:
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    steps = int(max(abs(p1[0] - p0[0]), abs(p1[1] - p0[1]))) + 1
    for t in np.linspace(0.0, 1.0, max(2, steps)):
        _draw_disc(rgb, p0[0] + t * (p1[0] - p0[0]),
                   p0[1] + t * (p1[1] - p0[1]), thickness, color)


def _draw_digit(rgb: np.ndarray, x: float, y: float, digit: int,
                scale: int, color) -> None:
    rows = DIGIT_FONT.get(int(digit) % 10)
    if rows is None:
        return
    height, width = rgb.shape[:2]
    for row_index, row in enumerate(rows):
        for column_index, cell in enumerate(row):
            if cell != "1":
                continue
            x0 = int(x) + column_index * scale
            y0 = int(y) + row_index * scale
            x1, y1 = x0 + scale, y0 + scale
            if x0 < 0 or y0 < 0 or x1 > width or y1 > height:
                continue
            rgb[y0:y1, x0:x1] = color


def annotate_image(rgb: np.ndarray, detected: np.ndarray,
                   projected: np.ndarray) -> np.ndarray:
    """Draw the detected corners (green, indexed) and the projection (red)."""
    height, width = rgb.shape[:2]
    radius = max(2, int(round(min(width, height) / 140.0)))
    scale = max(1, min(5, int(round(min(width, height) / 200.0))))
    thickness = max(1, radius // 3)
    for group in BLOCK_GROUPS:
        quad = detected[list(group)]
        for index in range(4):
            _draw_line(rgb, quad[index], quad[(index + 1) % 4], OUTLINE_COLOR, thickness)
    for point in projected:
        _draw_disc(rgb, point[0], point[1], max(1, radius // 2), PROJECTED_COLOR)
    for index, point in enumerate(detected):
        _draw_disc(rgb, point[0], point[1], radius, DETECTED_COLOR)
        _draw_digit(rgb, point[0] + radius + 2, point[1] - 3 * scale // 2,
                    index, scale, LABEL_COLOR)
    return rgb


def _write_png(path: str, rgb: np.ndarray) -> None:
    """Minimal RGB8 PNG writer (stdlib only)."""
    height, width = rgb.shape[:2]
    raw = bytearray()
    for row in rgb:
        raw.append(0)  # filter type 0
        raw.extend(row.tobytes())

    def chunk(tag: bytes, data: bytes) -> bytes:
        body = tag + data
        return (struct.pack(">I", len(data)) + body
                + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF))

    payload = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
        + chunk(b"IEND", b"")
    )
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as handle:
        handle.write(payload)


# ---------------------------------------------------------------------------
# cache + object helpers
# ---------------------------------------------------------------------------
def _camera_object(name: str):
    return bpy.data.objects.get(
        f"{builder.CAMERA_PREFIX}{builder.CAMERA_SUFFIX.get(name, '')}")


def corner_image_name(name: str) -> str:
    return f"AVM_Corners_{builder.CAMERA_SUFFIX.get(name, name)}"


def raw_image_path(name: str) -> str:
    """Cache of the **raw** render (reused by Export Falcon)."""
    return os.path.join(bpy.app.tempdir, IMAGE_DIR, f"{name}.png")


def corner_image_path(name: str) -> str:
    """Cache of the **annotated** render (shown in the Image Editor)."""
    return os.path.join(bpy.app.tempdir, IMAGE_DIR, f"{name}_annotated.png")


def corner_image(name: str):
    """The annotated image datablock of one camera, or ``None``."""
    return bpy.data.images.get(corner_image_name(name))


def show_image(context, image) -> bool:
    """Show ``image`` in an Image Editor, splitting one open if there is none.

    The detection bakes the annotation into a copy of the render (Blender cannot
    overlay arbitrary markers on an Image Editor without a GPU draw handler), so
    this is how the marked-up view reaches the screen.  Returns ``True`` when an
    editor is showing it.
    """
    screen = getattr(context, "screen", None)
    window = getattr(context, "window", None)
    if screen is None or image is None:
        return False
    for area in screen.areas:
        if area.type != "IMAGE_EDITOR":
            continue
        for space in area.spaces:
            if space.type == "IMAGE_EDITOR":
                space.image = image
        area.tag_redraw()
        return True

    candidates = [area for area in screen.areas if area.type != "IMAGE_EDITOR"]
    if not candidates:
        return False
    target = max(candidates, key=lambda area: area.width * area.height)
    region = next((item for item in target.regions if item.type == "WINDOW"), None)
    if region is None:
        return False
    before = set(screen.areas)
    try:
        with context.temp_override(window=window, screen=screen,
                                   area=target, region=region):
            bpy.ops.screen.area_split(direction="VERTICAL", factor=0.5)
    except Exception:
        return False
    created = [area for area in screen.areas if area not in before]
    if not created:
        return False
    new_area = created[0]
    new_area.type = "IMAGE_EDITOR"
    space = getattr(new_area.spaces, "active", None)
    if space is not None and space.type == "IMAGE_EDITOR":
        space.image = image
    else:
        for space in new_area.spaces:
            if space.type == "IMAGE_EDITOR":
                space.image = image
    new_area.tag_redraw()
    return True


def _load_annotated(name: str, path: str):
    image = bpy.data.images.get(corner_image_name(name))
    if image is None:
        if not os.path.exists(path):
            return None
        try:
            image = bpy.data.images.load(path)
        except Exception:
            return None
        image.name = corner_image_name(name)
    else:
        image.filepath = path
        try:
            image.reload()
        except Exception:
            pass
    return image


def _signature(settings, camera) -> str:
    """A cache key: scene revision + pose + intrinsics + render size.

    Deliberately excludes the render sample count: the detected corners are
    stable across samples, so a detection at any quality can seed the Export
    Falcon cache."""
    cam_settings = camera.data.opencv_cam
    intrinsics = cam_settings.intrinsics
    distortion = cam_settings.distortion
    render = bpy.context.scene.render
    field = settings.field_spec()
    values = [
        settings.revision,
        field.border_w, field.border_h, field.corner,
        field.inner_w, field.inner_h, field.core_w, field.core_h,
        settings.block_lift,
        camera.location[0], camera.location[1], camera.location[2],
        camera.rotation_euler[0], camera.rotation_euler[1], camera.rotation_euler[2],
        intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
        intrinsics.image_width, intrinsics.image_height,
        int(bool(intrinsics.auto_center)), int(bool(intrinsics.scale_to_render)),
        distortion.k1, distortion.k2, distortion.k3, distortion.k4,
        render.resolution_x, render.resolution_y,
    ]
    return ";".join(
        repr(round(float(value), 6)) if isinstance(value, (int, float)) else str(value)
        for value in values)


def cache_signature(settings, name: str) -> str:
    """The cache key of one camera (``""`` when it has no OpenCV camera)."""
    entry = settings.camera(name)
    camera = _camera_object(name)
    if entry is None or camera is None or getattr(camera.data, "opencv_cam", None) is None:
        return ""
    return _signature(settings, camera)


def is_cached(settings, name: str) -> bool:
    """Is a valid detection + raw image cached for this camera?"""
    entry = settings.camera(name)
    if entry is None or not entry.points_2d_ok:
        return False
    if not entry.points_2d_signature:
        return False
    if entry.points_2d_signature != cache_signature(settings, name):
        return False
    return os.path.exists(raw_image_path(name))


def clear(settings, name: str) -> str:
    """Drop one camera's detection: points, annotated image and cached render."""
    entry = settings.camera(name)
    if entry is None:
        return f"{name}: missing camera"
    entry.points_2d = [0.0] * 16
    entry.points_2d_ok = False
    entry.points_2d_error = 0.0
    entry.points_2d_revision = -1
    entry.points_2d_signature = ""
    image = bpy.data.images.get(corner_image_name(name))
    if image is not None:
        bpy.data.images.remove(image)
    for path in (raw_image_path(name), corner_image_path(name)):
        try:
            if os.path.exists(path):
                os.remove(path)
        except OSError:
            pass
    return f"{name}: cleared"


# ---------------------------------------------------------------------------
# render + detect
# ---------------------------------------------------------------------------
def detect_file(settings, name: str, path: str, samples: int = 64):
    """Detect one camera from an existing image file and cache the result.

    Returns ``(entry, status_text)``.  Also refreshes the raw-image cache, which
    is what makes a later Export Falcon reuse the render instead of repeating it.
    """
    entry = settings.camera(name)
    camera = _camera_object(name)
    if entry is None or camera is None:
        return None, f"{name}: missing camera"
    if not entry.enable:
        entry.points_2d_ok = False
        return None, f"{name}: disabled"
    cam_settings = getattr(camera.data, "opencv_cam", None)
    if cam_settings is None:
        return None, f"{name}: no OpenCV camera"
    rgb, gray = _load_image(path)
    if gray is None:
        entry.points_2d_ok = False
        return None, f"{name}: unreadable"
    record = next((item for item in io_mod.camera_records(settings)
                   if item["name"] == name), None)
    if record is None:
        entry.points_2d_ok = False
        return None, f"{name}: no camera record"
    points, projected, residual = detect_camera(
        gray, record, cam_settings, settings.field_spec())
    if points is None:
        entry.points_2d_ok = False
        return None, f"{name}: blocks not found"

    entry.points_2d = [float(value) for value in points.reshape(-1)]
    entry.points_2d_ok = True
    entry.points_2d_error = residual
    entry.points_2d_revision = settings.revision
    entry.points_2d_signature = _signature(settings, camera)

    annotated = annotate_image(
        (rgb * 255.0).round().clip(0, 255).astype(np.uint8), points, projected)
    _write_png(corner_image_path(name), annotated)
    _load_annotated(name, corner_image_path(name))
    try:  # keep the raw render for Export Falcon
        if os.path.abspath(path) != os.path.abspath(raw_image_path(name)):
            os.makedirs(os.path.dirname(raw_image_path(name)), exist_ok=True)
            shutil.copyfile(path, raw_image_path(name))
    except OSError:
        pass
    return entry, f"{name} 8/8 ({residual:.2f} px)"


def detect_one(context, settings, name: str, samples: int = 64,
               use_cache: bool = True):
    """Render and detect one camera; reuse the cached result when possible.

    Returns ``(entry, status_text)``.
    """
    entry = settings.camera(name)
    camera = _camera_object(name)
    if entry is None or camera is None:
        return None, f"{name}: missing camera"
    if not entry.enable:
        entry.points_2d_ok = False
        return None, f"{name}: disabled"
    if getattr(camera.data, "opencv_cam", None) is None:
        return None, f"{name}: no OpenCV camera"

    if use_cache and is_cached(settings, name):
        _load_annotated(name, corner_image_path(name))
        return entry, f"{name} cached ({entry.points_2d_error:.2f} px)"

    directory = tempfile.mkdtemp(prefix="avm_corner_")
    try:
        io_mod.render_cameras(context, settings, directory, samples=samples,
                              names=[name])
        path = os.path.join(directory, f"{name}.png")
        if not os.path.exists(path):
            entry.points_2d_ok = False
            return None, f"{name}: render failed"
        return detect_file(settings, name, path, samples=samples)
    finally:
        shutil.rmtree(directory, ignore_errors=True)


def detect_directory(settings, directory: str, samples: int = 64
                     ) -> Tuple[Dict[str, np.ndarray], str]:
    """Detect every enabled camera from ``<directory>/<name>.png``.

    Used by the material export, which has just rendered the images.  Stores the
    result on each camera's ``points_2d`` and returns ``({name: points}, status)``.
    """
    results: Dict[str, np.ndarray] = {}
    parts: List[str] = []
    for name in avm_layout.CAMERAS:
        entry = settings.camera(name)
        camera = _camera_object(name)
        if entry is None or camera is None:
            continue
        if not entry.enable:
            entry.points_2d_ok = False
            continue
        path = os.path.join(directory, f"{name}.png")
        if not os.path.exists(path):
            entry.points_2d_ok = False
            parts.append(f"{name}: no image")
            continue
        found, status = detect_file(settings, name, path, samples=samples)
        parts.append(status)
        if found is not None:
            results[name] = np.array(found.points_2d, dtype=float).reshape(8, 2)
    return results, " | ".join(parts)


def detect_corners(context, settings, samples: int = 64, use_cache: bool = True):
    """Detect every enabled camera, one render at a time (cache aware)."""
    results: Dict[str, np.ndarray] = {}
    parts: List[str] = []
    for name in avm_layout.CAMERAS:
        entry = settings.camera(name)
        if entry is None or not entry.enable:
            continue
        if _camera_object(name) is None:
            continue
        found, status = detect_one(context, settings, name, samples=samples,
                                   use_cache=use_cache)
        parts.append(status)
        if found is not None:
            results[name] = np.array(found.points_2d, dtype=float).reshape(8, 2)
    return results, " | ".join(parts)
