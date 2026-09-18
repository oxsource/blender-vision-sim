"""AVM ground coverage: camera footprints, blind spots and block visibility.

Pure Python (no ``bpy``).  The camera pose is the Blender object pose
(``location`` + XYZ euler in degrees), and the rays come from
:func:`core.camera_model.ray_from_pixel`, so the coverage matches exactly what
the Cycles fisheye camera renders.

Geometry
--------
For every sampled image-border pixel the ray is mapped into the world frame and
intersected with the ground plane ``z = 0``::

    d_cam  = ray_from_pixel(u, v, intr, dist)      # OpenCV camera frame
    d_local = M @ d_cam                            # M = diag(1, -1, -1)
    d_world = R_object @ d_local
    t = -C_z / d_world_z            (needs d_world_z < 0, i.e. pointing down)
    P = C + t * d_world

Rays that point at or above the horizon (``d_world_z >= 0``) have no ground
intersection; the polygon is then flagged **open** and those samples are
dropped instead of being silently clamped.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .. import camera_model, transform
from . import avm_layout

__all__ = [
    "Point", "Footprint", "CoverageReport",
    "euler_matrix", "object_matrix", "camera_centre", "rotation_of",
    "image_border_points", "valid_circle_points", "ground_point",
    "project_ground_point", "covers_ground_point", "footprint", "footprints",
    "point_in_polygon", "polygon_area", "coverage_report", "visibility_matrix",
]

Point = Tuple[float, float]


# ---------------------------------------------------------------------------
# pose
# ---------------------------------------------------------------------------
def euler_matrix(rotation_deg: Sequence[float]) -> Tuple[float, ...]:
    """Row-major 3x3 of Blender's XYZ euler (``R = Rz @ Ry @ Rx``)."""
    x, y, z = (math.radians(value) for value in rotation_deg)
    cx, sx = math.cos(x), math.sin(x)
    cy, sy = math.cos(y), math.sin(y)
    cz, sz = math.cos(z), math.sin(z)
    return (
        cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx,
        sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx,
        -sy, cy * sx, cy * cx,
    )


def object_matrix(location: Sequence[float],
                  rotation_deg: Sequence[float]) -> Tuple[float, ...]:
    """Row-major 4x4 camera-to-world matrix from a Blender object pose."""
    r = euler_matrix(rotation_deg)
    return (r[0], r[1], r[2], float(location[0]),
            r[3], r[4], r[5], float(location[1]),
            r[6], r[7], r[8], float(location[2]),
            0.0, 0.0, 0.0, 1.0)


def camera_centre(matrix: Sequence[float]) -> Tuple[float, float, float]:
    return float(matrix[3]), float(matrix[7]), float(matrix[11])


def rotation_of(matrix: Sequence[float]) -> Tuple[float, ...]:
    """Row-major 3x3 rotation out of a row-major 4x4 matrix."""
    return (matrix[0], matrix[1], matrix[2],
            matrix[4], matrix[5], matrix[6],
            matrix[8], matrix[9], matrix[10])


def _world_direction(u: float, v: float, intr: camera_model.Intrinsics,
                     dist: camera_model.Distortion,
                     matrix: Sequence[float]) -> Tuple[float, float, float]:
    ray = camera_model.ray_from_pixel(u, v, intr, dist)
    local = (ray[0], -ray[1], -ray[2])  # OpenCV camera frame -> Blender local
    return transform.mat3_vec(rotation_of(matrix), local)


# ---------------------------------------------------------------------------
# sampling
# ---------------------------------------------------------------------------
def image_border_points(intr: camera_model.Intrinsics,
                        samples: int = 256) -> List[Point]:
    """Evenly spaced samples along the rendered image rectangle perimeter."""
    width, height = float(intr.width), float(intr.height)
    perimeter = 2.0 * (width + height)
    points: List[Point] = []
    for index in range(max(4, int(samples))):
        s = perimeter * index / max(4, int(samples))
        if s < width:
            points.append((s, 0.0))
        elif s < width + height:
            points.append((width, s - width))
        elif s < 2.0 * width + height:
            points.append((width - (s - width - height), height))
        else:
            points.append((0.0, height - (s - 2.0 * width - height)))
    return points


def _rect_radius(cx: float, cy: float, width: float, height: float,
                 angle: float) -> float:
    """Distance from ``(cx, cy)`` to the image rectangle border along ``angle``."""
    dx, dy = math.cos(angle), math.sin(angle)
    best = 0.0
    for t in ((0.0 - cx) / dx if abs(dx) > 1e-12 else 0.0,
              (width - cx) / dx if abs(dx) > 1e-12 else 0.0,
              (0.0 - cy) / dy if abs(dy) > 1e-12 else 0.0,
              (height - cy) / dy if abs(dy) > 1e-12 else 0.0):
        if t <= 0.0:
            continue
        x, y = cx + t * dx, cy + t * dy
        if -1e-9 <= x <= width + 1e-9 and -1e-9 <= y <= height + 1e-9:
            best = t if best == 0.0 else min(best, t)
    return best


def valid_circle_points(intr: camera_model.Intrinsics,
                        dist: camera_model.Distortion,
                        samples: int = 256) -> List[Point]:
    """The fisheye valid domain (theta < 90 deg), clipped to the image rectangle."""
    radius = camera_model.fisheye_valid_radius_px(intr, dist)
    cx, cy = intr.cx, intr.cy
    points: List[Point] = []
    for index in range(max(4, int(samples))):
        angle = 2.0 * math.pi * index / max(4, int(samples))
        r = min(radius, _rect_radius(cx, cy, intr.width, intr.height, angle))
        points.append((cx + r * math.cos(angle), cy + r * math.sin(angle)))
    return points


def ground_point(u: float, v: float, intr: camera_model.Intrinsics,
                 dist: camera_model.Distortion,
                 matrix: Sequence[float]) -> Optional[Point]:
    """Where the ray through ``(u, v)`` meets ``z = 0``, or ``None``."""
    centre = camera_centre(matrix)
    direction = _world_direction(u, v, intr, dist, matrix)
    if direction[2] >= -1e-9 or centre[2] <= 0.0:
        return None
    t = -centre[2] / direction[2]
    if t <= 0.0:
        return None
    return (centre[0] + t * direction[0], centre[1] + t * direction[1])


def project_ground_point(point: Point, intr: camera_model.Intrinsics,
                         dist: camera_model.Distortion,
                         matrix: Sequence[float]) -> Optional[Point]:
    """A ground point -> image pixel ``(u, v)``, or ``None`` if not projectable.

    The exact inverse of :func:`ground_point`; used to seed the corner detector
    (and the visibility matrix).
    """
    centre = camera_centre(matrix)
    world = (point[0] - centre[0], point[1] - centre[1], -centre[2])
    if world[2] >= -1e-9:
        return None
    local = transform.mat3_vec(transform.mat3_transpose(rotation_of(matrix)), world)
    x, y, z = local[0], -local[1], -local[2]  # Blender local -> OpenCV camera
    if z <= 0.0:
        return None
    try:
        u, v, _ = camera_model.project_point(x, y, z, intr, dist)
    except (ValueError, ZeroDivisionError):
        return None
    if not (math.isfinite(u) and math.isfinite(v)):
        return None
    return u, v


def covers_ground_point(point: Point, intr: camera_model.Intrinsics,
                        dist: camera_model.Distortion,
                        matrix: Sequence[float]) -> bool:
    """Is a ground point inside this camera's rendered image?

    The exact test (the footprint polygon is only an approximation for open,
    horizon-cut views): project the ground point back through the fisheye model
    and check the pixel lands inside the image.
    """
    uv = project_ground_point(point, intr, dist, matrix)
    if uv is None:
        return False
    return 0.0 <= uv[0] <= intr.width and 0.0 <= uv[1] <= intr.height


# ---------------------------------------------------------------------------
# footprints
# ---------------------------------------------------------------------------
@dataclass
class Footprint:
    """One camera's ground footprint."""

    camera: str
    points: List[Point] = field(default_factory=list)
    closed: bool = True
    domain: str = "image"

    @property
    def area(self) -> float:
        return polygon_area(self.points) if self.closed else float("nan")


def footprint(camera: Dict, samples: int = 256,
              domain: str = "image") -> Footprint:
    """Ground footprint of one camera record (``avm_layout`` shape)."""
    intr = avm_layout.core_intrinsics(camera)
    dist = avm_layout.core_distortion(camera)
    matrix = object_matrix(camera["location"], camera["rotation"])
    border = (image_border_points(intr, samples) if domain == "image"
              else valid_circle_points(intr, dist, samples))
    points: List[Point] = []
    open_ = False
    for u, v in border:
        point = ground_point(u, v, intr, dist, matrix)
        if point is None:
            open_ = True
        else:
            points.append(point)
    return Footprint(camera=camera["name"], points=points,
                     closed=not open_, domain=domain)


def footprints(cameras: Iterable[Dict], samples: int = 256,
               domain: str = "image") -> List[Footprint]:
    return [footprint(camera, samples=samples, domain=domain)
            for camera in cameras if camera.get("enable", True)]


# ---------------------------------------------------------------------------
# polygon maths
# ---------------------------------------------------------------------------
def polygon_area(points: Sequence[Point]) -> float:
    """Absolute area of a simple polygon (shoelace formula)."""
    if len(points) < 3:
        return 0.0
    total = 0.0
    for index, (x0, y0) in enumerate(points):
        x1, y1 = points[(index + 1) % len(points)]
        total += x0 * y1 - x1 * y0
    return abs(total) * 0.5


def point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    """Ray casting test; ``False`` for an open / degenerate polygon."""
    if len(polygon) < 3:
        return False
    x, y = point
    inside = False
    count = len(polygon)
    for index in range(count):
        x0, y0 = polygon[index]
        x1, y1 = polygon[(index + 1) % count]
        if (y0 > y) != (y1 > y):
            x_cross = (x1 - x0) * (y - y0) / (y1 - y0) + x0
            if x < x_cross:
                inside = not inside
    return inside


# ---------------------------------------------------------------------------
# report
# ---------------------------------------------------------------------------
@dataclass
class CoverageReport:
    """Footprints plus the numbers a real-field evaluation needs."""

    footprints: List[Footprint]
    per_camera_area: Dict[str, float]
    union_area: float
    overlap_area: float
    blind_area: float
    field_area: float
    coverage_ratio: float
    visibility: Dict[str, Dict[str, bool]]
    open_polygons: List[str]

    @property
    def field_ok(self) -> bool:
        """Every block seen by at least one camera (all four corners)."""
        return all(any(seen.values()) for seen in self.visibility.values())

    def as_dict(self) -> Dict:
        return {
            "per_camera_area": self.per_camera_area,
            "union_area": self.union_area,
            "overlap_area": self.overlap_area,
            "blind_area": self.blind_area,
            "field_area": self.field_area,
            "coverage_ratio": self.coverage_ratio,
            "visibility": self.visibility,
            "open_polygons": self.open_polygons,
            "field_ok": self.field_ok,
        }


def _grid(bounds: Tuple[float, float, float, float], step: float) -> List[Point]:
    x_min, x_max, y_min, y_max = bounds
    columns = max(1, int(math.ceil((x_max - x_min) / step)))
    rows = max(1, int(math.ceil((y_max - y_min) / step)))
    return [(x_min + (column + 0.5) * step, y_min + (row + 0.5) * step)
            for row in range(rows) for column in range(columns)]


def _camera_models(camera: Dict):
    return (avm_layout.core_intrinsics(camera),
            avm_layout.core_distortion(camera),
            object_matrix(camera["location"], camera["rotation"]))


def visibility_matrix(cameras: Sequence[Dict],
                      blocks: Dict[str, Tuple[float, float, float, float]]
                      ) -> Dict[str, Dict[str, bool]]:
    """For each block, which cameras see all four of its corners."""
    models = {camera["name"]: _camera_models(camera) for camera in cameras
              if camera.get("enable", True)}
    result: Dict[str, Dict[str, bool]] = {}
    for name, (x0, y0, x1, y1) in blocks.items():
        corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
        result[name] = {
            camera: all(covers_ground_point(corner, intr, dist, matrix)
                        for corner in corners)
            for camera, (intr, dist, matrix) in models.items()
        }
    return result


def coverage_report(cameras: Sequence[Dict], field: avm_layout.FieldSpec,
                    samples: int = 256, step: float = 0.02,
                    domain: str = "image") -> CoverageReport:
    """Footprints + areas + block visibility over the field bounding box."""
    geo = avm_layout.geometry(field)
    bounds = (-geo.half_x, geo.half_x, -geo.half_y, geo.half_y)
    camera_footprints = footprints(cameras, samples=samples, domain=domain)
    models = {camera["name"]: _camera_models(camera) for camera in cameras
              if camera.get("enable", True)}

    cells = _grid(bounds, step)
    cell_area = step * step
    per_camera = {camera.camera: 0.0 for camera in camera_footprints}
    union = overlap = blind = 0
    for point in cells:
        hits = [name for name, (intr, dist, matrix) in models.items()
                if covers_ground_point(point, intr, dist, matrix)]
        for name in hits:
            per_camera[name] += cell_area
        if hits:
            union += 1
            if len(hits) >= 2:
                overlap += 1
        else:
            blind += 1

    field_area = (bounds[1] - bounds[0]) * (bounds[3] - bounds[2])
    return CoverageReport(
        footprints=camera_footprints,
        per_camera_area=per_camera,
        union_area=union * cell_area,
        overlap_area=overlap * cell_area,
        blind_area=blind * cell_area,
        field_area=field_area,
        coverage_ratio=(len(cells) - blind) / max(1, len(cells)),
        visibility=visibility_matrix(cameras, avm_layout.block_rects(field)),
        open_polygons=[camera.camera for camera in camera_footprints if not camera.closed],
    )
