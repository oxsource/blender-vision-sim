"""AVM plane-scene layout: the field equation, the calibration blocks and the
``points(camera)`` contract, plus the ``plane_scene`` Store JSON.

Pure Python (no ``bpy``).  It mirrors ``mediapipe_avm_calib`` so the exported
files stay interchangeable:

* ``persist/PlaneScenePerfs.kt`` - ``Model.sizing()`` / ``Model.points()``
* ``ui/scene/PlaneSceneConfig.kt`` - ``PlaneSceneGeometry.of()``

Units
-----
* the field parameters (``border`` / ``corner`` / ``inner`` / ``core``) are in
  **centimetres**, exactly like the HTML tool and the App sliders;
* every derived geometry value is in **metres** (Blender's unit);
* the world frame is the Blender vehicle frame: origin at the vehicle centre on
  the ground, ``+X`` right, ``+Y`` forward, ``+Z`` up.

The field, from the outside in::

    border | corner | inner | core | inner | corner | border

``core`` is the vehicle footprint, ``corner`` the four solid calibration blocks
and ``inner`` the gap between them; the whole field is
``(border + corner + inner) * 2 + core`` on both axes.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, replace
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .. import paths
from .. import camera_model

__all__ = [
    "CM_TO_M",
    "FRONT", "BACK", "LEFT", "RIGHT", "CAMERAS",
    "BLOCK_FRONT_LEFT", "BLOCK_FRONT_RIGHT", "BLOCK_BACK_LEFT", "BLOCK_BACK_RIGHT",
    "BLOCKS", "PRESETS",
    "FieldSpec", "FieldGeometry",
    "geometry", "block_rects", "scene_size", "points", "point_names",
    "load_preset", "preset_path", "field_from_preset", "cameras_from_preset",
    "core_intrinsics", "core_distortion",
    "to_store", "from_store", "size_from",
]

CM_TO_M = 0.01

FRONT, BACK, LEFT, RIGHT = "front", "back", "left", "right"
CAMERAS: Tuple[str, ...] = (FRONT, BACK, LEFT, RIGHT)

BLOCK_FRONT_LEFT, BLOCK_FRONT_RIGHT = "front_left", "front_right"
BLOCK_BACK_LEFT, BLOCK_BACK_RIGHT = "back_left", "back_right"
BLOCKS: Tuple[str, ...] = (BLOCK_FRONT_LEFT, BLOCK_FRONT_RIGHT,
                           BLOCK_BACK_LEFT, BLOCK_BACK_RIGHT)

#: bundled scene preset (written by ``scripts/solve_avm_defaults.py``)
SCENE_ID = "avm_scene"
PRESET_NAME = "default"


@dataclass(frozen=True)
class FieldSpec:
    """The editable field parameters, in centimetres."""

    border_w: float = 0.0
    border_h: float = 0.0
    corner: float = 100.0
    inner_w: float = 20.0
    inner_h: float = 80.0
    core_w: float = 240.0
    core_h: float = 480.0

    @property
    def scene_w(self) -> float:
        """Field width in cm (``Model.sizing()``)."""
        return self.core_w + 2.0 * (self.border_w + self.corner + self.inner_w)

    @property
    def scene_h(self) -> float:
        """Field height in cm (``Model.sizing()``)."""
        return self.core_h + 2.0 * (self.border_h + self.corner + self.inner_h)

    def replaced(self, **values: float) -> "FieldSpec":
        return replace(self, **values)


@dataclass(frozen=True)
class FieldGeometry:
    """Derived half extents, in metres (``PlaneSceneGeometry``)."""

    core_hx: float
    core_hy: float
    c_in_x: float
    c_in_y: float
    c_out_x: float
    c_out_y: float
    half_x: float
    half_y: float
    corner: float

    @property
    def scene_w(self) -> float:
        return 2.0 * self.half_x

    @property
    def scene_h(self) -> float:
        return 2.0 * self.half_y


def geometry(field: FieldSpec) -> FieldGeometry:
    """Half extents of every ring, in metres."""
    core_hx = field.core_w * CM_TO_M * 0.5
    core_hy = field.core_h * CM_TO_M * 0.5
    c_in_x = core_hx + field.inner_w * CM_TO_M
    c_in_y = core_hy + field.inner_h * CM_TO_M
    corner = field.corner * CM_TO_M
    c_out_x = c_in_x + corner
    c_out_y = c_in_y + corner
    return FieldGeometry(
        core_hx=core_hx,
        core_hy=core_hy,
        c_in_x=c_in_x,
        c_in_y=c_in_y,
        c_out_x=c_out_x,
        c_out_y=c_out_y,
        half_x=c_out_x + field.border_w * CM_TO_M,
        half_y=c_out_y + field.border_h * CM_TO_M,
        corner=corner,
    )


def scene_size(field: FieldSpec) -> Tuple[float, float]:
    """``(width, height)`` of the field in cm."""
    return field.scene_w, field.scene_h


def block_rects(field: FieldSpec) -> Dict[str, Tuple[float, float, float, float]]:
    """The four calibration blocks as ``(x0, y0, x1, y1)`` rectangles in metres."""
    geo = geometry(field)
    return {
        BLOCK_FRONT_LEFT: (-geo.c_out_x, geo.c_in_y, -geo.c_in_x, geo.c_out_y),
        BLOCK_FRONT_RIGHT: (geo.c_in_x, geo.c_in_y, geo.c_out_x, geo.c_out_y),
        BLOCK_BACK_LEFT: (-geo.c_out_x, -geo.c_out_y, -geo.c_in_x, -geo.c_in_y),
        BLOCK_BACK_RIGHT: (geo.c_in_x, -geo.c_out_y, geo.c_out_x, -geo.c_in_y),
    }


def point_names(camera: str) -> List[str]:
    """Names of the eight ``points_3d`` of ``camera``, in order."""
    if camera in (FRONT, BACK):
        rows = [(x, y) for y in ("outer", "inner") for x in ("outer_l", "inner_l",
                                                             "inner_r", "outer_r")]
    elif camera in (LEFT, RIGHT):
        rows = [(x, y) for x in ("outer", "inner") for y in ("back_outer", "back_inner",
                                                             "front_inner", "front_outer")]
    else:
        raise ValueError(f"unknown camera {camera!r}")
    return [f"{x}_{y}" for x, y in rows]


def points(camera: str, field: FieldSpec) -> List[List[float]]:
    """The eight ``points_3d`` of ``camera``, in metres.

    This is a line by line port of ``PlaneScenePerfs.Model.points()``: the
    ordering (and the ``back`` / ``right`` sign flip) is the 2D<->3D contract
    the config files rely on, so it must not be "tidied up".
    """
    if camera not in CAMERAS:
        raise ValueError(f"unknown camera {camera!r} (expected one of {CAMERAS})")
    geo = geometry(field)
    x1, x2 = geo.c_in_x, geo.c_out_x
    y1, y2 = geo.c_in_y, geo.c_out_y

    if camera in (FRONT, BACK):
        values = [(-x2, y2), (-x1, y2), (x1, y2), (x2, y2),
                  (-x2, y1), (-x1, y1), (x1, y1), (x2, y1)]
    else:  # LEFT, RIGHT
        values = [(-x2, -y2), (-x2, -y1), (-x2, y1), (-x2, y2),
                  (-x1, -y2), (-x1, -y1), (-x1, y1), (-x1, y2)]

    if camera in (BACK, RIGHT):  # front / left keep the values as-is
        values = [(-x, -y) for x, y in values]
    return [[x, y, 0.0] for x, y in values]


# ---------------------------------------------------------------------------
# bundled preset + the plane_scene Store JSON
# ---------------------------------------------------------------------------
def preset_path() -> str:
    """Absolute path of the bundled AVM default preset."""
    return paths.scene_preset_file(SCENE_ID, PRESET_NAME)


def load_preset(path: Optional[str] = None) -> Dict[str, Any]:
    """Load the bundled (or given) AVM default preset."""
    with open(path or preset_path(), "r", encoding="utf-8") as handle:
        preset = json.load(handle)
    if not isinstance(preset, dict) or "field" not in preset or "cameras" not in preset:
        raise ValueError("not an AVM scene preset (missing 'field'/'cameras')")
    return preset


def field_from_preset(preset: Dict[str, Any]) -> FieldSpec:
    """The preset's ``field`` block as a :class:`FieldSpec`."""
    field = preset["field"]
    return FieldSpec(**{key: float(field[key]) for key in (
        "border_w", "border_h", "corner", "inner_w", "inner_h", "core_w", "core_h"
    )})


def cameras_from_preset(preset: Dict[str, Any]) -> List[Dict[str, Any]]:
    """The preset's camera records, normalised to lists of floats."""
    cameras = []
    for camera in preset["cameras"]:
        cameras.append({
            "name": camera["name"],
            "enable": bool(camera.get("enable", True)),
            "location": [float(v) for v in camera["location"]],
            "rotation": [float(v) for v in camera["rotation"]],
            "K": [float(v) for v in camera["K"]],
            "D": [float(v) for v in camera["D"]],
            "output": [int(v) for v in camera.get("output", (1280, 960))],
        })
    return cameras


def core_intrinsics(camera: Dict[str, Any]) -> camera_model.Intrinsics:
    """A camera record's ``K`` as a :class:`camera_model.Intrinsics`.

    ``K`` is ``[fx, fy, cx, cy]`` and the size comes from the record's output
    resolution (the intrinsics belong to it, like any calibration).
    """
    fx, fy, cx, cy = (float(v) for v in camera["K"])
    width, height = (int(v) for v in camera.get("output", (1280, 960)))
    return camera_model.Intrinsics(fx=fx, fy=fy, cx=cx, cy=cy,
                                   width=width, height=height)


def core_distortion(camera: Dict[str, Any]) -> camera_model.Distortion:
    """A camera record's ``D`` (four fisheye coefficients) as a core model."""
    coefficients = [float(v) for v in camera["D"]]
    return camera_model.Distortion.from_coefficients(
        coefficients, model=camera_model.MODEL_FISHEYE
    )


def to_store(field: FieldSpec) -> Dict[str, Any]:
    """``PlaneScenePerfs.Store`` (the HTML / App interchange format)."""
    return {
        "border": _size_text(field.border_w, field.border_h),
        "corner": int(round(field.corner)),
        "inner": _size_text(field.inner_w, field.inner_h),
        "car": _size_text(field.core_w, field.core_h),
    }


def from_store(store: Dict[str, Any]) -> FieldSpec:
    """Parse a Store dict (or a full ``avm_scene`` export) tolerantly."""
    field = FieldSpec()
    border = size_from(store.get("border"))
    if border is not None:
        field = field.replaced(border_w=border[0], border_h=border[1])
    inner = size_from(store.get("inner"))
    if inner is not None:
        field = field.replaced(inner_w=inner[0], inner_h=inner[1])
    car = size_from(store.get("car", store.get("core")))
    if car is not None:
        field = field.replaced(core_w=car[0], core_h=car[1])
    corner = store.get("corner")
    if isinstance(corner, (int, float)):
        field = field.replaced(corner=float(corner))
    return field


def size_from(value: Any) -> Optional[Tuple[float, float]]:
    """Tolerant ``"WxH"`` / number / ``[w, h]`` / ``{width, height}`` parser.

    A port of the HTML tool's ``sizeFrom()``: it accepts the historical shapes
    so files written by any of the three tools keep loading.
    """
    if isinstance(value, str):
        parts = value.split("x")
        if len(parts) >= 2:
            try:
                return float(parts[0].strip()), float(parts[1].strip())
            except ValueError:
                return None
        return None
    if isinstance(value, (int, float)):
        return float(value), float(value)
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return float(value[0]), float(value[1])
    if isinstance(value, dict):
        w = value.get("width", value.get("w"))
        h = value.get("height", value.get("h"))
        if w is not None and h is not None:
            return float(w), float(h)
    return None


def _size_text(width: float, height: float) -> str:
    return f"{_num(width)}x{_num(height)}"


def _num(value: float) -> str:
    rounded = int(round(value))
    return str(rounded) if abs(value - rounded) < 1e-9 else f"{value:g}"


#: quick presets of the HTML tool (field sizes only, in cm); the "minibus" entry
#: mirrors the bundled preset
PRESETS = {
    "default": ("Default (cross car)", FieldSpec(
        border_w=10.0, border_h=10.0, corner=50.0,
        inner_w=0.0, inner_h=0.0, core_w=262.0, core_h=474.0)),
    "no_border": ("No outer border", FieldSpec(
        border_w=0.0, border_h=0.0, corner=50.0,
        inner_w=0.0, inner_h=0.0, core_w=262.0, core_h=474.0)),
    "large_corner": ("Large corner + gap", FieldSpec(
        border_w=10.0, border_h=10.0, corner=100.0,
        inner_w=20.0, inner_h=20.0, core_w=262.0, core_h=474.0)),
    "suv": ("SUV wide body", FieldSpec(
        border_w=20.0, border_h=20.0, corner=60.0,
        inner_w=10.0, inner_h=10.0, core_w=300.0, core_h=520.0)),
    "minibus": ("Minibus (bundled)", FieldSpec(
        border_w=0.0, border_h=0.0, corner=100.0,
        inner_w=20.0, inner_h=80.0, core_w=240.0, core_h=480.0)),
}
