"""Falcon AVM calibration config (a port of ``mediapipe_avm_calib`` JsonGenerator).

The app builds ``config.json`` from the plane scene, the four camera streams and
the bowl / mask / steering assets.  This module reproduces that structure (and
its field order) so the simulated scene can emit a drop-in ``vehicle_avm.json``
for the Falcon renderer.  Pure Python (no ``bpy``).

The asset / orbit / mask / steering defaults are mirrored from the app's
``res/values/bowl.xml`` and ``res/values/steering.xml``.
"""

from __future__ import annotations

import json
import re
from typing import Dict, Iterable, List, Sequence, Tuple

from . import avm_layout
from . import vehicle

__all__ = [
    "ASSETS", "BOWL_RADIUS", "MASK_PADDING", "MASK_SCALE",
    "MODEL_SCENE", "BA_OPT", "ORBIT", "STEERING",
    "bev_bound", "build_config", "dumps",
]

#: ``BowlScenePerfs.Assets`` defaults
ASSETS: Dict[str, str] = {
    "name": "filament_avm",
    "glb_file": "/vendor/etc/avmconfig/falcon/unlit_round_bowls.glb",
    "ibl_file": "/vendor/etc/avmconfig/falcon/ibl.ktx",
    # One directory for every shader artifact: the packed shaders.pack plus the
    # loose *.filamat files (the app resolves each material by id, see
    # filament_avm docs/specs/005_shader_pack).
    "shader_path": "/vendor/etc/avmconfig/falcon",
    "vehicle_model": "vehicle",
}

#: ``filament_bowl_radius`` / ``filament_mask_padding`` / ``filament_mask_scale``
BOWL_RADIUS = 15.0
MASK_PADDING = 0.2
MASK_SCALE = 20

#: ``filament_bowl_scene`` (one per camera, in the Camera enum order)
MODEL_SCENE = {
    "front": "NurbsPath.001",
    "back": "NurbsPath.002",
    "left": "NurbsPath.003",
    "right": "NurbsPath.004",
}

#: ``BowlScenePerfs.ba`` - only the side cameras refine cy
BA_OPT = {"front": False, "back": False, "left": True, "right": True}

#: ``BowlScenePerfs.Orbit`` defaults
ORBIT = {
    "pitchs": [65.0, 90.0, 65.0],
    "yaw": 90.0,
    "radius": 8.0,
    "mouse_sens": 0.3,
    "fov_y": 100.0,
    "near": 0.1,
    "far": 100.0,
}

#: ``SteeringPerfs`` defaults (``res/values/steering.xml``).  The vehicle
#: dimensions read :mod:`.vehicle`: the app already ships the same numbers, and
#: an ego mask or a vehicle projection anchored on them must be anchored on the
#: *rendered* car, not on a second copy of it (``docs/drive-scene-multicam.md``
#: section 5.6.3).
STEERING = {
    "enable": True,
    "wheel_base": vehicle.WHEEL_BASE_M,
    "rear_track": vehicle.REAR_TRACK_M,
    "length": [3.6, 0.0, 0.0],
    "segments": 64,
    "thickness": [0.04, 0.02, 0.03],
    "ground_above": 0.01,
    "steering_range": [-45.0, 45.0],
    "rear_center_offset": vehicle.REAR_CENTER_OFFSET_M,
    "color": ["#1010FF", "#FF1010", "#FFB000"],
    "blend_order": [32767, 32766, 32765],
    "body_width": vehicle.BODY_WIDTH_M,
    "back_prewarp": True,
}


def bev_bound(camera: str, radius: float = BOWL_RADIUS) -> List[float]:
    """``BowlScenePerfs.bound``: the half-bowl each camera renders into."""
    return {
        "front": [-radius, radius, 0.0, radius],
        "back": [-radius, radius, -radius, 0.0],
        "left": [-radius, 0.0, -radius, radius],
        "right": [0.0, radius, -radius, radius],
    }[camera]


def _stream(field: avm_layout.FieldSpec, record: Dict,
            points_2d: Sequence[Sequence[float]],
            input_size: Tuple[int, int]) -> Dict:
    """One ``StreamConf`` (field order matches the Kotlin LinkedHashMap)."""
    fx, fy, cx, cy = (float(value) for value in record["K"])
    name = record["name"]
    return {
        "name": name,
        "points_2d": [[round(float(u), 3), round(float(v), 3)]
                      for u, v in points_2d],
        "points_3d": [[float(x), float(y), float(z)]
                      for x, y, z in avm_layout.points(name, field)],
        "K": [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        "D": [float(value) for value in record["D"]],
        "input_size": [int(input_size[0]), int(input_size[1])],
        "model_scene": MODEL_SCENE.get(name, ""),
        "bev_bound": bev_bound(name),
        "orbit": [0.0, 0.0, 0.0],
        "enable": bool(points_2d),
        "ba_opt": BA_OPT.get(name, False),
    }


def build_config(field: avm_layout.FieldSpec,
                 cameras: Iterable[Dict],
                 points_2d: Dict[str, Sequence[Sequence[float]]],
                 input_sizes: Dict[str, Tuple[int, int]],
                 date_text: str = "") -> Dict:
    """The full Falcon ``config.json`` object, in the app's field order.

    ``cameras`` are the records of :func:`core.scenes.avm_layout.cameras_from_preset`
    shape (``name`` / ``K`` / ``D``); ``points_2d`` is ``{camera: [[u, v] * 8]}``
    (empty for an undetected camera) and ``input_sizes`` is ``{camera: (w, h)}``.
    """
    geo = avm_layout.geometry(field)
    # PlaneScenePerfs.bev() uses the scene *width* on both axes
    width = field.scene_w * avm_layout.CM_TO_M
    config: Dict = {
        "comment": ("#THIS config.json AUTO GENERATE BY VISION SIM AVM SCENE. "
                    f"DATE({date_text})"),
        "name": ASSETS["name"],
        "glb_file": ASSETS["glb_file"],
        "ibl_file": ASSETS["ibl_file"],
        "shader_path": ASSETS["shader_path"],
        "vehicle_model": ASSETS["vehicle_model"],
        "mask_overlay": {
            "refer": [geo.c_in_x, geo.c_in_y, BOWL_RADIUS],
            "padding": MASK_PADDING,
            "scale": MASK_SCALE,
        },
        "cameras": [
            _stream(field, record, points_2d.get(record["name"], []),
                    input_sizes.get(record["name"], (1280, 960)))
            for record in cameras
        ],
        "bev_coord": [-width, width, -width, width],
    }
    config.update({
        "orbit_pitchs": list(ORBIT["pitchs"]),
        "orbit_yaw": ORBIT["yaw"],
        "orbit_radius": ORBIT["radius"],
        "orbit_mouse_sens": ORBIT["mouse_sens"],
        "orbit_fov_y": ORBIT["fov_y"],
        "orbit_near": ORBIT["near"],
        "orbit_far": ORBIT["far"],
        "steering_line": dict(STEERING),
    })
    return config


#: an innermost array (no nested array/object): the ones the app keeps on one line
_INLINE_ARRAY = re.compile(r"\[[^\[\]{}]*\]")


def _inline_array(match: "re.Match") -> str:
    inner = match.group(0)[1:-1].strip()
    if not inner:
        return "[]"
    return "[" + ", ".join(part.strip() for part in inner.split(",")) + "]"


def dumps(config: Dict) -> str:
    """Pretty JSON with every scalar array on a single line.

    ``json.dumps(indent=2)`` puts one number per line; the app (Gson + its
    ``strip()``) keeps each innermost array on one line and only wraps the outer
    ``points_2d`` / ``points_3d`` / ``K``.  This reproduces that layout.
    """
    text = json.dumps(config, indent=2)
    return _INLINE_ARRAY.sub(_inline_array, text) + "\n"
