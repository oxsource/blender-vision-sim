"""AVM Scene parameter import / export and the four-camera render export.

Two parameter formats (``docs/avm-scene.md`` §8):

* **full** (``"avm_scene"``) - field + car + ground + blocks + the four cameras'
  ``location`` / ``rotation`` / ``K`` / ``D``; JSON, byte-stable round trip;
* **compact** (``"plane_scene"``) - the four HTML / App ``Store`` keys only
  (``border`` / ``corner`` / ``inner`` / ``car``, in cm).

The full format keeps the compact keys at the top level so the HTML tool can read
it; the extras live under ``"avm"`` in metres.  Camera intrinsics are read from /
written to ``camera.data.opencv_cam`` (the CV panels own them, §4.2).
"""

from __future__ import annotations

import json
import os
from typing import Dict, List, Optional

import bpy

from ....core import calibration_io
from ....core.scenes import avm_layout
from ... import apply as apply_mod
from . import builder

FORMAT_FULL = "avm_scene"
FORMAT_PLANE = "plane_scene"
VERSION = 1


# ---------------------------------------------------------------------------
# serialisation
# ---------------------------------------------------------------------------
def detect_format(data: Dict) -> str:
    """``"avm_scene"`` / ``"plane_scene"``; a bare Store counts as compact."""
    if not isinstance(data, dict):
        raise ValueError("expected a JSON/YAML object")
    fmt = data.get("format")
    if fmt == FORMAT_FULL:
        return FORMAT_FULL
    if fmt == FORMAT_PLANE:
        return FORMAT_PLANE
    if fmt not in (None, ""):
        raise ValueError(f"unknown format {fmt!r}")
    return FORMAT_FULL if "avm" in data else FORMAT_PLANE


def validate(data: Dict) -> None:
    """Raise ``ValueError`` with a readable message on a malformed file."""
    fmt = detect_format(data)
    for key in ("border", "inner", "car", "core"):
        value = data.get(key)
        if value is not None and avm_layout.size_from(value) is None:
            raise ValueError(
                f"{key!r} is not a size (expected 'WxH', a number, a pair or width/height)")
    corner = data.get("corner")
    if corner is not None and not isinstance(corner, (int, float)):
        raise ValueError("'corner' must be a number")
    if fmt != FORMAT_FULL:
        return
    avm = data.get("avm")
    if avm is None:
        return
    if not isinstance(avm, dict):
        raise ValueError("'avm' must be an object")
    for name in ("car_length", "car_width", "car_height", "car_clearance", "block_lift"):
        value = avm.get(name)
        if value is not None and not isinstance(value, (int, float)):
            raise ValueError(f"'avm.{name}' must be a number")
    ground = avm.get("ground")
    if ground is not None and avm_layout.size_from(ground) is None:
        raise ValueError("'avm.ground' is not a size")
    active = avm.get("active_camera")
    if active is not None and active not in avm_layout.CAMERAS:
        raise ValueError(f"'avm.active_camera' must be one of {avm_layout.CAMERAS}")
    cameras = avm.get("cameras")
    if cameras is None:
        return
    if not isinstance(cameras, list):
        raise ValueError("'avm.cameras' must be a list")
    for record in cameras:
        if not isinstance(record, dict) or "name" not in record:
            raise ValueError("every camera needs a 'name'")
        for key, length in (("location", 3), ("rotation", 3), ("K", 4), ("D", 4)):
            if key in record and len(record[key]) != length:
                raise ValueError(f"camera {record['name']!r}: '{key}' needs {length} values")


def camera_records(settings) -> List[Dict]:
    """The four cameras as export records (K/D read from ``opencv_cam``)."""
    records: List[Dict] = []
    for entry in settings.cameras:
        camera = bpy.data.objects.get(
            f"{builder.CAMERA_PREFIX}{builder.CAMERA_SUFFIX.get(entry.name, '')}")
        k, d = _read_intrinsics(camera)
        records.append({
            "name": entry.name,
            "enable": bool(entry.enable),
            "location": [float(value) for value in entry.location],
            "rotation": [float(value) for value in entry.rotation],
            "K": k,
            "D": d,
        })
    return records


def to_full(settings) -> Dict:
    """The full ``avm_scene`` parameter object."""
    data: Dict = {"format": FORMAT_FULL, "version": VERSION}
    data.update(avm_layout.to_store(settings.field_spec()))
    data["avm"] = {
        "units": "m",
        "car_follow_core": bool(settings.car_follow_core),
        "car_length": float(settings.car_length),
        "car_width": float(settings.car_width),
        "car_height": float(settings.car_height),
        "car_clearance": float(settings.car_clearance),
        "ground": f"{settings.ground_w:g}x{settings.ground_d:g}",
        "block_lift": float(settings.block_lift),
        "active_camera": settings.active_camera,
        "cameras": camera_records(settings),
    }
    return data


def to_plane(settings) -> Dict:
    """The compact ``plane_scene`` Store object (HTML / App compatible)."""
    data: Dict = {"format": FORMAT_PLANE, "version": VERSION}
    data.update(avm_layout.to_store(settings.field_spec()))
    return data


def dumps(data: Dict) -> str:
    return json.dumps(data, indent=2) + "\n"


def loads(text: str) -> Dict:
    """Parse JSON, falling back to the bundled YAML subset reader."""
    text = (text or "").strip()
    if not text:
        raise ValueError("empty parameter text")
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return calibration_io.parse_yaml_subset(text)


def read(path: str) -> Dict:
    with open(path, "r", encoding="utf-8") as handle:
        return loads(handle.read())


def write(path: str, data: Dict) -> str:
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(dumps(data))
    return path


# ---------------------------------------------------------------------------
# applying
# ---------------------------------------------------------------------------
def apply(settings, data: Dict) -> str:
    """Write a validated parameter object into the scene settings.

    Returns the detected format.  Callers rebuild afterwards; nothing is touched
    before :func:`validate` has passed, so a bad file never half-applies.
    """
    validate(data)
    fmt = detect_format(data)
    _set_field(settings, avm_layout.from_store(data))
    if fmt == FORMAT_FULL:
        _apply_avm(settings, data.get("avm") or {})
    return fmt


def _set_field(settings, field: avm_layout.FieldSpec) -> None:
    settings.border_w = field.border_w
    settings.border_h = field.border_h
    settings.corner = field.corner
    settings.inner_w = field.inner_w
    settings.inner_h = field.inner_h
    settings.core_w = field.core_w
    settings.core_h = field.core_h


def _apply_avm(settings, avm: Dict) -> None:
    if "car_follow_core" in avm:
        settings.car_follow_core = bool(avm["car_follow_core"])
    for name in ("car_length", "car_width", "car_height", "car_clearance", "block_lift"):
        if name in avm:
            setattr(settings, name, float(avm[name]))
    ground = avm.get("ground")
    if ground is not None:
        size = avm_layout.size_from(ground)
        if size is not None:
            settings.ground_w, settings.ground_d = size
    active = avm.get("active_camera")
    if active in avm_layout.CAMERAS:
        settings.active_camera = active
    for record in avm.get("cameras") or []:
        _apply_camera(settings, record)


def _apply_camera(settings, record: Dict) -> None:
    entry = settings.camera(record["name"])
    if entry is None:
        return
    if "enable" in record:
        entry.enable = bool(record["enable"])
    if "location" in record:
        entry.location = [float(value) for value in record["location"]]
    if "rotation" in record:
        entry.rotation = [float(value) for value in record["rotation"]]
    camera = bpy.data.objects.get(
        f"{builder.CAMERA_PREFIX}{builder.CAMERA_SUFFIX.get(entry.name, '')}")
    if camera is None:
        return
    cam_settings = camera.data.opencv_cam
    intrinsics = cam_settings.intrinsics
    if "K" in record:
        fx, fy, cx, cy = (float(value) for value in record["K"])
        intrinsics.fx, intrinsics.fy = fx, fy
        intrinsics.auto_center = False
        intrinsics.cx, intrinsics.cy = cx, cy
    if "D" in record:
        coefficients = [float(value) for value in record["D"]] + [0.0] * 4
        distortion = cam_settings.distortion
        distortion.model = "fisheye"
        distortion.enabled = True
        (distortion.k1, distortion.k2, distortion.k3,
         distortion.k4) = coefficients[:4]


def _read_intrinsics(camera) -> tuple:
    if camera is None:
        return [0.0] * 4, [0.0] * 4
    cam_settings = camera.data.opencv_cam
    intrinsics = cam_settings.intrinsics
    distortion = cam_settings.distortion
    return (
        [float(intrinsics.fx), float(intrinsics.fy),
         float(intrinsics.cx), float(intrinsics.cy)],
        [float(distortion.k1), float(distortion.k2),
         float(distortion.k3), float(distortion.k4)],
    )


# ---------------------------------------------------------------------------
# four-camera render export
# ---------------------------------------------------------------------------
def render_cameras(context, settings, directory: str, samples: int = 64) -> List[str]:
    """Render every enabled camera to ``<directory>/<name>.png`` at its own size.

    Each camera is rendered with its own ``K`` / ``D`` and output resolution; the
    scene render settings are saved and restored, so the export never changes the
    user's setup.
    """
    scene = context.scene
    os.makedirs(directory, exist_ok=True)
    render = scene.render
    saved = {
        "resolution_x": render.resolution_x,
        "resolution_y": render.resolution_y,
        "resolution_percentage": render.resolution_percentage,
        "filepath": render.filepath,
        "file_format": render.image_settings.file_format,
        "samples": scene.cycles.samples,
        "camera": scene.camera,
    }
    written: List[str] = []
    try:
        if scene.render.engine != "CYCLES":
            scene.render.engine = "CYCLES"
        scene.cycles.samples = int(samples)
        for name in avm_layout.CAMERAS:
            entry = settings.camera(name)
            if entry is None or not entry.enable:
                continue
            camera = bpy.data.objects.get(
                f"{builder.CAMERA_PREFIX}{builder.CAMERA_SUFFIX[name]}")
            if camera is None:
                continue
            cam_settings = camera.data.opencv_cam
            width, height = apply_mod.output_resolution(cam_settings, scene)
            render.resolution_x, render.resolution_y = width, height
            render.resolution_percentage = 100
            render.image_settings.file_format = "PNG"
            scene.camera = camera
            ok, messages = apply_mod.apply_settings(camera.data, cam_settings, scene)
            if not ok:
                raise RuntimeError(f"{name}: " + "; ".join(messages))
            path = os.path.join(directory, f"{name}.png")
            render.filepath = path
            bpy.ops.render.render(write_still=True)
            written.append(path)
    finally:
        render.resolution_x = saved["resolution_x"]
        render.resolution_y = saved["resolution_y"]
        render.resolution_percentage = saved["resolution_percentage"]
        render.filepath = saved["filepath"]
        render.image_settings.file_format = saved["file_format"]
        scene.cycles.samples = saved["samples"]
        scene.camera = saved["camera"]
        if saved["camera"] is not None and saved["camera"].type == "CAMERA":
            try:
                apply_mod.apply_settings(saved["camera"].data,
                                         saved["camera"].data.opencv_cam, scene)
            except Exception:
                pass
    return written
