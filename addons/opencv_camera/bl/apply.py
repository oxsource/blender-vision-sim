"""Translate settings into Cycles custom-camera parameters, pose and lens sync."""

from __future__ import annotations

from typing import Dict, List, Tuple

import bpy
from mathutils import Matrix

from ..core import camera_model, transform
from . import shader

#: OSL shader parameters per model family, written to ``camera.cycles_custom``.
#: Keep in sync with ``shaders/opencv_camera.osl`` and ``shaders/opencv_fisheye.osl``.
SHADER_PARAMS: Dict[str, Tuple[str, ...]] = {
    "poly": (
        "fx", "fy", "cx", "cy",
        "enable_distortion",
        "k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2",
        "undistort_iterations", "allow_off_sensor",
    ),
    "fisheye": (
        "fx", "fy", "cx", "cy",
        "enable_distortion",
        "k1", "k2", "k3", "k4",
        "undistort_iterations", "allow_off_sensor",
    ),
}


def param_group(model: str) -> str:
    """Which parameter family a distortion model uses."""
    return "fisheye" if model == camera_model.MODEL_FISHEYE else "poly"


def shader_params(model: str) -> Tuple[str, ...]:
    return SHADER_PARAMS[param_group(model)]


def output_resolution(settings, scene=None) -> Tuple[int, int]:
    """The image size the render should produce, per ``settings.output``."""
    output = settings.output
    if output.mode == "custom":
        return max(8, int(output.width)), max(8, int(output.height))
    if output.mode == "scene" or scene is None:
        scene = scene or bpy.context.scene
        return int(scene.render.resolution_x), int(scene.render.resolution_y)
    intrinsics = settings.intrinsics
    return max(8, int(intrinsics.image_width)), max(8, int(intrinsics.image_height))


def apply_render_resolution(scene, settings) -> Tuple[bool, List[str]]:
    """Write the output size into the scene render settings.

    Returns ``(changed, messages)``.  Does nothing when the output follows the
    scene (``mode == 'scene'``) or when driving the scene is switched off.
    """
    output = settings.output
    if output.mode == "scene" or not output.lock_scene_resolution:
        return False, []
    width, height = output_resolution(settings, scene)
    changed = (scene.render.resolution_x, scene.render.resolution_y,
               scene.render.resolution_percentage) != (width, height, 100)
    if changed:
        scene.render.resolution_x = width
        scene.render.resolution_y = height
        scene.render.resolution_percentage = 100
    return changed, []


def effective_intrinsics(settings, width: int = 0, height: int = 0) -> camera_model.Intrinsics:
    """Intrinsics for a given render resolution.

    The stored intrinsics belong to ``settings.intrinsics.image_width/height``:

    * same aspect ratio -> rescaled per axis, so the field of view is preserved
      (``scale_to_render`` off keeps the numbers verbatim);
    * different aspect ratio -> the render cannot match the calibration geometry,
      so the *pixel pitch* is kept (``fx``/``fy`` unchanged) and the principal
      point offset from the centre is preserved: the render is a centre crop of
      the calibrated image at the original pitch.

    The returned intrinsics always have a resolved (explicit) principal point.
    """
    intr = settings.intrinsics
    width = int(width or settings.intrinsics.image_width)
    height = int(height or settings.intrinsics.image_height)
    stored = camera_model.Intrinsics(
        fx=intr.fx, fy=intr.fy,
        cx=-1.0 if intr.auto_center else intr.cx,
        cy=-1.0 if intr.auto_center else intr.cy,
        width=int(intr.image_width), height=int(intr.image_height),
    )
    resolved = stored.resolved()
    same_size = (stored.width, stored.height) == (width, height)
    same_aspect = (
        stored.width > 0 and stored.height > 0 and height > 0
        and abs(stored.width / stored.height - width / height) < 1e-3
    )
    if same_size or not intr.scale_to_render:
        scaled = camera_model.Intrinsics(
            fx=resolved.fx, fy=resolved.fy, cx=resolved.cx, cy=resolved.cy,
            width=width, height=height,
        )
    elif same_aspect:
        scaled = stored.scaled(width, height)
    else:  # crop at the original pixel pitch
        scaled = camera_model.Intrinsics(
            fx=resolved.fx,
            fy=resolved.fy,
            cx=0.5 * width + (resolved.cx - 0.5 * stored.width),
            cy=0.5 * height + (resolved.cy - 0.5 * stored.height),
            width=width,
            height=height,
        )
    return scaled.resolved()


def custom_camera_values(settings, intr: camera_model.Intrinsics,
                         dist: camera_model.Distortion) -> Dict[str, float]:
    """Values for ``camera.cycles_custom[...]`` for the given intrinsics."""
    values = {
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "cx": float(intr.cx),
        "cy": float(intr.cy),
        "enable_distortion": 1 if dist.enabled else 0,
        "k1": float(dist.k1),
        "k2": float(dist.k2),
        "k3": float(dist.k3),
        "k4": float(dist.k4),
        "undistort_iterations": int(settings.distortion.iterations),
        "allow_off_sensor": 1,
    }
    if param_group(dist.model) == "poly":
        values.update({
            "k5": float(dist.k5),
            "k6": float(dist.k6),
            "p1": float(dist.p1),
            "p2": float(dist.p2),
        })
    return values


def apply_values(cam_data, settings, scene=None,
                 resolution: Tuple[int, int] = (0, 0)) -> Tuple[bool, List[str]]:
    """Push the current settings to ``camera.cycles_custom`` (no recompile).

    Used by the live ("auto apply") path: the shader is assumed to be attached
    and compiled already.  Also keeps the scene render size in sync when the
    output size drives it.  Returns ``(ok, messages)``.
    """
    messages: List[str] = []
    scene = scene or bpy.context.scene
    if scene is not None:
        apply_render_resolution(scene, settings)
    params = shader.cycles_custom_params(cam_data)
    if params is None:
        messages.append("Cycles is not enabled: camera.cycles_custom is unavailable")
        return False, messages

    width, height = resolution
    if not width or not height:
        width, height = scene.render.resolution_x, scene.render.resolution_y
    intr = effective_intrinsics(settings, width, height)
    dist = settings.core_distortion()
    try:
        values = custom_camera_values(settings, intr, dist)
    except NotImplementedError as exc:
        messages.append(str(exc))
        return False, messages

    missing = []
    for name in shader_params(dist.model):
        if name in params:
            params[name] = values[name]
        else:
            missing.append(name)
    if missing:
        messages.append(
            "shader parameters missing on this camera (stale bytecode?): "
            + ", ".join(missing)
        )
        return False, messages
    return True, messages


def apply_settings(cam_data, settings, scene=None,
                   resolution: Tuple[int, int] = (0, 0)) -> Tuple[bool, List[str]]:
    """Attach the matching shader, compile it and push the parameters to Cycles.

    Returns ``(ok, messages)``.  ``ok`` is ``False`` when the shader could not be
    compiled or the Cycles parameter group is missing - in that case the camera
    would render with a stale shader, which is worse than a hard failure.
    """
    messages: List[str] = []
    scene = scene or bpy.context.scene
    apply_render_resolution(scene, settings)
    shader.attach(cam_data, settings)
    ok, compile_messages = shader.ensure_compiled(cam_data)
    messages.extend(compile_messages)
    if not ok:
        return False, messages
    ok, value_messages = apply_values(cam_data, settings, scene, resolution)
    messages.extend(value_messages)
    return ok, messages


def apply_opencv_pose(obj, settings) -> None:
    """Set ``obj.matrix_world`` from an OpenCV world-to-camera pose."""
    pose = settings.pose
    world = tuple(pose.world_matrix) if pose.use_world_transform else None
    m = transform.object_matrix_from_opencv(pose.rotation, pose.translation, world)
    obj.matrix_world = Matrix(
        (m[0:4], m[4:8], m[8:12], m[12:16])
    )


def read_opencv_pose(obj, settings) -> Tuple[Tuple[float, ...], Tuple[float, float, float]]:
    """Read ``obj.matrix_world`` back as an OpenCV world-to-camera pose."""
    matrix = obj.matrix_world
    if settings.pose.use_world_transform:
        matrix = Matrix(
            (tuple(settings.pose.world_matrix[0:4]),
             tuple(settings.pose.world_matrix[4:8]),
             tuple(settings.pose.world_matrix[8:12]),
             tuple(settings.pose.world_matrix[12:16]))
        ).inverted() @ matrix
    rows = [tuple(matrix[r]) for r in range(4)]
    R_wc = (rows[0][0], rows[0][1], rows[0][2],
            rows[1][0], rows[1][1], rows[1][2],
            rows[2][0], rows[2][1], rows[2][2])
    centre = (rows[0][3], rows[1][3], rows[2][3])
    R_b = transform.mat3_transpose(R_wc)
    t_b = transform.mat3_vec(R_b, tuple(-c for c in centre))
    return transform.blender_to_world_to_camera(R_b, t_b)


def sync_intrinsics_from_lens(cam_data, settings, scene=None) -> List[str]:
    """Fill intrinsics from the camera's Blender lens/sensor/shift values."""
    scene = scene or bpy.context.scene
    width = scene.render.resolution_x
    height = scene.render.resolution_y
    fit = cam_data.sensor_fit
    if fit == "AUTO":
        fit = "HORIZONTAL" if width >= height else "VERTICAL"
    if fit == "HORIZONTAL":
        focal_px = transform.fx_from_lens_mm(cam_data.lens, cam_data.sensor_width, width)
        effective = cam_data.sensor_width
    else:
        focal_px = transform.fx_from_lens_mm(cam_data.lens, cam_data.sensor_height, height)
        effective = cam_data.sensor_height
    cx, cy = transform.principal_point_from_shift(
        cam_data.shift_x, cam_data.shift_y, width, height
    )
    intr = camera_model.Intrinsics(
        fx=focal_px, fy=focal_px, cx=cx, cy=cy, width=width, height=height
    )
    settings.set_from_core(intr, settings.core_distortion())
    return [
        f"lens {cam_data.lens:g}mm, sensor {effective:g}mm, fit {fit}, "
        f"resolution {width}x{height} -> fx=fy={focal_px:.3f}px, cx={cx:.3f}, cy={cy:.3f}"
    ]


def resolution_notes(settings, scene) -> List[str]:
    """Human readable notes about the stored vs. current resolution."""
    intr = settings.intrinsics
    width, height = scene.render.resolution_x, scene.render.resolution_y
    output = settings.output
    notes: List[str] = []
    if output.mode != "scene" and output.lock_scene_resolution:
        out_w, out_h = output_resolution(settings, scene)
        notes.append(f"output size {out_w}x{out_h} drives the scene resolution")
    if (intr.image_width, intr.image_height) != (width, height):
        if intr.scale_to_render and intr.image_height and height:
            same_aspect = abs(intr.image_width / intr.image_height - width / height) < 1e-3
        else:
            same_aspect = True
        if same_aspect:
            notes.append(
                f"calibrated at {intr.image_width}x{intr.image_height}, "
                f"rendering at {width}x{height}"
                + (" (intrinsics are rescaled)" if intr.scale_to_render else " (intrinsics used as-is)")
            )
        else:
            notes.append(
                f"aspect mismatch: {intr.image_width}x{intr.image_height} calibration "
                f"vs {width}x{height} render -> centre crop at the original pixel pitch "
                "(use the calibration aspect ratio to compare with real images)"
            )
    if abs(scene.render.pixel_aspect_x - scene.render.pixel_aspect_y) > 1e-6:
        notes.append("non-square pixel aspect breaks the OpenCV pixel model")
    if scene.render.engine != "CYCLES":
        notes.append("custom cameras only render in Cycles")
    distortion = settings.distortion
    if distortion.model == camera_model.MODEL_FISHEYE:
        notes.append("fisheye model: fx/fy are pinhole focal lengths, k1..k4 are theta terms")
    elif distortion.model == camera_model.MODEL_BROWN_CONRADY and (
        distortion.k4 or distortion.k5 or distortion.k6
    ):
        notes.append("k4/k5/k6 are non-zero: the rational model is used")
    return notes