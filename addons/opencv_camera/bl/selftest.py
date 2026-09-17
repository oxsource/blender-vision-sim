"""Render based self test.

Renders a small emissive sphere with the camera under test and compares the
measured image position against :mod:`core.camera_model`.  It validates the whole
chain (shader compile -> raster mapping -> distortion inversion), which is the
only way to catch a silently stale shader.

The test reuses the *current* scene (so it also works in background mode),
temporarily hiding the other objects and restoring every changed setting.
"""

from __future__ import annotations

import math
import os
import tempfile
from typing import Dict, List, Optional

import bpy
import numpy as np

from ..core import camera_model
from . import apply as apply_mod

#: test target as a fraction of the image, top-left origin, y down
TARGET_UV = (0.85, 0.70)
#: distance of the test sphere in metres
DISTANCE = 3.0
#: sphere radius relative to the distance (about 4 px at fx=800)
RADIUS_RATIO = 0.02
#: luminance threshold that separates the sphere from the black world
LUMA_THRESHOLD = 0.2


class SelfTestError(RuntimeError):
    """Raised when the self test cannot be run at all."""


def _centroid(image_path: str) -> tuple:
    image = bpy.data.images.load(image_path)
    try:
        width, height = image.size
        pixels = np.array(image.pixels[:], dtype=np.float32).reshape(height, width, 4)
    finally:
        bpy.data.images.remove(image)
    # weight by alpha so a non-black world (transparent film) cannot bias the
    # centroid, then drop everything below the emissive target
    luminance = pixels[..., :3].mean(axis=2) * pixels[..., 3]
    luminance[luminance < LUMA_THRESHOLD] = 0.0
    total = float(luminance.sum())
    if total <= 0.0:
        raise SelfTestError("the test render is black: the camera produced no usable rays")
    ys, xs = np.mgrid[0:height, 0:width]
    column = float((xs * luminance).sum() / total)
    row_topdown = float(height - 1 - (ys * luminance).sum() / total)
    return column, row_topdown


def target_position(
    intrinsics: camera_model.Intrinsics,
    distortion: camera_model.Distortion,
    distance: float = DISTANCE,
):
    """Camera-local position of the test target for the given intrinsics.

    The target pixel is chosen first (always inside the frame), the ray for that
    pixel is computed with :mod:`core.camera_model` and the target is placed along
    it.  The *predicted* image position is then obtained by forward projecting the
    target position again, so a wrong distortion inversion shows up as an offset.
    """
    u = TARGET_UV[0] * intrinsics.width
    v = TARGET_UV[1] * intrinsics.height
    x, y, _ = camera_model.ray_from_pixel(u, v, intrinsics, distortion)
    length = math.sqrt(x * x + y * y + 1.0)
    x, y, z = x / length, y / length, 1.0 / length
    # OpenCV camera frame (x right, y down, z forward) -> Blender camera local
    # (x right, y up, z backward)
    location = (x * distance, -y * distance, -z * distance)
    predicted = camera_model.project_point(x * distance, y * distance, z * distance,
                                           intrinsics, distortion)
    return location, (predicted[0] - 0.5, predicted[1] - 0.5)


def _mask_object(location) -> bpy.types.Object:
    """Build the emissive test sphere at the camera-local target position."""
    bpy.ops.mesh.primitive_uv_sphere_add(radius=RADIUS_RATIO * DISTANCE, location=location)
    sphere = bpy.context.active_object
    sphere.name = "__opencv_cam_selftest_target"

    material = bpy.data.materials.new("__opencv_cam_selftest_emit")
    material.use_nodes = True
    nodes = material.node_tree.nodes
    output = next(n for n in nodes if n.type == "OUTPUT_MATERIAL")
    emission = nodes.new("ShaderNodeEmission")
    emission.inputs["Color"].default_value = (1.0, 1.0, 1.0, 1.0)
    emission.inputs["Strength"].default_value = 1.0
    for link in list(output.inputs["Surface"].links):
        material.node_tree.links.remove(link)
    material.node_tree.links.new(emission.outputs["Emission"], output.inputs["Surface"])
    sphere.data.materials.append(material)
    return sphere


def run(
    cam_data,
    settings,
    scene: Optional[bpy.types.Scene] = None,
    resolution: int = 256,
    samples: int = 4,
    tolerance_px: float = 1.0,
) -> Dict:
    """Run the self test, returning a result dict (``passed`` included)."""
    scene = scene or bpy.context.scene
    if scene.render.engine != "CYCLES":
        raise SelfTestError("switch the render engine to Cycles first (custom cameras are Cycles only)")

    width = height = int(resolution)
    intrinsics = apply_mod.effective_intrinsics(settings, width, height)
    distortion = settings.core_distortion()
    location, predicted_px = target_position(intrinsics, distortion)

    view_layer = bpy.context.view_layer
    render = scene.render
    saved = {
        "camera": scene.camera,
        "active_object": view_layer.objects.active,
        "resolution_x": render.resolution_x,
        "resolution_y": render.resolution_y,
        "resolution_percentage": render.resolution_percentage,
        "filepath": render.filepath,
        "file_format": render.image_settings.file_format,
        "film_transparent": render.film_transparent,
        "view_transform": scene.view_settings.view_transform,
        "samples": scene.cycles.samples,
        "denoising": scene.cycles.use_denoising,
        "hide_render": [(o, o.hide_render) for o in scene.objects],
    }

    tmp_cam_data = None
    sphere = None
    temp_path = os.path.join(tempfile.mkdtemp(prefix="opencv_cam_selftest_"), "selftest.png")
    result: Dict

    try:
        for obj in scene.objects:
            obj.hide_render = True

        tmp_cam_data = cam_data.copy()
        params = getattr(tmp_cam_data, "cycles_custom", None)
        if params is None:
            raise SelfTestError("Cycles is not enabled: no custom camera parameters")
        values = apply_mod.custom_camera_values(settings, intrinsics, distortion)
        missing = [k for k in values if k not in params]
        if missing:
            raise SelfTestError("shader parameters missing: " + ", ".join(missing))
        for key, value in values.items():
            params[key] = value

        cam_obj = bpy.data.objects.new("__opencv_cam_selftest_cam", tmp_cam_data)
        scene.collection.objects.link(cam_obj)
        cam_obj.location = (0.0, 0.0, 0.0)
        cam_obj.rotation_euler = (0.0, 0.0, 0.0)
        scene.camera = cam_obj

        sphere = _mask_object(location)
        sphere.hide_render = False

        render.resolution_x = width
        render.resolution_y = height
        render.resolution_percentage = 100
        render.image_settings.file_format = "PNG"
        render.film_transparent = True
        render.filepath = temp_path
        scene.view_settings.view_transform = "Standard"
        scene.cycles.samples = int(samples)
        scene.cycles.use_denoising = False

        bpy.ops.render.render(write_still=True)
        measured = _centroid(temp_path)
        error = (measured[0] - predicted_px[0], measured[1] - predicted_px[1])
        distance = float(np.hypot(*error))
        result = {
            "passed": distance <= tolerance_px,
            "predicted": predicted_px,
            "measured": measured,
            "error": error,
            "error_px": distance,
            "tolerance_px": float(tolerance_px),
            "resolution": width,
            "samples": int(samples),
            "intrinsics": intrinsics,
            "distortion": distortion,
        }
    finally:
        if sphere is not None:
            mesh = sphere.data
            materials: List[bpy.types.Material] = [s.material for s in sphere.material_slots]
            bpy.data.objects.remove(sphere, do_unlink=True)
            bpy.data.meshes.remove(mesh)
            for material in materials:
                if material is not None and material.users == 0:
                    bpy.data.materials.remove(material)
        if tmp_cam_data is not None and tmp_cam_data.users == 0:
            bpy.data.cameras.remove(tmp_cam_data)
        for key in ("resolution_x", "resolution_y", "resolution_percentage", "filepath",
                    "film_transparent"):
            setattr(render, key, saved[key])
        render.image_settings.file_format = saved["file_format"]
        scene.view_settings.view_transform = saved["view_transform"]
        scene.cycles.samples = saved["samples"]
        scene.cycles.use_denoising = saved["denoising"]
        for obj, hidden in saved["hide_render"]:
            if obj.name in bpy.data.objects:
                obj.hide_render = hidden
        try:
            scene.camera = saved["camera"]
        except Exception:
            pass
        active = saved.get("active_object")
        if active is not None and active.name in bpy.data.objects:
            view_layer.objects.active = active
        if os.path.exists(temp_path):
            try:
                os.remove(temp_path)
            except OSError:
                pass
    return result