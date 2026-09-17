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

#: test target: azimuth around the principal point and how far out it sits,
#: expressed as a fraction of the model's usable image radius
TARGET_AZIMUTH_DEG = 35.0
TARGET_RADIUS_FRACTION = 0.85
#: largest undistorted slope (tan of the angle off axis) the target may reach.
#: The Brown-Conrady inversion has several roots beyond that, where the shader and
#: the reference model may pick different ones; 1.0 means 45 degrees.
MAX_TARGET_SLOPE = 1.0
#: distance of the test sphere in metres
DISTANCE = 3.0
#: sphere radius relative to the distance (about 4 px at fx=800)
RADIUS_RATIO = 0.02
#: luminance threshold that separates the sphere from the black world
LUMA_THRESHOLD = 0.2


class SelfTestError(RuntimeError):
    """Raised when the self test cannot be run at all."""


def test_resolution(settings, long_side: int = 256) -> tuple:
    """Render size for the self test: the calibration aspect ratio is preserved.

    Testing a 1280x960 camera with a square render would stretch (or crop) it, so
    the short side follows the calibrated aspect ratio.
    """
    intrinsics = settings.intrinsics
    width, height = int(intrinsics.image_width), int(intrinsics.image_height)
    if width <= 0 or height <= 0:
        return int(long_side), int(long_side)
    if width >= height:
        return int(long_side), max(8, int(round(long_side * height / width)))
    return max(8, int(round(long_side * width / height))), int(long_side)


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


def target_pixel(intrinsics: camera_model.Intrinsics,
                 distortion: camera_model.Distortion,
                 sphere_radius_px: float = 0.0) -> tuple:
    """A pixel that keeps the test target inside the image *and* inside the
    model's valid domain.

    The usable radius is where theta reaches ~90 degrees for fisheye, otherwise
    the image corner; it is additionally capped so the measured blob (plus a
    margin) cannot be clipped by the image border, which would bias the centroid.
    """
    res = intrinsics.resolved()
    azimuth = math.radians(TARGET_AZIMUTH_DEG)
    direction = (math.cos(azimuth), math.sin(azimuth))

    radius = 0.5 * math.hypot(res.width, res.height)
    if distortion.enabled and distortion.model == camera_model.MODEL_FISHEYE:
        radius = min(radius, camera_model.fisheye_valid_radius_px(res, distortion))

    margin = sphere_radius_px + 2.0
    limits = (min(res.cx, res.width - res.cx) - margin,
              min(res.cy, res.height - res.cy) - margin)
    for limit, component in zip(limits, direction):
        if abs(component) > 1e-6:
            radius = min(radius, max(4.0, limit) / abs(component))

    radius *= TARGET_RADIUS_FRACTION
    # the polynomial inversion becomes multi-valued far off axis, so cap the
    # slope there; the fisheye Newton inversion is monotonic and needs no cap
    polynomial = not (distortion.enabled and distortion.model == camera_model.MODEL_FISHEYE)
    for _ in range(12 if polynomial else 0):
        u = res.cx + radius * direction[0]
        v = res.cy + radius * direction[1]
        ray = camera_model.ray_from_pixel(u, v, intrinsics, distortion)
        slope = math.hypot(ray[0], ray[1]) / abs(ray[2]) if ray[2] else float("inf")
        if slope <= MAX_TARGET_SLOPE or radius <= 4.0:
            break
        radius = max(4.0, radius * MAX_TARGET_SLOPE / slope)
    return res.cx + radius * direction[0], res.cy + radius * direction[1]


def target_position(
    intrinsics: camera_model.Intrinsics,
    distortion: camera_model.Distortion,
    distance: float = DISTANCE,
    pixel: Optional[tuple] = None,
):
    """Camera-local position of the test target for the given intrinsics.

    The target pixel is chosen first (inside the frame and inside the model's
    valid domain), the ray for that pixel is computed with
    :mod:`core.camera_model` and the target is placed along it.  The *predicted*
    image position is then obtained by forward projecting the target position
    again, so a wrong distortion inversion shows up as an offset.
    """
    if pixel is None:
        u, v = target_pixel(intrinsics, distortion,
                            sphere_radius_px=RADIUS_RATIO * 0.5 * (intrinsics.fx + intrinsics.fy))
    else:
        u, v = float(pixel[0]), float(pixel[1])
    x, y, z = camera_model.ray_from_pixel(u, v, intrinsics, distortion)
    length = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / length, y / length, z / length
    # OpenCV camera frame (x right, y down, z forward) -> Blender camera local
    # (x right, y up, z backward)
    location = (x * distance, -y * distance, -z * distance)
    predicted = camera_model.project_point(x * distance, y * distance, z * distance,
                                           intrinsics, distortion)
    return location, (predicted[0] - 0.5, predicted[1] - 0.5)


def _mask_object(location) -> bpy.types.Object:
    """Build the emissive test target at the camera-local target position.

    A flat quad facing the camera is used instead of a sphere: a sphere seen far
    off axis projects to an asymmetric blob whose centroid is biased away from
    the ray (several pixels at 50+ degrees), while a small patch perpendicular to
    the ray projects symmetrically about it.
    """
    bpy.ops.mesh.primitive_plane_add(size=2.0 * RADIUS_RATIO * DISTANCE, location=location)
    sphere = bpy.context.active_object
    sphere.name = "__opencv_cam_selftest_target"
    from mathutils import Matrix, Vector
    to_camera = -Vector(location)
    if to_camera.length > 0.0:
        rotation = to_camera.to_track_quat("Z", "Y").to_matrix().to_4x4()
        sphere.matrix_world = Matrix.Translation(Vector(location)) @ rotation

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
    target_pixel_override: Optional[tuple] = None,
) -> Dict:
    """Run the self test, returning a result dict (``passed`` included).

    ``target_pixel_override`` tests one specific pixel instead of the automatic
    target (used to probe large off-axis angles).
    """
    scene = scene or bpy.context.scene
    if scene.render.engine != "CYCLES":
        raise SelfTestError("switch the render engine to Cycles first (custom cameras are Cycles only)")

    width, height = test_resolution(settings, resolution)
    intrinsics = apply_mod.effective_intrinsics(settings, width, height)
    distortion = settings.core_distortion()
    location, predicted_px = target_position(intrinsics, distortion,
                                             pixel=target_pixel_override)

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
            "resolution": (width, height),
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