"""Create fully configured OpenCV cameras.

Blender does not let add-ons extend ``Camera.type`` (the enum lives in RNA), so
"pick the OpenCV camera when adding a camera" is implemented the Blender way: an
operator in :menuselection:`Add ▸ Camera` that creates a camera already set to
*Lens Type = Custom* with the matching bundled shader and parameters.

Models:

* ``fisheye`` - ``opencv_fisheye.osl`` (Kannala-Brandt, k1..k4)
* ``brown_conrady`` / ``rational`` - ``opencv_camera.osl``
* ``pinhole`` - ``opencv_camera.osl`` with distortion disabled (ideal pinhole
  with OpenCV intrinsics)
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import bpy

from ..core import camera_model, presets
from . import apply as apply_mod

#: model -> (label, distortion model, distortion enabled)
MODELS = {
    "fisheye": ("Fisheye (OpenCV equidistant)", camera_model.MODEL_FISHEYE, True),
    "brown_conrady": ("Brown-Conrady (radtan)", camera_model.MODEL_BROWN_CONRADY, True),
    "rational": ("Rational polynomial", camera_model.MODEL_RATIONAL, True),
    "pinhole": ("Pinhole (no distortion)", camera_model.MODEL_BROWN_CONRADY, False),
}


def default_name(model: str) -> str:
    return {
        "fisheye": "FisheyeCamera",
        "brown_conrady": "OpenCVCamera",
        "rational": "RationalCamera",
        "pinhole": "PinholeCamera",
    }.get(model, "OpenCVCamera")


def configure_from_record(camera: bpy.types.Object, record: Dict,
                          scene: bpy.types.Scene) -> Tuple[bool, List[str]]:
    """Write a calibration record's K / D / output onto a camera and compile it.

    ``record`` is an ``avm_layout.cameras_from_preset`` entry (``K`` = fx/fy/cx/cy,
    ``D`` = the four fisheye coefficients, ``output`` = the calibration size).  The
    AVM Scene and the Drive Scene configure their cameras through this one
    function, so their intrinsics can never drift apart.
    """
    settings = camera.data.opencv_cam
    intrinsics = settings.intrinsics
    fx, fy, cx, cy = record.get("K", (0.0, 0.0, 0.0, 0.0))
    intrinsics.fx, intrinsics.fy = float(fx), float(fy)
    intrinsics.auto_center = False
    intrinsics.cx, intrinsics.cy = float(cx), float(cy)
    width, height = record.get("output", (1280, 960))
    intrinsics.image_width, intrinsics.image_height = int(width), int(height)
    intrinsics.scale_to_render = True

    distortion = settings.distortion
    distortion.model = camera_model.MODEL_FISHEYE
    distortion.enabled = True
    coefficients = list(record.get("D", (0.0, 0.0, 0.0, 0.0))) + [0.0] * 4
    (distortion.k1, distortion.k2, distortion.k3,
     distortion.k4) = (float(value) for value in coefficients[:4])
    return apply_mod.apply_settings(camera.data, settings, scene)


def resolve_camera(context):
    """The camera to work on, or ``None``.

    Prefers the active object when it is a camera, otherwise falls back to
    ``context.camera`` (the camera shown in Object Data Properties) so the panel
    operators keep working while some other object is selected.
    """
    obj = getattr(context, "active_object", None)
    if obj is not None and obj.type == "CAMERA":
        return obj
    for candidate in (getattr(context, "camera", None),
                      getattr(getattr(context, "scene", None), "camera", None)):
        if candidate is not None and candidate.type == "CAMERA":
            return candidate
    return None


def add_camera(
    scene: bpy.types.Scene,
    model: str = "fisheye",
    preset: Optional[str] = None,
    name: str = "",
    use_rig: bool = False,
    location: Optional[Tuple[float, float, float]] = None,
) -> Tuple[bpy.types.Object, List[str]]:
    """Create and configure a camera; returns ``(object, messages)``.

    ``preset`` may be a bundled preset identifier, ``presets.DEFAULTS`` / ``None``
    (the add-on's default camera values) or ``presets.CURRENT`` (copy the settings
    of the active camera).
    """
    messages: List[str] = []
    if model not in MODELS:
        raise ValueError(f"unknown camera model {model!r}")
    _, distortion_model, distortion_enabled = MODELS[model]

    cam_data = bpy.data.cameras.new(name or default_name(model))
    camera = bpy.data.objects.new(cam_data.name, cam_data)
    scene.collection.objects.link(camera)
    if location is not None:
        camera.location = location

    settings = cam_data.opencv_cam
    settings.distortion.model = distortion_model
    settings.distortion.enabled = distortion_enabled

    source = None
    active = getattr(scene, "camera", None)
    if preset in (None, presets.DEFAULTS):
        messages.append("add-on default camera values")
    elif preset == presets.CURRENT and active is not None and active.type == "CAMERA":
        source = active.data.opencv_cam
    elif preset == presets.CURRENT:
        messages.append("no camera to copy from: using the add-on defaults")
    else:
        calibration = presets.load_preset(preset)
        settings.set_from_core(calibration.intrinsics, calibration.distortion)
        settings.distortion.model = distortion_model  # the operator decides the model
        settings.distortion.enabled = distortion_enabled
        messages.append(
            f"preset {preset}: {calibration.width}x{calibration.height}, "
            f"fx={calibration.intrinsics.fx:.2f}"
        )
    if source is not None:
        settings.set_from_core(source.core_intrinsics(), source.core_distortion())
        settings.distortion.model = distortion_model
        settings.distortion.enabled = distortion_enabled
        messages.append(f"copied the settings of {active.name}")

    ok, apply_messages = apply_mod.apply_settings(cam_data, settings, scene)
    messages.extend(apply_messages)
    if not ok:
        messages.append("the camera was created but its shader/parameters are not ready")

    if use_rig:
        rig = bpy.data.objects.new(f"{camera.name}_rig", None)
        rig.empty_display_type = "PLAIN_AXES"
        rig.empty_display_size = 0.5
        rig.location = camera.location
        scene.collection.objects.link(rig)
        camera.parent = rig
        camera.matrix_parent_inverse = rig.matrix_world.inverted()
        created_rig = True
    else:
        created_rig = False

    if scene.camera is None:
        scene.camera = camera
    for obj in scene.objects:
        obj.select_set(False)
    camera.select_set(True)
    bpy.context.view_layer.objects.active = camera
    messages.append(
        f"{camera.name}: {MODELS[model][0]}"
        + (", parented to a rig empty" if created_rig else "")
    )
    return camera, messages