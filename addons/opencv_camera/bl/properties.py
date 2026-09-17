"""Add-on properties.

The add-on owns its own :class:`OpenCVCameraSettings` (stored on the camera data
-block) and uses it as the single source of truth.  The values Cycles exposes on
``camera.cycles_custom`` are only ever *written* by :mod:`bl.apply`.

Defaults come from a real surround-view camera (AVM minibus front camera,
1280x960, OpenCV fisheye coefficients) so the add-on is usable for a visual
check right after installation; replace them with your own calibration.
"""

from __future__ import annotations

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)

from ..core import camera_model


def _root_settings(group):
    """The :class:`OpenCVCameraSettings` that owns a nested property group."""
    cam_data = group.id_data
    if not isinstance(cam_data, bpy.types.Camera):
        return None
    return getattr(cam_data, "opencv_cam", None)


def _apply_live(group, context=None, full: bool = False):
    """Push changes to Cycles while the user edits (see ``auto_apply``).

    ``full`` also (re)attaches and compiles the shader - needed when the
    distortion model changes, because that swaps the shader.
    """
    settings = _root_settings(group)
    cam_data = group.id_data
    if settings is None or not isinstance(cam_data, bpy.types.Camera):
        return
    if not settings.auto_apply or cam_data.type != "CUSTOM":
        return
    from . import apply as apply_mod
    scene = context.scene if context is not None else bpy.context.scene
    try:
        fn = apply_mod.apply_settings if full else apply_mod.apply_values
        ok, messages = fn(cam_data, settings, scene)
        settings.status = "live" if ok else "live apply failed: " + "; ".join(messages)[:120]
        if ok and settings.preview.on_change:
            from . import preview
            preview.schedule_preview(cam_data, settings, scene)
    except Exception as exc:  # never let an update callback raise into the UI
        settings.status = f"live apply error: {type(exc).__name__}: {exc}"


def _update_values(self, context):
    _apply_live(self, context)


def _update_model(self, context):
    _apply_live(self, context, full=True)


def _update_pose(self, context):
    """Live extrinsics: keep the camera object in sync with R/t."""
    settings = _root_settings(self)
    obj = getattr(self, "id_data", None)
    if settings is None or obj is None:
        return
    if not settings.auto_apply:
        return
    owner = next((o for o in bpy.context.scene.objects if o.data is obj), None)
    if owner is None:
        return
    from . import apply as apply_mod
    try:
        apply_mod.apply_opencv_pose(owner, settings)
    except Exception as exc:
        settings.status = f"pose error: {type(exc).__name__}: {exc}"

#: defaults of the reference camera (AVM minibus, front, 1280x960)
AVM_FRONT_INTRINSICS = {
    "fx": 317.77563818112867,
    "fy": 318.0250964604786,
    "cx": 636.2327868307656,
    "cy": 477.8201435641188,
    "image_width": 1280,
    "image_height": 960,
}
AVM_FRONT_DISTORTION = (
    0.08476733270570755,
    0.043184113434448945,
    -0.037989564107367736,
    0.009428162434166068,
)


class IntrinsicsSettings(bpy.types.PropertyGroup):
    fx: FloatProperty(
        name="fx",
        description="Focal length in pixels along X",
        default=AVM_FRONT_INTRINSICS["fx"],
        min=1e-6,
        precision=4,
        update=_update_values,
    )
    fy: FloatProperty(
        name="fy",
        description="Focal length in pixels along Y",
        default=AVM_FRONT_INTRINSICS["fy"],
        min=1e-6,
        precision=4,
        update=_update_values,
    )
    auto_center: BoolProperty(
        name="Auto Center",
        description="Keep the principal point at the image centre",
        default=False,
        update=_update_values,
    )
    cx: FloatProperty(
        name="cx",
        description="Principal point u in pixels (origin top-left)",
        default=AVM_FRONT_INTRINSICS["cx"],
        precision=3,
        update=_update_values,
    )
    cy: FloatProperty(
        name="cy",
        description="Principal point v in pixels (origin top-left)",
        default=AVM_FRONT_INTRINSICS["cy"],
        precision=3,
        update=_update_values,
    )
    image_width: IntProperty(
        name="Width",
        description="Resolution the intrinsics were calibrated for",
        default=AVM_FRONT_INTRINSICS["image_width"],
        min=1,
        update=_update_values,
    )
    image_height: IntProperty(
        name="Height",
        description="Resolution the intrinsics were calibrated for",
        default=AVM_FRONT_INTRINSICS["image_height"],
        min=1,
        update=_update_values,
    )
    scale_to_render: BoolProperty(
        name="Scale To Render",
        description="Rescale the intrinsics when the render resolution differs "
        "from the calibrated one (keeps the same field of view)",
        default=True,
        update=_update_values,
    )


class DistortionSettings(bpy.types.PropertyGroup):
    model: EnumProperty(
        name="Model",
        description="OpenCV distortion model; the coefficient meaning depends on it",
        items=[
            ("fisheye", "Fisheye (equidistant)",
             "OpenCV fisheye / Kannala-Brandt: k1..k4 are the theta polynomial terms"),
            ("brown_conrady", "Brown-Conrady",
             "OpenCV plumb_bob / radtan: k1, k2, p1, p2, k3"),
            ("rational", "Rational",
             "OpenCV rational_polynomial: adds the numerator terms k4, k5, k6"),
        ],
        default="fisheye",
        update=_update_model,
    )
    enabled: BoolProperty(
        name="Enable Distortion",
        description="Apply distortion. Disable for an ideal pinhole / rectified view",
        default=True,
        update=_update_values,
    )
    k1: FloatProperty(
        name="k1",
        description="Fisheye: theta^2 term | Brown-Conrady: radial k1",
        default=AVM_FRONT_DISTORTION[0],
        precision=6,
        update=_update_values,
    )
    k2: FloatProperty(
        name="k2",
        description="Fisheye: theta^4 term | Brown-Conrady: radial k2",
        default=AVM_FRONT_DISTORTION[1],
        precision=6,
        update=_update_values,
    )
    p1: FloatProperty(
        name="p1",
        description="Brown-Conrady tangential p1 (unused by the fisheye model)",
        default=0.0,
        precision=6,
        update=_update_values,
    )
    p2: FloatProperty(
        name="p2",
        description="Brown-Conrady tangential p2 (unused by the fisheye model)",
        default=0.0,
        precision=6,
        update=_update_values,
    )
    k3: FloatProperty(
        name="k3",
        description="Fisheye: theta^6 term | Brown-Conrady: radial k3",
        default=AVM_FRONT_DISTORTION[2],
        precision=6,
        update=_update_values,
    )
    k4: FloatProperty(
        name="k4",
        description="Fisheye: theta^8 term | Rational model: numerator k4",
        default=AVM_FRONT_DISTORTION[3],
        precision=6,
        update=_update_values,
    )
    k5: FloatProperty(name="k5", description="Rational model numerator k5", default=0.0, precision=6)
    k6: FloatProperty(name="k6", description="Rational model numerator k6", default=0.0, precision=6)
    discard_invalid_rays: BoolProperty(
        name="Discard Invalid Rays",
        description="Where the distortion inversion diverges (very strong distortion or a pixel "
        "far outside the calibrated image), drop the ray instead of keeping a best effort "
        "direction - dropped pixels render black",
        default=False,
    )
    iterations: IntProperty(
        name="Iterations",
        description="Iterations used to invert the distortion "
        "(fixed point for Brown-Conrady, Newton for fisheye)",
        default=20,
        min=1,
        max=100,
        update=_update_values,
    )


#: common output sizes (identifier -> (width, height))
OUTPUT_PRESETS = {
    "1280x960": (1280, 960),
    "1920x1080": (1920, 1080),
    "1280x720": (1280, 720),
    "640x480": (640, 480),
    "3840x2160": (3840, 2160),
    "1920x1200": (1920, 1200),
}


#: enum items for the output preset.  A *list* (not a callable) so the default can
#: be a string identifier - with callable items bpy requires an index.
OUTPUT_PRESET_ITEMS = [
    (key, key.replace("x", " x "), "") for key in OUTPUT_PRESETS
] + [("custom", "Custom", "Use the width/height fields")]


def _update_output_preset(self, context):
    # note: ``self`` is the OutputSettings group itself, not the root settings
    if self.preset in OUTPUT_PRESETS:
        width, height = OUTPUT_PRESETS[self.preset]
        self.width = width
        self.height = height
    _apply_output(self, context)


def _update_output(self, context):
    _apply_output(self, context)


def _apply_output(group, context=None):
    """Keep the scene render size in sync with the output settings."""
    settings = _root_settings(group)
    if settings is None or not settings.output.lock_scene_resolution:
        return
    if settings.output.mode == "scene":
        return
    from . import apply as apply_mod
    scene = context.scene if context is not None else bpy.context.scene
    try:
        changed, messages = apply_mod.apply_render_resolution(scene, settings)
        if changed or messages:
            settings.status = "resolution set" if not messages else messages[0][:120]
        if settings.auto_apply and group.id_data.type == "CUSTOM":
            apply_mod.apply_values(group.id_data, settings, scene)
    except Exception as exc:
        settings.status = f"output error: {type(exc).__name__}: {exc}"


class OutputSettings(bpy.types.PropertyGroup):
    mode: EnumProperty(
        name="Output Size",
        description="Where the render/output image size comes from",
        items=[
            ("calibration", "Calibration Size",
             "Render at the size the intrinsics were calibrated for (real camera output size)"),
            ("custom", "Custom", "Use the width/height below"),
            ("scene", "Scene Settings", "Leave Blender's render resolution alone"),
        ],
        default="calibration",
        update=_update_output,
    )
    preset: EnumProperty(
        name="Preset",
        description="Common sensor output sizes",
        items=OUTPUT_PRESET_ITEMS,
        default="1280x960",
        update=_update_output_preset,
    )
    width: IntProperty(
        name="Width", description="Output width in pixels", default=1280, min=8, max=16384,
        update=_update_output,
    )
    height: IntProperty(
        name="Height", description="Output height in pixels", default=960, min=8, max=16384,
        update=_update_output,
    )
    lock_scene_resolution: BoolProperty(
        name="Drive Scene Resolution",
        description="Write the output size into Blender's render resolution whenever settings are "
        "applied, so F12 always produces the camera's real output size",
        default=True,
        update=_update_output,
    )


class CalibrationSettings(bpy.types.PropertyGroup):
    filepath: StringProperty(
        name="File",
        description="Calibration file (OpenCV YAML/XML-ish YAML, JSON, ROS camera_info, Kalibr)",
        subtype="FILE_PATH",
        default="",
    )
    last_import: StringProperty(name="Last Import", default="", options={"HIDDEN"})


class PoseSettings(bpy.types.PropertyGroup):
    rotation: FloatVectorProperty(
        name="R",
        description="Row-major 3x3 rotation, world to camera (OpenCV convention)",
        size=9,
        default=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        precision=6,
        update=_update_pose,
    )
    translation: FloatVectorProperty(
        name="t",
        description="Translation, world to camera, OpenCV convention (meters)",
        size=3,
        default=(0.0, 0.0, 0.0),
        precision=6,
        update=_update_pose,
    )
    use_world_transform: BoolProperty(
        name="Custom World Frame",
        description="Apply an extra transform (e.g. ROS/OpenCV world to Blender world)",
        default=False,
    )
    world_matrix: FloatVectorProperty(
        name="World Matrix",
        description="Row-major 4x4 mapping the calibration world frame to the Blender world",
        size=16,
        default=(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        precision=6,
    )


class PreviewSettings(bpy.types.PropertyGroup):
    size: EnumProperty(
        name="Size",
        description="Long side of the preview render (the aspect ratio follows the calibration)",
        items=[
            ("256", "256 px", "Fastest"),
            ("384", "384 px", "Default"),
            ("512", "512 px", ""),
            ("720", "720 px", "Largest"),
        ],
        default="384",
    )
    samples: IntProperty(
        name="Samples",
        description="Cycles samples for the preview",
        default=16,
        min=1,
        max=512,
    )
    denoise: BoolProperty(name="Denoise", default=True)
    on_change: BoolProperty(
        name="Preview On Change",
        description="Re-render the preview shortly after a parameter changes "
        "(about 0.6 s debounce); needs a UI and a Cycles GPU/CPU render per change",
        default=False,
    )


class OpenCVCameraSettings(bpy.types.PropertyGroup):
    intrinsics: PointerProperty(type=IntrinsicsSettings)
    distortion: PointerProperty(type=DistortionSettings)
    calibration: PointerProperty(type=CalibrationSettings)
    pose: PointerProperty(type=PoseSettings)
    preview: PointerProperty(type=PreviewSettings)
    output: PointerProperty(type=OutputSettings)
    show_raw_params: BoolProperty(
        name="Show Cycles Raw Parameters",
        description="Show the raw parameter list that Cycles generates from the OSL shader "
        "parameters (duplicated by the panels below and shown as plain numbers)",
        default=False,
    )
    auto_apply: BoolProperty(
        name="Live Apply",
        description="Push parameter changes to the camera immediately. Only applies once the "
        "camera uses Lens Type = Custom (press Apply to Camera once to set that up)",
        default=True,
    )
    status: StringProperty(name="Status", default="", options={"HIDDEN"})

    # -- helpers used by operators/UI ------------------------------------
    def core_intrinsics(self, width: int = 0, height: int = 0) -> camera_model.Intrinsics:
        """Current settings as a :class:`core.camera_model.Intrinsics`."""
        intr = self.intrinsics
        auto = intr.auto_center
        return camera_model.Intrinsics(
            fx=intr.fx,
            fy=intr.fy,
            cx=-1.0 if auto else intr.cx,
            cy=-1.0 if auto else intr.cy,
            width=int(width or intr.image_width),
            height=int(height or intr.image_height),
        )

    def core_distortion(self) -> camera_model.Distortion:
        """Current settings as a :class:`core.camera_model.Distortion`."""
        d = self.distortion
        model = d.model
        if model == "brown_conrady" and (d.k4 or d.k5 or d.k6):
            model = camera_model.MODEL_RATIONAL
        return camera_model.Distortion(
            k1=d.k1, k2=d.k2, k3=d.k3, k4=d.k4, k5=d.k5, k6=d.k6,
            p1=d.p1, p2=d.p2, model=model, enabled=d.enabled,
        )

    def set_from_core(self, intr: camera_model.Intrinsics, dist: camera_model.Distortion) -> None:
        """Write core model objects back into the property groups."""
        self.intrinsics.fx = intr.fx
        self.intrinsics.fy = intr.fy
        self.intrinsics.image_width = intr.width
        self.intrinsics.image_height = intr.height
        if intr.auto_center:
            self.intrinsics.auto_center = True
            self.intrinsics.cx = -1.0
            self.intrinsics.cy = -1.0
        else:
            self.intrinsics.auto_center = False
            self.intrinsics.cx = intr.cx
            self.intrinsics.cy = intr.cy
        d = self.distortion
        d.model = dist.model
        d.enabled = dist.enabled
        d.k1, d.k2, d.k3, d.k4, d.k5, d.k6 = dist.k1, dist.k2, dist.k3, dist.k4, dist.k5, dist.k6
        d.p1, d.p2 = dist.p1, dist.p2


#: Classes registered by :func:`register` / unregistered by :func:`unregister`.
_CLASSES = (
    IntrinsicsSettings,
    DistortionSettings,
    CalibrationSettings,
    PoseSettings,
    PreviewSettings,
    OutputSettings,
    OpenCVCameraSettings,
)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Camera.opencv_cam = PointerProperty(
        name="OpenCV Camera",
        description="OpenCV intrinsics / distortion settings for this camera",
        type=OpenCVCameraSettings,
    )


def unregister() -> None:
    del bpy.types.Camera.opencv_cam
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)