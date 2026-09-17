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
    )
    fy: FloatProperty(
        name="fy",
        description="Focal length in pixels along Y",
        default=AVM_FRONT_INTRINSICS["fy"],
        min=1e-6,
        precision=4,
    )
    auto_center: BoolProperty(
        name="Auto Center",
        description="Keep the principal point at the image centre",
        default=False,
    )
    cx: FloatProperty(
        name="cx",
        description="Principal point u in pixels (origin top-left)",
        default=AVM_FRONT_INTRINSICS["cx"],
        precision=3,
    )
    cy: FloatProperty(
        name="cy",
        description="Principal point v in pixels (origin top-left)",
        default=AVM_FRONT_INTRINSICS["cy"],
        precision=3,
    )
    image_width: IntProperty(
        name="Width",
        description="Resolution the intrinsics were calibrated for",
        default=AVM_FRONT_INTRINSICS["image_width"],
        min=1,
    )
    image_height: IntProperty(
        name="Height",
        description="Resolution the intrinsics were calibrated for",
        default=AVM_FRONT_INTRINSICS["image_height"],
        min=1,
    )
    scale_to_render: BoolProperty(
        name="Scale To Render",
        description="Rescale the intrinsics when the render resolution differs "
        "from the calibrated one (keeps the same field of view)",
        default=True,
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
    )
    enabled: BoolProperty(
        name="Enable Distortion",
        description="Apply distortion. Disable for an ideal pinhole / rectified view",
        default=True,
    )
    k1: FloatProperty(
        name="k1",
        description="Fisheye: theta^2 term | Brown-Conrady: radial k1",
        default=AVM_FRONT_DISTORTION[0],
        precision=6,
    )
    k2: FloatProperty(
        name="k2",
        description="Fisheye: theta^4 term | Brown-Conrady: radial k2",
        default=AVM_FRONT_DISTORTION[1],
        precision=6,
    )
    p1: FloatProperty(
        name="p1",
        description="Brown-Conrady tangential p1 (unused by the fisheye model)",
        default=0.0,
        precision=6,
    )
    p2: FloatProperty(
        name="p2",
        description="Brown-Conrady tangential p2 (unused by the fisheye model)",
        default=0.0,
        precision=6,
    )
    k3: FloatProperty(
        name="k3",
        description="Fisheye: theta^6 term | Brown-Conrady: radial k3",
        default=AVM_FRONT_DISTORTION[2],
        precision=6,
    )
    k4: FloatProperty(
        name="k4",
        description="Fisheye: theta^8 term | Rational model: numerator k4",
        default=AVM_FRONT_DISTORTION[3],
        precision=6,
    )
    k5: FloatProperty(name="k5", description="Rational model numerator k5", default=0.0, precision=6)
    k6: FloatProperty(name="k6", description="Rational model numerator k6", default=0.0, precision=6)
    iterations: IntProperty(
        name="Iterations",
        description="Iterations used to invert the distortion "
        "(fixed point for Brown-Conrady, Newton for fisheye)",
        default=20,
        min=1,
        max=100,
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
    )
    translation: FloatVectorProperty(
        name="t",
        description="Translation, world to camera, OpenCV convention (meters)",
        size=3,
        default=(0.0, 0.0, 0.0),
        precision=6,
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


class OpenCVCameraSettings(bpy.types.PropertyGroup):
    intrinsics: PointerProperty(type=IntrinsicsSettings)
    distortion: PointerProperty(type=DistortionSettings)
    calibration: PointerProperty(type=CalibrationSettings)
    pose: PointerProperty(type=PoseSettings)
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