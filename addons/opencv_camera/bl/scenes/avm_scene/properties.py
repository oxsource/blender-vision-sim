"""AVM Scene properties.

The scene owns its settings on the **Scene** (``scene.avm_scene``), so the panels
need no selected object.  Camera *intrinsics* are deliberately **not** here: they
stay on ``camera.data.opencv_cam`` and are edited in the existing
``CV Intrinsics`` / ``CV Presets`` panels (see
``docs/avm-scene.md`` §4.2).  Only the camera *mount pose* lives here.

Every editable value schedules a debounced rebuild (dragging a slider fires one
update per mouse move, so the rebuild has to wait until the user stops).
"""

from __future__ import annotations

import bpy
from bpy.props import (
    BoolProperty,
    CollectionProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)

from ....core.scenes import avm_cameras, vehicle
from .. import base
from ..base import SceneDefinition  # noqa: F401  (re-exported for convenience)
from . import DEFINITION

#: enum items must be a plain module level list (annotations are re-evaluated).
#: The four rig roles come from ``core.scenes.avm_cameras`` - the Drive Scene
#: reads the same list, so the two panels cannot offer different cameras
#: (``docs/drive-scene-multicam.md`` section 4).
CAMERA_ITEMS = avm_cameras.enum_items()


def _schedule(self, context) -> None:
    """Debounced rebuild for any layout change."""
    from . import controller
    controller.schedule_rebuild(context)


def _update_visibility(self, context) -> None:
    """Layer flags only toggle hide_render / hide_set - no geometry rebuild."""
    from . import builder
    builder.apply_visibility(self)


def _schedule_active(self, context) -> None:
    """``active_camera`` also re-applies the render resolution."""
    from . import controller
    controller.apply_active_camera(context)
    controller.schedule_rebuild(context)


class AVMCameraSettings(bpy.types.PropertyGroup):
    """One camera's scene-level state (pose + intrinsics live on the object)."""

    name: StringProperty(name="Name", default="camera")
    enable: BoolProperty(name="Enable", default=True, update=_schedule)
    #: detected block corners, in the points_3d order (8 x [u, v]); hidden and
    #: read through the corner detector / the material export
    points_2d: FloatVectorProperty(
        name="Points 2D", size=16, default=(0.0,) * 16, options={"HIDDEN"})
    points_2d_ok: BoolProperty(
        name="Points 2D Detected", default=False, options={"HIDDEN"})
    points_2d_error: FloatProperty(
        name="Points 2D RMS", default=0.0, options={"HIDDEN"})
    #: cache key captured at detection time (scene revision + pose/intrinsics/
    #: output/samples); a re-detect with the same key reuses the result
    points_2d_revision: IntProperty(
        name="Points 2D Revision", default=-1, options={"HIDDEN"})
    points_2d_signature: StringProperty(
        name="Points 2D Signature", default="", options={"HIDDEN"})


class AVMSceneSettings(bpy.types.PropertyGroup):
    """The whole AVM Scene layout (Scene level)."""

    # -- field (cm, the unit of the HTML tool and the App sliders) -----------
    border_w: FloatProperty(name="Border W", default=0.0, min=0.0, max=2000.0,
                            description="Outer margin, width direction [cm]",
                            update=_schedule)
    border_h: FloatProperty(name="Border H", default=0.0, min=0.0, max=2000.0,
                            description="Outer margin, height direction [cm]",
                            update=_schedule)
    corner: FloatProperty(name="Corner", default=100.0, min=1.0, max=2000.0,
                          description="Calibration block edge length [cm]",
                          update=_schedule)
    inner_w: FloatProperty(name="Inner W", default=20.0, min=0.0, max=2000.0,
                           description="Gap between the car and the blocks, width [cm]",
                           update=_schedule)
    inner_h: FloatProperty(name="Inner H", default=80.0, min=0.0, max=2000.0,
                           description="Gap between the car and the blocks, height [cm]",
                           update=_schedule)
    core_w: FloatProperty(name="Core W", default=240.0, min=10.0, max=2000.0,
                          description="Car footprint width [cm]",
                          update=_schedule)
    core_h: FloatProperty(name="Core H", default=480.0, min=10.0, max=2000.0,
                          description="Car footprint length [cm]",
                          update=_schedule)

    # -- car ----------------------------------------------------------------
    # The defaults are the shared minibus (:mod:`core.scenes.vehicle`) so the AVM
    # car and the Drive car are the same car by construction; ``build()`` still
    # overwrites the height with the preset's camera mount height.
    car_follow_core: BoolProperty(
        name="Follow Core", default=True,
        description="Use the core footprint as the car length/width",
        update=_schedule)
    car_length: FloatProperty(name="Length", default=vehicle.BODY_LENGTH_M,
                              min=0.1, max=30.0, unit="LENGTH", update=_schedule)
    car_width: FloatProperty(name="Width", default=vehicle.BODY_WIDTH_M,
                             min=0.1, max=30.0, unit="LENGTH", update=_schedule)
    car_height: FloatProperty(
        name="Height", default=vehicle.BODY_HEIGHT_M, min=0.05, max=10.0,
        unit="LENGTH",
        description="Body height; keep it at or above the camera mount height so "
                    "the cameras sit on the body",
        update=_schedule)
    car_clearance: FloatProperty(name="Clearance", default=vehicle.GROUND_CLEARANCE_M,
                                 min=0.0, max=2.0, unit="LENGTH", update=_schedule)

    # -- ground / blocks ----------------------------------------------------
    use_ground_model: BoolProperty(
        name="Real Ground Mesh", default=True,
        description="Use the real AVM bowl mesh as the ground (the surface the "
                    "Falcon app projects the cameras onto); off = a flat plane",
        update=_schedule)
    ground_model: StringProperty(
        name="Ground Model",
        description="Custom ground model (.glb/.gltf/.fbx/.obj); empty = the bundled "
                    "real AVM bowl that ships with the add-on",
        subtype="FILE_PATH",
        default="",
        update=_schedule)
    ground_radius: FloatProperty(
        name="Bowl Radius", default=15.0, min=2.0, max=100.0,
        unit="LENGTH",
        description="Radius of the real ground mesh [m]; the bowl is scaled around "
                    "the vehicle and its floor stays flat at z = 0",
        update=_schedule)
    ground_rim_height: FloatProperty(
        name="Rim Height", default=5.0, min=0.0, max=50.0,
        unit="LENGTH",
        description="Height of the ground mesh rim above the floor [m]",
        update=_schedule)
    ground_w: FloatProperty(name="Ground W", default=30.0, min=1.0, max=500.0,
                            unit="LENGTH", update=_schedule)
    ground_d: FloatProperty(name="Ground D", default=30.0, min=1.0, max=500.0,
                            unit="LENGTH", update=_schedule)
    block_lift: FloatProperty(name="Block Lift", default=0.001, min=0.0, max=0.05,
                              unit="LENGTH", precision=4,
                              description="Blocks hover this far above the ground (z-fighting)",
                              update=_schedule)
    sun_energy: FloatProperty(
        name="Sun Energy", default=3.0, min=0.0, max=100.0,
        description="Key light strength; a shadowless fill is derived from it",
        update=_schedule)
    sun_shadow: BoolProperty(
        name="Sun Shadow", default=False,
        description="Let the sun cast shadows. Off by default: cast shadows are dark "
        "ground patches that a black-region corner detector can mistake for blocks",
        update=_schedule)

    # -- props around the vehicle (like the real scene) ---------------------
    prop_pedestrians: IntProperty(
        name="Pedestrians", default=3, min=0, max=16,
        description="Blocky pedestrians standing around the field",
        update=_schedule)
    prop_boxes: IntProperty(
        name="Crates", default=4, min=0, max=16,
        description="Plastic crates on the floor around the field",
        update=_schedule)
    prop_carts: IntProperty(
        name="Carts", default=1, min=0, max=8,
        description="Small pallet carts with casters",
        update=_schedule)

    # -- ground text --------------------------------------------------------
    ground_title: StringProperty(
        name="Title",
        description="Text painted on the ground outside the field",
        default="AVM 仿真标定场地",
        update=_schedule)
    label_font: StringProperty(
        name="Font",
        description="Font file with CJK glyphs for the ground text; empty = "
                    "auto-detect a system font (Blender's built-in font has no CJK)",
        subtype="FILE_PATH",
        default="",
        update=_schedule)
    logo_enabled: BoolProperty(
        name="Show Logo", default=True,
        description="Draw the logo decal before the title text",
        update=_schedule)
    logo_image: StringProperty(
        name="Logo Image",
        description="Custom logo image (e.g. a PNG with alpha); empty = the "
                    "add-on's bundled logo, which ships with the package",
        subtype="FILE_PATH",
        default="",
        update=_schedule)
    logo_size: FloatProperty(
        name="Logo Size", default=1.0, min=0.1, max=10.0, unit="LENGTH",
        description="Logo width in metres; the height follows the image aspect ratio",
        update=_schedule)

    # -- layers (toggle visibility only, no rebuild) ------------------------
    show_ground: BoolProperty(name="Ground", default=True, update=_update_visibility)
    show_blocks: BoolProperty(name="Calibration Blocks", default=True, update=_update_visibility)
    show_car: BoolProperty(name="Car", default=True, update=_update_visibility)
    show_cameras: BoolProperty(name="Cameras", default=True, update=_update_visibility)
    show_props: BoolProperty(name="Props", default=True, update=_update_visibility)
    show_labels: BoolProperty(name="Ground Text", default=True, update=_update_visibility)
    show_coverage: BoolProperty(name="Coverage", default=False, update=_update_visibility)
    show_sun: BoolProperty(name="Sun", default=True, update=_update_visibility)

    # -- cameras ------------------------------------------------------------
    active_camera: EnumProperty(
        name="Active Camera", items=CAMERA_ITEMS, default="front",
        description="Camera that drives the render resolution (all four share it)",
        update=_schedule_active)
    cameras: CollectionProperty(type=AVMCameraSettings)

    # -- bowl export --------------------------------------------------------
    bowl_include_vehicle: BoolProperty(
        name="Include Vehicle", default=False,
        description="Also export the vehicle in Export Bowl; the GLB node is named "
                    "'vehicle' (the scene object stays AVM_Car)")

    # -- state --------------------------------------------------------------
    root: PointerProperty(
        name="Root",
        description="AVM_Root empty; its presence is what makes the panels show",
        type=bpy.types.Object)

    #: rebuild counter, handy in tests / the status line
    revision: IntProperty(name="Revision", default=0, options={"HIDDEN"})

    # -- parameter import / export -----------------------------------------
    io_status: StringProperty(name="Status", default="", options={"HIDDEN"})

    # -- coverage (cached by the analyze operator, §16) ---------------------
    coverage_status: StringProperty(name="Coverage", default="", options={"HIDDEN"})
    coverage_matrix: StringProperty(name="Visibility", default="", options={"HIDDEN"})

    # -- corner detection (cached by the detect operator, §16.7) ------------
    corners_status: StringProperty(name="Corners", default="", options={"HIDDEN"})

    # -- helpers ------------------------------------------------------------
    def field_spec(self):
        """The layout as a pure-Python :class:`core.scenes.avm_layout.FieldSpec`."""
        from ....core.scenes import avm_layout
        return avm_layout.FieldSpec(
            border_w=self.border_w, border_h=self.border_h,
            corner=self.corner, inner_w=self.inner_w, inner_h=self.inner_h,
            core_w=self.core_w, core_h=self.core_h,
        )

    def camera(self, name: str):
        for entry in self.cameras:
            if entry.name == name:
                return entry
        return None

    def active_camera_settings(self):
        return self.camera(self.active_camera)

    def car_size(self):
        """``(length, width)`` in metres."""
        if self.car_follow_core:
            return self.core_h * 0.01, self.core_w * 0.01
        return self.car_length, self.car_width


_CLASSES = (AVMCameraSettings, AVMSceneSettings)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.avm_scene = PointerProperty(
        name="AVM Scene",
        description="AVM plane scene layout (blocks, car, cameras, ground)",
        type=AVMSceneSettings,
    )


def unregister() -> None:
    del bpy.types.Scene.avm_scene
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
