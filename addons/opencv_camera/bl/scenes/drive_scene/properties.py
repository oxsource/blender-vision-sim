"""Drive Scene properties.

The scene owns its settings on the **Scene** (``scene.drive_scene``), so the
panels need no selected object.  Camera *intrinsics* are deliberately **not**
here: they stay on ``camera.data.opencv_cam`` and are edited in the existing
``CV Intrinsics`` / ``CV Presets`` panels, exactly like in the AVM Scene.

The same boundary governs :attr:`DriveSceneSettings.cameras`: an entry holds
**only** "is this camera recorded?" and nothing else.  ``K`` / ``D`` / ``output``
and the mount pose are the camera's *calibration*, and calibration has a single
source - the AVM Scene's bundled preset, read through
``bl.camera_factory.configure_from_record`` (``docs/drive-scene-multicam.md``
sections 3-4).  A second copy here is exactly the drift this scene is built to
avoid, so there is not one.

Every editable value schedules a debounced rebuild (dragging a slider fires one
update per mouse move, so the rebuild has to wait until the user stops).
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import bpy
from bpy.props import (
    BoolProperty,
    CollectionProperty,
    EnumProperty,
    FloatProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)

from ....core.scenes import avm_cameras, vehicle
from .. import base
from ..base import SceneDefinition  # noqa: F401  (re-exported for convenience)
from . import DEFINITION

#: enum items must be a plain module level list (annotations are re-evaluated).
#: The four rig roles come from ``core.scenes.avm_cameras``, i.e. the very list
#: the AVM Scene builds its cameras from.
CAMERA_ITEMS = avm_cameras.enum_items()

#: enum items must be a plain module level list (annotations are re-evaluated)
GROUND_TEXTURES = [
    ("concrete", "Concrete", "Poured concrete - mid grey, matte, gently blotchy"),
    ("asphalt", "Asphalt", "Asphalt - near black, matte, light aggregate speckle"),
    ("epoxy", "Epoxy", "Epoxy coating - light grey, semi-gloss, almost even"),
    ("checker", "Checker", "High contrast checker tiles"),
    ("plain", "Plain", "Flat grey - no texture at all"),
]

DRIVE_PROFILES = [
    ("constant", "Constant", "Hold one speed for the whole drive"),
    ("trapezoid", "Trapezoid", "Accelerate to the cruise speed, cruise, brake to a stop"),
]

#: Clip export quality presets.  Only the render *cost* changes - the output
#: size and the camera (K / D) are identical at every setting.  ``high`` pins
#: nothing but the sample count, i.e. it reproduces the historical export.
CLIP_QUALITIES = [
    ("draft", "Draft", "Fastest: 8 samples, denoised, 2 light bounces"),
    ("balanced", "Balanced", "24 samples, denoised, 4 light bounces"),
    ("high", "High", "64 samples, the scene's own denoise / bounce settings"),
]

CLIP_QUALITY_PRESETS = {
    "draft": {
        "samples": 8,
        "denoise": True,
        "max_bounces": 2,
        "diffuse_bounces": 1,
        "glossy_bounces": 1,
        "adaptive_threshold": 0.1,
        "caustics": False,
    },
    "balanced": {
        "samples": 24,
        "denoise": True,
        "max_bounces": 4,
        "diffuse_bounces": 2,
        "glossy_bounces": 2,
        "adaptive_threshold": 0.05,
        "caustics": False,
    },
    "high": {
        "samples": 64,
    },
}

#: Render devices the export may ask for.  The OpenCV camera is an OSL shader,
#: which Cycles only evaluates on CPU and NVIDIA OptiX, so ``gpu`` is a no-op
#: (with a warning) on every other backend.
CLIP_DEVICES = [
    ("cpu", "CPU", "Always correct - the only backend on macOS / Metal"),
    ("gpu", "GPU (OptiX)", "Faster, but only NVIDIA OptiX can run the OSL camera"),
]


def clip_quality_preset(key: str) -> dict:
    """The Cycles settings for a quality key (unknown keys fall back to balanced)."""
    return dict(CLIP_QUALITY_PRESETS.get(str(key), CLIP_QUALITY_PRESETS["balanced"]))


def _schedule(self, context) -> None:
    """Debounced rebuild for any layout change."""
    from . import controller
    controller.schedule_rebuild(context)


def _update_visibility(self, context) -> None:
    """Layer flags only toggle hide_render / hide_set - no geometry rebuild."""
    from . import builder
    builder.apply_visibility(self)


def _schedule_active(self, context) -> None:
    """``active_camera`` switches the render camera straight away.

    Only ``scene.camera`` changes - no camera parameter and no geometry - so the
    debounced rebuild would be pure latency here.  It is still scheduled: the
    active camera is part of what a rebuild restores.
    """
    from . import builder, controller
    builder.apply_active_camera(getattr(context, "scene", None), self)
    controller.schedule_rebuild(context)


class DriveCameraSettings(bpy.types.PropertyGroup):
    """One camera's Drive-Scene state: **only** whether it is recorded.

    Anything that describes the camera itself (K / D / output / mount pose) is
    deliberately absent - see the module docstring.
    """

    name: StringProperty(name="Name", default="camera")
    enable: BoolProperty(
        name="Record", default=True,
        description="Record this camera: one PNG per frame and one mp4 per "
                    "camera. The transparent-chassis algorithm needs all four, "
                    "so they are on by default; every extra camera costs one "
                    "more render per frame")


def _ensure_camera_entries(settings) -> None:
    """Keep ``settings.cameras`` in step with :data:`avm_cameras.CAMERAS`.

    The rig roles are code-level constants, not user data, so the collection is
    a mirror of them rather than something the user adds to.  Entries are added
    (enabled) and stale names dropped.

    Called from the **builder**, never from a panel's ``draw()``: a panel may be
    repainted while Blender holds the file in a state where writing to an ID is
    not allowed, and a UI pass has no business changing the scene anyway.  The
    readers below therefore tolerate a not-yet-populated collection instead of
    filling it themselves.
    """
    wanted = list(avm_cameras.CAMERAS)
    known = [entry.name for entry in settings.cameras]
    for index in range(len(known) - 1, -1, -1):
        if known[index] not in wanted:
            settings.cameras.remove(index)
    known = {entry.name for entry in settings.cameras}
    for key in wanted:
        if key not in known:
            settings.cameras.add().name = key


class DriveSceneSettings(bpy.types.PropertyGroup):
    """The whole Drive Scene layout (Scene level)."""

    # -- the car park (m) ---------------------------------------------------
    aisle_length: FloatProperty(
        name="Aisle Length", default=48.0, min=6.0, max=300.0, unit="LENGTH",
        description="Length of the parking aisle; the slab grows past it when "
                    "the drive needs more room",
        update=_schedule)
    aisle_width: FloatProperty(
        name="Aisle Width", default=6.0, min=2.5, max=20.0, unit="LENGTH",
        description="Width of the driving lane between the two parking rows",
        update=_schedule)
    bay_depth: FloatProperty(
        name="Bay Depth", default=5.2, min=3.0, max=8.0, unit="LENGTH",
        description="Depth of a parking bay (aisle edge to the wall)",
        update=_schedule)
    bay_width: FloatProperty(
        name="Bay Width", default=2.5, min=1.8, max=4.0, unit="LENGTH",
        description="Width of one parking bay; the dividers are painted every "
                    "bay_width along both rows",
        update=_schedule)
    show_bays: BoolProperty(
        name="Lane Markings", default=True,
        description="Paint the bay dividers, the aisle edge lines and the dashed "
                    "centre line - the visual features a transparent-chassis "
                    "algorithm is verified against",
        update=_schedule)
    pillar_count: IntProperty(
        name="Pillars", default=2, min=0, max=16,
        description="Square columns per side, evenly spaced along the aisle; a "
                    "bay whose column would reach into it stays empty",
        update=_schedule)
    parked_cars: IntProperty(
        name="Parked / Row", default=4, min=0, max=24,
        description="Parked vehicles per row, spread over the free bays (nose in, "
                    "the forms and paints cycle so a rebuild always looks the same)",
        update=_schedule)
    bay_numbers: BoolProperty(
        name="Bay Numbers", default=True,
        description="Paint A01 / B01 ... in the aisle in front of every bay - "
                    "landmarks to compare a reconstruction against",
        update=_schedule)
    ground_texture: EnumProperty(
        name="Floor", items=GROUND_TEXTURES, default="concrete",
        description="Floor material; concrete / asphalt / epoxy are procedural "
                    "(subtle multi-scale mottling, no texture file), checker and "
                    "plain are control experiments",
        update=_schedule)
    light_energy: FloatProperty(
        name="Ceiling Light", default=2000.0, min=0.0, max=50000.0,
        description="Power of the soft ceiling panel [W]; it covers the whole lot, "
                    "so the floor is lit evenly instead of in bright pools "
                    "(2000 W puts the concrete near 0.4 with nothing clipped)",
        update=_schedule)
    shadows: BoolProperty(
        name="Shadows", default=True,
        description="Let the ceiling panel cast shadows (the AVM Scene keeps them "
                    "off for its black-region corner detector; a drive clip wants "
                    "the lighting a real car park has)",
        update=_schedule)

    # -- vehicle (m) - the shared minibus geometry (:mod:`core.scenes.vehicle`),
    # the same numbers the AVM Scene's car is built from
    car_length: FloatProperty(name="Length", default=vehicle.BODY_LENGTH_M,
                              min=1.0, max=30.0, unit="LENGTH", update=_schedule)
    car_width: FloatProperty(name="Width", default=vehicle.BODY_WIDTH_M,
                             min=0.5, max=10.0, unit="LENGTH", update=_schedule)
    car_height: FloatProperty(name="Height", default=vehicle.BODY_HEIGHT_M,
                              min=0.5, max=10.0, unit="LENGTH", update=_schedule)
    car_clearance: FloatProperty(name="Clearance", default=vehicle.GROUND_CLEARANCE_M,
                                 min=0.0, max=2.0, unit="LENGTH", update=_schedule)

    # -- drive --------------------------------------------------------------
    drive_distance: FloatProperty(
        name="Distance", default=20.0, min=0.0, max=500.0, unit="LENGTH",
        description="How far the vehicle drives; the run is centred on the lot",
        update=_schedule)
    drive_speed: FloatProperty(
        name="Speed", default=3.0, min=0.05, max=40.0, unit="VELOCITY",
        update=_schedule)
    drive_accel: FloatProperty(
        name="Accel", default=1.0, min=0.05, max=10.0, unit="ACCELERATION",
        description="Acceleration and braking of the trapezoid profile; unused "
                    "by the constant profile",
        update=_schedule)
    drive_profile: EnumProperty(
        name="Profile", items=DRIVE_PROFILES, default="constant",
        description="constant holds one speed; trapezoid accelerates to the "
                    "cruise speed, cruises, then brakes to a stop",
        update=_schedule)
    drive_fps: IntProperty(
        name="FPS", default=10, min=1, max=120,
        description="Frames per second of the clip; a frame is rendered at every "
                    "step (the per-frame travel is speed / fps)",
        update=_schedule)
    drive_heading: FloatProperty(
        name="Heading", default=0.0, min=-180.0, max=180.0,
        description="Driving direction [deg]; 0 = +Y (the aisle), the vehicle's nose",
        update=_schedule)

    # -- clip export --------------------------------------------------------
    clip_quality: EnumProperty(
        name="Quality", items=CLIP_QUALITIES, default="draft",
        description="Render quality of the exported clip; the image size and the "
                    "camera stay the same at every setting, only the render cost "
                    "changes (the video is always H.264)")
    clip_device: EnumProperty(
        name="Device", items=CLIP_DEVICES, default="cpu",
        description="Cycles render device for the export. The OpenCV camera is an "
                    "OSL shader, which Cycles only evaluates on CPU and NVIDIA "
                    "OptiX; asking for GPU on any other backend falls back to CPU")
    clip_keep_frames: BoolProperty(
        name="Keep Frames", default=True,
        description="Also keep the PNG sequence in the export. A validation run "
                    "consumes the video, and the stills are only the encoder's "
                    "input, so turning this off leaves a clip that is ~100x "
                    "smaller (measured on the default clip: 138 MB of PNG against "
                    "1.0 MB of H.264). The scenario in clip.json can re-render "
                    "them at any time")

    # -- cameras ------------------------------------------------------------
    # Which camera the viewport / F12 shows, and which ones a clip records.  No
    # calibration here - see the module docstring.
    active_camera: EnumProperty(
        name="Active Camera", items=CAMERA_ITEMS, default=avm_cameras.FRONT,
        description="Camera the viewport and F12 render; the clip records every "
                    "camera whose Record box is ticked, in a fixed order",
        update=_schedule_active)
    cameras: CollectionProperty(type=DriveCameraSettings)

    # -- layers (toggle visibility only, no rebuild) ------------------------
    show_ground: BoolProperty(name="Floor", default=True, update=_update_visibility)
    show_walls: BoolProperty(name="Walls", default=True, update=_update_visibility)
    show_parked: BoolProperty(name="Parked Cars", default=True, update=_update_visibility)
    show_car: BoolProperty(name="Car", default=True, update=_update_visibility)

    # -- state --------------------------------------------------------------
    root: PointerProperty(
        name="Root",
        description="DRIVE_Root empty; its presence is what makes the panels show",
        type=bpy.types.Object)

    #: rebuild counter, handy in tests / the status line
    revision: IntProperty(name="Revision", default=0, options={"HIDDEN"})
    #: last recording's report
    clip_status: StringProperty(name="Clip", default="", options={"HIDDEN"})

    # -- helpers ------------------------------------------------------------
    def ensure_cameras(self) -> None:
        """Mirror :data:`avm_cameras.CAMERAS` into :attr:`cameras`.

        The builder calls this on every rebuild; the panel and the recorder only
        read, so neither can write to the file from a ``draw()``.
        """
        _ensure_camera_entries(self)

    def camera(self, key: str) -> Optional[DriveCameraSettings]:
        """The record switch of one camera (``None`` for an unknown key)."""
        for entry in self.cameras:
            if entry.name == key:
                return entry
        return None

    def recorded_cameras(self) -> List[str]:
        """The camera keys a clip records, in :data:`avm_cameras.CAMERAS` order.

        Order is the contract: it is the order of ``frames.csv``'s column groups
        and of ``clip.json``'s ``cameras`` array, so "which column belongs to
        which camera" is answerable without counting.

        A role with no entry counts as **enabled** - that is the switch's own
        default - so a scene that has not been rebuilt since the collection was
        introduced records all four rather than silently recording none.
        """
        disabled = {entry.name for entry in self.cameras if not entry.enable}
        return [key for key in avm_cameras.CAMERAS if key not in disabled]

    def reset_cameras(self) -> None:
        """Put the per-camera record switches back to their defaults (all on)."""
        self.cameras.clear()
        _ensure_camera_entries(self)

    def lot_size(self) -> Tuple[float, float]:
        """``(length, width)`` of the slab [m].

        The slab always covers the whole drive plus one bay, so a long drive
        never runs off the floor even when the aisle is short.
        """
        length = max(self.aisle_length, self.drive_distance + 2.0 * self.bay_depth)
        return length, self.aisle_width + 2.0 * self.bay_depth

    def start_point(self) -> Tuple[float, float]:
        """Where the vehicle starts: the run is centred on the lot [m]."""
        from ....core.scenes import drive_path
        forward = drive_path.forward(self.drive_heading)
        return (-forward[0] * self.drive_distance / 2.0,
                -forward[1] * self.drive_distance / 2.0)

    def plan(self):
        """The motion plan (:mod:`core.scenes.drive_path`) of these settings."""
        from ....core.scenes import drive_path
        return drive_path.plan(
            distance=self.drive_distance, speed=self.drive_speed,
            accel=self.drive_accel, profile=self.drive_profile,
            fps=self.drive_fps, heading=self.drive_heading,
            start=self.start_point())


_CLASSES = (DriveCameraSettings, DriveSceneSettings)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.drive_scene = PointerProperty(
        name="Drive Scene",
        description="Indoor car-park drive: layout, vehicle and the drive clip",
        type=DriveSceneSettings,
    )


def unregister() -> None:
    del bpy.types.Scene.drive_scene
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
