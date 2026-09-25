"""Road Scene properties.

The scene owns its settings on ``scene.road_scene``.  Camera *intrinsics* are
deliberately **not** here: they stay on ``camera.data.opencv_cam`` and are edited
in the existing ``CV Intrinsics`` / ``CV Presets`` panels, exactly like the AVM
and Drive Scenes; the only camera setting this scene owns is *which ones a clip
records*.  Clip quality / device / keep-frames are the shared recorder's, too.
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

from ....core.scenes import avm_cameras, road_path, road_track, vehicle
from .. import base, clip_core
from ..base import SceneDefinition  # noqa: F401  (re-exported for convenience)
from . import DEFINITION

#: enum items must be a plain module level list (annotations are re-evaluated)
CAMERA_ITEMS = avm_cameras.enum_items()

GROUND_TEXTURES = [
    ("asphalt", "Asphalt", "Asphalt - near black, matte, light aggregate speckle"),
    ("concrete", "Concrete", "Poured concrete - mid grey, matte, gently blotchy"),
    ("epoxy", "Epoxy", "Epoxy coating - light grey, semi-gloss, almost even"),
    ("checker", "Checker", "High contrast checker tiles"),
    ("plain", "Plain", "Flat dark - no texture at all"),
]

DRIVE_PROFILES = [
    ("scenario", "Scenario",
     "Start at rest, accelerate to the cruise, slow at the zebra crossing, and "
     "brake to a stop exactly back at the start (geometry-aware)"),
    ("constant", "Constant", "Hold one speed for the whole drive"),
    ("trapezoid", "Trapezoid", "Accelerate to the cruise speed, cruise, brake to a stop"),
]

#: loop sizes the scene can build.  ``custom`` uses the dimension sliders below.
TRACK_ITEMS = [(key, preset.label, preset.description)
               for key, preset in road_track.TRACK_PRESETS.items()]
TRACK_ITEMS.append(("custom", "Custom", "Use the straight / radius / ramp sliders below"))

DIRECTIONS = [
    ("forward", "Forward", "Drive around the loop nose-first"),
    ("reverse", "Reverse", "Drive around the loop backwards (reversing)"),
]

CLIP_QUALITIES = clip_core.CLIP_QUALITIES
CLIP_DEVICES = [
    ("cpu", "CPU", "Always correct - the only backend on macOS / Metal"),
    ("gpu", "GPU (OptiX)", "Faster, but only NVIDIA OptiX can run the OSL camera"),
]

LOOP_SEGMENT = "loop"

#: the whole loop, or one named segment - the M5 matrix drives segments one at a
#: time so a clip's label is unambiguous
SEGMENT_ITEMS = [(LOOP_SEGMENT, "Whole loop",
                  "Drive the whole closed loop (speed x direction on every type)")]
for _name in road_track.DEFAULT_SEGMENT_NAMES:
    _label = _name.replace("_", " ").title()
    if _name.startswith("ramp_up"):
        _label = "Up slope"
    elif _name.startswith("ramp_down"):
        _label = "Down slope"
    SEGMENT_ITEMS.append((_name, _label, f"Drive the {_name} segment once"))


def _schedule(self, context) -> None:
    from . import controller
    controller.schedule_rebuild(context)


def _schedule_active(self, context) -> None:
    from . import builder, controller
    builder.apply_active_camera(getattr(context, "scene", None), self)
    controller.schedule_rebuild(context)


def _apply_track_preset(self, context) -> None:
    """Write a named loop's dimensions into the sliders, then rebuild."""
    preset = road_track.TRACK_PRESETS.get(self.track_preset)
    if preset is not None:
        self.straight_length = preset.straight
        self.curve_radius = preset.radius
        self.ramp_rise = preset.ramp_rise
        self.ramp_length = preset.ramp_length
        self.road_width = preset.road_width
        self.shoulder_width = preset.shoulder
    _schedule(self, context)


class RoadCameraSettings(bpy.types.PropertyGroup):
    """One camera's Road-Scene state: **only** whether it is recorded."""

    name: StringProperty(name="Name", default="camera")
    enable: BoolProperty(
        name="Record", default=True,
        description="Record this camera: one PNG per frame and one mp4 per camera")


def _ensure_camera_entries(settings) -> None:
    wanted = list(avm_cameras.CAMERAS)
    known = [entry.name for entry in settings.cameras]
    for index in range(len(known) - 1, -1, -1):
        if known[index] not in wanted:
            settings.cameras.remove(index)
    known = {entry.name for entry in settings.cameras}
    for key in wanted:
        if key not in known:
            settings.cameras.add().name = key


class RoadSceneSettings(bpy.types.PropertyGroup):
    """The whole Road Scene layout (Scene level)."""

    # -- the track (m) ------------------------------------------------------
    track_preset: EnumProperty(
        name="Loop", items=TRACK_ITEMS, default="compact",
        description="Named loop size: Compact (~124 m, a whole scenario lap in "
                    "~25-28 s) or Full (~248 m); Custom uses the sliders below",
        update=_apply_track_preset)
    straight_length: FloatProperty(
        name="Straight", default=road_track.DEFAULT_STRAIGHT_M,
        min=10.0, max=300.0, unit="LENGTH",
        description="Length of the two long straights; the loop scales with it",
        update=_schedule)
    curve_radius: FloatProperty(
        name="Curve Radius", default=road_track.DEFAULT_RADIUS_M,
        min=4.0, max=80.0, unit="LENGTH",
        description="Radius of the four 90 degree curves",
        update=_schedule)
    curve_bank_deg: FloatProperty(
        name="Curve Bank", default=0.0, min=-15.0, max=15.0,
        description="Peak lateral road bank on curves; tapers to zero at each segment join",
        update=_schedule)
    ramp_rise: FloatProperty(
        name="Ramp Rise", default=road_track.DEFAULT_RAMP_RISE_M,
        min=0.0, max=8.0, unit="LENGTH",
        description="Height gained on the up ramp (and lost on the down ramp)",
        update=_schedule)
    ramp_length: FloatProperty(
        name="Ramp Length", default=road_track.DEFAULT_RAMP_LENGTH_M,
        min=2.0, max=80.0, unit="LENGTH",
        description="Length of each ramp; a longer ramp is a gentler slope",
        update=_schedule)
    road_width: FloatProperty(
        name="Road Width", default=road_track.DEFAULT_ROAD_WIDTH_M,
        min=2.5, max=24.0, unit="LENGTH",
        description="Width of the driving surface (both lanes)",
        update=_schedule)
    shoulder_width: FloatProperty(
        name="Shoulder", default=road_track.DEFAULT_SHOULDER_M,
        min=0.0, max=6.0, unit="LENGTH",
        description="Width of the paved shoulder outside the lane markings",
        update=_schedule)
    ground_texture: EnumProperty(
        name="Surface", items=GROUND_TEXTURES, default="asphalt",
        description="Road surface material; asphalt / concrete / epoxy are "
                    "procedural, checker and plain are control experiments",
        update=_schedule)
    show_markings: BoolProperty(
        name="Lane Markings", default=True,
        description="Paint the edge lines and the dashed centre line - the "
                    "features a transparent-chassis algorithm is verified against",
        update=_schedule)
    show_crosswalk: BoolProperty(
        name="Zebra Crossing", default=True,
        description="Paint a zebra crossing just before the up ramp, with two "
                    "pedestrians waiting at its ends",
        update=_schedule)

    # -- parking (the manoeuvre at the start of the loop) -------------------
    parking: BoolProperty(
        name="Parking Bay", default=True,
        description="Start parked in a bay beside the road, pull out onto it, "
                    "and reverse back into the bay at the end - the clip then "
                    "contains a reversing manoeuvre",
        update=_schedule)
    parking_bays: IntProperty(
        name="Bays", default=3, min=1, max=12,
        description="Number of perpendicular bays painted at the start of the "
                    "road; the ego car uses the first one",
        update=_schedule)
    parking_speed: FloatProperty(
        name="Parking Speed", default=2.0, min=0.2, max=10.0, unit="VELOCITY",
        description="Speed of the pull-out / reverse-in manoeuvre",
        update=_schedule)

    # -- roadside props -----------------------------------------------------
    pedestrians: IntProperty(
        name="Pedestrians", default=6, min=0, max=48,
        description="People standing on the shoulders, alternating sides",
        update=_schedule)
    trees: IntProperty(
        name="Trees", default=10, min=0, max=80,
        description="Trees on the grass outside the embankment",
        update=_schedule)
    lamps: IntProperty(
        name="Street Lamps", default=8, min=0, max=40,
        description="Street lamps on the shoulder, arm over the road",
        update=_schedule)
    signs: IntProperty(
        name="Signs", default=3, min=0, max=24,
        description="Roadside signs on the shoulder",
        update=_schedule)
    animate_pedestrians: BoolProperty(
        name="Walk Pedestrians", default=False,
        description="Key the pedestrians walking along the shoulder. This makes "
                    "the scene dynamic, so the renderer can no longer cache it "
                    "across frames (slower) - it is the dynamic-content case for "
                    "M5 / CH-014",
        update=_schedule)
    pedestrian_speed: FloatProperty(
        name="Walk Speed", default=1.2, min=0.1, max=6.0, unit="VELOCITY",
        description="Walking speed of the animated pedestrians",
        update=_schedule)

    # -- lighting (dim world + one even sun, the Drive Scene's flat look) ---
    light_energy: FloatProperty(
        name="Sun", default=4.0, min=0.0, max=20.0,
        description="Sun strength [W/m2]; the world ambient is dim, so the loop "
                    "is lit by this one even light",
        update=_schedule)

    # -- vehicle (m) --------------------------------------------------------
    car_length: FloatProperty(name="Length", default=vehicle.BODY_LENGTH_M,
                              min=1.0, max=30.0, unit="LENGTH", update=_schedule)
    car_width: FloatProperty(name="Width", default=vehicle.BODY_WIDTH_M,
                             min=0.5, max=10.0, unit="LENGTH", update=_schedule)
    car_height: FloatProperty(name="Height", default=vehicle.BODY_HEIGHT_M,
                              min=0.5, max=10.0, unit="LENGTH", update=_schedule)
    car_clearance: FloatProperty(name="Clearance", default=vehicle.GROUND_CLEARANCE_M,
                                 min=0.0, max=2.0, unit="LENGTH", update=_schedule)

    # -- drive --------------------------------------------------------------
    drive_speed: FloatProperty(
        name="Cruise", default=7.0, min=0.05, max=40.0, unit="VELOCITY",
        description="Cruise speed (the scenario profile never exceeds it); "
                    "7 m/s is about 25 km/h",
        update=_schedule)
    drive_profile: EnumProperty(
        name="Profile", items=DRIVE_PROFILES, default="scenario",
        description="scenario starts at rest, slows at the zebra crossing and "
                    "brakes to a stop back at the start; constant / trapezoid are "
                    "the Drive Scene's profiles (control cases)",
        update=_schedule)
    drive_accel: FloatProperty(
        name="Accel", default=2.5, min=0.05, max=10.0, unit="ACCELERATION",
        description="Acceleration limit of the speed profile",
        update=_schedule)
    drive_decel: FloatProperty(
        name="Decel", default=2.5, min=0.05, max=10.0, unit="ACCELERATION",
        description="Braking limit of the speed profile; higher = a shorter "
                    "braking distance and a shorter clip",
        update=_schedule)
    slow_speed: FloatProperty(
        name="Crossing Speed", default=3.5, min=0.05, max=40.0, unit="VELOCITY",
        description="Speed the scenario profile slows to at the zebra crossing "
                    "(~11 km/h; raise it for a shorter clip, lower it for a "
                    "slower crossing)",
        update=_schedule)
    drive_fps: IntProperty(
        name="FPS", default=10, min=1, max=120,
        description="Frames per second of the clip",
        update=_schedule)
    drive_direction: EnumProperty(
        name="Direction", items=DIRECTIONS, default="forward",
        description="Forward drives nose-first; reverse drives the loop backwards",
        update=_schedule)
    drive_loops: FloatProperty(
        name="Loops", default=1.0, min=0.05, max=20.0,
        description="How many times around the loop (whole-loop drive only)",
        update=_schedule)
    drive_segment: EnumProperty(
        name="Segment", items=SEGMENT_ITEMS, default=LOOP_SEGMENT,
        description="Drive the whole loop or exactly one named segment - the "
                    "'road type x speed x direction' matrix picks segments",
        update=_schedule)

    # -- clip export --------------------------------------------------------
    clip_quality: EnumProperty(
        name="Quality", items=CLIP_QUALITIES, default="draft",
        description="Render quality of the exported clip; the image size and the "
                    "camera stay the same at every setting")
    clip_device: EnumProperty(
        name="Device", items=CLIP_DEVICES, default="cpu",
        description="Cycles render device for the export; the OpenCV camera is "
                    "an OSL shader, so only CPU and NVIDIA OptiX are valid")
    clip_keep_frames: BoolProperty(
        name="Keep Frames", default=True,
        description="Also keep the PNG sequence in the export")

    # -- cameras ------------------------------------------------------------
    active_camera: EnumProperty(
        name="Active Camera", items=CAMERA_ITEMS, default=avm_cameras.FRONT,
        description="Camera the viewport and F12 render; the clip records every "
                    "camera whose Record box is ticked",
        update=_schedule_active)
    cameras: CollectionProperty(type=RoadCameraSettings)

    # -- state --------------------------------------------------------------
    root: PointerProperty(
        name="Root",
        description="ROAD_Root empty; its presence is what makes the panels show",
        type=bpy.types.Object)
    revision: IntProperty(name="Revision", default=0, options={"HIDDEN"})
    clip_status: StringProperty(name="Clip", default="", options={"HIDDEN"})

    # -- helpers ------------------------------------------------------------
    def ensure_cameras(self) -> None:
        _ensure_camera_entries(self)

    def camera(self, key: str) -> Optional[RoadCameraSettings]:
        for entry in self.cameras:
            if entry.name == key:
                return entry
        return None

    def recorded_cameras(self) -> List[str]:
        disabled = {entry.name for entry in self.cameras if not entry.enable}
        return [key for key in avm_cameras.CAMERAS if key not in disabled]

    def reset_cameras(self) -> None:
        self.cameras.clear()
        _ensure_camera_entries(self)

    def track(self) -> road_track.RoadTrack:
        """The closed loop of the selected preset, or of the custom sliders."""
        if self.track_preset in road_track.TRACK_PRESETS:
            return road_track.preset_track(
                self.track_preset, center=True, curve_bank_deg=self.curve_bank_deg)
        return road_track.default_track(
            straight=self.straight_length, radius=self.curve_radius,
            ramp_rise=self.ramp_rise, ramp_length=self.ramp_length,
            curve_bank_deg=self.curve_bank_deg, center=True)

    def parking_radius(self) -> float:
        """Bay centre's distance from the centreline: the quarter-arc radius."""
        return self.road_width / 2.0 + self.shoulder_width + self.car_length / 2.0

    def parking_entry_s(self) -> float:
        """Where the pull-out arc meets the centreline (bay sits behind it)."""
        return self.parking_radius() + 2.0

    def parking_arc(self, track: road_track.RoadTrack):
        """The bay + pull-out arc, or ``None`` when parking is off."""
        if not self.parking:
            return None
        return road_track.parking_arc(track, self.parking_entry_s(),
                                      self.parking_radius())

    def segment_span(self, track: road_track.RoadTrack) -> Tuple[float, float]:
        """``(start_distance, loops)`` for the selected segment / whole loop.

        A reversing segment drive starts at the segment's **end** and runs back
        through it, so "reverse the up ramp" really is the up ramp - not the
        straight behind its start.
        """
        if self.drive_segment != LOOP_SEGMENT:
            for segment in track.segments:
                if segment.name == self.drive_segment:
                    start = (segment.start.s + segment.length
                             if self.drive_direction == "reverse" else segment.start.s)
                    return start, segment.length / track.length
        return 0.0, float(self.drive_loops)

    def plan(self):
        track = self.track()
        start, loops = self.segment_span(track)
        if self.drive_profile == road_path.SCENARIO:
            zones = ()
            if self.show_crosswalk:
                zone = road_track.crosswalk_slow_zone(track)
                if zone is not None:
                    zones = (zone,)
            if self.parking and self.drive_segment == LOOP_SEGMENT:
                return road_path.parking_plan(
                    track, s_entry=self.parking_entry_s(),
                    radius=self.parking_radius(), cruise=self.drive_speed,
                    parking_speed=self.parking_speed, slow_speed=self.slow_speed,
                    accel=self.drive_accel, decel=self.drive_decel,
                    fps=self.drive_fps, slow_zones=zones)
            return road_path.plan(
                track, speed=self.drive_speed, direction=self.drive_direction,
                profile=self.drive_profile, accel=self.drive_accel,
                decel=self.drive_decel, fps=self.drive_fps, loops=loops,
                start_distance=start, slow_speed=self.slow_speed,
                slow_zones=zones)
        return road_path.plan(
            track, speed=self.drive_speed, direction=self.drive_direction,
            profile=self.drive_profile, accel=self.drive_accel, fps=self.drive_fps,
            loops=loops, start_distance=start)


_CLASSES = (RoadCameraSettings, RoadSceneSettings)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.road_scene = PointerProperty(
        name="Road Scene",
        description="Closed test road: straights, curves, ramps and roadside props",
        type=RoadSceneSettings,
    )


def unregister() -> None:
    del bpy.types.Scene.road_scene
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
