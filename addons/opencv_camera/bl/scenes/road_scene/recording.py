"""Record a Road Scene clip: the scene's binding of :mod:`bl.scenes.clip_core`.

The engine is shared with the Drive Scene; this module is only the Road Scene's
profile.  Its clip differs in three deliberate ways, all of them M5's:

* ``frames.csv`` carries the vertical pose (``z_m``, ``pitch_deg``, ``roll_deg``)
  and the matrix labels (``segment``, ``road_type``, ``direction``) after the
  Drive Scene's seven columns;
* ``clip.json`` has ``motion`` and ``segments`` blocks describing the loop and
  every labelled piece of it, so a boundary result can name the road type it came
  from;
* ``version`` is **3** (the ``cameras`` array stays, so a v2 reader still parses
  the file; the new columns are additions, not renames).

The rows come from :mod:`core.scenes.road_path`, the very plan the vehicle was
keyframed with, so a PNG and its CSV row cannot disagree.
"""

from __future__ import annotations

from typing import Dict, List, Sequence

from ....core.scenes import road_path, vehicle
from .. import clip_core
from . import builder

FORMAT = "road_clip"
#: 2 added the ``cameras`` array; 3 adds the vertical pose columns and the loop
#: segment labels to a clip whose ``cameras`` array is unchanged
VERSION = 3
DEFAULT_NAME = "road_scene"

FRAME_PATTERN = clip_core.FRAME_PATTERN
VIDEO_PATTERN = clip_core.VIDEO_PATTERN
OUTPUT_VIEW_TRANSFORM = clip_core.OUTPUT_VIEW_TRANSFORM
OUTPUT_LOOK = clip_core.OUTPUT_LOOK

RenderCancelled = clip_core.RenderCancelled
ClipJob = clip_core.ClipJob


def _motion_meta(settings, plan) -> Dict:
    track = plan.track
    return {
        "motion": {
            "distance_m": plan.distance,
            "profile": plan.profile,
            "cruise_speed_mps": plan.cruise_speed,
            "accel_mps2": plan.accel,
            "decel_mps2": float(getattr(settings, "drive_decel", plan.accel)),
            "slow_speed_mps": float(getattr(settings, "slow_speed", 0.0)),
            "direction": plan.direction,
            "loops": plan.loops,
            "start_distance_m": plan.start_distance,
            "loop_length_m": round(track.length, 6),
            "road_width_m": float(settings.road_width),
            "shoulder_m": float(settings.shoulder_width),
            "parking": bool(getattr(settings, "parking", False)),
            "parking_speed_mps": float(getattr(settings, "parking_speed", 0.0)),
            "vehicle_frame": ("world: X/Y/Z euler, Z up, nose +Y at yaw 0; "
                              "z/pitch carry the road slope"),
        },
        "segments": track.describe(),
        # the per-frame vehicle signals an AVM algorithm consumes, and where
        # they live in frames.csv (speed is the Drive Scene's column)
        "signals": {
            "columns": ["speed_mps", "steering_deg", "gear"],
            "speed": "speed_mps",
            "steering": "steering_deg",
            "steering_frame": ("front-wheel angle, left positive; bicycle model "
                               "tan(delta) = wheel_base * curvature, sign flips "
                               "when reversing"),
            "gear": "gear",
            "gear_values": ["P", "R", "D"],
        },
    }


def _scene_meta(settings, plan) -> Dict:
    return {"road": {
        "preset": str(getattr(settings, "track_preset", "custom")),
        "surface": settings.ground_texture,
        "markings": bool(settings.show_markings),
        "crosswalk": bool(settings.show_crosswalk),
        "ramp_rise_m": float(settings.ramp_rise),
        "ramp_length_m": float(settings.ramp_length),
        "curve_radius_m": float(settings.curve_radius),
        "curve_bank_deg": float(getattr(settings, "curve_bank_deg", 0.0)),
        "pedestrians": int(settings.pedestrians),
        "animate_pedestrians": bool(settings.animate_pedestrians),
    }}


def _vehicle_block(settings) -> Dict:
    return vehicle.block(
        length=float(settings.car_length), width=float(settings.car_width),
        height=float(settings.car_height),
        clearance=float(settings.car_clearance))


def _persistent_data(settings) -> bool:
    """A static road may be cached; walking pedestrians may not."""
    return not bool(getattr(settings, "animate_pedestrians", False))


PROFILE = clip_core.ClipProfile(
    format=FORMAT, version=VERSION, default_name=DEFAULT_NAME,
    light_prefix=builder.LIGHT_PREFIX, camera_name=builder.camera_name,
    csv_text=road_path.csv_text, motion_meta=_motion_meta, scene_meta=_scene_meta,
    vehicle_block=_vehicle_block, persistent_data=_persistent_data)


def default_filename() -> str:
    return clip_core.default_filename(PROFILE)


def frame_name(index: int, camera: str) -> str:
    return clip_core.frame_name(index, camera)


def video_name(camera: str) -> str:
    return clip_core.video_name(camera)


def camera_mounts(cameras: Sequence[str]) -> Dict:
    return clip_core.camera_mounts(PROFILE, cameras)


def camera_size(key: str):
    return clip_core.camera_size(PROFILE, key)


def render_clip(context, settings, directory: str, samples: int = 0) -> Dict:
    return clip_core.render_clip(PROFILE, context, settings, directory, samples)


def export_zip(context, settings, filepath: str, samples: int = 0) -> Dict:
    return clip_core.export_zip(PROFILE, context, settings, filepath, samples)
