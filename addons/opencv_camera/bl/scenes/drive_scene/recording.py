"""Record a Drive Scene clip: the scene's binding of :mod:`bl.scenes.clip_core`.

The engine (still/video/CSV/JSON, per-frame progress, device resolution) lives in
:mod:`opencv_camera.bl.scenes.clip_core` and is shared with the Road Scene.  This
module is only the Drive Scene's profile: its object prefixes, its ``drive`` and
``lot`` blocks in ``clip.json``, its default file name and its plan/CSV writers.

Contract v2: one named camera group per recorded camera.  ``frames.csv`` and
``clip.json``'s ``cameras`` order is ``avm_cameras.CAMERAS`` filtered by what the
user enabled.  The rows come from :mod:`core.scenes.drive_path`, the very plan
the vehicle was keyframed with, so a PNG and its CSV row cannot disagree.
"""

from __future__ import annotations

from typing import Dict, List, Sequence

from ....core.scenes import drive_path, vehicle
from .. import clip_core
from . import builder

#: 1 = a single unnamed ``camera`` (front only); 2 = a ``cameras`` array, one per
#: recorded camera, with named ``frames.csv`` column groups.
VERSION = 2
FORMAT = "drive_clip"
#: the default zip name of [Export Clip…] - a stable name, not the .blend's
DEFAULT_NAME = "drive_scene"

FRAME_PATTERN = clip_core.FRAME_PATTERN
VIDEO_PATTERN = clip_core.VIDEO_PATTERN
OUTPUT_VIEW_TRANSFORM = clip_core.OUTPUT_VIEW_TRANSFORM
OUTPUT_LOOK = clip_core.OUTPUT_LOOK
VIDEO_FORMAT = clip_core.VIDEO_FORMAT
VIDEO_CODEC = clip_core.VIDEO_CODEC
VIDEO_CRF = clip_core.VIDEO_CRF

RenderCancelled = clip_core.RenderCancelled
ClipJob = clip_core.ClipJob


def _motion_meta(settings, plan) -> Dict:
    return {"drive": {
        "distance_m": plan.distance,
        "profile": plan.profile,
        "cruise_speed_mps": plan.cruise_speed,
        "accel_mps2": plan.accel,
        "heading_deg": plan.heading,
        "start_xy": list(plan.start),
        "vehicle_frame": "world: X/Y/Z euler, Z up, nose +Y at yaw 0",
    }}


def _scene_meta(settings, plan) -> Dict:
    return {"lot": {
        "texture": settings.ground_texture,
        "markings": bool(settings.show_bays),
        "aisle_width_m": float(settings.aisle_width),
        "bay_depth_m": float(settings.bay_depth),
        "bay_width_m": float(settings.bay_width),
    }}


def _vehicle_block(settings) -> Dict:
    return vehicle.block(
        length=float(settings.car_length), width=float(settings.car_width),
        height=float(settings.car_height),
        clearance=float(settings.car_clearance))


PROFILE = clip_core.ClipProfile(
    format=FORMAT, version=VERSION, default_name=DEFAULT_NAME,
    light_prefix=builder.LIGHT_PREFIX, camera_name=builder.camera_name,
    csv_text=drive_path.csv_text, motion_meta=_motion_meta,
    scene_meta=_scene_meta, vehicle_block=_vehicle_block)


# -- thin wrappers so the rest of the add-on / tests keep their imports ------
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


def camera_entry(key: str, width: int, height: int) -> Dict:
    return clip_core.camera_entry(PROFILE, key, width, height)


def video_field(cameras: Sequence[str]) -> str:
    return clip_core.video_field(cameras)


def compute_device_type() -> str:
    return clip_core.compute_device_type()


def gpu_can_render_osl_camera(device_type: str) -> bool:
    return clip_core.gpu_can_render_osl_camera(device_type)


def enabled_osl_gpu_available() -> bool:
    return clip_core.enabled_osl_gpu_available()


def resolve_device(requested: str, messages: List[str]) -> str:
    return clip_core.resolve_device(requested, messages)


def format_duration(seconds) -> str:
    return clip_core.format_duration(seconds)


def progress_text(done: int, total: int, eta) -> str:
    return clip_core.progress_text(done, total, eta)


def clip_quality_preset(key: str) -> dict:
    return clip_core.clip_quality_preset(key)


def still_size(path: str):
    return clip_core.still_size(path)


def video_encode_meta(fps, view_transform=OUTPUT_VIEW_TRANSFORM, look=OUTPUT_LOOK) -> Dict:
    return clip_core.video_encode_meta(fps, view_transform, look)


def encode_video(directory, plan, camera, output_path,
                 view_transform=OUTPUT_VIEW_TRANSFORM, look=OUTPUT_LOOK) -> str:
    return clip_core.encode_video(directory, plan, camera, output_path,
                                  view_transform, look)


def render_clip(context, settings, directory: str, samples: int = 0) -> Dict:
    return clip_core.render_clip(PROFILE, context, settings, directory, samples)


def export_zip(context, settings, filepath: str, samples: int = 0) -> Dict:
    return clip_core.export_zip(PROFILE, context, settings, filepath, samples)
