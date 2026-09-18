"""Record a drive clip: one PNG per frame, plus the per-frame truth.

The output of this module is the contract with the algorithm side:

* ``frame_%04d.png`` - the front camera image of that frame;
* ``frames.csv`` - one row per frame: time, distance, speed and world pose;
* ``clip.json`` - everything needed to reproduce the clip (path and speed
  parameters, camera K / D and vehicle-frame mount pose, render settings).

The rows come from :mod:`core.scenes.drive_path`, the very plan the vehicle was
keyframed with, so a PNG and its CSV row can never describe different poses.
"""

from __future__ import annotations

import datetime
import json
import math
import os
from typing import Dict, List

import bpy

from ....core.scenes import drive_path
from ... import apply as apply_mod
from . import builder

FORMAT = "drive_clip"
VERSION = 1


def frame_name(index: int) -> str:
    return f"frame_{index:04d}.png"


def clip_meta(scene: bpy.types.Scene, settings, plan, samples: int) -> Dict:
    """The clip's self-description (what was rendered / driven / recorded)."""
    width, height = apply_mod.render_resolution(scene)
    camera = bpy.data.objects.get(builder.CAMERA_NAME)
    entry: Dict = {
        "name": builder.CAMERA_NAME,
        "model": "fisheye",
        "output": [int(width), int(height)],
        "mount": "vehicle",
    }
    if camera is not None:
        intrinsics = apply_mod.effective_intrinsics(
            camera.data.opencv_cam, width, height)
        distortion = camera.data.opencv_cam.distortion
        entry["K"] = [float(intrinsics.fx), float(intrinsics.fy),
                      float(intrinsics.cx), float(intrinsics.cy)]
        entry["D"] = [float(distortion.k1), float(distortion.k2),
                      float(distortion.k3), float(distortion.k4)]
        entry["mount"] = {
            "frame": "vehicle",
            "location": [float(value) for value in camera.location],
            "rotation_deg": [math.degrees(float(value))
                             for value in camera.rotation_euler],
        }
    return {
        "format": FORMAT,
        "version": VERSION,
        "created": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "fps": plan.fps,
        "frames": len(plan.frames),
        "duration_s": round(plan.duration, 6),
        "drive": {
            "distance_m": plan.distance,
            "profile": plan.profile,
            "cruise_speed_mps": plan.cruise_speed,
            "accel_mps2": plan.accel,
            "heading_deg": plan.heading,
            "start_xy": list(plan.start),
            "vehicle_frame": "world: X/Y/Z euler, Z up, nose +Y at yaw 0",
        },
        "render": {
            "engine": "CYCLES",
            "samples": int(samples),
            "resolution": [int(width), int(height)],
        },
        "lot": {
            "texture": settings.ground_texture,
            "markings": bool(settings.show_bays),
            "aisle_width_m": float(settings.aisle_width),
            "bay_depth_m": float(settings.bay_depth),
            "bay_width_m": float(settings.bay_width),
        },
        "camera": entry,
        "frames_csv": "frames.csv",
        "time_base": "simulated: t = frame / fps, there is no absolute clock",
    }


def render_clip(context, settings, directory: str, samples: int = 64) -> Dict:
    """Render every frame of the plan and write the clip's files.

    The scene's render settings (filepath, format, samples, camera, current
    frame) are saved and restored, so recording never changes the user's setup.
    """
    scene = context.scene
    plan = settings.plan()
    camera = bpy.data.objects.get(builder.CAMERA_NAME)
    if camera is None:
        raise RuntimeError("the Drive Scene has no front camera (build it first)")
    os.makedirs(directory, exist_ok=True)

    render = scene.render
    # the file around the scene usually carries lights of its own (a fresh
    # Blender scene has a point light); they would change the clip's lighting
    # from one recording to the next, so they are muted for the duration
    other_lights = [(light, light.hide_render) for light in scene.objects
                    if light.type == "LIGHT"
                    and not light.name.startswith(builder.LIGHT_PREFIX)]
    saved = {
        "filepath": render.filepath,
        "file_format": render.image_settings.file_format,
        "samples": scene.cycles.samples,
        "camera": scene.camera,
        "frame": scene.frame_current,
    }
    written: List[str] = []
    try:
        if scene.render.engine != "CYCLES":
            scene.render.engine = "CYCLES"
        scene.cycles.samples = int(samples)
        for light, _ in other_lights:
            light.hide_render = True
        render.image_settings.file_format = "PNG"
        scene.camera = camera
        ok, messages = apply_mod.apply_settings(
            camera.data, camera.data.opencv_cam, scene)
        if not ok:
            raise RuntimeError("; ".join(messages))
        for frame in plan.frames:
            scene.frame_set(frame.index)
            render.filepath = os.path.join(directory, frame_name(frame.index))
            bpy.ops.render.render(write_still=True)
            written.append(render.filepath)
    finally:
        for light, hidden in other_lights:
            light.hide_render = hidden
        render.filepath = saved["filepath"]
        render.image_settings.file_format = saved["file_format"]
        scene.cycles.samples = saved["samples"]
        scene.camera = saved["camera"]
        scene.frame_set(saved["frame"])

    csv_path = os.path.join(directory, "frames.csv")
    with open(csv_path, "w", encoding="utf-8") as handle:
        handle.write(drive_path.csv_text(plan))
    meta_path = os.path.join(directory, "clip.json")
    with open(meta_path, "w", encoding="utf-8") as handle:
        handle.write(json.dumps(clip_meta(scene, settings, plan, samples), indent=2) + "\n")
    written.extend([csv_path, meta_path])
    return {"directory": directory, "frames": len(plan.frames), "files": written,
            "plan": plan}
