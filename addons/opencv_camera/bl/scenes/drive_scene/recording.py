"""Record a drive clip: one PNG per frame, plus the per-frame truth.

The output of this module is the contract with the algorithm side:

* ``frame_%04d.png`` - the front camera image of that frame;
* ``frames.csv`` - one row per frame: time, distance, speed, the vehicle's world
  pose and the camera's **world** pose (vehicle pose composed with the fixed
  mount pose);
* ``clip.json`` - everything needed to reproduce the clip (path and speed
  parameters, camera K / D and vehicle-frame mount pose, render settings);
* ``clip.mp4`` - the same frames as an H.264 video (``export_zip`` only).

The rows come from :mod:`core.scenes.drive_path`, the very plan the vehicle was
keyframed with, so a PNG and its CSV row can never describe different poses.

:class:`ClipJob` is the stepwise engine: the modal export operator renders one
frame per UI tick (so it can show progress / an ETA and be cancelled), while
:func:`render_clip` / :func:`export_zip` drive the very same job synchronously
for tests and headless use.
"""

from __future__ import annotations

import datetime
import json
import math
import os
import shutil
import tempfile
import time
import zipfile
from typing import Dict, List, Optional

import bpy

from ....core.scenes import drive_path
from ... import apply as apply_mod
from ... import compat
from . import builder
from . import properties as drive_properties

FORMAT = "drive_clip"
VERSION = 1
#: the video inside the exported zip
VIDEO_NAME = "clip.mp4"
#: the default zip name of [Export Clip…] - a stable name, not the .blend's
DEFAULT_NAME = "drive_scene"

#: Cycles compute backends that can evaluate the OSL camera shader.  OSL is
#: supported on CPU and NVIDIA OptiX only - Metal / CUDA / HIP / oneAPI fall
#: back to a wrong camera, so the export refuses to use them.
_OSL_CAMERA_DEVICES = ("OPTIX",)


class RenderCancelled(Exception):
    """The user aborted the render (ESC / the render window was closed)."""


def default_filename() -> str:
    """The default ``[Export Clip…]`` file name (``drive_scene.zip``)."""
    return f"{DEFAULT_NAME}.zip"


def frame_name(index: int) -> str:
    return f"frame_{index:04d}.png"


def compute_device_type() -> str:
    """The Cycles preference's active compute device type (``"NONE"`` if unset).

    Read through the Cycles add-on preferences; every access is guarded so a
    missing / disabled Cycles add-on cannot break the export.
    """
    try:
        addon = bpy.context.preferences.addons.get("cycles")
        preferences = getattr(addon, "preferences", None) if addon is not None else None
        return str(getattr(preferences, "compute_device_type", "NONE") or "NONE")
    except Exception:
        return "NONE"


def gpu_can_render_osl_camera(device_type: str) -> bool:
    """True when the GPU backend can evaluate the OSL camera (OptiX only)."""
    return str(device_type).upper() in _OSL_CAMERA_DEVICES


def enabled_osl_gpu_available() -> bool:
    """True when an OptiX GPU device is enabled in the Cycles preferences."""
    try:
        addon = bpy.context.preferences.addons.get("cycles")
        preferences = getattr(addon, "preferences", None) if addon is not None else None
        if preferences is None:
            return False
        preferences.get_devices()
        for device in preferences.devices:
            if (getattr(device, "use", False)
                    and gpu_can_render_osl_camera(getattr(device, "type", ""))):
                return True
    except Exception:
        pass
    return False


def resolve_device(requested: str, messages: List[str]) -> str:
    """The Cycles device to render with: ``"CPU"`` or ``"GPU"``.

    CPU is always correct.  GPU is only usable when the active compute device is
    OptiX (the one backend that runs OSL camera shaders) and an OptiX device is
    enabled; any other case is reported and falls back to CPU so the clip's
    camera stays correct.
    """
    if str(requested).lower() != "gpu":
        return "CPU"
    device_type = compute_device_type()
    if gpu_can_render_osl_camera(device_type) and enabled_osl_gpu_available():
        return "GPU"
    if gpu_can_render_osl_camera(device_type):
        reason = "no OptiX GPU device is enabled in the Cycles preferences"
    else:
        reason = f"{device_type} cannot run the OSL camera"
    messages.append(
        f"the OpenCV camera is an OSL shader, which Cycles evaluates on CPU or "
        f"OptiX only; {reason} - rendering on CPU instead")
    return "CPU"


def format_duration(seconds: Optional[float]) -> str:
    """``m:ss`` (or ``h:mm:ss``) for an ETA; ``--:--`` when unknown."""
    if seconds is None:
        return "--:--"
    try:
        value = float(seconds)
    except (TypeError, ValueError):
        return "--:--"
    if math.isnan(value) or math.isinf(value) or value < 0:
        return "--:--"
    total = int(round(value))
    hours, rest = divmod(total, 3600)
    minutes, secs = divmod(rest, 60)
    if hours:
        return f"{hours}:{minutes:02d}:{secs:02d}"
    return f"{minutes}:{secs:02d}"


def progress_text(done: int, total: int, eta: Optional[float]) -> str:
    """``frame k/N · xx% · ETA m:ss`` for the panel / status line."""
    if total <= 0:
        return "export: nothing to render"
    percent = int(round(100.0 * done / total))
    return f"frame {done}/{total} · {percent}% · ETA {format_duration(eta)}"


def camera_mount() -> drive_path.Mount:
    """The front camera's fixed pose in the vehicle frame (metres, degrees)."""
    camera = bpy.data.objects.get(builder.CAMERA_NAME)
    if camera is None:
        return drive_path.Mount()
    return drive_path.Mount(
        location=tuple(float(value) for value in camera.location),
        rotation_deg=tuple(math.degrees(float(value)) for value in camera.rotation_euler),
    )


def clip_meta(scene: bpy.types.Scene, settings, plan, samples: int,
              video: str = "", quality: str = "", device: str = "") -> Dict:
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
            "device": device,
            "samples": int(samples),
            "quality": quality,
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
        "video": video,
        "time_base": "simulated: t = frame / fps, there is no absolute clock",
    }


class ClipJob:
    """A stepwise clip render shared by the modal operator and the sync API.

    ``start()`` applies the render settings (quality preset, device, camera),
    ``step()`` renders exactly one frame, and ``finish()`` writes the truth, the
    video and the zip before restoring everything.  ``cancel()`` restores without
    writing anything.  The caller chooses the cadence: a modal timer in the UI,
    a plain loop in tests / headless runs.
    """

    def __init__(self, context, settings, filepath: str = "", directory: str = "",
                 samples: int = 0, encode: bool = True, pack: bool = True,
                 video: str = "") -> None:
        self.context = context
        self.scene = context.scene
        self.settings = settings
        self.filepath = filepath
        self.directory = directory
        self.samples = int(samples)
        self.encode = bool(encode)
        self.pack = bool(pack)
        self.video = video or (VIDEO_NAME if encode else "")
        self.messages: List[str] = []
        self.plan = None
        self.frames: List = []
        self.total = 0
        self.done = 0
        self.written: List[str] = []
        self.quality = ""
        self.device = "CPU"
        self.sample_count = 0
        self._camera = None
        self._saved: Dict = {}
        self._other_lights: List = []
        self._started = False
        self._finished = False
        self._owns_directory = False
        self._t0 = 0.0

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> "ClipJob":
        if self._started:
            return self
        camera = bpy.data.objects.get(builder.CAMERA_NAME)
        if camera is None:
            raise RuntimeError("the Drive Scene has no front camera (build it first)")
        self._camera = camera
        self.plan = self.settings.plan()
        self.frames = list(self.plan.frames)
        self.total = len(self.frames)
        if not self.directory:
            self.directory = tempfile.mkdtemp(prefix="drive_clip_")
            self._owns_directory = True
        os.makedirs(self.directory, exist_ok=True)
        try:
            self._apply()
        except Exception:
            self._restore()
            raise
        self.done = 0
        self._t0 = time.time()
        self._started = True
        return self

    def _apply(self) -> None:
        scene = self.scene
        render = scene.render
        cycles = scene.cycles
        quality = str(getattr(self.settings, "clip_quality", "draft"))
        preset = drive_properties.clip_quality_preset(quality)
        if self.samples > 0:
            preset["samples"] = self.samples
        self.quality = quality
        self.sample_count = int(preset["samples"])
        self.device = resolve_device(
            str(getattr(self.settings, "clip_device", "cpu")), self.messages)
        # the file around the scene usually carries lights of its own (a fresh
        # Blender scene has a point light); they would change the clip's lighting
        # from one recording to the next, so they are muted for the duration
        self._other_lights = [
            (light, light.hide_render) for light in scene.objects
            if light.type == "LIGHT"
            and not light.name.startswith(builder.LIGHT_PREFIX)]
        self._saved = {
            "filepath": render.filepath,
            "file_format": render.image_settings.file_format,
            "engine": render.engine,
            "device": cycles.device,
            "samples": cycles.samples,
            "denoising": cycles.use_denoising,
            "adaptive": cycles.use_adaptive_sampling,
            "adaptive_threshold": cycles.adaptive_threshold,
            "max_bounces": cycles.max_bounces,
            "diffuse_bounces": cycles.diffuse_bounces,
            "glossy_bounces": cycles.glossy_bounces,
            "caustics_reflective": cycles.caustics_reflective,
            "caustics_refractive": cycles.caustics_refractive,
            "persistent": render.use_persistent_data,
            "threads_mode": render.threads_mode,
            "threads": render.threads,
            "frame_start": scene.frame_start,
            "frame_end": scene.frame_end,
            "frame_step": scene.frame_step,
            "camera": scene.camera,
            "frame": scene.frame_current,
        }
        if render.engine != "CYCLES":
            render.engine = "CYCLES"
        cycles.device = self.device
        cycles.samples = self.sample_count
        cycles.use_denoising = bool(preset.get("denoise", self._saved["denoising"]))
        cycles.max_bounces = int(preset.get("max_bounces", self._saved["max_bounces"]))
        cycles.diffuse_bounces = int(preset.get("diffuse_bounces",
                                                self._saved["diffuse_bounces"]))
        cycles.glossy_bounces = int(preset.get("glossy_bounces",
                                               self._saved["glossy_bounces"]))
        if "adaptive_threshold" in preset:
            cycles.use_adaptive_sampling = True
            cycles.adaptive_threshold = float(preset["adaptive_threshold"])
        if "caustics" in preset:
            cycles.caustics_reflective = bool(preset["caustics"])
            cycles.caustics_refractive = bool(preset["caustics"])
        # the scene is static apart from the vehicle, so caching it across the
        # per-frame stills is free quality-wise and saves the sync every frame
        render.use_persistent_data = True
        render.threads_mode = "AUTO"
        for light, _ in self._other_lights:
            light.hide_render = True
        render.image_settings.file_format = "PNG"
        scene.camera = self._camera
        ok, messages = apply_mod.apply_settings(
            self._camera.data, self._camera.data.opencv_cam, scene)
        if not ok:
            raise RuntimeError("; ".join(messages))
        scene.frame_start = self.frames[0].index
        scene.frame_end = self.frames[-1].index
        scene.frame_step = 1

    def _restore(self) -> None:
        if not self._saved:
            return
        scene = self.scene
        render = scene.render
        cycles = scene.cycles
        saved = self._saved
        self._saved = {}
        for light, hidden in self._other_lights:
            light.hide_render = hidden
        render.filepath = saved["filepath"]
        render.image_settings.file_format = saved["file_format"]
        render.engine = saved["engine"]
        cycles.device = saved["device"]
        cycles.samples = saved["samples"]
        cycles.use_denoising = saved["denoising"]
        cycles.use_adaptive_sampling = saved["adaptive"]
        cycles.adaptive_threshold = saved["adaptive_threshold"]
        cycles.max_bounces = saved["max_bounces"]
        cycles.diffuse_bounces = saved["diffuse_bounces"]
        cycles.glossy_bounces = saved["glossy_bounces"]
        cycles.caustics_reflective = saved["caustics_reflective"]
        cycles.caustics_refractive = saved["caustics_refractive"]
        render.use_persistent_data = saved["persistent"]
        render.threads_mode = saved["threads_mode"]
        render.threads = saved["threads"]
        scene.frame_step = saved["frame_step"]
        scene.frame_start = saved["frame_start"]
        scene.frame_end = saved["frame_end"]
        scene.camera = saved["camera"]
        scene.frame_set(saved["frame"])

    # -- stepping -----------------------------------------------------------
    def step(self) -> None:
        """Render the next frame (one call = one frame)."""
        if self.done >= self.total:
            return
        frame = self.frames[self.done]
        render = self.scene.render
        self.scene.frame_set(frame.index)
        render.filepath = os.path.join(self.directory, frame_name(frame.index))
        result = bpy.ops.render.render(write_still=True)
        if "CANCELLED" in result:
            raise RenderCancelled("the clip render was cancelled")
        self.written.append(render.filepath)
        self.done += 1

    # -- progress -----------------------------------------------------------
    def elapsed(self) -> float:
        return max(0.0, time.time() - self._t0) if self._t0 else 0.0

    def eta_seconds(self) -> Optional[float]:
        if self.done <= 0 or self.done >= self.total:
            return None
        return self.elapsed() / self.done * (self.total - self.done)

    def status_text(self) -> str:
        return progress_text(self.done, self.total, self.eta_seconds())

    # -- finish / cancel ----------------------------------------------------
    def _write_truth(self) -> Dict:
        csv_path = os.path.join(self.directory, "frames.csv")
        with open(csv_path, "w", encoding="utf-8") as handle:
            handle.write(drive_path.csv_text(self.plan, camera_mount()))
        meta = clip_meta(self.scene, self.settings, self.plan, self.sample_count,
                         video=self.video, quality=self.quality, device=self.device)
        meta_path = os.path.join(self.directory, "clip.json")
        with open(meta_path, "w", encoding="utf-8") as handle:
            handle.write(json.dumps(meta, indent=2) + "\n")
        self.written.extend([csv_path, meta_path])
        return meta

    def _pack(self) -> None:
        parent = os.path.dirname(os.path.abspath(self.filepath))
        if parent:
            os.makedirs(parent, exist_ok=True)
        with zipfile.ZipFile(self.filepath, "w", zipfile.ZIP_DEFLATED) as archive:
            for name in sorted(os.listdir(self.directory)):
                archive.write(os.path.join(self.directory, name), arcname=name)

    def finish(self) -> Dict:
        if self._finished:
            return self.report()
        try:
            if self.encode:
                encode_video(self.directory, self.plan,
                             os.path.join(self.directory, VIDEO_NAME))
            meta = self._write_truth()
            if self.pack:
                self._pack()
        finally:
            self._restore()
            if self._owns_directory:
                shutil.rmtree(self.directory, ignore_errors=True)
        self._finished = True
        return {"directory": self.directory, "frames": self.total,
                "files": list(self.written), "plan": self.plan, "meta": meta,
                "filepath": self.filepath, "quality": self.quality,
                "device": self.device, "messages": list(self.messages)}

    def cancel(self) -> None:
        self._restore()
        if self._owns_directory:
            shutil.rmtree(self.directory, ignore_errors=True)
        self._finished = True

    def report(self) -> Dict:
        return {"directory": self.directory, "frames": self.total,
                "files": list(self.written), "plan": self.plan,
                "filepath": self.filepath, "quality": self.quality,
                "device": self.device, "messages": list(self.messages)}


def encode_video(directory: str, plan, output_path: str) -> str:
    """Encode the rendered PNG sequence into an H.264 mp4 with Blender's FFmpeg.

    A throwaway sequencer scene turns the stills into a movie **without
    re-rendering** the 3D scene; it is removed again whatever happens.
    """
    scene = bpy.data.scenes.new("__drive_encode__")
    try:
        scene.render.fps = max(1, int(round(plan.fps)))
        scene.render.fps_base = 1.0
        compat.enable_movie_output(scene.render.image_settings)
        scene.render.ffmpeg.format = "MPEG4"
        scene.render.ffmpeg.codec = "H264"
        scene.render.ffmpeg.constant_rate_factor = "HIGH"
        scene.render.filepath = output_path
        scene.render.use_sequencer = True
        scene.frame_start = plan.frames[0].index
        scene.frame_end = plan.frames[-1].index
        strips = compat.sequence_strips(scene)
        strip = strips.new_image("clip", os.path.join(directory, frame_name(plan.frames[0].index)),
                                 1, plan.frames[0].index)
        for frame in plan.frames[1:]:
            strip.elements.append(frame_name(frame.index))
        result = bpy.ops.render.render(animation=True, scene=scene.name)
        if "CANCELLED" in result:
            raise RenderCancelled("the clip encoding was cancelled")
    finally:
        bpy.data.scenes.remove(scene)
    return output_path


def render_clip(context, settings, directory: str, samples: int = 0,
                video: str = "") -> Dict:
    """Render every frame of the plan into ``directory`` (no video / zip).

    The render cost comes from ``settings.clip_quality``; a positive ``samples``
    overrides its sample count (tests / scripts).  Only the render cost changes -
    the output size and the camera (K / D) are the same at every quality.  Every
    render setting that is touched is saved and restored, so recording never
    changes the user's setup.
    """
    job = ClipJob(context, settings, directory=directory, samples=samples,
                  encode=False, pack=False, video=video)
    job.start()
    try:
        while job.done < job.total:
            job.step()
        return job.finish()
    except RenderCancelled:
        job.cancel()
        raise


def export_zip(context, settings, filepath: str, samples: int = 0) -> Dict:
    """Render the clip, encode the video and pack it all into one zip.

    The zip holds the PNG sequence, ``frames.csv`` (per-frame speed, vehicle and
    camera pose), ``clip.json`` (K / D, mount pose, drive and render parameters)
    and ``clip.mp4``.  The PNGs are written to a temporary directory and removed
    once the zip is closed.
    """
    job = ClipJob(context, settings, filepath=filepath, samples=samples,
                  encode=True, pack=True)
    job.start()
    try:
        while job.done < job.total:
            job.step()
        return job.finish()
    except RenderCancelled:
        job.cancel()
        return {"filepath": filepath, "frames": 0, "plan": settings.plan(),
                "cancelled": True}
